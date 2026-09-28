// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package first.robot.util;

import static org.wpilib.units.Units.Seconds;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Nat;
import org.wpilib.units.measure.Time;

import first.robot.util.Constants.RobotGeometryConstants;

/**
 * Fuses swerve odometry with vision observations using a closed-form Kalman gain, maintaining both a
 * raw odometry pose and a vision-corrected estimate.
 *
 * <p>The two optional behaviours — scaling odometry down when the robot is tilted, and clamping the
 * estimate inside the field walls — are injected as predicates rather than read from NetworkTables
 * here, so the filter stays independent of the dashboard and testable without one. On the robot,
 * pass {@code Tuning.kOdometryTiltCompensation} and {@code Tuning.kWallClamp}.
 */
public class PoseEstimator {
  private static final double kPoseBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> kOdometryStdDevs = new Matrix<>(VecBuilder.fill(1, 1, 0.2));
  private static final double kMaxTiltDegrees = 25.0;

  private final SwerveDriveKinematics m_kinematics;
  private final BooleanSupplier m_tiltCompensationEnabled;
  private final BooleanSupplier m_wallClampEnabled;

  private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer = TimeInterpolatableBuffer
      .createBuffer(kPoseBufferSizeSec);
  private final Matrix<N3, N1> m_qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private Pose2d m_odometryPose = Pose2d.ZERO;
  private Pose2d m_estimatedPose = Pose2d.ZERO;
  private Rotation2d m_gyroOffset = Rotation2d.ZERO;
  private SwerveModulePosition[] m_lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  public PoseEstimator(
      SwerveDriveKinematics kinematics,
      BooleanSupplier tiltCompensationEnabled,
      BooleanSupplier wallClampEnabled) {
    m_kinematics = kinematics;
    m_tiltCompensationEnabled = tiltCompensationEnabled;
    m_wallClampEnabled = wallClampEnabled;

    for (int i = 0; i < 3; ++i) {
      m_qStdDevs.set(i, 0, Math.pow(kOdometryStdDevs.get(i, 0), 2));
    }
  }

  /** Reset the pose estimate and odometry pose to the given pose. */
  public void resetPose(
      Rotation2d gyroRotation, SwerveModulePosition[] wheelPositions, Pose2d pose) {
    m_gyroOffset = pose.getRotation().minus(gyroRotation);
    m_estimatedPose = pose;
    m_odometryPose = pose;
    m_lastWheelPositions = wheelPositions;
    m_poseBuffer.clear();
  }

  public Pose2d getEstimatedPose() {
    return m_estimatedPose;
  }

  /** Adds a new odometry observation from the drive subsystem. */
  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = m_kinematics.toTwist2d(m_lastWheelPositions, observation.wheelPositions());
    double tiltScale = tiltScale(observation);
    twist = new Twist2d(twist.dx * tiltScale, twist.dy * tiltScale, twist.dtheta * tiltScale);
    m_lastWheelPositions = observation.wheelPositions();

    Pose2d lastOdometryPose = m_odometryPose;
    m_odometryPose = m_odometryPose.plus(twist.exp());

    // Replace odometry heading with gyro if present
    observation.yaw().ifPresent(
        gyroAngle -> m_odometryPose = new Pose2d(m_odometryPose.getTranslation(), gyroAngle.plus(m_gyroOffset)));

    m_poseBuffer.addSample(observation.timestamp().in(Seconds), m_odometryPose);

    // Apply the odometry delta to the vision-corrected estimate. The delta runs from the previous
    // pose to the current one; composing the inverse would walk the estimate backwards.
    setEstimatedPose(m_estimatedPose.plus(new Transform2d(lastOdometryPose, m_odometryPose)));
  }

  /** Adds a new vision pose observation from the vision subsystem. */
  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (m_poseBuffer.getInternalBuffer().lastKey() - kPoseBufferSizeSec > observation.timestamp().in(Seconds)) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = m_poseBuffer.getSample(observation.timestamp().in(Seconds));
    if (sample.isEmpty()) {
      return;
    }

    // Shift the estimate back to sample time, correct it there, then carry it forward again
    var sampleToOdometryTransform = new Transform2d(sample.get(), m_odometryPose);
    var odometryToSampleTransform = new Transform2d(m_odometryPose, sample.get());
    Pose2d estimateAtTime = m_estimatedPose.plus(odometryToSampleTransform);

    Transform2d correction = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());
    Transform2d scaledCorrection = scaleByKalmanGain(correction, observation.stdDevs());

    setEstimatedPose(estimateAtTime.plus(scaledCorrection).plus(sampleToOdometryTransform));
  }

  /**
   * Get the estimated pose at a past timestamp by projecting the current estimate backward using the
   * odometry buffer.
   */
  public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
    return m_poseBuffer.getSample(timestamp)
        .map(oldOdometryPose -> m_estimatedPose.transformBy(
            new Transform2d(m_odometryPose, oldOdometryPose)));
  }

  /**
   * The single write path for the estimate, so the wall clamp cannot be forgotten on one of the two
   * paths that update it.
   */
  private void setEstimatedPose(Pose2d pose) {
    m_estimatedPose = m_wallClampEnabled.getAsBoolean()
        ? FieldConstants.clampToFieldBounds(
            pose,
            RobotGeometryConstants.kRobotLengthWithBumpers,
            RobotGeometryConstants.kRobotWidthWithBumpers)
        : pose;
  }

  /**
   * How much to trust wheel travel while the robot is tilted, e.g. driving over a ramp: travel on an
   * incline does not translate 1:1 to movement in the field plane.
   */
  private double tiltScale(OdometryObservation observation) {
    if (!m_tiltCompensationEnabled.getAsBoolean()
        || observation.pitch().isEmpty()
        || observation.roll().isEmpty()) {
      return 1.0;
    }

    double cosProduct = observation.pitch().get().getCos() * observation.roll().get().getCos();
    double tiltDegrees = Math.abs(Math.toDegrees(Math.acos(cosProduct)));

    return Math.clamp(1.0 - MathUtil.inverseLerp(0, kMaxTiltDegrees, tiltDegrees), 0.0, 1.0);
  }

  /**
   * Scales a vision correction by the closed-form Kalman gain for a continuous filter with A = 0 and
   * C = I. See wpimath/algorithms.md.
   */
  private Transform2d scaleByKalmanGain(Transform2d correction, Matrix<N3, N1> visionStdDevs) {
    var scaled = new double[3];
    double[] components = {
        correction.getX(), correction.getY(), correction.getRotation().getRadians()
    };

    for (int row = 0; row < 3; ++row) {
      double q = m_qStdDevs.get(row, 0);
      double r = visionStdDevs.get(row, 0) * visionStdDevs.get(row, 0);
      double gain = q == 0.0 ? 0.0 : q / (q + Math.sqrt(q * r));
      scaled[row] = gain * components[row];
    }

    return new Transform2d(scaled[0], scaled[1], Rotation2d.fromRadians(scaled[2]));
  }

  // MARK: - Record types

  public record OdometryObservation(
      Time timestamp,
      SwerveModulePosition[] wheelPositions,
      Optional<Rotation2d> pitch,
      Optional<Rotation2d> roll,
      Optional<Rotation2d> yaw) {
  }

  public record VisionObservation(
      Time timestamp,
      Pose3d visionPose,
      Matrix<N3, N1> stdDevs) {
  }
}
