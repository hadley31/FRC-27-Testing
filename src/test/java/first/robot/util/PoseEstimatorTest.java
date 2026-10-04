package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

import first.robot.util.PoseEstimator.OdometryObservation;
import first.robot.util.PoseEstimator.VisionObservation;

/**
 * What a vision observation is allowed to do to the estimate, and in particular what it is not
 * allowed to do to a pose reset.
 *
 * <p>The wall clamp and tilt compensation are off throughout, so that what moves the estimate here
 * is only ever odometry and vision.
 */
public class PoseEstimatorTest {
  /** A stationary robot: every observation reports the wheels in the same place. */
  private static final SwerveModulePosition[] STOPPED = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  /** A tight standard deviation, the kind a close multi-tag solve earns. */
  private static final Matrix<N3, N1> CONFIDENT = new Matrix<>(VecBuilder.fill(0.05, 0.05, 0.1));

  private final PoseEstimator m_estimator =
      new PoseEstimator(kinematics(), () -> false, () -> false);

  // MARK: - A reset is not negotiable

  /**
   * The failure this exists to catch: reset the pose, then hand over a sighting captured before the
   * reset. It describes a robot that no longer exists, and applying it drags the estimate back to
   * where the robot used to be -- which on a dashboard reads as the reset not having taken.
   */
  @Test
  void aVisionObservationFromBeforeAResetCannotUndoIt() {
    Pose2d resetPose = new Pose2d(5.0, 3.0, Rotation2d.ZERO);

    odometryAt(9.98);
    m_estimator.resetPose(Rotation2d.ZERO, STOPPED, resetPose);
    odometryAt(10.02);

    m_estimator.addVisionObservation(visionAt(9.99, new Pose2d(1.0, 1.0, Rotation2d.ZERO)));

    assertEquals(resetPose, m_estimator.getEstimatedPose(),
        "a sighting from before the reset must not move the estimate");
  }

  /**
   * The same guarantee for the awkward case: a frame captured a hair after the reset, but still
   * before the first odometry sample that followed it. The buffer cannot say where the robot was at
   * that instant, and answering with its earliest sample anyway -- which is what an interpolating
   * buffer does by default -- would replay a pre-reset sighting against the post-reset pose.
   */
  @Test
  void aVisionObservationTheOdometryBufferCannotPlaceIsRejected() {
    odometryAt(10.00);
    odometryAt(10.02);

    // Well inside the buffer's two second span, so only the near end can reject it.
    m_estimator.addVisionObservation(visionAt(9.50, new Pose2d(1.0, 1.0, Rotation2d.ZERO)));

    assertEquals(Pose2d.ZERO, m_estimator.getEstimatedPose(),
        "an observation older than every odometry sample has nothing to be replayed against");
  }

  // MARK: - How much one observation is worth

  /**
   * One frame is evidence, not an answer. The gain is set by the ratio of the odometry and vision
   * standard deviations, and odometry standard deviations given in metres rather than millimetres
   * make it near one -- at which point the estimate simply becomes whatever the last frame said, and
   * a single bad frame moves the robot across the field.
   */
  @Test
  void oneVisionObservationMovesTheEstimateByAFractionOfTheWay() {
    odometryAt(10.00);
    odometryAt(10.02);

    m_estimator.addVisionObservation(visionAt(10.01, new Pose2d(1.0, 0.0, Rotation2d.ZERO)));

    double moved = m_estimator.getEstimatedPose().getX();

    assertTrue(moved > 0.0, "an observation should move the estimate toward itself, got " + moved);
    assertTrue(moved < 0.25,
        "one frame should be a nudge rather than a teleport, got " + moved + " m of a 1 m correction");
  }

  /** Enough of them, though, and the estimate arrives. */
  @Test
  void repeatedVisionObservationsConvergeOnWhatTheySay() {
    odometryAt(10.00);

    for (int i = 1; i <= 200; i++) {
      double timestamp = 10.0 + i * 0.02;
      odometryAt(timestamp);
      m_estimator.addVisionObservation(
          visionAt(timestamp - 0.005, new Pose2d(1.0, 0.0, Rotation2d.ZERO)));
    }

    assertEquals(1.0, m_estimator.getEstimatedPose().getX(), 0.02,
        "a stationary robot told the same thing repeatedly should end up believing it");
  }

  // MARK: - Fixtures

  /** A stationary odometry sample, which only moves the clock on and fills the pose buffer. */
  private void odometryAt(double timestampSeconds) {
    m_estimator.addOdometryObservation(new OdometryObservation(
        Seconds.of(timestampSeconds),
        STOPPED,
        Optional.empty(),
        Optional.empty(),
        Optional.empty()));
  }

  private static VisionObservation visionAt(double timestampSeconds, Pose2d pose) {
    return new VisionObservation(Seconds.of(timestampSeconds), new Pose3d(pose), CONFIDENT);
  }

  /** A square chassis, since nothing here turns and the exact track width cannot matter. */
  private static SwerveDriveKinematics kinematics() {
    return new SwerveDriveKinematics(
        new Translation2d(0.3, 0.3),
        new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3),
        new Translation2d(-0.3, -0.3));
  }
}
