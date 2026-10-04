package first.robot;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.fields.Field;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;

import first.lib.util.GeometryUtil;
import first.robot.mechanism.drive.Drive;
import first.robot.mechanism.flywheel.Flywheel;
import first.robot.mechanism.hood.Hood;
import first.robot.mechanism.turret.Turret;
import first.robot.mechanism.vision.apriltag.AprilTagVision;
import first.robot.mechanism.vision.apriltag.AprilTagVisionProcessor;
import first.robot.util.Constants.RobotGeometryConstants;
import first.robot.util.FieldConstants;
import first.robot.util.PoseEstimator;
import first.robot.util.Tuning;
import first.robot.util.TurretSnapshot;

/**
 * The robot's derived state: where it is, how fast it is moving, where its turret is pointing, and
 * the conditions the commands key off.
 *
 * <p>This is the single place that composes raw mechanism readings into field-frame geometry, so
 * that consumers ask questions rather than repeating transforms. It depends on the mechanisms it
 * actually reads rather than on {@link Robot} as a whole, which keeps it constructible in a test.
 */
public class RobotState {
  private static final AngularVelocity kFeedingFlywheelFloor = RPM.of(1600);
  private static final AngularVelocity kShootingFlywheelFloor = RPM.of(1200);
  private static final AngularVelocity kFeedingFlywheelTolerance = RPM.of(500);
  private static final double kPoseBufferSeconds = 1.0;

  private final Drive m_drive;
  private final Turret m_turret;
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final AprilTagVision m_vision;
  private final PoseEstimator m_poseEstimator;

  private final Trigger m_isPreparedToShootTrigger;
  private final Trigger m_isFeedingTrigger;
  private final Trigger m_inAllianceZoneTrigger;

  private final AprilTagVisionProcessor m_visionProcessor;

  private final Field2d m_field2d = new Field2d();

  private final TimeInterpolatableBuffer<Pose2d> m_robotPoseBuffer = TimeInterpolatableBuffer
      .createBuffer(kPoseBufferSeconds);

  /**
   * @param field the tag layout the vision filter measures tag distances against. Passed in rather
   *              than loaded here so that it is the same layout instance the cameras solve against:
   *              a second load would be a second copy to keep in step, and a filter judging
   *              observations by a layout the cameras never used would be judging them by the wrong
   *              field.
   */
  public RobotState(Drive drive, Turret turret, Hood hood, Flywheel flywheel, AprilTagVision vision,
      PoseEstimator poseEstimator, Field field) {
    m_drive = drive;
    m_turret = turret;
    m_hood = hood;
    m_flywheel = flywheel;
    m_vision = vision;
    m_poseEstimator = poseEstimator;

    // The drivetrain's own velocity, not the velocity implied by the estimate: the vision filter
    // uses it to price timestamp error, and reading it from the estimate would make how much an
    // observation is trusted depend on the estimate that observation is about to correct.
    m_visionProcessor = new AprilTagVisionProcessor(
        field,
        m_drive::getRobotRelativeSpeeds,
        Tuning.kSingleTagEstimation,
        m_poseEstimator::addVisionObservation);

    m_inAllianceZoneTrigger = new Trigger(this::inAllianceZone)
        .debounce(Seconds.of(0.2), DebounceType.FALLING);

    m_isFeedingTrigger = m_turret.isNearTargetTrigger()
        .and(() -> m_flywheel.isNearTarget(kFeedingFlywheelTolerance))
        .and(() -> m_flywheel.getCurrentMeasurement().gt(kFeedingFlywheelFloor))
        .and(RobotModeTriggers.teleop())
        .and(m_inAllianceZoneTrigger.negate());

    m_isPreparedToShootTrigger = m_flywheel.isNearTargetTrigger()
        .and(m_hood.isNearTargetTrigger())
        .and(m_turret.isNearTargetTrigger())
        .and(() -> m_flywheel.getTarget().gt(kShootingFlywheelFloor))
        .debounce(Seconds.of(0.1), DebounceType.FALLING);

    Tunables.publish("Field", m_field2d);
  }

  public void periodic() {
    // Handle odometry and vision observations
    m_drive.getOdometryObservations().forEach(m_poseEstimator::addOdometryObservation);
    m_visionProcessor.process(m_vision.getLatestObservations());

    Time timestamp = RobotController.getMeasureTime();
    Pose2d robotPose = getRobotPose();

    Logger.recordOutput("RobotState/Toggles/FixedTurretModeEnabled", isFixedTurretModeEnabled());
    Logger.recordOutput("RobotState/Toggles/AutoAimEnabled", isAutoAimEnabled());
    Logger.recordOutput("RobotState/Triggers/InAllianceZone", m_inAllianceZoneTrigger.getAsBoolean());
    Logger.recordOutput("RobotState/Triggers/IsPreparedToShoot", m_isPreparedToShootTrigger.getAsBoolean());
    Logger.recordOutput("RobotState/Triggers/IsFeeding", m_isFeedingTrigger.getAsBoolean());
    Logger.recordOutput("RobotState/Geometry/TurretPose", getTurretPose());
    Logger.recordOutput("RobotState/Odometry/FieldRelativeSpeeds", getFieldRelativeSpeeds());
    Logger.recordOutput("RobotState/Odometry/FieldRelativeTurretSpeeds", getFieldRelativeTurretSpeeds());
    Logger.recordOutput("RobotState/Odometry/RobotPose", getRobotPose());
    Logger.recordOutput("RobotState/Odometry/SimRobotPose", m_poseEstimator.getOdometryPose());

    differentiatePose(timestamp, robotPose).ifPresent(speeds -> {
      Logger.recordOutput("RobotState/Experimental/PoseDerivedRobotRelativeSpeeds", speeds);
      Logger.recordOutput("RobotState/Experimental/PoseDerivedFieldRelativeSpeeds",
          speeds.toFieldRelative(robotPose.getRotation()));
    });

    m_robotPoseBuffer.addSample(timestamp.in(Seconds), robotPose);
    m_field2d.setRobotPose(robotPose);
  }

  /**
   * Differentiates the pose buffer, as a cross-check against the velocity the drivetrain reports.
   *
   * @return empty on the first cycle, or when two samples share a timestamp
   */
  private Optional<ChassisVelocities> differentiatePose(Time timestamp, Pose2d robotPose) {
    var previous = m_robotPoseBuffer.getInternalBuffer().lastEntry();
    if (previous == null) {
      return Optional.empty();
    }

    double dt = timestamp.in(Seconds) - previous.getKey();
    if (dt <= 0.0) {
      return Optional.empty();
    }

    Twist2d twist = robotPose.minus(previous.getValue()).log();
    return Optional.of(new ChassisVelocities(twist.dx / dt, twist.dy / dt, twist.dtheta / dt));
  }

  // MARK: - Pose

  public Pose2d getRobotPose() {
    return m_poseEstimator.getEstimatedPose();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPose(m_drive.getRawGyroAngle(), m_drive.getModulePositions(), pose);

    // A simulated field is rendered from the pose just teleported, and keeps a trail of recent poses
    // to render camera latency from that the teleport does not reach. Unless it is told, it spends
    // the next frame or two rendering the robot part way back to where it came from -- stamped after
    // the reset, so the filter cannot recognise those frames as stale and undoes the reset on the
    // strength of them. A no-op on a real robot.
    m_vision.resetRobotPose(pose);
  }

  /** The robot pose projected forward by {@code seconds} of its current motion. */
  public Pose2d getExpRobotPose(double seconds) {
    return getRobotPose().plus(getRobotRelativeSpeeds().toTwist2d(seconds).exp());
  }

  public Pose2d getTurretPose(Pose2d robotPose) {
    return robotPose.plus(getRobotToTurretTransform());
  }

  public Pose2d getTurretPose() {
    return getTurretPose(getRobotPose());
  }

  public Pose2d getExpTurretPose(double seconds) {
    return getTurretPose(getExpRobotPose(seconds));
  }

  /**
   * The turret's field-relative position, orientation and velocity, projected forward by
   * {@code latency} so that a solution computed from it is aimed at where the robot will be once the
   * mechanisms have actually responded.
   */
  public TurretSnapshot getTurretSnapshot(Time latency) {
    Pose2d robotPose = getExpRobotPose(latency.in(Seconds));
    Translation2d turretGround = getTurretPose(robotPose).getTranslation();

    return new TurretSnapshot(
        // The robot pose is yaw-only, so the turret's height above the floor is just its height
        // above the robot origin.
        new Translation3d(
            turretGround.getX(),
            turretGround.getY(),
            RobotGeometryConstants.kRobotToTurret3d.getZ()),
        robotPose.getRotation(),
        getFieldRelativeTurretSpeeds(robotPose));
  }

  // MARK: - Turret geometry

  public Transform2d getRobotToTurretTransform() {
    return new Transform2d(RobotGeometryConstants.kRobotToTurret, new Rotation2d(getTurretAngle()));
  }

  public Transform3d getRobotToTurretTransform3d() {
    return new Transform3d(RobotGeometryConstants.kRobotToTurret3d,
        GeometryUtil.rotation3dFromYaw(getTurretAngle()));
  }

  public Transform3d getRobotToTurretCamera() {
    return getRobotToTurretTransform3d().plus(RobotGeometryConstants.kTurretToCamera);
  }

  // MARK: - Velocity

  public ChassisVelocities getRobotRelativeSpeeds() {
    return m_drive.getRobotRelativeSpeeds();
  }

  public ChassisVelocities getFieldRelativeSpeeds() {
    return getRobotRelativeSpeeds().toFieldRelative(getRobotPose().getRotation());
  }

  public ChassisVelocities getFieldRelativeTurretSpeeds() {
    return getFieldRelativeTurretSpeeds(getRobotPose());
  }

  /**
   * The field-relative velocity of the turret pivot, which differs from the chassis's whenever the
   * chassis is rotating: the pivot rides on a lever arm, so it sweeps even in place.
   */
  public ChassisVelocities getFieldRelativeTurretSpeeds(Pose2d robotPose) {
    ChassisVelocities robotSpeeds = getFieldRelativeSpeeds();

    // v = omega x r, with r the mounting offset rotated into the field frame.
    Translation2d leverArm = RobotGeometryConstants.kRobotToTurret.rotateBy(robotPose.getRotation());

    return robotSpeeds.plus(new ChassisVelocities(
        -leverArm.getY() * robotSpeeds.omega,
        leverArm.getX() * robotSpeeds.omega,
        0.0));
  }

  // MARK: - Mechanism readings

  public Angle getTurretAngle() {
    return m_turret.getCurrentMeasurement();
  }

  public Angle getHoodAngle() {
    return m_hood.getCurrentMeasurement();
  }

  public AngularVelocity getFlywheelVelocity() {
    return m_flywheel.getCurrentMeasurement();
  }

  // MARK: - Conditions

  public Trigger isPreparedToShootTrigger() {
    return m_isPreparedToShootTrigger;
  }

  public Trigger isFeedingTrigger() {
    return m_isFeedingTrigger;
  }

  public Trigger inAllianceZoneTrigger() {
    return m_inAllianceZoneTrigger;
  }

  /** Whether the robot is on its own side of the hub. */
  public boolean inAllianceZone() {
    double hubX = FieldConstants.getHubPosition3d().getX();
    double robotX = getRobotPose().getX();

    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE
        ? robotX < hubX
        : robotX > hubX;
  }

  // MARK: - Driver toggles

  public void setFixedTurretMode(boolean enabled) {
    Tuning.kFixedTurretMode.set(enabled);
  }

  public boolean isFixedTurretModeEnabled() {
    return Tuning.kFixedTurretMode.getAsBoolean();
  }

  public boolean isAutoAimEnabled() {
    return Tuning.kAutoAim.getAsBoolean();
  }

  public boolean isAutoAimAndFixedTurretModeEnabled() {
    return isAutoAimEnabled() && isFixedTurretModeEnabled();
  }
}
