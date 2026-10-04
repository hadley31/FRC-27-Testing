package first.robot.util;

import first.lib.tuning.Toggle;

/**
 * Every feature toggle and tunable table on the robot, and the behaviour each one defaults to in a
 * match.
 *
 * <p>One declaration site is the point: the defaults below are the competition configuration, so
 * they can be reviewed in a single diff, and a robot that has never seen a dashboard plays a valid
 * match. Nothing here should be read from inside a mechanism or an algorithm — see the conventions
 * below.
 *
 * <h2>Where toggles get read</h2>
 *
 * <ul>
 *   <li><b>Robot-scoped classes</b> that already depend on this robot's configuration — {@code
 *       RobotState}, the command factories — may read these directly.
 *   <li><b>Algorithms and filters</b> take the value injected instead, as a {@code BooleanSupplier}
 *       parameter or a field on a value object. {@code PoseEstimator} and {@code ShotCalculationUtil}
 *       work this way, which is what keeps them unit-testable without NetworkTables.
 * </ul>
 *
 * <p>Class initialization publishes the NT topics, so this class must first be touched from the
 * {@code Robot} constructor or later — by then {@code RobotBase} has started the NT server and
 * loaded any persisted values.
 */
public final class Tuning {
  private Tuning() {
  }

  // MARK: - Shot

  /**
   * The shot tables the robot actually shoots with, editable from a dashboard while it runs.
   *
   * <p>Defaults to the table committed in {@link ShotProfile}, so this is the competition
   * configuration until someone edits it, and is that again after a reset. It persists across reboots
   * so a tuning session survives the restarts between matches — watch the published {@code IsDefault}
   * output, and export the result into {@link ShotProfile} rather than leaving it on the robot.
   */
  public static final TunableShotProfile kScoringShotProfile = TunableShotProfile.persistent(
      "/Tuning/ShotProfile/Scoring",
      ShotProfile.kScoringActuationLatency,
      ShotProfile.kScoringTable);

  /**
   * Whether shots lead the target to cancel the velocity fuel inherits from a moving turret.
   * Turning this off is the first thing to try when shots drift consistently to one side while
   * moving but land while stationary.
   */
  public static final Toggle kVelocityCompensation =
      Toggle.of("/Tuning/Shot/VelocityCompensation", true);

  // MARK: - Turret

  /** Whether the turret tracks the hub on its own. */
  public static final Toggle kAutoAim = Toggle.of("/Tuning/Turret/AutoAim", false);

  /**
   * Whether the turret holds a fixed angle. Orthogonal to {@link #kAutoAim}: with both set, the
   * chassis does the aiming instead of the turret.
   */
  public static final Toggle kFixedTurretMode = Toggle.of("/Tuning/Turret/FixedTurretMode", false);

  // MARK: - Vision

  /**
   * Whether one-tag solves correct the pose estimate at all, or only multi-tag ones do.
   *
   * <p>On by default, because a single tag is still a measurement and the filter prices it as one:
   * most of a match is spent where only one tag is in view, and a robot that ignored those frames
   * would run on odometry alone for most of it. Turn it off when the estimate is visibly being
   * pulled about and the log shows the frames doing the pulling were one-tag ones — a camera with a
   * drifting calibration or a mounting transform measured wrong shows up there first, since a
   * one-tag solve has no second tag to contradict it.
   */
  public static final Toggle kSingleTagEstimation =
      Toggle.of("/Tuning/Vision/SingleTagEstimation", true);

  // MARK: - Odometry

  /** Whether the pose estimate is clamped to keep the robot footprint inside the field walls. */
  public static final Toggle kWallClamp = Toggle.of("/Tuning/Odometry/WallClamp", true);

  /** Whether wheel travel is scaled down while the robot is tilted, e.g. crossing a ramp. */
  public static final Toggle kOdometryTiltCompensation =
      Toggle.of("/Tuning/Odometry/TiltCompensation", false);
}
