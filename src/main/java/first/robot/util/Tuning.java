package first.robot.util;

import first.lib.tuning.LoggedTunableBoolean;

/**
 * Every feature toggle and tunable table on the robot, and the behaviour each one defaults to in a
 * match.
 *
 * <p>One declaration site is the point: the defaults below are the competition configuration, so
 * they can be reviewed in a single diff, and a robot that has never seen a dashboard plays a valid
 * match. Nothing here should be read from inside a mechanism or an algorithm — see the conventions
 * below.
 *
 * <h2>Where these get read</h2>
 *
 * <ul>
 *   <li><b>Robot-scoped classes</b> that already depend on this robot's configuration — {@code
 *       RobotState}, the command factories — may read these directly.
 *   <li><b>Algorithms and filters</b> take the value injected instead, as a {@code BooleanSupplier}
 *       parameter or a field on a value object. {@code PoseEstimator} and {@code ShotCalculationUtil}
 *       work this way, which is what keeps them unit-testable without NetworkTables.
 * </ul>
 *
 * <h2>Toggles versus tuned values</h2>
 *
 * <p>Everything below is a {@code LoggedTunableBoolean.of}, which resets to its committed default
 * every boot — that default <em>is</em> the competition behaviour. A value you actually tune
 * — a gain, a setpoint, an offset — wants the {@code persistent} factory of the matching
 * {@code LoggedTunable} type instead, so it survives the restarts between matches.
 * {@code LoggedTunables.refresh()} then raises one driver-station alert naming every persistent
 * value that no longer matches the code. Declare those here too, under their own section.
 *
 * <p>Do not reach for {@code persistent} on a behaviour switch: a persisted kill switch from a
 * Thursday debug session will quietly still be set at Saturday's event.
 *
 * <p>A mechanism's gains and setpoint are already covered — {@code TuningState} claims them under
 * the mechanism's own name, so they need no declaration here.
 *
 * <p>Class initialization publishes the NT topics, so this class must first be touched from the
 * {@code Robot} constructor or later — by then {@code RobotBase} has started the NT server and
 * loaded any persisted values. That ordering is load-bearing for a {@code LoggedTunable}: touched
 * earlier, there is no server to persist to.
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
  public static final LoggedTunableBoolean kVelocityCompensation =
      LoggedTunableBoolean.of("/Tuning/Shot/VelocityCompensation", true);

  // MARK: - Turret

  /**
   * Whether the turret tracks the hub on its own, whenever no command has claimed it.
   *
   * <p>Read inside the turret's default command rather than gating it, so it can be flipped at any
   * time -- see {@code RobotCommandFactory#trackHubWithTurret}. With it off the turret simply holds
   * position when idle. Either way an explicit command, such as auto-aim, takes the turret and the
   * toggle has no say until that command ends.
   */
  public static final LoggedTunableBoolean kAutoAim =
      LoggedTunableBoolean.of("/Tuning/Turret/AutoAim", false);

  /**
   * Whether the turret parks at a fixed angle and the <em>chassis</em> does the aiming instead.
   *
   * <p>The shot solution is unchanged; only which joint executes it differs. The chassis heading is
   * read out of the same solve the turret would have used, by
   * {@code ShotCalculationUtil#chassisHeadingForTurretBore}, so the chassis inherits the turret
   * offset and velocity lead rather than aiming by a cruder rule.
   *
   * <p>Safe to flip mid-match: auto-aim picks the chassis up on the rising edge and gives it back on
   * the falling one. Expect a longer time-to-shot while it is set, since a loaded chassis turns a
   * good deal slower than a turret.
   */
  public static final LoggedTunableBoolean kFixedTurretMode =
      LoggedTunableBoolean.of("/Tuning/Turret/FixedTurretMode", false);

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
  public static final LoggedTunableBoolean kSingleTagEstimation =
      LoggedTunableBoolean.of("/Tuning/Vision/SingleTagEstimation", true);

  // MARK: - Odometry

  /** Whether the pose estimate is clamped to keep the robot footprint inside the field walls. */
  public static final LoggedTunableBoolean kWallClamp = LoggedTunableBoolean.of("/Tuning/Odometry/WallClamp", true);

  /** Whether wheel travel is scaled down while the robot is tilted, e.g. crossing a ramp. */
  public static final LoggedTunableBoolean kOdometryTiltCompensation =
      LoggedTunableBoolean.of("/Tuning/Odometry/TiltCompensation", false);
}
