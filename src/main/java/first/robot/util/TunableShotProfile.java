package first.robot.util;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.wpilib.units.measure.Time;

import first.lib.tuning.TunableDoubleArray;

/**
 * A {@link ShotProfile} a dashboard can edit while the robot is running.
 *
 * <p>The shot tables are the one part of the shooter that can only be found empirically, and the loop
 * of "guess a number, redeploy, shoot one ball, guess again" is what makes finding them slow. This
 * publishes the table as four columns a dashboard can plot and drag — one shared range axis and one
 * value per curve — and folds accepted edits into the live profile within a cycle.
 *
 * <h2>How an edit reaches the solver</h2>
 *
 * <p>{@link #profile} hands out one {@link ShotProfile} that never changes identity: edits rewrite the
 * three interpolating maps inside it in place. So {@link ShotCalculationUtil} needs to know nothing
 * about any of this, callers may hold the profile in a field, and a command already mid-shot picks up a
 * new table on its next lookup.
 *
 * <h2>When the rebuild happens</h2>
 *
 * <p>In the dashboard-input refresh AdvantageKit runs at the top of every cycle, before any user code.
 * That timing is deliberate: no command can run against a table mid-rebuild, and every command in a
 * cycle sees the same table. The four columns are refreshed from here rather than registering
 * themselves, so a consistent snapshot does not depend on registration order.
 *
 * <h2>Safety</h2>
 *
 * <p>The columns default to the committed table, so a robot that has never seen a dashboard — or one
 * whose dashboard publishes nonsense — shoots exactly what is committed in {@link ShotProfile}. A
 * snapshot that fails {@link ShotProfileTable#validate} leaves the previous table in place and
 * publishes why. Because a persistent table survives a reboot, {@code IsDefault} is published every
 * cycle: a tuned table is otherwise invisible, and the failure mode to design against is a robot
 * arriving at a match still carrying Thursday evening's experiment.
 *
 * <p>{@code AsJava} carries the live table as the Java literal that declares it. Pasting that into
 * {@link ShotProfile} and committing it is how a tuning session is supposed to end — see
 * {@link ShotProfileTable#toJava}.
 *
 * <h2>Published layout</h2>
 *
 * <p>Under the path given, all four columns being arrays of equal length, one entry per range:
 *
 * <ul>
 *   <li>{@code DistancesFeet} — the shared range axis, strictly increasing
 *   <li>{@code TimeOfFlightSeconds}, {@code HoodAngleDegrees}, {@code FlywheelRpm} — one curve each
 *   <li>{@code Accepted}, {@code RejectionReason}, {@code IsDefault}, {@code AsJava} — read-only
 * </ul>
 */
public final class TunableShotProfile extends LoggedNetworkInput {
  private final String m_path;
  private final ShotProfileTable m_competitionDefault;
  private final TunableDoubleArray m_distances;
  private final TunableDoubleArray m_timeOfFlight;
  private final TunableDoubleArray m_hoodAngle;
  private final TunableDoubleArray m_flywheelVelocity;
  private final ShotProfile m_profile;

  private ShotProfileTable m_accepted;

  private TunableShotProfile(
      String path, Time actuationLatency, ShotProfileTable competitionDefault, boolean persistent) {
    m_path = path;
    m_competitionDefault = competitionDefault;
    m_accepted = competitionDefault;
    m_profile = competitionDefault.toProfile(actuationLatency);

    m_distances = TunableDoubleArray.owned(
        path + "/DistancesFeet", competitionDefault.distanceColumn(), persistent);
    m_timeOfFlight = TunableDoubleArray.owned(
        path + "/TimeOfFlightSeconds", competitionDefault.timeOfFlightColumn(), persistent);
    m_hoodAngle = TunableDoubleArray.owned(
        path + "/HoodAngleDegrees", competitionDefault.hoodAngleColumn(), persistent);
    m_flywheelVelocity = TunableDoubleArray.owned(
        path + "/FlywheelRpm", competitionDefault.flywheelColumn(), persistent);

    Logger.registerDashboardInput(this);
  }

  /**
   * A profile whose table resets to the committed one every boot.
   *
   * @param path absolute NT path to publish under, e.g. {@code "/Tuning/ShotProfile/Scoring"}
   * @param actuationLatency the actuation latency, which is not tunable here — it describes mechanism
   *     lag rather than the range table, and nothing about it is easier to find by dragging a chart
   * @param competitionDefault the committed table
   */
  public static TunableShotProfile of(
      String path, Time actuationLatency, ShotProfileTable competitionDefault) {
    return new TunableShotProfile(path, actuationLatency, competitionDefault, false);
  }

  /**
   * A profile whose table is saved on the robot and restored at boot, so a tuning session survives the
   * reboots between matches.
   *
   * <p>This is what {@link first.lib.tuning.Toggle}'s notes mean by a value you would be annoyed to
   * re-enter: measured data, not a behaviour switch. Watch {@code IsDefault}, and export the result into
   * source rather than treating the robot's {@code networktables.json} as where the tuning lives.
   */
  public static TunableShotProfile persistent(
      String path, Time actuationLatency, ShotProfileTable competitionDefault) {
    return new TunableShotProfile(path, actuationLatency, competitionDefault, true);
  }

  /** The live profile. Safe to hold: edits rewrite its tables rather than replacing it. */
  public ShotProfile profile() {
    return m_profile;
  }

  /** The table currently in effect, which is the committed one until a dashboard edit is accepted. */
  public ShotProfileTable table() {
    return m_accepted;
  }

  /** Whether the live table still matches the one committed in source. */
  public boolean isDefault() {
    return m_accepted.equals(m_competitionDefault);
  }

  /** Discards any dashboard edits and republishes the committed table. */
  public void resetToDefault() {
    m_distances.set(m_competitionDefault.distanceColumn());
    m_timeOfFlight.set(m_competitionDefault.timeOfFlightColumn());
    m_hoodAngle.set(m_competitionDefault.hoodAngleColumn());
    m_flywheelVelocity.set(m_competitionDefault.flywheelColumn());
  }

  @Override
  public void periodic() {
    m_distances.periodic();
    m_timeOfFlight.periodic();
    m_hoodAngle.periodic();
    m_flywheelVelocity.periodic();

    var result = ShotProfileTable.validate(
        m_distances.get(),
        m_timeOfFlight.get(),
        m_hoodAngle.get(),
        m_flywheelVelocity.get());

    result.table().ifPresent(table -> {
      // Rebuild only on a real change. Filling three interpolating maps is cheap, but AsJava is not
      // something to serialize at 50 Hz, and a log full of identical tables hides the edits.
      if (!table.equals(m_accepted)) {
        m_accepted = table;
        table.applyTo(m_profile);
        Logger.recordOutput(m_path + "/AsJava", table.toJava());
      }
    });

    Logger.recordOutput(m_path + "/Accepted", result.isAccepted());
    Logger.recordOutput(m_path + "/RejectionReason", result.rejection());
    Logger.recordOutput(m_path + "/IsDefault", isDefault());
  }
}
