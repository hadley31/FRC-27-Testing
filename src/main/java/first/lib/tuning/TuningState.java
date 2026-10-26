package first.lib.tuning;

import org.wpilib.units.Measure;

import first.lib.mechanism.Gains;
import first.lib.util.UnitDoubleConverter;

/**
 * A tunable setpoint and its gains, shared by every mechanism's tuning command.
 *
 * <p>Both halves persist and both are logged as AdvantageKit inputs -- see {@link LoggedTunable} --
 * so a tuning session survives a reboot and a log replays the numbers the robot really ran. Paths
 * are nested under {@link LoggedTunables#kRoot} and the mechanism's name, so a {@code Turret} tuned
 * in degrees gets {@code /Tuning/Turret/Setpoint (Degree)} alongside {@code /Tuning/Turret/kP}.
 *
 * @param <U> the measure this mechanism's setpoint is expressed in
 */
public class TuningState<U extends Measure<?>> {
  private final LoggedTunableMeasure<U> m_setpoint;
  private final LoggedTunableGains m_gains;

  /**
   * @param namePrefix the name this mechanism's values are filed under, typically its own name
   * @param setpointType the unit the setpoint is stored, displayed and typed in
   */
  public TuningState(String namePrefix, UnitDoubleConverter<U> setpointType) {
    String pathPrefix = LoggedTunables.kRoot + "/" + namePrefix;

    m_setpoint = LoggedTunableMeasure.owned(
        pathPrefix + "/Setpoint", setpointType, setpointType.of(0.0));
    m_gains = new LoggedTunableGains(pathPrefix);
  }

  /** The current setpoint. */
  public U setpoint() {
    return m_setpoint.get();
  }

  /**
   * Writes a setpoint back out to the dashboard.
   *
   * <p>Converted through the published unit rather than taken as a bare magnitude, so a caller
   * handing over a setpoint in some other unit of the same dimension -- {@code Rotations.of(1)} for a
   * mechanism published in degrees -- writes the value it means rather than the number 1.
   *
   * <p>Inputs refresh once per loop before user code, so {@link #setpoint()} keeps returning the old
   * value until the next cycle.
   */
  public void setSetpoint(U setpoint) {
    m_setpoint.set(setpoint);
  }

  /** The current gains, as plain numbers for an IO implementation to apply. */
  public Gains gains() {
    return m_gains.gains();
  }

  /**
   * Whether a <em>gain</em> changed during this loop's refresh.
   *
   * <p>Only the gains, deliberately. Applying gains to a Talon is a retrying blocking CAN config
   * write, whereas a setpoint is pushed every loop regardless, so folding a setpoint edit into this
   * would buy a config write for nothing. See {@link LoggedTunable#changedThisLoop()} for why this
   * is a one-loop query and why a consumer must also apply the gains once when it starts.
   */
  public boolean gainsChangedThisLoop() {
    return m_gains.changedThisLoop();
  }

  /**
   * Refreshes the setpoint and the gains from NetworkTables, or from the log during replay.
   *
   * <p>Call once per loop, before anything reads either. {@code G3Mechanism.registerTuningCommand}
   * arranges this, and the ordering is load-bearing: {@code Scheduler} runs
   * {@code addPeriodic} callbacks before it runs any command, so the tuning command always reads
   * values refreshed in the same cycle rather than the previous one's.
   */
  public void periodic() {
    m_setpoint.periodic();
    m_gains.periodic();
  }
}
