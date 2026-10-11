package first.lib.tuning;

import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.Measure;

import first.lib.util.UnitDoubleConverter;

/** Bundles a tunable setpoint with its {@link TunableGains}, shared by every mechanism's tuneCommand. */
public class TuningState<U extends Measure<?>> {
  private final TunableDouble m_setpoint;
  private final UnitDoubleConverter<U> m_setpointType;
  private final TunableGains m_gains;

  public TuningState(String namePrefix, UnitDoubleConverter<U> setpointType) {
    m_setpoint = Tunables.addDouble(namePrefix + " Setpoint", 0.0);
    m_setpointType = setpointType;
    m_gains = new TunableGains(namePrefix);
  }

  public U setpoint() {
    return m_setpointType.of(m_setpoint.get());
  }

  /**
   * Writes a setpoint back out to the dashboard.
   *
   * <p>Converted through the published unit rather than taken as a bare magnitude, so a caller
   * handing over a setpoint in some other unit of the same dimension -- {@code Rotations.of(1)} for a
   * mechanism published in degrees -- writes the value it means rather than the number 1.
   */
  public void setSetpoint(U setpoint) {
    m_setpoint.set(m_setpointType.toDouble(setpoint));
  }

  public TunableGains gains() {
    return m_gains;
  }

  public boolean hasChanged() {
    return m_setpoint.hasChanged() || m_gains.hasChanged();
  }
}
