package first.lib.mechanism;

import java.util.function.DoubleFunction;

import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.Measure;

/** Bundles a tunable setpoint with its {@link TunableGains}, shared by every mechanism's tuneCommand. */
public class TuningState<U extends Measure<?>> {
  private final TunableDouble m_setpoint;
  private final DoubleFunction<U> m_unitFactory;
  private final TunableGains m_gains;

  public TuningState(String namePrefix, DoubleFunction<U> unitFactory) {
    m_setpoint = Tunables.addDouble(namePrefix + " Setpoint", 0.0);
    m_unitFactory = unitFactory;
    m_gains = new TunableGains(namePrefix);
  }

  public U setpoint() {
    return m_unitFactory.apply(m_setpoint.get());
  }

  public void setSetpoint(U setpoint) {
    m_setpoint.set(setpoint.magnitude());
  }

  public TunableGains gains() {
    return m_gains;
  }

  public boolean hasChanged() {
    return m_setpoint.hasChanged() || m_gains.hasChanged();
  }
}
