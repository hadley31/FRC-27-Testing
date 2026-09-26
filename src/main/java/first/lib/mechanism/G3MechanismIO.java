package first.lib.mechanism;

import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base IO interface for every mechanism: the commands that are common to all of them, on top of
 * the {@link LoggedIO} reporting contract.
 *
 * @param <I> the inputs class this IO populates
 */
public interface G3MechanismIO<I extends LoggableInputs> extends LoggedIO<I> {
  public void setGains(TunableGains gains);

  public void halt();
}
