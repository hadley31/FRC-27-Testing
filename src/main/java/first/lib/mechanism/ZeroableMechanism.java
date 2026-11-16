package first.lib.mechanism;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.command3.Command;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.Measure;

import first.lib.tuning.LoggedTunableMeasure;
import first.lib.tuning.LoggedTunables;
import first.lib.tuning.TunableCommand;
import first.lib.util.UnitDoubleConverter;

/**
 * A mechanism whose measurement means a place rather than a rate, and which therefore has a zero
 * that can be lost: an angle or a position, not a velocity.
 *
 * <p>The zero is one stored number -- the raw reading the mechanism gives when it is physically at
 * zero -- and {@link ZeroableMechanismIO} applies it, so nothing above here sees a raw sensor
 * value. {@link #zero()} is the whole operation: put the mechanism where zero should be, call it,
 * and the offset that makes it so is saved and handed to the IO.
 *
 * <p>The constructor puts that on the dashboard twice over: as a button at
 * {@code Tuning/Zero/<mechanism name>}, and as the offset itself at
 * {@code /Tuning/<mechanism name>/ZeroOffset (<unit>)}, which a person can read and type into -- a
 * number off yesterday's log goes back in without a code deploy. A typed offset reaches the IO at
 * the next restart rather than straight away, because the hand-off happens here; the button is the
 * path that takes effect immediately.
 *
 * @param <T> the IO interface this mechanism talks to
 * @param <I> the inputs class {@code T} populates
 * @param <U> the measure this mechanism's position is expressed in
 */
public abstract class ZeroableMechanism<T extends ZeroableMechanismIO<I, U>, I extends LoggableInputs, U extends Measure<?>>
    extends G3Mechanism<T, I, U> {
  /** Where every mechanism's zero button is published, so they sit together in one folder. */
  public static final String kZeroCommandRoot = "Tuning/Zero";

  /**
   * The stored zero offset.
   *
   * <p>Persistent, so a zero found at 4pm is still there after the next code deploy -- that is the
   * whole point. Logged as an AdvantageKit input for the same reason: a robot running a stored
   * offset is by definition running a number its source does not contain, so a replay has to take
   * the offset from the log rather than from today's code. It is also why every zeroed mechanism
   * shows up in {@link LoggedTunables}' driver-station alert, which is that alert doing its job.
   */
  private final LoggedTunableMeasure<U> m_zeroOffset;

  protected ZeroableMechanism(T io, I inputs, UnitDoubleConverter<U> measureType) {
    super(io, inputs, measureType);

    m_zeroOffset = LoggedTunableMeasure.persistent(
        LoggedTunables.kRoot + "/" + getName() + "/ZeroOffset", measureType, measureType.of(0.0));

    // The point of storing the offset at all: one found yesterday has to reach the IO at boot, and
    // nothing else would take it there.
    io.setZeroOffset(m_zeroOffset.get());

    Tunables.publish(
        "%s/%s".formatted(kZeroCommandRoot, getName()),
        new TunableCommand(zeroCommand(), getRegisteredScheduler()));
  }

  /** This mechanism's stored zero offset, which survives a reboot. */
  public final LoggedTunableMeasure<U> getZeroOffset() {
    return m_zeroOffset;
  }

  /**
   * Takes wherever the mechanism is right now as its zero: the IO applies it, and the offset it
   * worked out is stored so it is still there tomorrow.
   */
  public void zero() {
    m_zeroOffset.set(getIO().zero());
  }

  /**
   * The command the dashboard button runs.
   *
   * <p>It claims <b>no</b> mechanisms, deliberately. Zeroing happens with the robot disabled and
   * someone's hands on the mechanism, and the scheduler refuses to schedule a command requiring a
   * mechanism that is not {@code controllableDuringDisabled} -- so a command that took ownership
   * would be a button that does nothing exactly when it is needed. There is nothing to own anyway:
   * the work is a single value write that finishes on the tick it starts.
   */
  public Command zeroCommand() {
    return Command.noRequirements(coroutine -> zero()).named("Zero %s".formatted(getName()));
  }
}
