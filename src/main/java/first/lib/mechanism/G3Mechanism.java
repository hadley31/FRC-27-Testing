package first.lib.mechanism;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Trigger;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.Measure;

import first.lib.tuning.TunableCommand;
import first.lib.tuning.TuningState;
import first.lib.util.UnitDoubleConverter;

/**
 * A mechanism driven to a single measured quantity -- an angle, a position, a speed -- through one
 * IO layer.
 *
 * <p>A class rather than an interface, which is what lets it hold the three things every mechanism
 * has: its IO, its inputs, and its tuning state. Those used to be a field and a {@code getIO()}
 * override repeated in every mechanism, plus an identity-keyed cache standing in for the field a
 * tuning state could not have. A subclass now declares what it is and nothing else; see
 * {@code Turret}.
 *
 * <p>The constructor wires the mechanism up as well as building it -- the inputs are refreshed every
 * loop and the tuning command is published -- so the registration cannot be left out of a new
 * mechanism by forgetting a line. That does hand {@code this} to the scheduler and to
 * {@link Tunables} before the subclass's own constructor has run, which is safe here because
 * nothing dereferences it until the scheduler first runs: a periodic callback and a published
 * tunable are both read later. A subclass whose {@link #getName()} depended on its own fields would
 * break that, which is reason enough not to write one.
 *
 * @param <T> the IO interface this mechanism talks to
 * @param <I> the inputs class {@code T} populates
 * @param <U> the measure this mechanism is commanded in
 */
public abstract class G3Mechanism<T extends G3MechanismIO<I>, I extends LoggableInputs, U extends Measure<?>>
    implements Mechanism, LoggedComponent<T, I> {
  private final T m_io;
  private final I m_inputs;
  private final UnitDoubleConverter<U> m_measureType;
  private final TuningState<U> m_tuningState;

  /**
   * @param io the IO layer to command and read
   * @param inputs the inputs object {@code io} fills, supplied by the subclass for its dimension
   * @param measureType the unit this mechanism is tuned in -- passed rather than left to an
   *     overridable method, because the tuning state built below needs it before a subclass
   *     constructor could have set anything
   */
  protected G3Mechanism(T io, I inputs, UnitDoubleConverter<U> measureType) {
    m_io = io;
    m_inputs = inputs;
    m_measureType = measureType;
    m_tuningState = new TuningState<>(getName(), measureType);

    getRegisteredScheduler().addPeriodic(this::update);

    // The tuning state owns the refresh of its own values rather than letting AdvantageKit refresh
    // them one at a time, so a cycle never hands the controller a setpoint from this loop and gains
    // from the last. Sideloaded callbacks run before any command in the same Scheduler.run(), which
    // is what makes that refresh land ahead of the tuning command that reads it.
    getRegisteredScheduler().addPeriodic(m_tuningState::periodic);

    // Published at Tuning/Commands/<mechanism name>, so every mechanism's button sits together in
    // one place, under the same word as the setpoint and gains the button exists to exercise. It
    // replaces the commands v2 SmartDashboard.putData a command could once be handed to directly;
    // see TunableCommand for why the binding has to be written out in v3.
    Tunables.publish(
        "Tuning/Commands/%s".formatted(getName()),
        new TunableCommand(tuningCommand(), getRegisteredScheduler()));
  }

  @Override
  public final T getIO() {
    return m_io;
  }

  @Override
  public final I getInputs() {
    return m_inputs;
  }

  /**
   * Resolves the two inherited defaults, which Java will not pick between: {@link Mechanism} and
   * {@link LoggedComponent} each supply a {@code getName()}. They agree -- both are the class's
   * simple name -- but a mechanism's name is the one the scheduler and every command name is built
   * from, so that is the one deferred to here.
   */
  @Override
  public String getName() {
    return Mechanism.super.getName();
  }

  /**
   * A mechanism that is its own component: there is no outer mechanism to be filed beneath.
   *
   * @return {@code this}
   */
  @Override
  public final G3Mechanism<T, I, U> getMechanism() {
    return this;
  }

  /**
   * Just the mechanism's name, rather than {@code getMechanism()/getName()} as it would be for a
   * component with an owner -- which here is the same name twice.
   */
  @Override
  public String getLogPrefix() {
    return getName();
  }

  public abstract U getCurrentMeasurement();

  public abstract U getTarget();

  public abstract void setTarget(U target);

  public void halt() {
    getIO().halt();
  }

  public abstract boolean isNear(U measure, U tolerance);

  public boolean isNear(U measure) {
    return isNear(measure, getDefaultTolerance());
  }

  public boolean isNearTarget(U tolerance) {
    return isNear(getTarget(), tolerance);
  }

  public boolean isNearTarget() {
    return isNearTarget(getDefaultTolerance());
  }

  public Trigger isNearTargetTrigger(U tolerance) {
    return new Trigger(() -> isNearTarget(tolerance));
  }

  public Trigger isNearTargetTrigger() {
    return isNearTargetTrigger(getDefaultTolerance());
  }

  public Trigger isNearTrigger(Supplier<U> targetSupplier, U tolerance) {
    return new Trigger(() -> isNear(targetSupplier.get(), tolerance));
  }

  public Trigger isNearTrigger(U target, U tolerance) {
    return isNearTrigger(() -> target, tolerance);
  }

  public Trigger isNearTrigger(Supplier<U> targetSupplier) {
    return isNearTrigger(targetSupplier, getDefaultTolerance());
  }

  public Trigger isNearTrigger(U target) {
    return isNearTrigger(() -> target, getDefaultTolerance());
  }

  public NeedsNameBuilderStage setTargetCommand(Supplier<U> targetSupplier) {
    return run(coroutine -> {
      while (true) {
        setTarget(targetSupplier.get());
        coroutine.yield();
      }
    });
  }

  public NeedsNameBuilderStage setTargetCommand(U target) {
    return setTargetCommand(() -> target);
  }

  public Command haltCommand() {
    return run(coroutine -> {
      halt();
      coroutine.park();
    }).named("Halt %s".formatted(getName()));
  }

  public abstract U getDefaultTolerance();

  public Command tuningCommand() {
    TuningState<U> tuning = getTuningState();

    return run(coroutine -> {
      // Apply once on entry. changedThisLoop() is true only for the cycle a change lands in, so a
      // gain edited before this command was scheduled would otherwise never reach the controller.
      getIO().setGains(tuning.gains());
      setTarget(tuning.setpoint());

      while (true) {
        // One apply per edit, which is what this was always trying to say. The gains used to be
        // org.wpilib.tunable values whose hasChanged() flag nothing ever cleared, so a single edit
        // left this issuing a retrying blocking CAN config write every loop for the rest of the
        // enable.
        if (tuning.gainsChangedThisLoop()) {
          getIO().setGains(tuning.gains());
        }

        setTarget(tuning.setpoint());
        coroutine.yield();
      }
    }).named("Tune %s".formatted(getName()));
  }

  /**
   * The unit this mechanism's measurements are tuned in: whatever a person wants to read and type on
   * a dashboard, which is not necessarily the unit the inputs are logged in.
   *
   * <p>Supplied by the unit-specialized subclasses -- {@code AngleMechanism} and friends -- alongside
   * the default tolerance, since both are properties of the dimension rather than of the mechanism.
   */
  public final UnitDoubleConverter<U> getMeasureType() {
    return m_measureType;
  }

  /** This mechanism's published setpoint and gains. */
  public final TuningState<U> getTuningState() {
    return m_tuningState;
  }
}
