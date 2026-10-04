package first.lib.mechanism;

import first.lib.tuning.TuningState;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Trigger;
import org.wpilib.units.Measure;

public interface G3Mechanism<T extends G3MechanismIO<I>, I extends LoggableInputs, U extends Measure<?>>
    extends Mechanism, LoggedComponent<T, I> {
  /**
   * Resolves the two inherited defaults, which Java will not pick between: {@link Mechanism} and
   * {@link LoggedComponent} each supply a {@code getName()}. They agree -- both are the class's
   * simple name -- but a mechanism's name is the one the scheduler and every command name is built
   * from, so that is the one deferred to here.
   */
  @Override
  public default String getName() {
    return Mechanism.super.getName();
  }

  /**
   * A mechanism that is its own component: there is no outer mechanism to be filed beneath.
   *
   * @return {@code this}
   */
  @Override
  public default G3Mechanism<T, I, U> getMechanism() {
    return this;
  }

  /**
   * Just the mechanism's name, rather than {@code getMechanism()/getName()} as it would be for a
   * component with an owner -- which here is the same name twice.
   */
  @Override
  public default String getLogPrefix() {
    return getName();
  }

  public U getCurrentMeasurement();

  public U getTarget();

  public void setTarget(U target);

  public default void halt() {
    getIO().halt();
  }

  public boolean isNear(U measure, U tolerance);

  public default boolean isNear(U measure) {
    return isNear(measure, getDefaultTolerance());
  }

  public default boolean isNearTarget(U tolerance) {
    return isNear(getTarget(), tolerance);
  }

  public default boolean isNearTarget() {
    return isNearTarget(getDefaultTolerance());
  }

  public default Trigger isNearTargetTrigger(U tolerance) {
    return new Trigger(() -> isNearTarget(tolerance));
  }

  public default Trigger isNearTargetTrigger() {
    return isNearTargetTrigger(getDefaultTolerance());
  }

  public default Trigger isNearTrigger(Supplier<U> targetSupplier, U tolerance) {
    return new Trigger(() -> isNear(targetSupplier.get(), tolerance));
  }

  public default Trigger isNearTrigger(U target, U tolerance) {
    return isNearTrigger(() -> target, tolerance);
  }

  public default Trigger isNearTrigger(Supplier<U> targetSupplier) {
    return isNearTrigger(targetSupplier, getDefaultTolerance());
  }

  public default Trigger isNearTrigger(U target) {
    return isNearTrigger(() -> target, getDefaultTolerance());
  }

  public default NeedsNameBuilderStage setTargetCommand(Supplier<U> targetSupplier) {
    return run(coroutine -> {
      while (true) {
        setTarget(targetSupplier.get());
        coroutine.yield();
      }
    });
  }

  public default NeedsNameBuilderStage setTargetCommand(U target) {
    return setTargetCommand(() -> target);
  }

  public default Command haltCommand() {
    return run(coroutine -> {
      halt();
      coroutine.park();
    }).named("Halt %s".formatted(getName()));
  }

  public U getDefaultTolerance();

  public default Command tuningCommand() {
    return run(coroutine -> {
      TuningState<U> tuning = getTuningState();
      setTarget(tuning.setpoint());

      while (true) {
        if (tuning.hasChanged()) {
          getIO().setGains(tuning.gains());
        }

        setTarget(tuning.setpoint());
        coroutine.yield();
      }
    }).named("Tune %s".formatted(getName()));
  }

  public TuningState<U> getTuningState();
}
