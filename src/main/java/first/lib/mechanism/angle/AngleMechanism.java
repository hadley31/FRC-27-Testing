package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.ZeroableMechanism;
import first.lib.util.UnitDoubleConverter;

/**
 * A mechanism commanded to an angle.
 *
 * <p>Zeroable, because an angle names a place: these angles are in the zeroed frame, and the raw
 * sensor readings behind them never leave {@link AngleMechanismIO}. See {@link ZeroableMechanism}.
 */
public abstract class AngleMechanism<T extends AngleMechanismIO>
    extends ZeroableMechanism<T, AngleMechanismInputsAutoLogged, Angle> {
  protected AngleMechanism(T io) {
    super(io, new AngleMechanismInputsAutoLogged(), UnitDoubleConverter.DEGREES);
  }

  @Override
  public Angle getCurrentMeasurement() {
    return getInputs().currentAngle;
  }

  @Override
  public Angle getTarget() {
    return getInputs().targetAngle;
  }

  /** Unaffected by the zero offset: moving the frame does not change how fast it is turning. */
  public AngularVelocity getCurrentAngularVelocity() {
    return getInputs().currentAngularVelocity;
  }

  @Override
  public void setTarget(Angle angle) {
    getIO().setTargetAngle(angle);
  }

  @Override
  public boolean isNear(Angle measure, Angle tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  public Angle getDefaultTolerance() {
    return Degrees.of(0.5);
  }
}
