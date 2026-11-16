package first.lib.mechanism.position;

import static org.wpilib.units.Units.Inches;

import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import first.lib.mechanism.ZeroableMechanism;
import first.lib.util.UnitDoubleConverter;

/**
 * A mechanism commanded to a position along a travel.
 *
 * <p>Zeroable, because a position names a place: these positions are in the zeroed frame, and the
 * raw sensor readings behind them never leave {@link PositionMechanismIO}. See
 * {@link ZeroableMechanism}.
 */
public abstract class PositionMechanism<T extends PositionMechanismIO>
    extends ZeroableMechanism<T, PositionMechanismInputsAutoLogged, Distance> {
  protected PositionMechanism(T io) {
    super(io, new PositionMechanismInputsAutoLogged(), UnitDoubleConverter.INCHES);
  }

  @Override
  public Distance getCurrentMeasurement() {
    return getInputs().currentPosition;
  }

  @Override
  public Distance getTarget() {
    return getInputs().targetPosition;
  }

  /** Unaffected by the zero offset: moving the frame does not change how fast it is travelling. */
  public LinearVelocity getCurrentVelocity() {
    return getInputs().currentVelocity;
  }

  @Override
  public void setTarget(Distance position) {
    getIO().setTargetPosition(position);
  }

  @Override
  public boolean isNear(Distance measure, Distance tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  public Distance getDefaultTolerance() {
    return Inches.of(0.25);
  }
}
