package first.lib.mechanism.position;

import static org.wpilib.units.Units.Inches;

import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import first.lib.mechanism.G3Mechanism;

public interface PositionMechanism<IO extends PositionMechanismIO>
    extends G3Mechanism<IO, PositionMechanismInputsAutoLogged, Distance> {
  @Override
  public default Distance getCurrentMeasurement() {
    return getInputs().currentPosition;
  }

  @Override
  public default Distance getTarget() {
    return getInputs().targetPosition;
  }

  public default LinearVelocity getCurrentVelocity() {
    return getInputs().currentVelocity;
  }

  @Override
  public default void setTarget(Distance position) {
    getIO().setTargetPosition(position);
  }

  @Override
  default boolean isNear(Distance measure, Distance tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  default Distance getDefaultTolerance() {
    return Inches.of(0.25);
  }
}
