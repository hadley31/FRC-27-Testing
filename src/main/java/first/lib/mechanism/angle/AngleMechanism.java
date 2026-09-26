package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3Mechanism;

public interface AngleMechanism<IO extends AngleMechanismIO>
    extends G3Mechanism<IO, AngleMechanismInputsAutoLogged, Angle> {
  @Override
  public default Angle getCurrentMeasurement() {
    return getInputs().currentAngle;
  }

  @Override
  public default Angle getTarget() {
    return getInputs().targetAngle;
  }

  public default AngularVelocity getCurrentAngularVelocity() {
    return getInputs().currentAngularVelocity;
  }

  @Override
  public default void setTarget(Angle angle) {
    getIO().setTargetAngle(angle);
  }

  @Override
  public default boolean isNear(Angle measure, Angle tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  default Angle getDefaultTolerance() {
    return Degrees.of(0.5);
  }
}
