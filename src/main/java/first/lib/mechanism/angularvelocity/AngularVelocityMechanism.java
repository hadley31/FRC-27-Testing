package first.lib.mechanism.angularvelocity;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3Mechanism;

public interface AngularVelocityMechanism<IO extends AngularVelocityMechanismIO>
    extends G3Mechanism<IO, AngularVelocityMechanismInputsAutoLogged, AngularVelocity> {
  @Override
  public default AngularVelocity getCurrentMeasurement() {
    return getInputs().currentAngularVelocity;
  }

  @Override
  public default AngularVelocity getTarget() {
    return getInputs().targetAngularVelocity;
  }

  public default Angle getCurrentAngle() {
    return getInputs().currentAngle;
  }

  @Override
  public default void setTarget(AngularVelocity angularVelocity) {
    getIO().setTargetAngularVelocity(angularVelocity);
  }

  @Override
  default boolean isNear(AngularVelocity measure, AngularVelocity tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  default AngularVelocity getDefaultTolerance() {
    return RPM.of(30);
  }
}
