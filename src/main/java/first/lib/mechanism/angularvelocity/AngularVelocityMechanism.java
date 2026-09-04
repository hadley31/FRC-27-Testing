package first.lib.mechanism.angularvelocity;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3Mechanism;

public interface AngularVelocityMechanism<T extends AngularVelocityMechanismIO>
    extends G3Mechanism<T, AngularVelocity> {
  @Override
  public default AngularVelocity getCurrentMeasurement() {
    return getIO().getCurrentAngularVelocity();
  }

  @Override
  public default AngularVelocity getTarget() {
    return getIO().getTargetAngularVelocity();
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
