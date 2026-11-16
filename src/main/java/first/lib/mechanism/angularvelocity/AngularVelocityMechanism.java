package first.lib.mechanism.angularvelocity;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3Mechanism;
import first.lib.util.UnitDoubleConverter;

/**
 * A mechanism commanded to a speed rather than to a place -- a flywheel, a set of rollers.
 *
 * <p>Not zeroable, deliberately: a speed has no frame to establish, so there is no offset to lose.
 */
public abstract class AngularVelocityMechanism<T extends AngularVelocityMechanismIO>
    extends G3Mechanism<T, AngularVelocityMechanismInputsAutoLogged, AngularVelocity> {
  protected AngularVelocityMechanism(T io) {
    super(io, new AngularVelocityMechanismInputsAutoLogged(), UnitDoubleConverter.RPM);
  }

  @Override
  public AngularVelocity getCurrentMeasurement() {
    return getInputs().currentAngularVelocity;
  }

  @Override
  public AngularVelocity getTarget() {
    return getInputs().targetAngularVelocity;
  }

  public Angle getCurrentAngle() {
    return getInputs().currentAngle;
  }

  @Override
  public void setTarget(AngularVelocity angularVelocity) {
    getIO().setTargetAngularVelocity(angularVelocity);
  }

  @Override
  public boolean isNear(AngularVelocity measure, AngularVelocity tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  public AngularVelocity getDefaultTolerance() {
    return RPM.of(30);
  }
}
