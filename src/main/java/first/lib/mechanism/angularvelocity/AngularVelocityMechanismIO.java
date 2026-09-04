package first.lib.mechanism.angularvelocity;

import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3MechanismIO;

public interface AngularVelocityMechanismIO extends G3MechanismIO {
  public AngularVelocity getCurrentAngularVelocity();

  public AngularVelocity getTargetAngularVelocity();

  public void setTargetAngularVelocity(AngularVelocity angularVelocity);
}
