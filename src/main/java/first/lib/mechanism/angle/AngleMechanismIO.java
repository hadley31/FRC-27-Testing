package first.lib.mechanism.angle;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3MechanismIO;

public interface AngleMechanismIO extends G3MechanismIO {
  public Angle getCurrentAngle();

  public Angle getTargetAngle();

  public AngularVelocity getCurrentAngularVelocity();

  public void setTargetAngle(Angle angle);
}
