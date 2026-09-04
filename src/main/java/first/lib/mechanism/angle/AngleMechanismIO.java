package first.lib.mechanism.angle;

import org.wpilib.units.measure.Angle;

import first.lib.mechanism.G3MechanismIO;

public interface AngleMechanismIO extends G3MechanismIO {
  public Angle getAngle();

  public Angle getTargetAngle();

  public void setTargetAngle(Angle angle);
}
