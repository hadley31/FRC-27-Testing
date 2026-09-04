package first.lib.mechanism.position;

import org.wpilib.units.measure.Distance;

import first.lib.mechanism.G3MechanismIO;

public interface PositionMechanismIO extends G3MechanismIO {
  public Distance getPosition();

  public Distance getTargetPosition();

  public void setTargetPosition(Distance position);
}
