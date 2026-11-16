package first.robot.mechanism.flywheel;

import first.lib.mechanism.angularvelocity.AngularVelocityMechanism;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;

public class Flywheel extends AngularVelocityMechanism<AngularVelocityMechanismIO> {
  public Flywheel(AngularVelocityMechanismIO io) {
    super(io);
  }
}
