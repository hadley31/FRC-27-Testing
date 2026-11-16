package first.robot.mechanism.feeder;

import first.lib.mechanism.angularvelocity.AngularVelocityMechanism;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;

public class Feeder extends AngularVelocityMechanism<AngularVelocityMechanismIO> {
  public Feeder(AngularVelocityMechanismIO io) {
    super(io);
  }
}
