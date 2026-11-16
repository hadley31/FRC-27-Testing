package first.robot.mechanism.intake;

import first.lib.mechanism.angularvelocity.AngularVelocityMechanism;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;

/**
 * The intake's rollers, which run independently of the pivot that carries them: the arm can be
 * commanded down while the rollers idle, and the rollers can be purged while it is stowed.
 */
public class IntakeRollers extends AngularVelocityMechanism<AngularVelocityMechanismIO> {
  public IntakeRollers(AngularVelocityMechanismIO io) {
    super(io);
  }
}
