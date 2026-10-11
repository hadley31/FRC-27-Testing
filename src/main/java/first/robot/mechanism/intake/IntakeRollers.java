package first.robot.mechanism.intake;

import first.lib.mechanism.angularvelocity.AngularVelocityMechanism;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismInputsAutoLogged;

/**
 * The intake's rollers, which run independently of the pivot that carries them: the arm can be
 * commanded down while the rollers idle, and the rollers can be purged while it is stowed.
 */
public class IntakeRollers implements AngularVelocityMechanism<AngularVelocityMechanismIO> {
  private final AngularVelocityMechanismIO m_io;
  private final AngularVelocityMechanismInputsAutoLogged m_inputs = new AngularVelocityMechanismInputsAutoLogged();

  public IntakeRollers(AngularVelocityMechanismIO io) {
    m_io = io;
    getRegisteredScheduler().addPeriodic(this::update);
    registerTuningCommand();
  }

  @Override
  public AngularVelocityMechanismIO getIO() {
    return m_io;
  }

  @Override
  public AngularVelocityMechanismInputsAutoLogged getInputs() {
    return m_inputs;
  }
}
