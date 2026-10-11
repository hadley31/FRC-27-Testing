package first.robot.mechanism.flywheel;

import first.lib.mechanism.angularvelocity.AngularVelocityMechanism;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismInputsAutoLogged;

public class Flywheel implements AngularVelocityMechanism<AngularVelocityMechanismIO> {
  private final AngularVelocityMechanismIO m_io;
  private final AngularVelocityMechanismInputsAutoLogged m_inputs = new AngularVelocityMechanismInputsAutoLogged();

  public Flywheel(AngularVelocityMechanismIO io) {
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
