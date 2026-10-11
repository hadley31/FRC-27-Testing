package first.robot.mechanism.hood;

import first.lib.mechanism.angle.AngleMechanism;
import first.lib.mechanism.angle.AngleMechanismIO;
import first.lib.mechanism.angle.AngleMechanismInputsAutoLogged;

public class Hood implements AngleMechanism<AngleMechanismIO> {
  private final AngleMechanismIO m_io;
  private final AngleMechanismInputsAutoLogged m_inputs = new AngleMechanismInputsAutoLogged();

  public Hood(AngleMechanismIO io) {
    m_io = io;
    getRegisteredScheduler().addPeriodic(this::update);
    registerTuningCommand();
  }

  @Override
  public AngleMechanismIO getIO() {
    return m_io;
  }

  @Override
  public AngleMechanismInputsAutoLogged getInputs() {
    return m_inputs;
  }
}
