package first.robot.mechanism.turret;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;

import first.lib.tuning.TuningState;
import first.lib.mechanism.angle.AngleMechanism;
import first.lib.mechanism.angle.AngleMechanismInputsAutoLogged;
import first.lib.mechanism.angle.AngleMechanismIO;

public class Turret implements AngleMechanism<AngleMechanismIO> {
  private final AngleMechanismIO m_io;
  private final AngleMechanismInputsAutoLogged m_inputs = new AngleMechanismInputsAutoLogged();

  public Turret(AngleMechanismIO io) {
    m_io = io;
  }

  @Override
  public AngleMechanismIO getIO() {
    return m_io;
  }

  @Override
  public AngleMechanismInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public TuningState<Angle> getTuningState() {
    return new TuningState<>(getName(), Degrees::of);
  }
}
