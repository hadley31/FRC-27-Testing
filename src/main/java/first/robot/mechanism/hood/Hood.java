package first.robot.mechanism.hood;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;

import first.lib.mechanism.TuningState;
import first.lib.mechanism.angle.AngleMechanism;
import first.lib.mechanism.angle.AngleMechanismIO;

public class Hood implements AngleMechanism<AngleMechanismIO> {
  private final AngleMechanismIO m_io;

  public Hood(AngleMechanismIO io) {
    m_io = io;
  }

  @Override
  public AngleMechanismIO getIO() {
    return m_io;
  }

  @Override
  public TuningState<Angle> getTuningState() {
    return new TuningState<>(getName(), Degrees::of);
  }
}
