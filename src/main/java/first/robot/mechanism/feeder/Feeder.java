package first.robot.mechanism.feeder;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.TuningState;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanism;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;

public class Feeder implements AngularVelocityMechanism<AngularVelocityMechanismIO> {
  private final AngularVelocityMechanismIO m_io;

  public Feeder(AngularVelocityMechanismIO io) {
    m_io = io;
  }

  @Override
  public AngularVelocityMechanismIO getIO() {
    return m_io;
  }

  @Override
  public TuningState<AngularVelocity> getTuningState() {
    return new TuningState<>(getName(), RPM::of);
  }
}
