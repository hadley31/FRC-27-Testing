package first.lib.mechanism.angularvelocity;

import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.TunableGains;

public class TalonFXAngularVelocityMechanismIO implements AngularVelocityMechanismIO {
  private AngularVelocity m_angularVelocity = RPM.zero();
  private AngularVelocity m_targetAngularVelocity = RPM.zero();

  @Override
  public AngularVelocity getCurrentAngularVelocity() {
    return m_angularVelocity;
  }

  @Override
  public AngularVelocity getTargetAngularVelocity() {
    return m_targetAngularVelocity;
  }

  @Override
  public void setTargetAngularVelocity(AngularVelocity angularVelocity) {
    m_targetAngularVelocity = angularVelocity;
  }

  @Override
  public void halt() {

  }

  @Override
  public void setGains(TunableGains gains) {
    // todo
  }
}
