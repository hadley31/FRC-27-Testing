package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;

import first.lib.mechanism.TunableGains;

public class TalonFXAngleMechanismIO implements AngleMechanismIO {
  public static class TalonFXAngleMechanismIOConfig {

  }

  private Angle m_angle = Degrees.zero();
  private Angle m_targetAngle = Degrees.zero();

  public TalonFXAngleMechanismIO() {
    // config motor
  }

  @Override
  public Angle getAngle() {
    return m_angle;
  }

  @Override
  public Angle getTargetAngle() {
    return m_targetAngle;
  }

  @Override
  public void setTargetAngle(Angle angle) {
    m_targetAngle = angle;
  }

  @Override
  public void halt() {

  }

  @Override
  public void setGains(TunableGains gains) {
    // todo
  }
}
