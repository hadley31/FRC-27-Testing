package first.lib.mechanism.angle;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import first.lib.mechanism.PhoenixUtil;
import first.lib.mechanism.TunableGains;

public class TalonFXAngleMechanismIO implements AngleMechanismIO {
  private final TalonFX m_motor;
  private final StatusSignal<Angle> m_position;
  private final StatusSignal<AngularVelocity> m_angularVelocity;

  private final PositionTorqueCurrentFOC m_control;

  public TalonFXAngleMechanismIO(int port, CANBus bus) {
    m_motor = new TalonFX(port, bus);
    m_position = m_motor.getPosition();
    m_angularVelocity = m_motor.getVelocity();
    m_control = new PositionTorqueCurrentFOC(0);
  }

  @Override
  public Angle getCurrentAngle() {
    return m_position.getValue();
  }

  @Override
  public Angle getTargetAngle() {
    return m_control.getPositionMeasure();
  }

  @Override
  public AngularVelocity getCurrentAngularVelocity() {
    return m_angularVelocity.getValue();
  }

  @Override
  public void setTargetAngle(Angle angle) {
    m_motor.setControl(m_control.withPosition(angle));
  }

  @Override
  public void halt() {
    m_motor.stopMotor();
  }

  @Override
  public void setGains(TunableGains gains) {
    var slot0Configs = new Slot0Configs()
        .withKS(gains.kS())
        .withKV(gains.kV())
        .withKA(gains.kA())
        .withKG(gains.kG())
        .withKP(gains.kP())
        .withKI(gains.kI())
        .withKD(gains.kD());
    PhoenixUtil.tryUntilOk(() -> m_motor.getConfigurator().apply(slot0Configs));
  }
}
