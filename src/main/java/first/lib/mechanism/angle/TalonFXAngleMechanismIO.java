package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import first.lib.util.PhoenixUtil;
import first.lib.mechanism.Gains;

public class TalonFXAngleMechanismIO implements AngleMechanismIO {
  private final TalonFX m_motor;
  private final StatusSignal<Angle> m_position;
  private final StatusSignal<AngularVelocity> m_angularVelocity;

  private final PositionTorqueCurrentFOC m_control;

  /**
   * The rotor reading that counts as zero.
   *
   * <p>Kept and applied in software rather than written into the Talon, which is what
   * {@link #setZeroOffset} asks for: a {@code setPosition} or a magnet-offset config is a blocking
   * CAN write, and this arrives whenever someone edits the stored value. It also keeps the offset in
   * one place -- a seeded encoder would disagree with the stored number the moment the Talon
   * rebooted and came back up at whatever it was reading.
   */
  private Angle m_zeroOffset = Rotations.of(0);

  public TalonFXAngleMechanismIO(int port, CANBus bus) {
    m_motor = new TalonFX(port, bus);
    m_position = m_motor.getPosition();
    m_angularVelocity = m_motor.getVelocity();
    m_control = new PositionTorqueCurrentFOC(0);
  }

  @Override
  public void updateInputs(AngleMechanismInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(m_position, m_angularVelocity);
    // Both angles reported in the zeroed frame; the velocity needs no offset, since moving the
    // frame does not change how fast the rotor is turning.
    inputs.currentAngle = m_position.getValue().minus(m_zeroOffset);
    inputs.currentAngularVelocity = m_angularVelocity.getValue();
    inputs.targetAngle = m_control.getPositionMeasure().minus(m_zeroOffset);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    m_motor.setControl(m_control.withPosition(angle.plus(m_zeroOffset)));
  }

  @Override
  public Angle zero() {
    // The signal as refreshed by the last updateInputs, which is this loop's: refreshing again here
    // would be a second CAN read for a value already in hand.
    m_zeroOffset = m_position.getValue();
    return m_zeroOffset;
  }

  @Override
  public void setZeroOffset(Angle offset) {
    m_zeroOffset = offset;
  }

  @Override
  public void halt() {
    m_motor.stopMotor();
  }

  @Override
  public void setGains(Gains gains) {
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
