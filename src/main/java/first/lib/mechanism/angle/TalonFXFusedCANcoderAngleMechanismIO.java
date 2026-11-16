package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import first.lib.mechanism.Gains;
import first.lib.mechanism.ZeroableMechanismIO;
import first.lib.util.PhoenixUtil;

/**
 * An angle mechanism driven by a TalonFX reading a fused CANcoder, whose zero lives in the
 * CANcoder's {@code MagnetOffset} configuration rather than anywhere in this program.
 *
 * <h2>Why a CANcoder is what makes a stored zero mean anything</h2>
 *
 * <p>A bare TalonFX's integrated sensor is absolute only within one <em>rotor</em> rotation. Behind
 * a reduction that is a fraction of a degree of mechanism travel, and the turn count is gone at the
 * next power cycle -- so no stored number can put the frame back, however it is stored. A CANcoder
 * on the mechanism side is absolute over the mechanism's own rotation, which is what lets a zero
 * found yesterday still be true tomorrow. That is the reason this class exists alongside
 * {@link TalonFXAngleMechanismIO}, which can only hold a zero until the robot is switched off.
 *
 * <h2>Why the configuration, and not a field</h2>
 *
 * <p>{@code MagnetOffset} is added to the CANcoder's reported position inside the sensor, so the
 * position this IO reads is already zeroed and {@link #updateInputs} has no arithmetic to do and
 * nothing to remember. The device is then the single source of truth: {@link #zero()} reads the
 * offset back off it rather than from a copy here that could disagree with it. Both of those cost a
 * blocking CAN round trip, which is why they happen when someone presses a button or when the robot
 * boots, and never in a loop.
 *
 * <p>Two things the sensor does to a value written here are worth knowing. {@code MagnetOffset} is
 * clamped to +/-1 rotation, which is never a real limit -- the offset only ever has to cancel a raw
 * magnet reading, itself within one rotation -- but a nonsense value typed into the stored offset by
 * hand is clamped rather than rejected. And it is quantized to 1/4096 of a rotation, so the zero the
 * sensor keeps can sit up to about 0.04 degrees from the one asked for. {@link #zero()} returns the
 * value it asked for rather than reading back the rounded one, which is deliberate: re-applying that
 * same value at the next boot rounds into the same bucket, so the frame is reproducible even though
 * the number on the dashboard is a hair finer than the sensor can hold.
 *
 * <p>One caveat worth knowing on the field: the TalonFX's fused position tracks the rotor between
 * syncs with the CANcoder, so a freshly trimmed offset lands as the two re-sync rather than on the
 * instant. The offset {@link #zero()} returns is right regardless -- it is computed from the trim,
 * not read back after it -- so what gets stored is correct even if the reported position takes a
 * moment to agree.
 */
public class TalonFXFusedCANcoderAngleMechanismIO implements AngleMechanismIO {
  private final TalonFX m_motor;
  private final CANcoder m_encoder;

  /**
   * CANcoder rotations per mechanism rotation, needed because {@code MagnetOffset} is in the
   * sensor's rotations while every angle crossing this interface is in the mechanism's. One when the
   * CANcoder is on the mechanism's own output, which is the usual way to mount it.
   */
  private final double m_sensorToMechanismRatio;

  private final StatusSignal<Angle> m_position;
  private final StatusSignal<AngularVelocity> m_angularVelocity;

  private final PositionTorqueCurrentFOC m_control;

  /**
   * @param motorId the TalonFX's CAN id
   * @param encoderId the CANcoder's CAN id
   * @param bus the bus both devices are on
   * @param rotorToSensorRatio rotor rotations per CANcoder rotation -- the reduction between the
   *     motor and wherever the CANcoder is mounted
   * @param sensorToMechanismRatio CANcoder rotations per mechanism rotation, {@code 1.0} for a
   *     CANcoder on the mechanism's output
   */
  public TalonFXFusedCANcoderAngleMechanismIO(
      int motorId,
      int encoderId,
      CANBus bus,
      double rotorToSensorRatio,
      double sensorToMechanismRatio) {
    m_motor = new TalonFX(motorId, bus);
    m_encoder = new CANcoder(encoderId, bus);
    m_sensorToMechanismRatio = sensorToMechanismRatio;

    m_position = m_motor.getPosition();
    m_angularVelocity = m_motor.getVelocity();
    m_control = new PositionTorqueCurrentFOC(0);

    // Only the feedback sub-config, so this does not reset gains the tuning command has applied.
    PhoenixUtil.tryUntilOk(() -> m_motor.getConfigurator().apply(
        new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(encoderId)
            .withRotorToSensorRatio(rotorToSensorRatio)
            .withSensorToMechanismRatio(sensorToMechanismRatio)));
  }

  @Override
  public void updateInputs(AngleMechanismInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(m_position, m_angularVelocity);
    // Already zeroed, by the sensor. Nothing to subtract and nothing to remember.
    inputs.currentAngle = m_position.getValue();
    inputs.currentAngularVelocity = m_angularVelocity.getValue();
    inputs.targetAngle = m_control.getPositionMeasure();
  }

  @Override
  public void setTargetAngle(Angle angle) {
    m_motor.setControl(m_control.withPosition(angle));
  }

  @Override
  public Angle zero() {
    MagnetSensorConfigs magnet = refreshedMagnetConfig();

    // The sensor adds MagnetOffset to what it reports, so taking away exactly the position showing
    // now -- in the sensor's rotations -- puts zero right here.
    magnet.MagnetOffset -= m_position.getValue().in(Rotations) * m_sensorToMechanismRatio;

    PhoenixUtil.tryUntilOk(() -> m_encoder.getConfigurator().apply(magnet));

    return mechanismOffsetOf(magnet);
  }

  @Override
  public void setZeroOffset(Angle offset) {
    MagnetSensorConfigs magnet = refreshedMagnetConfig();
    magnet.MagnetOffset = -offset.in(Rotations) * m_sensorToMechanismRatio;

    PhoenixUtil.tryUntilOk(() -> m_encoder.getConfigurator().apply(magnet));
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

  /**
   * The CANcoder's magnet configuration as the device currently holds it.
   *
   * <p>Read back rather than kept, so the sensor stays the one place the zero lives -- and so the
   * direction and discontinuity point someone set in Tuner X survive an offset being written here.
   */
  private MagnetSensorConfigs refreshedMagnetConfig() {
    var magnet = new MagnetSensorConfigs();
    PhoenixUtil.tryUntilOk(() -> m_encoder.getConfigurator().refresh(magnet));
    return magnet;
  }

  /**
   * The offset in the terms {@link ZeroableMechanismIO} defines it, {@code raw - reported}, given a
   * magnet config. The sign flips because {@code MagnetOffset} is added to what the sensor reports
   * while the contract's offset is taken away from it.
   */
  private Angle mechanismOffsetOf(MagnetSensorConfigs magnet) {
    return Rotations.of(-magnet.MagnetOffset / m_sensorToMechanismRatio);
  }
}
