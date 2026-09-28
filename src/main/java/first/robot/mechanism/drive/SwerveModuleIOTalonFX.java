package first.robot.mechanism.drive;

import static first.robot.util.Constants.ElectricalConstants.CAN_BUS;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.Queue;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import first.lib.mechanism.PhoenixUtil;

/**
 * A module driven by two Talon FXs with a CANcoder on the azimuth, configured from the constants
 * Tuner X generates.
 *
 * <p>The drive motor's feedback is scaled by the gearing so that a "rotation" here is a rotation of
 * the wheel, which is all this class needs to trade rotations for metres. The azimuth runs against
 * the CANcoder directly with continuous wrap enabled, so a commanded angle always takes the shorter
 * way round.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  private static final double kConnectedDebounceSeconds = 0.5;

  private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> m_constants;

  /** Metres of travel per rotation of the wheel. */
  private final double m_metersPerWheelRotation;

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_steerEncoder;

  private final VelocityVoltage m_driveVelocityVoltage = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC m_driveVelocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);
  private final PositionVoltage m_steerPositionVoltage = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC m_steerPositionTorqueCurrent = new PositionTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> m_drivePosition;
  private final StatusSignal<AngularVelocity> m_driveVelocity;
  private final StatusSignal<Voltage> m_driveAppliedVoltage;
  private final StatusSignal<Current> m_driveCurrent;

  private final StatusSignal<Angle> m_steerPosition;
  private final StatusSignal<Angle> m_steerAbsolutePosition;
  private final StatusSignal<AngularVelocity> m_steerVelocity;
  private final StatusSignal<Voltage> m_steerAppliedVoltage;
  private final StatusSignal<Current> m_steerCurrent;

  private final Queue<Double> m_timestampQueue;
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_steerPositionQueue;

  private final Debouncer m_driveConnectedDebouncer = new Debouncer(kConnectedDebounceSeconds,
      DebounceType.FALLING);
  private final Debouncer m_steerConnectedDebouncer = new Debouncer(kConnectedDebounceSeconds,
      DebounceType.FALLING);
  private final Debouncer m_steerEncoderConnectedDebouncer = new Debouncer(kConnectedDebounceSeconds,
      DebounceType.FALLING);

  public SwerveModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    m_constants = constants;
    m_metersPerWheelRotation = 2.0 * Math.PI * constants.WheelRadius;

    m_driveMotor = new TalonFX(constants.DriveMotorId, CAN_BUS);
    m_steerMotor = new TalonFX(constants.SteerMotorId, CAN_BUS);
    m_steerEncoder = new CANcoder(constants.EncoderId, CAN_BUS);

    configureDriveMotor();
    configureSteerMotor();
    configureSteerEncoder();

    m_drivePosition = m_driveMotor.getPosition();
    m_driveVelocity = m_driveMotor.getVelocity();
    m_driveAppliedVoltage = m_driveMotor.getMotorVoltage();
    m_driveCurrent = m_driveMotor.getStatorCurrent();

    m_steerPosition = m_steerMotor.getPosition();
    m_steerAbsolutePosition = m_steerEncoder.getAbsolutePosition();
    m_steerVelocity = m_steerMotor.getVelocity();
    m_steerAppliedVoltage = m_steerMotor.getMotorVoltage();
    m_steerCurrent = m_steerMotor.getStatorCurrent();

    // Only the two positions feed odometry, so only they are worth the bus bandwidth to publish at
    // the odometry rate. Everything else is diagnostic.
    BaseStatusSignal.setUpdateFrequencyForAll(PhoenixOdometryThread.getFrequency(),
        m_drivePosition, m_steerPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50),
        m_driveVelocity, m_driveAppliedVoltage, m_driveCurrent,
        m_steerAbsolutePosition, m_steerVelocity, m_steerAppliedVoltage, m_steerCurrent);
    ParentDevice.optimizeBusUtilizationForAll(m_driveMotor, m_steerMotor, m_steerEncoder);

    var odometryThread = PhoenixOdometryThread.getInstance();
    m_timestampQueue = odometryThread.makeTimestampQueue();
    m_drivePositionQueue = odometryThread.registerSignal(m_drivePosition.clone());
    m_steerPositionQueue = odometryThread.registerSignal(m_steerPosition.clone());
  }

  private void configureDriveMotor() {
    var config = m_constants.DriveMotorInitialConfigs;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = m_constants.DriveMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.Slot0 = m_constants.DriveMotorGains;

    // Scaling feedback by the gearing is what lets everything above this class speak in wheel
    // rotations instead of motor rotations.
    config.Feedback.SensorToMechanismRatio = m_constants.DriveMotorGearRatio;

    config.TorqueCurrent.PeakForwardTorqueCurrent = m_constants.SlipCurrent;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -m_constants.SlipCurrent;
    config.CurrentLimits.StatorCurrentLimit = m_constants.SlipCurrent;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    PhoenixUtil.tryUntilOk(() -> m_driveMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(() -> m_driveMotor.setPosition(Rotations.of(0)));
  }

  private void configureSteerMotor() {
    var config = m_constants.SteerMotorInitialConfigs;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = m_constants.SteerMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.Slot0 = m_constants.SteerMotorGains;

    config.Feedback.FeedbackRemoteSensorID = m_constants.EncoderId;
    config.Feedback.FeedbackSensorSource = switch (m_constants.FeedbackSource) {
      case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
      case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
      case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
      default -> throw new IllegalArgumentException(
          "Unsupported steer feedback source: " + m_constants.FeedbackSource);
    };
    config.Feedback.RotorToSensorRatio = m_constants.SteerMotorGearRatio;

    // Lets a commanded angle wrap through +/-180 degrees instead of unwinding the long way.
    config.ClosedLoopGeneral.ContinuousWrap = true;

    PhoenixUtil.tryUntilOk(() -> m_steerMotor.getConfigurator().apply(config));
  }

  private void configureSteerEncoder() {
    var config = m_constants.EncoderInitialConfigs;
    config.MagnetSensor.MagnetOffset = m_constants.EncoderOffset;
    config.MagnetSensor.SensorDirection = m_constants.EncoderInverted
        ? SensorDirectionValue.Clockwise_Positive
        : SensorDirectionValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(() -> m_steerEncoder.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    var driveStatus = BaseStatusSignal.refreshAll(
        m_drivePosition, m_driveVelocity, m_driveAppliedVoltage, m_driveCurrent);
    var steerStatus = BaseStatusSignal.refreshAll(
        m_steerPosition, m_steerVelocity, m_steerAppliedVoltage, m_steerCurrent);
    var steerEncoderStatus = BaseStatusSignal.refreshAll(m_steerAbsolutePosition);

    inputs.driveConnected = m_driveConnectedDebouncer.calculate(driveStatus.isOK());
    inputs.drivePosition = Meters
        .of(m_drivePosition.getValue().in(Rotations) * m_metersPerWheelRotation);
    inputs.driveVelocity = MetersPerSecond
        .of(m_driveVelocity.getValue().in(RotationsPerSecond) * m_metersPerWheelRotation);
    inputs.driveAppliedVoltage = m_driveAppliedVoltage.getValue();
    inputs.driveCurrent = m_driveCurrent.getValue();

    inputs.steerConnected = m_steerConnectedDebouncer.calculate(steerStatus.isOK());
    inputs.steerEncoderConnected = m_steerEncoderConnectedDebouncer.calculate(steerEncoderStatus.isOK());
    inputs.steerPosition = new Rotation2d(m_steerPosition.getValue());
    inputs.steerAbsolutePosition = new Rotation2d(m_steerAbsolutePosition.getValue());
    inputs.steerVelocity = m_steerVelocity.getValue();
    inputs.steerAppliedVoltage = m_steerAppliedVoltage.getValue();
    inputs.steerCurrent = m_steerCurrent.getValue();

    inputs.odometryTimestampSeconds = m_timestampQueue.stream()
        .mapToDouble(Double::doubleValue)
        .toArray();
    var drivePositions = m_drivePositionQueue.stream()
        .mapToDouble(rotations -> rotations * m_metersPerWheelRotation)
        .toArray();
    var steerPositions = m_steerPositionQueue.stream()
        .map(Rotation2d::fromRotations)
        .toArray(Rotation2d[]::new);
    inputs.odometryPositions = new SwerveModulePosition[drivePositions.length];
    for (int i = 0; i < drivePositions.length; i++) {
      inputs.odometryPositions[i] = new SwerveModulePosition(drivePositions[i], steerPositions[i]);
    }

    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_steerPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(LinearVelocity velocity) {
    double wheelRotationsPerSecond = velocity.in(MetersPerSecond) / m_metersPerWheelRotation;

    m_driveMotor.setControl(switch (m_constants.DriveMotorClosedLoopOutput) {
      case Voltage -> m_driveVelocityVoltage.withVelocity(wheelRotationsPerSecond);
      case TorqueCurrentFOC -> m_driveVelocityTorqueCurrent.withVelocity(wheelRotationsPerSecond);
    });
  }

  @Override
  public void setSteerPosition(Rotation2d position) {
    m_steerMotor.setControl(switch (m_constants.SteerMotorClosedLoopOutput) {
      case Voltage -> m_steerPositionVoltage.withPosition(position.getRotations());
      case TorqueCurrentFOC -> m_steerPositionTorqueCurrent.withPosition(position.getRotations());
    });
  }

  @Override
  public void halt() {
    m_driveMotor.stopMotor();
    m_steerMotor.stopMotor();
  }
}
