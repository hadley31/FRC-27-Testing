package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.LinearVelocity;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * A module backed by a physics model instead of hardware, so the drivetrain can be driven in
 * simulation and in replay.
 *
 * <p>The sim is always voltage controlled, so it carries its own gains rather than the ones Tuner X
 * produced for the real motors. Odometry is sampled once per loop: the high-frequency thread exists
 * to compensate for CAN latency, and there is none here.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private static final DCMotor kDriveGearbox = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor kSteerGearbox = DCMotor.getKrakenX60Foc(1);

  private static final double kDriveKp = 0.05;
  private static final double kDriveKd = 0.0;
  private static final double kDriveKs = 0.0;
  /** Volts per radian per second at the wheel. */
  private static final double kDriveKv = 0.14486;
  private static final double kSteerKp = 8.0;
  private static final double kSteerKd = 0.0;

  private static final double kMaxVoltage = 12.0;

  /** Metres of travel per radian of wheel rotation, which for a rolling wheel is its radius. */
  private final double m_metersPerWheelRadian;

  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_steerSim;

  private final PIDController m_driveController = new PIDController(kDriveKp, 0.0, kDriveKd);
  private final PIDController m_steerController = new PIDController(kSteerKp, 0.0, kSteerKd);

  private boolean m_driveClosedLoop = false;
  private boolean m_steerClosedLoop = false;
  private double m_driveFeedforwardVolts = 0.0;
  private double m_driveAppliedVolts = 0.0;
  private double m_steerAppliedVolts = 0.0;

  public SwerveModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    m_metersPerWheelRadian = constants.WheelRadius;

    m_driveSim = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            kDriveGearbox, constants.DriveInertia, constants.DriveMotorGearRatio),
        kDriveGearbox);
    m_steerSim = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            kSteerGearbox, constants.SteerInertia, constants.SteerMotorGearRatio),
        kSteerGearbox);

    m_steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    if (m_driveClosedLoop) {
      m_driveAppliedVolts = m_driveFeedforwardVolts
          + m_driveController.calculate(m_driveSim.getAngularVelocity());
    } else {
      m_driveController.reset();
    }

    if (m_steerClosedLoop) {
      m_steerAppliedVolts = m_steerController.calculate(m_steerSim.getAngularPosition());
    } else {
      m_steerController.reset();
    }

    m_driveSim.setInputVoltage(Math.clamp(m_driveAppliedVolts, -kMaxVoltage, kMaxVoltage));
    m_steerSim.setInputVoltage(Math.clamp(m_steerAppliedVolts, -kMaxVoltage, kMaxVoltage));
    m_driveSim.update(Drive.kLoopPeriod.in(Seconds));
    m_steerSim.update(Drive.kLoopPeriod.in(Seconds));

    inputs.driveConnected = true;
    inputs.drivePosition = Meters.of(m_driveSim.getAngularPosition() * m_metersPerWheelRadian);
    inputs.driveVelocity = MetersPerSecond.of(m_driveSim.getAngularVelocity() * m_metersPerWheelRadian);
    inputs.driveAppliedVoltage = Volts.of(m_driveAppliedVolts);
    inputs.driveCurrent = Amps.of(Math.abs(m_driveSim.getCurrentDraw()));

    inputs.steerConnected = true;
    inputs.steerEncoderConnected = true;
    inputs.steerPosition = new Rotation2d(m_steerSim.getAngularPosition());
    inputs.steerAbsolutePosition = inputs.steerPosition;
    inputs.steerVelocity = RadiansPerSecond.of(m_steerSim.getAngularVelocity());
    inputs.steerAppliedVoltage = Volts.of(m_steerAppliedVolts);
    inputs.steerCurrent = Amps.of(Math.abs(m_steerSim.getCurrentDraw()));

    inputs.odometryTimestampSeconds = new double[] { Timer.getTimestamp() };
    inputs.odometryPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(inputs.drivePosition, inputs.steerPosition) };
  }

  @Override
  public void setDriveVelocity(LinearVelocity velocity) {
    double wheelRadiansPerSecond = velocity.in(MetersPerSecond) / m_metersPerWheelRadian;

    m_driveClosedLoop = true;
    m_driveFeedforwardVolts = kDriveKs * Math.signum(wheelRadiansPerSecond)
        + kDriveKv * wheelRadiansPerSecond;
    m_driveController.setSetpoint(wheelRadiansPerSecond);
  }

  @Override
  public void setSteerPosition(Rotation2d position) {
    m_steerClosedLoop = true;
    m_steerController.setSetpoint(position.getRadians());
  }

  @Override
  public void halt() {
    m_driveClosedLoop = false;
    m_steerClosedLoop = false;
    m_driveAppliedVolts = 0.0;
    m_steerAppliedVolts = 0.0;
  }
}
