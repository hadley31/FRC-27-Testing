package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

/**
 * The drivetrain's physical description, in the form Phoenix's module code expects.
 *
 * <p>Every value here belongs to one particular robot and has to be measured or tuned on it. The
 * shape of this file matches what the Tuner X swerve generator emits, so its output can be pasted in
 * to replace the placeholders below.
 *
 * @see <a href=
 *      "https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html">Tuner X
 *      Swerve Project Generator</a>
 */
public final class TunerConstants {
  private TunerConstants() {
  }

  /** Free speed of a module at 12 V, which is what module setpoints are desaturated against. */
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

  public static final int kPigeonId = 1;

  // MARK: - Gains

  private static final Slot0Configs kSteerGains = new Slot0Configs()
      .withKP(100.0)
      .withKI(0.0)
      .withKD(0.5)
      .withKS(0.1)
      .withKV(1.91)
      .withKA(0.0)
      .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  private static final Slot0Configs kDriveGains = new Slot0Configs()
      .withKP(0.1)
      .withKI(0.0)
      .withKD(0.0)
      .withKS(0.0)
      .withKV(0.124);

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // MARK: - Hardware

  private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  /** Without a Phoenix Pro licence the fused and synced modes fall back to RemoteCANcoder. */
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  /** The stator current at which the wheels break traction. Tune this on the robot. */
  private static final Current kSlipCurrent = Amps.of(120.0);

  private static final double kDriveGearRatio = 7.363636363636365;
  private static final double kSteerGearRatio = 15.42857142857143;

  /** Drive motor turns per turn of the azimuth, from the azimuth dragging the drive stage with it. */
  private static final double kCouplingGearRatio = 3.8181818181818183;

  private static final Distance kWheelRadius = Inches.of(2.167);

  private static final TalonFXConfiguration kDriveInitialConfigs = new TalonFXConfiguration();

  private static final TalonFXConfiguration kSteerInitialConfigs = new TalonFXConfiguration()
      // An azimuth needs very little torque, so a low limit buys brownout headroom for free.
      .withCurrentLimits(new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(60))
          .withStatorCurrentLimitEnable(true));

  private static final CANcoderConfiguration kEncoderInitialConfigs = new CANcoderConfiguration();

  // MARK: - Simulation

  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  // MARK: - Modules

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kModuleFactory =
      new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCouplingGearRatio)
          .withWheelRadius(kWheelRadius)
          .withDriveMotorGains(kDriveGains)
          .withSteerMotorGains(kSteerGains)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withDriveMotorType(kDriveMotorType)
          .withSteerMotorType(kSteerMotorType)
          .withFeedbackSource(kSteerFeedbackType)
          .withDriveMotorInitialConfigs(kDriveInitialConfigs)
          .withSteerMotorInitialConfigs(kSteerInitialConfigs)
          .withEncoderInitialConfigs(kEncoderInitialConfigs)
          .withDriveInertia(kDriveInertia)
          .withSteerInertia(kSteerInertia)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withSteerFrictionVoltage(kSteerFrictionVoltage);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final int kFrontLeftDriveMotorId = 3;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 1;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.0);
  private static final Distance kFrontLeftXPos = Inches.of(10);
  private static final Distance kFrontLeftYPos = Inches.of(10);

  private static final int kFrontRightDriveMotorId = 5;
  private static final int kFrontRightSteerMotorId = 4;
  private static final int kFrontRightEncoderId = 2;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(0.0);
  private static final Distance kFrontRightXPos = Inches.of(10);
  private static final Distance kFrontRightYPos = Inches.of(-10);

  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 6;
  private static final int kBackLeftEncoderId = 3;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(0.0);
  private static final Distance kBackLeftXPos = Inches.of(-10);
  private static final Distance kBackLeftYPos = Inches.of(10);

  private static final int kBackRightDriveMotorId = 9;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 4;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.0);
  private static final Distance kBackRightXPos = Inches.of(-10);
  private static final Distance kBackRightYPos = Inches.of(-10);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontLeft = kModuleFactory
      .createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          kFrontLeftXPos,
          kFrontLeftYPos,
          kInvertLeftSide,
          true,
          false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kFrontRight = kModuleFactory
      .createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          kFrontRightXPos,
          kFrontRightYPos,
          kInvertRightSide,
          true,
          false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackLeft = kModuleFactory
      .createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          kBackLeftXPos,
          kBackLeftYPos,
          kInvertLeftSide,
          true,
          false);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> kBackRight = kModuleFactory
      .createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          kBackRightXPos,
          kBackRightYPos,
          kInvertRightSide,
          true,
          false);

  /** Where each module sits relative to the robot's centre, in the same order {@link Drive} uses. */
  public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kFrontLeft.LocationX, kFrontLeft.LocationY),
      new Translation2d(kFrontRight.LocationX, kFrontRight.LocationY),
      new Translation2d(kBackLeft.LocationX, kBackLeft.LocationY),
      new Translation2d(kBackRight.LocationX, kBackRight.LocationY),
  };
}
