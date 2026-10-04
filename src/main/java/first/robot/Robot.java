// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static first.robot.util.Constants.ElectricalConstants.CAN_BUS;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import first.lib.mechanism.angle.TalonFXAngleMechanismIO;
import first.lib.mechanism.angularvelocity.TalonFXAngularVelocityMechanismIO;
import first.lib.tuning.Toggle;
import first.lib.util.LoggedTracer;
import first.robot.mechanism.drive.Drive;
import first.robot.mechanism.drive.GyroIO;
import first.robot.mechanism.drive.GyroIOPigeon2;
import first.robot.mechanism.drive.SwerveModuleIO;
import first.robot.mechanism.drive.SwerveModuleIOSim;
import first.robot.mechanism.drive.SwerveModuleIOTalonFX;
import first.robot.mechanism.drive.TunerConstants;
import first.robot.mechanism.feeder.Feeder;
import first.robot.mechanism.flywheel.Flywheel;
import first.robot.mechanism.hood.Hood;
import first.robot.mechanism.turret.Turret;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOReplay;
import first.robot.mechanism.vision.apriltag.AprilTagVision;
import first.robot.mechanism.vision.apriltag.AprilTagVisionFactory;
import first.robot.mechanism.vision.apriltag.AprilTagVisionFactoryPhotonVision;
import first.robot.mechanism.vision.apriltag.AprilTagVisionFactoryPhotonVisionSim;
import first.robot.mechanism.vision.apriltag.RobotHeadingSource;
import first.robot.mode.CompetitionAutoFactory;
import first.robot.mode.CompetitionTeleopFactory;
import first.robot.util.Constants;
import first.robot.util.Constants.RobotModeConstants.Mode;
import first.robot.util.Constants.VisionConstants;
import first.robot.util.PoseEstimator;
import first.robot.util.SchedulerLogger;
import first.robot.util.Tuning;

/**
 * A plain {@link LoggedRobot}: AdvantageKit drives the main loop through
 * {@code IterativeRobotBase}, and the robot modes come from the driver station rather than from
 * selected opmodes.
 *
 * <p>Commands v3 supports this arrangement directly. Bindings made here in the constructor (while
 * the robot is disabled and no opmode is selected) land in the scheduler's global binding scope, and
 * bindings made from {@link #autonomousInit()} or {@link #teleopInit()} land in a robot-mode scope
 * that the scheduler drops on its own when the mode ends.
 */
public class Robot extends LoggedRobot {
  public final Drive drive;
  public final Flywheel flywheel;
  public final Turret turret;
  public final Hood hood;
  public final Feeder feeder;
  public final AprilTagVision vision;
  public final RobotState state;

  public Robot() {
    // AdvantageKit must be configured and started before anything else is constructed, so that IO
    // implementations can log or replay their first set of inputs on the very first cycle.
    configureLogging();

    drive = new Drive(
        createGyroIO(),
        createSwerveModuleIO(TunerConstants.kFrontLeft),
        createSwerveModuleIO(TunerConstants.kFrontRight),
        createSwerveModuleIO(TunerConstants.kBackLeft),
        createSwerveModuleIO(TunerConstants.kBackRight));
    flywheel = new Flywheel(new TalonFXAngularVelocityMechanismIO(0, CAN_BUS));
    turret = new Turret(new TalonFXAngleMechanismIO(1, CAN_BUS));
    hood = new Hood(new TalonFXAngleMechanismIO(2, CAN_BUS));
    feeder = new Feeder(new TalonFXAngularVelocityMechanismIO(3, CAN_BUS));

    var poseEstimator = new PoseEstimator(drive.getKinematics(), Tuning.kOdometryTiltCompensation, Tuning.kWallClamp);

    Field field = Fields.DEFAULT_FIELD.loadField();

    // The two poses handed over here are deliberately different. A single-tag solve wants the best
    // heading available, which is the fused estimate: the trig solve returns the heading it was
    // given, so an observation built on the estimate disagrees with it about rotation by nothing and
    // leaves heading to the multi-tag solves that can actually measure it. The simulated cameras,
    // on the other hand, must be posed from odometry, because rendering sightings from the
    // vision-corrected pose would only ever confirm the correction they were rendered from.
    vision = createVisionFactory(
        field,
        () -> poseEstimator.getEstimatedPose().getRotation(),
        poseEstimator::getOdometryPose)
        .createVision(VisionConstants.kCameras);

    state = new RobotState(drive, turret, hood, flywheel, vision, poseEstimator, field);

    var teleopFactory = new CompetitionTeleopFactory(this);
    var autoFactory = new CompetitionAutoFactory(this);
    var autoChooser = autoFactory.createAutoChooser();

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    RobotModeTriggers.teleop().whileTrue(teleopFactory.getTeleopCommand());

    Scheduler.getDefault().addPeriodic(Toggle::logNonDefaults);
    Scheduler.getDefault().addPeriodic(state::periodic);
  }

  /**
   * How AprilTag cameras are read in the current mode.
   *
   * <p>Each mode names its own factory, and they are free to disagree: a robot reading its cameras
   * some other way would change only the {@code REAL} line and go on being simulated by
   * PhotonVision's tooling. See {@link AprilTagVisionFactory}.
   *
   * @param field                   the tag layout the cameras solve against, and in simulation the
   *                                layout that gets rendered
   * @param headingSource           the robot heading single-tag solves need
   * @param groundTruthPoseSupplier where the robot really is, used only in simulation
   */
  private static AprilTagVisionFactory createVisionFactory(
      Field field,
      RobotHeadingSource headingSource,
      Supplier<Pose2d> groundTruthPoseSupplier) {
    return switch (Constants.RobotModeConstants.CURRENT_MODE) {
      case REAL -> new AprilTagVisionFactoryPhotonVision(field, headingSource);

      case SIM -> new AprilTagVisionFactoryPhotonVisionSim(
          field, headingSource, groundTruthPoseSupplier);

      // Replay reads nothing, since the log supplies the inputs, and must not construct a real
      // camera: that would open NetworkTables subscriptions and run a coprocessor version check for
      // values that are about to be overwritten from the log.
      case REPLAY -> config -> new AprilTagCameraIOReplay();
    };
  }

  /** The gyro for the current mode. Simulation runs without one and lets odometry infer heading. */
  private static GyroIO createGyroIO() {
    return Constants.RobotModeConstants.CURRENT_MODE == Mode.REAL
        ? new GyroIOPigeon2(TunerConstants.kPigeonId, CAN_BUS)
        : new GyroIO() {
        };
  }

  private static SwerveModuleIO createSwerveModuleIO(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    return Constants.RobotModeConstants.CURRENT_MODE == Mode.REAL
        ? new SwerveModuleIOTalonFX(constants)
        : new SwerveModuleIOSim(constants);
  }

  private void configureLogging() {
    Logger.recordMetadata("ProjectName", "FRC-27-Testing");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());

    switch (Constants.RobotModeConstants.CURRENT_MODE) {
      case REAL -> {
        // Log to a USB stick ("/U/logs") and publish live data to NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    Scheduler.getDefault().run();
    LoggedTracer.record("Scheduler/run");
    SchedulerLogger.refresh(Scheduler.getDefault());
    LoggedTracer.record("SchedulerLogger/refresh");

    // Every span opened this loop should have been closed by now, so anything still open never
    // reached its endTrace -- an early return or a thrown exception in between. Dropping them here
    // keeps a leaked entry from making that name's next measurement run from a stale start, and
    // publishes the offenders so the mistake is visible.
    LoggedTracer.clearTraces();
  }
}
