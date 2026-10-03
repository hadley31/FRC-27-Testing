package first.robot.util;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.List;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import com.ctre.phoenix6.CANBus;

import first.lib.util.GeometryUtil;
import first.robot.mechanism.vision.apriltag.AprilTagCameraConfig;

public class Constants {

  public static final double kControllerDeadband = 0.05;

  public static final class RobotModeConstants {
    /** How the robot code is being run, which selects the AdvantageKit logging configuration. */
    public enum Mode {
      /** Running on a real robot. */
      REAL,
      /** Running a physics simulator. */
      SIM,
      /** Replaying a log file from a real robot. */
      REPLAY
    }

    /** Which mode to use when not running on a real robot. Set to REPLAY to replay a log. */
    public static final Mode kSimMode = Mode.SIM;

    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : kSimMode;
  }

  public static final class DriveConstants {
    public static final LinearVelocity kMaxLinearVelocity = FeetPerSecond.of(10);
    public static final AngularVelocity kMaxAngularVelocity = RotationsPerSecond.of(5);
  }

  public static final class ElectricalConstants {
    public static final CANBus CAN_BUS = new CANBus("CANivore");
  }

  public static final class RobotGeometryConstants {
    public static final Translation3d kRobotToTurret3d = new Translation3d(Inches.of(-3.75), Inches.of(7.25),
        Inches.of(13.25));
    public static final Translation2d kRobotToTurret = kRobotToTurret3d.toTranslation2d();

    public static final Transform3d kTurretToCamera = new Transform3d(Inches.of(6.359), Inches.of(0), Inches.of(1.792),
        GeometryUtil.rotation3dFromPitch(Degrees.of(-24)));

    // Where each AprilTag camera sits relative to the robot origin. Bolted to the chassis rather
    // than riding the turret, so unlike the turret camera above these are true at every turret
    // angle and need no per-loop correction. All three are pitched up slightly, to put the tags
    // nearer the middle of the frame than the floor is.
    public static final Transform3d kRobotToRearCamera = new Transform3d(
        new Translation3d(Inches.of(-9.394), Inches.of(-12.564), Inches.of(20.659)),
        new Rotation3d(Degrees.zero(), Degrees.of(-14), Degrees.of(160)));

    public static final Transform3d kRobotToRightCamera = new Transform3d(
        new Translation3d(Inches.of(-5.587), Inches.of(-13.648), Inches.of(20.6695)),
        new Rotation3d(Degrees.zero(), Degrees.of(-14), Degrees.of(-60)));

    public static final Transform3d kRobotToLeftCamera = new Transform3d(
        new Translation3d(Inches.of(1.054), Inches.of(14.429), Inches.of(10.172)),
        new Rotation3d(Degrees.zero(), Degrees.of(-12), Degrees.of(70)));

    public static final Distance kRobotWidthWithBumpers = Inches.of(38.438);
    public static final Distance kRobotLengthWithBumpers = Inches.of(31.256);
  }

  public static final class VisionConstants {
    /**
     * Every AprilTag camera on the robot, declared once.
     *
     * <p>Nothing here says how a camera is read: {@code AprilTagVisionFactory} turns each of these
     * into a real camera, a simulated one or a replay placeholder depending on the mode. Adding a
     * camera to the robot means adding a line here and nothing else.
     */
    public static final List<AprilTagCameraConfig> kCameras = List.of(
        new AprilTagCameraConfig("Rear", RobotGeometryConstants.kRobotToRearCamera),
        new AprilTagCameraConfig("Right", RobotGeometryConstants.kRobotToRightCamera),
        new AprilTagCameraConfig("Left", RobotGeometryConstants.kRobotToLeftCamera));
  }
}
