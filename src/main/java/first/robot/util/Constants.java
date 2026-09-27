package first.robot.util;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import com.ctre.phoenix6.CANBus;

import first.lib.util.GeometryUtil;

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

    public static final Distance kRobotWidthWithBumpers = Inches.of(38.438);
    public static final Distance kRobotLengthWithBumpers = Inches.of(31.256);
  }
}
