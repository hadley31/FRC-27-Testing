package first.robot.util;

import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.framework.RobotBase;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

public class Constants {
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

    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : kSimMode;

    public static final double kControllerDeadband = 0.05;

    public static final LinearVelocity kMaxDriveLinearVelocity = FeetPerSecond.of(10);
    public static final AngularVelocity kMaxDriveAngularVelocity = RotationsPerSecond.of(5);
}
