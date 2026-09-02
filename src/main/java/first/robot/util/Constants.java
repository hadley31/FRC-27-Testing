package first.robot.util;

import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

public class Constants {
    public static final double kControllerDeadband = 0.05;

    public static final LinearVelocity kMaxDriveLinearVelocity = FeetPerSecond.of(10);
    public static final AngularVelocity kMaxDriveAngularVelocity = RotationsPerSecond.of(5);
}
