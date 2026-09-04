package first.robot.util;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public record ShotParameters(
    Angle targetTurretAngle,
    Angle targetHoodAngle,
    AngularVelocity targetFlywheelSpeed) {
}
