package first.robot.util;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public record ShotParameters(
    Angle m_targetTurretAngle,
    Angle m_targetHoodAngle,
    AngularVelocity m_targetFlywheelSpeed) {
}
