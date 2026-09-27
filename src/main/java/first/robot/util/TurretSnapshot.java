package first.robot.util;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * An immutable, field-relative picture of the turret at one instant.
 *
 * <p>This is the whole of the robot that {@link ShotCalculationUtil} needs, which lets the shot math
 * stay a pure function of its inputs. {@code RobotState} produces these; the calculator consumes
 * them and knows nothing else about the robot.
 *
 * @param position the turret pivot's field-relative position, including its height above the floor
 * @param robotRotation the chassis's field-relative heading — the frame turret setpoints are
 *     measured in, so a bearing has to be taken relative to it
 * @param fieldRelativeVelocity the velocity of the turret pivot itself: the chassis velocity plus
 *     the lever-arm term contributed by the chassis's rotation
 */
public record TurretSnapshot(
    Translation3d position,
    Rotation2d robotRotation,
    ChassisVelocities fieldRelativeVelocity) {
}
