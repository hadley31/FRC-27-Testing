package first.robot.util;

import static org.wpilib.units.Units.Meters;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

import choreo.util.ChoreoAllianceFlipUtil.Flipper;

public final class FieldConstants {
  public static final Field FIELD = Fields.DEFAULT_FIELD.loadField();
  public static final Distance FIELD_LENGTH = Meters.of(FIELD.getFieldLength());
  public static final Distance FIELD_WIDTH = Meters.of(FIELD.getFieldWidth());

  private static final Translation3d HUB_POSITION = new Translation3d(
      4.6256194,
      FIELD_WIDTH.in(Meters) / 2.0,
      1.8);

  private FieldConstants() {
  }

  public static Translation3d getHubPosition3d() {
    return Flipper.FRC_CURRENT.flip(HUB_POSITION);
  }

  public static Translation2d getHubPosition2d() {
    return getHubPosition3d().toTranslation2d();
  }

  /**
   * Clamps a pose so that a robot of the given footprint stays entirely inside the field walls.
   *
   * <p>Field geometry lives here rather than in the pose estimator so that the estimator does not
   * need to know what field it is on.
   *
   * @param robotLength the robot's X extent including bumpers
   * @param robotWidth the robot's Y extent including bumpers
   */
  public static Pose2d clampToFieldBounds(Pose2d pose, Distance robotLength, Distance robotWidth) {
    // Half-extents of the rotated robot rectangle. For an L x W
    // rectangle rotated by theta:
    //   halfExtentX = (|L*cos| + |W*sin|) / 2
    //   halfExtentY = (|L*sin| + |W*cos|) / 2
    Rotation2d rotation = pose.getRotation();
    double cos = Math.abs(rotation.getCos());
    double sin = Math.abs(rotation.getSin());
    double length = robotLength.in(Meters);
    double width = robotWidth.in(Meters);

    double halfExtentX = (length * cos + width * sin) / 2.0;
    double halfExtentY = (length * sin + width * cos) / 2.0;

    return new Pose2d(
        new Translation2d(
            Math.clamp(pose.getX(), halfExtentX, FIELD_LENGTH.in(Meters) - halfExtentX),
            Math.clamp(pose.getY(), halfExtentY, FIELD_WIDTH.in(Meters) - halfExtentY)),
        rotation);
  }
}
