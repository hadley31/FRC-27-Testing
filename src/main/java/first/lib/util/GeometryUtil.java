package first.lib.util;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;

import choreo.util.ChoreoAllianceFlipUtil.Flipper;

public class GeometryUtil {
  public static Rotation3d rotation3dFromYaw(Angle angle) {
    return new Rotation3d(Degrees.zero(), Degrees.zero(), angle);
  }

  public static Rotation3d rotation3dFromPitch(Angle angle) {
    return new Rotation3d(Degrees.zero(), angle, Degrees.zero());
  }

  public static Transform3d transform3dFromYaw(Angle angle) {
    return new Transform3d(Translation3d.ZERO, rotation3dFromYaw(angle));
  }

  public static Transform3d transform3dFromTranslation(Translation3d translation) {
    return new Transform3d(translation, Rotation3d.ZERO);
  }

  public static Transform3d transform3dFromRotation(Rotation3d rotation) {
    return new Transform3d(Translation3d.ZERO, rotation);
  }

  public static Transform3d transform3dFromPose(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  public static Transform2d transform2dFromTranslation(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.ZERO);
  }

  public static Transform2d transform2dFromRotation(Rotation2d rotation) {
    return new Transform2d(Translation2d.ZERO, rotation);
  }

  public static Transform2d transform2dFromPose(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Pose2d pose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.ZERO);
  }

  public static Pose2d pose2dFromTransform(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  public static Pose3d pose3dFromTranslation(Translation3d translation) {
    return new Pose3d(translation, Rotation3d.ZERO);
  }

  public static Pose3d pose3dFromTransform(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  public static Translation2d flip(Translation2d translation) {
    return Flipper.FRC_CURRENT.flip(translation);
  }

  public static Pose2d flip(Pose2d pose) {
    return Flipper.FRC_CURRENT.flip(pose);
  }

  public static Translation3d flip(Translation3d translation) {
    return Flipper.FRC_CURRENT.flip(translation);
  }

  public static Pose3d flip(Pose3d pose) {
    return Flipper.FRC_CURRENT.flip(pose);
  }
}
