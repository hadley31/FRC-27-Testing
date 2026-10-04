package first.robot.mechanism.vision.apriltag;

import org.wpilib.fields.Field;

/**
 * Cameras read from PhotonVision coprocessors on a real robot.
 *
 * @param field         the tag layout the coprocessors solve against, which must be the layout they
 *                      were given: a filter judging observations by a layout the cameras never used
 *                      is judging them by the wrong field
 * @param headingSource the robot heading single-tag solves need; see {@link RobotHeadingSource} for
 *                      what it must be
 */
public record AprilTagVisionFactoryPhotonVision(Field field, RobotHeadingSource headingSource)
    implements AprilTagVisionFactory {
  @Override
  public AprilTagCameraIO createCameraIO(AprilTagCameraConfig config) {
    return new AprilTagCameraIOPhotonVision(config, field, headingSource);
  }
}
