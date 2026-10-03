package first.robot.mechanism.vision.apriltag;

import java.util.Set;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Time;

/**
 * One robot pose estimate produced by one AprilTag camera, whole again.
 *
 * <p>The camera reports these a few fields at a time, spread across the index-aligned arrays of
 * {@link AprilTagCameraIO.AprilTagCameraIOInputs} so that the logger can serialize them; this
 * record is what {@link AprilTagCamera} reassembles from one index of those arrays every loop. It
 * is never itself serialized, which is why it can hold the two things the inputs cannot: the
 * camera that produced it, which is an IO layer with no struct representation and would be
 * redundant with the log path these are published under anyway, and a variable-length set of tag
 * IDs, which no fixed-size struct could hold.
 *
 * @param cameraIO                the camera that produced this observation
 * @param observedRobotPose       the robot pose the camera solved for
 * @param timestamp               when the frame behind this estimate was captured
 * @param tags                    the IDs of the tags this estimate was solved from
 * @param ambiguity               how ambiguous the solve was, 0 (unambiguous) to 1, or -1 if the
 *                                camera reported none
 * @param reprojectionErrorPixels the residual the solve left behind, or -1 if the camera reported
 *                                none, which a single-tag solve always does
 * @param measuredTagRanges       how far the camera measured its tags to be, in metres, in no
 *                                particular order and possibly empty; see
 *                                {@link AprilTagCameraIO.AprilTagCameraIOInputs#tagRangesMeters}
 */
public record AprilTagPoseObservation(
    AprilTagCameraIO cameraIO,
    Pose3d observedRobotPose,
    Time timestamp,
    Set<Integer> tags,
    double ambiguity,
    double reprojectionErrorPixels,
    double[] measuredTagRanges) {

  /** Whether {@code ambiguity} is a number the camera actually computed. */
  public boolean hasAmbiguity() {
    return ambiguity >= 0.0;
  }

  /** Whether {@code reprojectionErrorPixels} is a number the camera actually computed. */
  public boolean hasReprojectionError() {
    return reprojectionErrorPixels >= 0.0;
  }
}
