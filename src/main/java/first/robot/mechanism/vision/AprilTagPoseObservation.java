package first.robot.mechanism.vision;

import java.util.Set;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.util.struct.StructSerializable;

/**
 * A single robot pose estimate produced by one AprilTag camera.
 *
 * <p>{@code cameraIO} is deliberately left out of the serialized form: the camera is already part
 * of the log path these observations are published under, so repeating it in every record would be
 * redundant, and an IO layer has no meaningful struct representation anyway. It is carried in
 * memory only, so that commands and the vision subsystem can tell which camera an observation came
 * from. Observations that come back out of a log (i.e. during replay) therefore have a
 * {@code null} camera until someone reattaches one with {@link #withCameraIO}; {@link
 * AprilTagCamera} does this for its own observations every loop.
 *
 * @param cameraIO           the camera that produced this observation, or {@code null} if it was
 *                           deserialized and has not been reattached yet (never serialized)
 * @param tags               the IDs of the tags this estimate was solved from
 * @param timestamp          when the frame behind this estimate was captured
 * @param observedRobotPose  the robot pose the camera solved for
 */
public final record AprilTagPoseObservation(
    AprilTagCameraIO cameraIO,
    Pose3d observedRobotPose,
    Time timestamp,
    Set<Integer> tags,
    double ambiguity) implements StructSerializable {

  public sealed interface AprilTagPoseObservationResult {
    public AprilTagPoseObservation observation();

    public boolean accepted();

    sealed interface AprilTagPoseObservationAccepted extends AprilTagPoseObservationResult {
      Vector<N3> stdDevs();

      @Override
      default boolean accepted() {
        return true;
      }
    }

    sealed interface AprilTagPoseObservationRejected extends AprilTagPoseObservationResult {
      String getReason();

      default boolean accepted() {
        return false;
      }
    }

    public final record AcceptedAprilTagPoseObservation(AprilTagPoseObservation observation, Vector<N3> stdDevs)
        implements AprilTagPoseObservationAccepted {
    }

    public final record AverageTagDistanceTooLargeRejection(AprilTagPoseObservation observation,
        Distance averageTagDistance)
        implements AprilTagPoseObservationRejected {
      @Override
      public String getReason() {
        return "Average tag distance too large: " + averageTagDistance.toShortString();
      }
    }

    public final record ObservedPoseTooHighRejection(AprilTagPoseObservation observation,
        Distance distanceAboveGround)
        implements AprilTagPoseObservationRejected {
      @Override
      public String getReason() {
        return "Observed position too elevated: " + distanceAboveGround.toShortString();
      }
    }

    public final record ObservationTooAmbiguousRejection(AprilTagPoseObservation observation, double ambiguity)
        implements AprilTagPoseObservationRejected {
      @Override
      public String getReason() {
        return "Tag result too ambiguous: " + ambiguity;
      }
    }

    public final record StaleObservationRejection(AprilTagPoseObservation observation, Time latency)
        implements AprilTagPoseObservationRejected {
      @Override
      public String getReason() {
        return "Observation too stale: " + latency.toShortString();
      }
    }
  }

}
