package first.robot.command;

import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.robot.mechanism.vision.AprilTagPoseObservation;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.AcceptedAprilTagPoseObservation;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.AprilTagPoseObservationAccepted;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.AprilTagPoseObservationRejected;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.AverageTagDistanceTooLargeRejection;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.ObservedPoseTooHighRejection;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.StaleObservationRejection;
import first.robot.util.PoseEstimator.VisionObservation;

public class AprilTagVisionProcessor {
  private static final Distance MAX_AVERAGE_TAG_DISTANCE = Feet.of(25.0);
  private static final Distance MAX_DISTANCE_ABOVE_GROUND = Feet.of(1.0);
  private static final Time MAX_LATENCY = Seconds.of(0.1);

  private final Field m_field;
  private final Consumer<VisionObservation> m_observationConsumer;

  public AprilTagVisionProcessor(Field field,
      Consumer<VisionObservation> observationConsumer) {
    m_field = field;
    m_observationConsumer = observationConsumer;
  }

  public void process(List<AprilTagPoseObservation> observations) {
    observations.stream().map(this::processObservation).forEach(result -> {
      String logPrefix = "Vision/%s".formatted(result.observation().cameraIO().getName());

      boolean observationAccepted = true;
      String rejectionReason = "";
      Pose3d observedRobotPose = result.observation().observedRobotPose();

      switch (result) {
        case AprilTagPoseObservationAccepted accepted:
          submitAcceptedObservation(accepted);
          break;
        case AprilTagPoseObservationRejected rejected:
          observationAccepted = false;
          rejectionReason = rejected.getReason();
          break;
      }

      Logger.recordOutput(logPrefix + "/Accepted", observationAccepted);
      Logger.recordOutput(logPrefix + "/ObservedRobotPose", observedRobotPose);
      Logger.recordOutput(logPrefix + "/RejectionReason", rejectionReason);
    });
  }

  private AprilTagPoseObservationResult processObservation(AprilTagPoseObservation observation) {
    if (observation.observedRobotPose().getMeasureZ().gt(MAX_DISTANCE_ABOVE_GROUND)) {
      return new ObservedPoseTooHighRejection(observation, observation.observedRobotPose().getMeasureZ());
    }

    Distance avgTagDistance = Meters.of(observation.tags().stream().map(m_field::getTagPose)
        .flatMap(Optional::stream)
        .mapToDouble(x -> x.getTranslation().getDistance(observation.observedRobotPose().getTranslation())).average()
        .orElse(Double.POSITIVE_INFINITY));

    if (avgTagDistance.gt(MAX_AVERAGE_TAG_DISTANCE)) {
      return new AverageTagDistanceTooLargeRejection(observation, avgTagDistance);
    }

    Time latency = RobotController.getMeasureTime().minus(observation.timestamp());
    if (latency.gt(MAX_LATENCY)) {
      return new StaleObservationRejection(observation, latency);
    }

    int tagCount = observation.tags().size();

    double stdDevX = 1.0;
    double stdDevY = 1.0;
    double stdDevTheta = 1.0;

    double tagCountFactor = tagCount > 1 ? 10 : 1;
    double distanceFactor = Math.pow(avgTagDistance.in(Meters), 2);

    stdDevX *= distanceFactor / tagCountFactor;
    stdDevY *= distanceFactor / tagCountFactor;
    stdDevTheta *= distanceFactor / tagCountFactor;

    return new AcceptedAprilTagPoseObservation(observation, VecBuilder.fill(stdDevX, stdDevY, stdDevTheta));
  }

  private void submitAcceptedObservation(AprilTagPoseObservationAccepted acceptedResult) {
    VisionObservation visionObservation = new VisionObservation(
        acceptedResult.observation().timestamp(),
        acceptedResult.observation().observedRobotPose(),
        acceptedResult.stdDevs());
    m_observationConsumer.accept(visionObservation);
  }
}
