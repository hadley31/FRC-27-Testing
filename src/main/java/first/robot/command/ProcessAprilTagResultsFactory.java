package first.robot.command;

import static org.wpilib.units.Units.Meters;

import java.util.Optional;
import java.util.function.Consumer;

import org.wpilib.command3.Command;
import org.wpilib.fields.Field;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.measure.Distance;

import first.robot.mechanism.vision.AprilTagPoseObservation;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.AverageTagDistanceTooLarge;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.ObservationAccepted;
import first.robot.mechanism.vision.AprilTagPoseObservation.AprilTagPoseObservationResult.ObservedPositionTooHighRejection;
import first.robot.mechanism.vision.AprilTagVision;

public class ProcessAprilTagResultsFactory {
  private static final Distance MAX_AVERAGE_TAG_DISTANCE = Meters.of(5.0);
  private static final Distance MAX_DISTANCE_ABOVE_GROUND = Meters.of(0.2);

  private final Field m_field;
  private final AprilTagVision m_vision;
  private final Consumer<AprilTagPoseObservationResult> m_observationConsumer;

  public ProcessAprilTagResultsFactory(Field field, AprilTagVision vision,
      Consumer<AprilTagPoseObservationResult> observationConsumer) {
    m_field = field;
    m_vision = vision;
    m_observationConsumer = observationConsumer;
  }

  public Command build() {
    return Command.requiring(m_vision).executing(coroutine -> {
      while (true) {
        m_vision.getLatestObservations().stream().map(this::processObservation).forEach(m_observationConsumer);
        coroutine.yield();
      }
    }).named("Process Vision Results");
  }

  private AprilTagPoseObservationResult processObservation(AprilTagPoseObservation observation) {
    if (observation.observedRobotPose().getMeasureZ().gt(MAX_DISTANCE_ABOVE_GROUND)) {
      return new ObservedPositionTooHighRejection(observation, observation.observedRobotPose().getMeasureZ());
    }

    Distance avgTagDistance = Meters.of(observation.tags().stream().map(m_field::getTagPose)
        .flatMap(Optional::stream)
        .mapToDouble(x -> x.getTranslation().getDistance(observation.observedRobotPose().getTranslation())).average()
        .orElse(Double.POSITIVE_INFINITY));

    if (avgTagDistance.gt(MAX_AVERAGE_TAG_DISTANCE)) {
      return new AverageTagDistanceTooLarge(observation, avgTagDistance);
    }

    double stdDevX = 1.0;
    double stdDevY = 1.0;
    double stdDevTheta = 1.0;

    return new ObservationAccepted(observation, VecBuilder.fill(stdDevX, stdDevY, stdDevTheta));
  }
}
