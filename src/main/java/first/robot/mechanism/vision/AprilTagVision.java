package first.robot.mechanism.vision;

import java.util.List;
import java.util.stream.Stream;

import org.wpilib.command3.Mechanism;

import first.lib.mechanism.LoggedComponent;
import first.lib.mechanism.LoggedMultiComponentMechanism;

public class AprilTagVision implements Mechanism, LoggedMultiComponentMechanism {
  private final List<AprilTagCamera> m_cameras;

  public AprilTagVision(AprilTagCameraIO... cameraIOs) {
    m_cameras = Stream.of(cameraIOs).map(AprilTagCamera::new).toList();

    getRegisteredScheduler().addPeriodic(this::updateComponents);
  }

  @Override
  public List<? extends LoggedComponent<?, ?>> getComponents() {
    return m_cameras;
  }

  public List<AprilTagPoseObservation> getLatestObservations() {
    return m_cameras.stream().map(AprilTagCamera::getObservation).filter(x -> x != null).toList();
  }
}
