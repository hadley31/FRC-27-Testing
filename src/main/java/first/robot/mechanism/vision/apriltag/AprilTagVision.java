package first.robot.mechanism.vision.apriltag;

import java.util.List;
import java.util.stream.Stream;

import first.lib.mechanism.LoggedComponent;
import first.lib.mechanism.LoggedMultiComponentMechanism;

public class AprilTagVision implements LoggedMultiComponentMechanism {
  private final List<AprilTagCamera> m_cameras;

  public AprilTagVision(AprilTagCameraIO... cameraIOs) {
    m_cameras = Stream.of(cameraIOs).map(AprilTagCamera::new).toList();

    getRegisteredScheduler().addPeriodic(this::updateComponents);
  }

  @Override
  public List<? extends LoggedComponent<?, ?>> getComponents() {
    return m_cameras;
  }

  /**
   * Returns every observation from every camera since the last loop. A camera can contribute more
   * than one, so this is not one entry per camera.
   */
  public List<AprilTagPoseObservation> getLatestObservations() {
    return m_cameras.stream().map(AprilTagCamera::getObservations).flatMap(List::stream).toList();
  }
}
