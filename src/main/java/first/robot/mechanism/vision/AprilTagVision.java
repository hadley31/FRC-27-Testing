package first.robot.mechanism.vision;

import java.util.List;
import java.util.stream.Stream;

import org.wpilib.command3.Mechanism;

import first.lib.LoggedIOMechanismContainer;
import first.lib.mechanism.LoggedInputContainer;

public class AprilTagVision implements Mechanism, LoggedIOMechanismContainer {
  private final List<AprilTagCamera> m_cameras;

  public AprilTagVision(AprilTagCameraIO... cameraIOs) {
    m_cameras = Stream.of(cameraIOs).map(AprilTagCamera::new).toList();

    getRegisteredScheduler().addPeriodic(this::updateIOs);
  }

  @Override
  public List<? extends LoggedInputContainer<?, ?>> getIOContainers() {
    return m_cameras;
  }

  public List<AprilTagPoseObservation> getLatestObservations() {
    return m_cameras.stream().map(AprilTagCamera::getObservation).filter(x -> x != null).toList();
  }
}
