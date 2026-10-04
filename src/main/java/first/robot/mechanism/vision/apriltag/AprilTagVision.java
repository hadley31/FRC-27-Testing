package first.robot.mechanism.vision.apriltag;

import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Stream;

import org.wpilib.math.geometry.Pose2d;

import first.lib.mechanism.LoggedComponent;
import first.lib.mechanism.LoggedMultiComponentMechanism;

public class AprilTagVision implements LoggedMultiComponentMechanism {
  private final List<AprilTagCamera> m_cameras;
  private final Consumer<Pose2d> m_onPoseReset;

  public AprilTagVision(AprilTagCameraIO... cameraIOs) {
    this(pose -> {
    }, cameraIOs);
  }

  /**
   * @param onPoseReset what to tell about a pose reset, which outside simulation is nothing. See
   *                    {@link #onPoseReset}.
   */
  public AprilTagVision(Consumer<Pose2d> onPoseReset, AprilTagCameraIO... cameraIOs) {
    m_cameras = Stream.of(cameraIOs).map(AprilTagCamera::new).toList();
    m_onPoseReset = onPoseReset;

    getRegisteredScheduler().addPeriodic(this::updateComponents);
  }

  /**
   * Tells the vision layer that the pose estimate has been teleported rather than driven.
   *
   * <p>A no-op on a real robot, where moving the estimate does not move the robot and the cameras go
   * on reporting where it really is. In simulation the cameras are rendered from the estimate's own
   * odometry pose, so a reset moves the simulated robot too -- and the simulation keeps a history of
   * where it has been that has to be told, or it spends the next frame or two rendering the robot
   * part way back to where it was. See {@link PhotonVisionSim#resetRobotPose}.
   */
  public void onPoseReset(Pose2d pose) {
    m_onPoseReset.accept(pose);
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
