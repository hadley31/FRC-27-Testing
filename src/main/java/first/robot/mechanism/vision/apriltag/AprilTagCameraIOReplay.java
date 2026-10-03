package first.robot.mechanism.vision.apriltag;

import org.wpilib.math.geometry.Transform3d;

/**
 * A camera that reads nothing, for replay, where the log supplies the inputs and the IO layer is
 * only there to be named.
 */
public class AprilTagCameraIOReplay implements AprilTagCameraIO {
  private final String m_cameraName;
  private final Transform3d m_robotToCamera;

  /**
   * @param config the camera being replayed. Its name selects which camera's recorded inputs these
   *               become, so it must match the name the log was written under; its mounting
   *               transform is carried through unchanged, since where the camera sat is a property
   *               of the robot rather than something the log recorded.
   */
  public AprilTagCameraIOReplay(AprilTagCameraConfig config) {
    m_cameraName = config.name();
    m_robotToCamera = config.robotToCamera();
  }

  @Override
  public void updateInputs(AprilTagCameraIOInputsAutoLogged inputs) {
  }

  @Override
  public String getName() {
    return m_cameraName;
  }

  @Override
  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  @Override
  public int getPipelineIndex() {
    return -1;
  }

  @Override
  public void setPipelineIndex(int pipelineId) {
  }
}
