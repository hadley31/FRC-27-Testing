package first.robot.mechanism.vision.apriltag;

/**
 * A camera that reads nothing, for replay, where the log supplies every input.
 *
 * <p>Nothing to hold and nothing to do. What the camera is called and where it sits come from its
 * {@link AprilTagCameraConfig}, which {@link AprilTagCamera} already has, and everything the log
 * recorded arrives through the inputs object; so a replay IO is the absence of an IO, spelled out.
 */
public class AprilTagCameraIOReplay implements AprilTagCameraIO {
  @Override
  public void updateInputs(AprilTagCameraIOInputsAutoLogged inputs) {
  }

  @Override
  public int getPipelineIndex() {
    return -1;
  }

  @Override
  public void setPipelineIndex(int pipelineId) {
  }
}
