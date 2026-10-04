package first.robot.mechanism.vision.apriltag;

/**
 * A camera that reads nothing, for replay, where the log supplies every input.
 *
 * <p>Nothing to do: everything the camera recorded arrives through the inputs object, so a replay IO
 * is the absence of an IO, spelled out. It still carries the declaration, because that is how
 * {@link AprilTagCamera} learns what this camera is called -- and the name is what selects its
 * recorded inputs out of the log, so a replay camera that did not know its own name would read
 * nothing back.
 *
 * @param config the camera this stands in for
 */
public record AprilTagCameraIOReplay(AprilTagCameraConfig config) implements AprilTagCameraIO {
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

  @Override
  public AprilTagCameraConfig getConfig() {
    return config;
  }
}
