package first.robot.mechanism.vision.apriltag;

/**
 * Builds the IO layer for a declared camera.
 *
 * <p>What varies between a real robot, a simulation and a log replay is only how a camera is read,
 * never which cameras exist or where they are mounted. This interface is that seam: the cameras are
 * declared as {@link AprilTagCameraConfig}s once, and one factory — chosen by
 * {@link AprilTagVisionFactory} for the current mode — turns every declaration into the IO that
 * mode needs. Nothing above the IO layer has to branch on the mode, and a test can supply cameras
 * of its own by passing a factory rather than by pretending to be a mode.
 */
@FunctionalInterface
public interface AprilTagCameraIOFactory {
  /**
   * Returns the IO to read {@code config}'s camera through.
   *
   * @param config the camera to build for
   */
  public AprilTagCameraIO create(AprilTagCameraConfig config);
}
