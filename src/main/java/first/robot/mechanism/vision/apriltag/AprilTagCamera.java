package first.robot.mechanism.vision.apriltag;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import first.lib.mechanism.LoggedComponent;

/**
 * One AprilTag camera: its declaration, the IO that reads it, and the inputs that come back, turned
 * into whole {@link AprilTagPoseObservation}s for whatever consumes them.
 *
 * <p>Logged as {@code <vision>/<camera name>}. The name is the declared one rather than this class's
 * -- every camera on the robot is this class -- and it is also what selects a camera's recorded
 * inputs during replay, so renaming a camera in {@link AprilTagCameraConfig} orphans its old logs.
 */
public class AprilTagCamera implements LoggedComponent<AprilTagCameraIO, AprilTagCameraIOInputsAutoLogged> {
  private final AprilTagVision m_vision;
  private final AprilTagCameraConfig m_config;
  private final AprilTagCameraIO m_io;
  private final AprilTagCameraIOInputsAutoLogged m_inputs;

  /**
   * @param vision the vision mechanism this camera belongs to, which is what its inputs are logged
   *               beneath. A camera is constructed by that mechanism for this reason.
   * @param io     how to read the camera, and -- through {@link AprilTagCameraIO#getConfig()} -- the
   *               declaration it was built from, which is what this camera is named after and what
   *               its observations are attributed to. Held here rather than asked for per use: a
   *               config is immutable declaration data, so there is nothing to re-read, and every
   *               IO can only ever answer with the config it was built from.
   */
  public AprilTagCamera(AprilTagVision vision, AprilTagCameraIO io) {
    m_vision = vision;
    m_config = io.getConfig();
    m_io = io;
    m_inputs = new AprilTagCameraIOInputsAutoLogged();
  }

  @Override
  public AprilTagCameraIO getIO() {
    return m_io;
  }

  @Override
  public AprilTagCameraIOInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public String getName() {
    return m_config.name();
  }

  @Override
  public AprilTagVision getMechanism() {
    return m_vision;
  }

  /**
   * Returns every estimate this camera reported during the loop that just ran, oldest first, as
   * observations attributed to this camera. Empty when the camera published nothing new.
   */
  public List<AprilTagPoseObservation> getObservations() {
    // The arrays are index-aligned and the IO layer owes us equal lengths, so in normal
    // operation this is just their common length. Taking the shortest keeps a mismatched or
    // truncated log from indexing out of bounds part way through the loop; it costs a trailing
    // entry in a case that should not arise at all.
    int count = Math.min(
        Math.min(m_inputs.observedRobotPoses.length, m_inputs.timestampsSeconds.length),
        Math.min(
            Math.min(m_inputs.ambiguities.length, m_inputs.reprojectionErrorsPixels.length),
            Math.min(m_inputs.tagIds.length, m_inputs.tagRangesMeters.length)));

    List<AprilTagPoseObservation> observations = new ArrayList<>(count);

    for (int i = 0; i < count; i++) {
      observations.add(new AprilTagPoseObservation(
          m_config,
          m_inputs.observedRobotPoses[i],
          Seconds.of(m_inputs.timestampsSeconds[i]),
          Arrays.stream(m_inputs.tagIds[i]).boxed().collect(Collectors.toUnmodifiableSet()),
          m_inputs.ambiguities[i],
          m_inputs.reprojectionErrorsPixels[i],
          m_inputs.tagRangesMeters[i]));
    }

    return observations;
  }
}
