package first.robot.mechanism.vision.apriltag;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import first.lib.mechanism.LoggedComponent;

public class AprilTagCamera implements LoggedComponent<AprilTagCameraIO, AprilTagCameraIOInputsAutoLogged> {
  private final AprilTagCameraConfig m_config;
  private final String m_name;
  private final String m_logName;
  private final AprilTagCameraIO m_io;
  private final AprilTagCameraIOInputsAutoLogged m_inputs;

  /**
   * @param config the camera as declared, which is what its observations are attributed to. Taken
   *               here rather than read back off {@code io} because where a camera sits and what it
   *               is called are declaration data: the IO was built from this same config, so asking
   *               it would only be a round trip through a device that cannot know better.
   * @param io     how to read the camera
   */
  public AprilTagCamera(AprilTagCameraConfig config, AprilTagCameraIO io) {
    m_config = config;
    m_io = io;
    m_inputs = new AprilTagCameraIOInputsAutoLogged();
    m_name = config.name();
    m_logName = "Camera/" + m_name;
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
  public String getLogName() {
    return m_logName;
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
