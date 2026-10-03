package first.robot.mechanism.vision.apriltag;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import first.lib.mechanism.LoggedComponent;

public class AprilTagCamera implements LoggedComponent<AprilTagCameraIO, AprilTagCameraIOInputsAutoLogged> {
  private final String m_name;
  private final String m_logName;
  private final AprilTagCameraIO m_io;
  private final AprilTagCameraIOInputsAutoLogged m_inputs;

  public AprilTagCamera(AprilTagCameraIO io) {
    m_io = io;
    m_inputs = new AprilTagCameraIOInputsAutoLogged();
    m_name = io.getName();
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
          m_io,
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
