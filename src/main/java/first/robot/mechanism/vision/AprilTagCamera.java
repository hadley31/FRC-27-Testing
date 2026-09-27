package first.robot.mechanism.vision;

import java.util.Arrays;
import java.util.stream.Collectors;

import first.lib.mechanism.LoggedInputContainer;

public class AprilTagCamera implements LoggedInputContainer<AprilTagCameraIO, AprilTagCameraIOInputsAutoLogged> {
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

  public AprilTagPoseObservation getObservation() {
    return new AprilTagPoseObservation(
        m_io,
        m_inputs.observedRobotPose,
        m_inputs.timestamp,
        Arrays.stream(m_inputs.tags).boxed().collect(Collectors.toUnmodifiableSet()),
        m_inputs.ambiguity);
  }
}
