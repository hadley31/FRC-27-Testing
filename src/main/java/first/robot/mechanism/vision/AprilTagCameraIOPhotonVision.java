package first.robot.mechanism.vision;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.system.Timer;

public class AprilTagCameraIOPhotonVision implements AprilTagCameraIO {
  private final String m_cameraName;

  public AprilTagCameraIOPhotonVision(String cameraName) {
    m_cameraName = cameraName;
  }

  @Override
  public void updateInputs(AprilTagCameraIOInputsAutoLogged inputs) {
    inputs.timestamp = Seconds.of(Timer.getTimestamp());
    inputs.observedRobotPose = new Pose3d();
    inputs.ambiguity = 0.0;
    inputs.tags = new int[] { 1, 2 };
    inputs.pipelineId = -1;
  }

  @Override
  public String getName() {
    return m_cameraName;
  }

  @Override
  public int getPipelineId() {
    return -1;
  }

  @Override
  public void setPipelineId(int pipelineId) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPipelineId'");
  }
}
