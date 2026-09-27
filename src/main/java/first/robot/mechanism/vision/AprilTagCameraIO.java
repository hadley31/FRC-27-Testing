package first.robot.mechanism.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Time;

import first.lib.mechanism.LoggedIO;

public interface AprilTagCameraIO extends LoggedIO<AprilTagCameraIOInputsAutoLogged> {
  @AutoLog
  public static class AprilTagCameraIOInputs {
    public Time timestamp = null;
    public Pose3d observedRobotPose = null;
    public int[] tags = new int[0];
    public double ambiguity = 0.0;
    public int pipelineId = -1;
  }

  public String getName();

  public int getPipelineId();

  public void setPipelineId(int pipelineId);
}
