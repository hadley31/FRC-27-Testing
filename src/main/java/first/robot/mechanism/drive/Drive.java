package first.robot.mechanism.drive;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import choreo.trajectory.SwerveSample;

public class Drive implements Mechanism {
  private void drive(ChassisVelocities speeds) {
    // no-op
  }

  public Pose2d getPose() {
    return new Pose2d();
  }

  public void resetPose(Pose2d pose) {
    // no-op
  }

  /** Drives the robot to follow a single sample of a Choreo trajectory. */
  public void followSample(SwerveSample sample) {
    drive(sample.getChassisVelocities());
  }

  public Command driveCommand(Supplier<ChassisVelocities> speeds) {
    return run(coroutine -> {
      while (true) {
        drive(speeds.get());
        coroutine.yield();
      }
    }).named("Drive");
  }
}
