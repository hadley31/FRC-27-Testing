package first.robot.mechanism.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import choreo.trajectory.SwerveSample;

public class Drive implements Mechanism {
  private final PIDController m_pathXController = new PIDController(1.0, 0.0, 0.0);
  private final PIDController m_pathYController = new PIDController(1.0, 0.0, 0.0);
  private final PIDController m_pathThetaController = new PIDController(1.0, 0.0, 0.0);

  public Drive() {

  }

  private void drive(ChassisVelocities speeds) {

  }

  public Pose2d getPose() {
    return new Pose2d();
  }

  public void resetPose(Pose2d pose) {

  }

  /** Drives the robot to follow a single sample of a Choreo trajectory. */
  public void followSample(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = getPose();
    var targetPose = sample.getPose();

    var targetSpeeds = sample.getChassisVelocities();
    targetSpeeds.vx += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vy += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omega += m_pathThetaController.calculate(pose.getRotation().getRadians(),
        sample.heading);

    Logger.recordOutput("Drive/PathFollower/TargetPose", targetPose);
    Logger.recordOutput("Drive/PathFollower/Error/X", targetPose.getMeasureX().minus(pose.getMeasureX()));
    Logger.recordOutput("Drive/PathFollower/Error/Y", targetPose.getMeasureY().minus(pose.getMeasureY()));
    Logger.recordOutput("Drive/PathFollower/Error/Rotation",
        targetPose.getRotation().minus(pose.getRotation()).getMeasure());

    // Convert field-relative speeds to robot-relative and run
    ChassisVelocities robotRelativeSpeeds = new ChassisVelocities(
        targetSpeeds.vx,
        targetSpeeds.vy,
        targetSpeeds.omega).toRobotRelative(pose.getRotation());
    drive(robotRelativeSpeeds);
  }

  /** Drives the robot to follow a single sample of a Choreo trajectory. */
  public void followSampleSimple(SwerveSample sample) {
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
