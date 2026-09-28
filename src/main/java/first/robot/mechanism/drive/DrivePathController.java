package first.robot.mechanism.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import choreo.trajectory.SwerveSample;

public class DrivePathController {
  private final Supplier<Pose2d> m_poseSupplier;
  private final Consumer<ChassisVelocities> m_velocitiesConsumer;

  private final PIDController m_pathXController = new PIDController(1.0, 0.0, 0.0);
  private final PIDController m_pathYController = new PIDController(1.0, 0.0, 0.0);
  private final PIDController m_pathThetaController = new PIDController(1.0, 0.0, 0.0);

  public DrivePathController(Supplier<Pose2d> poseSupplier, Consumer<ChassisVelocities> velocitiesConsumer) {
    m_poseSupplier = poseSupplier;
    m_velocitiesConsumer = velocitiesConsumer;
  }

  public void followSample(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = m_poseSupplier.get();
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
    m_velocitiesConsumer.accept(robotRelativeSpeeds);
  }
}
