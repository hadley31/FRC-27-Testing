package first.robot.mechanism.drive;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import choreo.trajectory.SwerveSample;

public class DrivePathController {
  private final Supplier<Pose2d> m_poseSupplier;
  private final BiConsumer<ChassisVelocities, Translation2d[]> m_velocitiesConsumer;

  private final PIDController m_pathXController = new PIDController(7.0, 0.0, 0.0);
  private final PIDController m_pathYController = new PIDController(7.0, 0.0, 0.0);
  private final PIDController m_pathThetaController = new PIDController(7.0, 0.0, 0.0);

  public DrivePathController(
      Supplier<Pose2d> poseSupplier,
      BiConsumer<ChassisVelocities, Translation2d[]> velocitiesConsumer) {
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

    Logger.recordOutput("RobotState/Odometry/Trajectory/TargetPose", targetPose);
    Logger.recordOutput("RobotState/Odometry/Trajectory/Error/X", targetPose.getMeasureX().minus(pose.getMeasureX()));
    Logger.recordOutput("RobotState/Odometry/Trajectory/Error/Y", targetPose.getMeasureY().minus(pose.getMeasureY()));
    Logger.recordOutput("RobotState/Odometry/Trajectory/Error/Rotation",
        targetPose.getRotation().minus(pose.getRotation()).getMeasure());

    // Convert field-relative speeds to robot-relative and run
    ChassisVelocities robotRelativeSpeeds = new ChassisVelocities(
        targetSpeeds.vx,
        targetSpeeds.vy,
        targetSpeeds.omega).toRobotRelative(pose.getRotation());

    m_velocitiesConsumer.accept(robotRelativeSpeeds, moduleForces(sample, pose.getRotation()));
  }

  /**
   * The force the trajectory expects each module to be exerting, turned robot-relative.
   *
   * <p>Choreo solves the whole trajectory against the robot's mass and its motors, so alongside
   * where to be it knows what force each corner has to produce to get there. Those come out
   * field-relative, like every other vector on a sample, and the modules reason in their own frame,
   * so the robot's heading is rotated out here rather than four times further down.
   *
   * <p>A sample carrying no forces reports four zeroes rather than nothing, so an older trajectory
   * or one generated without them simply feeds forward nothing and needs no special case.
   */
  private static Translation2d[] moduleForces(SwerveSample sample, Rotation2d robotRotation) {
    double[] forcesX = sample.moduleForcesX();
    double[] forcesY = sample.moduleForcesY();
    Translation2d[] forces = new Translation2d[Math.min(forcesX.length, forcesY.length)];

    for (int i = 0; i < forces.length; i++) {
      forces[i] = new Translation2d(forcesX[i], forcesY[i]).rotateBy(robotRotation.unaryMinus());
    }

    Logger.recordOutput("RobotState/Odometry/Trajectory/ModuleForces", forces);

    return forces;
  }
}
