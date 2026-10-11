package first.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

import first.robot.mechanism.drive.DriveHeadingController;

/**
 * Turns what the driver is asking for into the robot-relative velocities the drivetrain takes.
 *
 * <p>Each {@code get...Supplier} is a different way of deciding the chassis's rotation while the
 * translation stays the driver's: the rotate stick, or a heading held by a controller. They hand
 * back a {@link Supplier} rather than a value, because that is what {@code Drive.driveCommand}
 * wants -- one supplier evaluated afresh every cycle for as long as the command runs.
 *
 * <p>What this class does <em>not</em> do is decide which heading is worth holding. Pointing at the
 * hub, facing a feeding station, aiming a turret bore -- those need the field, the shot solution and
 * the robot's own state, none of which belong to a stick mapper. A heading arrives here already
 * chosen, as a {@link Rotation2d}, and the {@link DriveHeadingController} that turns it into a
 * rotation rate is injected rather than built: this class depends on the drive layer, and nothing in
 * the drive layer depends on it.
 */
public class SwerveInputStream {
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private final DriveHeadingController m_headingController;
  private final DoubleSupplier m_forwardSupplier;
  private final DoubleSupplier m_leftSupplier;
  private final DoubleSupplier m_rotateSupplier;

  private SwerveInputStream(
      Supplier<Pose2d> robotPoseSupplier,
      DriveHeadingController headingController,
      DoubleSupplier forwardSupplier,
      DoubleSupplier leftSupplier,
      DoubleSupplier rotateSupplier) {
    m_robotPoseSupplier = robotPoseSupplier;
    m_headingController = headingController;
    m_forwardSupplier = forwardSupplier;
    m_leftSupplier = leftSupplier;
    m_rotateSupplier = rotateSupplier;
  }

  public static SwerveInputStream of(
      Supplier<Pose2d> robotPoseSupplier,
      DriveHeadingController headingController,
      DoubleSupplier forwardSupplier,
      DoubleSupplier leftSupplier,
      DoubleSupplier rotateSupplier) {
    return new SwerveInputStream(
        robotPoseSupplier, headingController, forwardSupplier, leftSupplier, rotateSupplier);
  }

  /**
   * A stream for a context with no driver in it: the sticks read zero, so the only velocity it can
   * produce is whatever a heading controller asks for.
   *
   * <p>Still a driver-controls concern -- it describes the absence of one. This is the stream behind
   * the commands that aim the chassis on the robot's own account, in auto or from a button, where
   * there is no stick input to blend with and translation should stay put.
   */
  public static SwerveInputStream of(
      Supplier<Pose2d> robotPoseSupplier, DriveHeadingController headingController) {
    return new SwerveInputStream(
        robotPoseSupplier, headingController, () -> 0.0, () -> 0.0, () -> 0.0);
  }

  public static double applyControllerStickMapping(double value, double deadband) {
    return MathUtil.applyDeadband(-value * Math.abs(value), deadband);
  }

  /** Translation and rotation both straight from the sticks. */
  public Supplier<ChassisVelocities> getNormalDriveSupplier() {
    return () -> toRobotRelative(getRotateVelocitySupplier().get(), m_robotPoseSupplier.get());
  }

  /**
   * Driver translation, with rotation taken over by the heading controller holding whatever
   * {@code targetAngleSupplier} reports.
   *
   * <p>Re-read every cycle rather than captured, so the heading may be one that moves -- tracking a
   * point the robot is driving past, following a shot solution -- and not only a fixed bearing.
   */
  public Supplier<ChassisVelocities> getHeadingLockedDriveSupplier(
      Supplier<Rotation2d> targetAngleSupplier) {
    return () -> {
      Pose2d robotPose = m_robotPoseSupplier.get();
      AngularVelocity omega =
          m_headingController.calculate(robotPose.getRotation(), targetAngleSupplier.get());

      return toRobotRelative(omega, robotPose);
    };
  }

  /**
   * Whether the heading lock has settled to within {@code tolerance}, as of the last cycle that
   * actually drove it. Reports the controller's last calculation, so it is stale unless a
   * heading-locked command is running, and false before any has run.
   */
  public boolean isAtTargetAngle(Angle tolerance) {
    return m_headingController.isNearTarget(tolerance);
  }

  /**
   * The driver's translation with the given rotation rate, turned into the robot's own frame.
   *
   * <p>The sticks are field-relative -- pushing away from the driver station drives away from it
   * whichever way the robot happens to be facing -- and {@code Drive.drive} takes robot-relative
   * velocities, so the robot's heading is rotated out here. The rotation rate is the same number in
   * either frame and passes through untouched.
   *
   * @param robotPose passed in rather than fetched, so the translation is rotated by the same
   *                  heading the rotation rate was computed from rather than one read a moment later
   */
  private ChassisVelocities toRobotRelative(AngularVelocity omega, Pose2d robotPose) {
    return new ChassisVelocities(
        getForwardVelocitySupplier().get(),
        getLeftVelocitySupplier().get(),
        omega).toRobotRelative(robotPose.getRotation());
  }

  private Supplier<LinearVelocity> getForwardVelocitySupplier() {
    return () -> Constants.DriveConstants.kMaxLinearVelocity.times(m_forwardSupplier.getAsDouble());
  }

  private Supplier<LinearVelocity> getLeftVelocitySupplier() {
    return () -> Constants.DriveConstants.kMaxLinearVelocity.times(m_leftSupplier.getAsDouble());
  }

  private Supplier<AngularVelocity> getRotateVelocitySupplier() {
    return () -> Constants.DriveConstants.kMaxAngularVelocity.times(m_rotateSupplier.getAsDouble());
  }
}
