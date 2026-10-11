package first.robot.command;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * The chassis heading an auto-aim run wants, and whether it wants one at all.
 *
 * <p>Written by the auto-aim command's body each cycle, and read by the chassis-aim command it forks
 * and by the gate deciding whether to feed. A holder rather than a field on
 * {@link RobotCommandFactory}, so two auto-aim commands built from one factory cannot tread on each
 * other's goal, and so {@link #isAimed} can be tested without standing up a scheduler.
 *
 * <p>Not thread-safe and does not need to be: the scheduler is single-threaded, and everything that
 * touches one of these runs inside it.
 */
class ChassisAimGoal {
  private Rotation2d m_heading = Rotation2d.ZERO;
  private boolean m_chassisAiming = false;

  /**
   * @param chassisAiming whether the chassis is the thing doing the aiming this cycle. False leaves
   *                      {@code heading} recorded but makes {@link #isAimed} abstain.
   */
  void set(Rotation2d heading, boolean chassisAiming) {
    m_heading = heading;
    m_chassisAiming = chassisAiming;
  }

  Rotation2d heading() {
    return m_heading;
  }

  /**
   * Whether the aim is good enough to shoot through.
   *
   * <p>Measured against the live pose rather than read out of the heading controller, so this
   * answers a question about where the robot is pointed rather than about whether some controller
   * happened to run this cycle.
   *
   * <p>The rotation-rate term is not redundant with the tolerance. A chassis swinging through its
   * target passes within tolerance for a cycle or two on the way past, and in fixed-turret mode that
   * is the worst moment to shoot: the solution leads for the turret pivot's velocity, the pivot rides
   * a lever arm off the chassis centre, so a fast swing produces a large lead that is chasing its own
   * cause. The aim is only worth believing once the rotation has settled.
   *
   * <p>Abstains -- reports true -- whenever the chassis is not aiming, so a turret-aimed shot is
   * gated by exactly what it was before this existed.
   */
  boolean isAimed(
      Rotation2d currentHeading,
      AngularVelocity rotationRate,
      Angle tolerance,
      AngularVelocity settledRate) {
    if (!m_chassisAiming) {
      return true;
    }

    return m_heading.minus(currentHeading).getMeasure().abs(Degrees) <= tolerance.in(Degrees)
        && rotationRate.abs(DegreesPerSecond) <= settledRate.in(DegreesPerSecond);
  }
}
