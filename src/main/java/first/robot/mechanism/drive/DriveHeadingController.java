package first.robot.mechanism.drive;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Turns a heading the chassis should be holding into the rotation rate that gets it there.
 *
 * <p>Extracted so it can be tested, in the same spirit as {@link DriveForceFeedforward}, and sited
 * beside {@link DrivePathController} because it is the same kind of thing: a collaborator of the
 * drivetrain rather than a part of it, owning its own controller state and knowing nothing about
 * commands. {@link Drive} deliberately stops at "make the robot move at this velocity", so deciding
 * what rate to ask for belongs out here.
 *
 * <p>It knows nothing about <em>why</em> a heading is wanted. Pointing at the hub, holding a bearing
 * for a driver, facing a feeding station -- those are strategy, and they reach this class as a
 * {@link Rotation2d} and nothing more. That is the seam that lets both the driver's aim-assist and
 * the robot's own auto-aim share one implementation.
 *
 * <p>Two things here are easy to get wrong and hard to notice, hence
 * {@code DriveHeadingControllerTest}. The error has to wrap, or a robot facing just clockwise of a
 * target behind it unwinds the long way round -- which looks like a slow robot, not a bug. And the
 * output has to saturate rather than scale, because the gain is tuned for how fast to correct a
 * small error; past the error where it saturates, the right behaviour is to turn at the fastest rate
 * the chassis has, not at whatever larger number the gain asked for.
 */
public class DriveHeadingController {
  /**
   * Proportional gain, in radians per second of chassis rotation per radian of heading error.
   *
   * <p>{@link DrivePathController} uses 7.0 on the same axis. Whether the two should be the same
   * number is a tuning question and not this class's business: a path follower corrects heading
   * error alongside a feedforward that is already producing most of the rotation, so it can afford
   * a stiffer gain than a controller that is the sole source of rotation.
   */
  public static final double kDefaultHeadingP = 5.0;

  private final PIDController m_controller;
  private final double m_maxRadiansPerSecond;
  private final String m_logKey;

  /**
   * Whether {@link #calculate} has ever run.
   *
   * <p>Tracked because a fresh {@link PIDController} reports zero error, which is indistinguishable
   * from being exactly on target. {@link #isNearTarget} has to answer "no" for a controller that has
   * not run, or a caller gating on it would be told the chassis is aimed before anything had aimed
   * it.
   */
  private boolean m_hasCalculated = false;

  /**
   * @param maxAngularVelocity the rate the output saturates at
   * @param logKey             what this controller logs under, so that two of them -- a driver's and
   *                           the robot's own -- do not overwrite each other's telemetry
   */
  public DriveHeadingController(AngularVelocity maxAngularVelocity, String logKey) {
    this(kDefaultHeadingP, maxAngularVelocity, logKey);
  }

  /**
   * @param maxAngularVelocity taken as a parameter rather than read from {@code Constants}, which is
   *                           what keeps this class unit-testable and free of any one robot's
   *                           configuration -- the same reason {@code ShotCalculationUtil} is handed
   *                           its toggle instead of reaching for {@code Tuning}
   */
  public DriveHeadingController(
      double headingP, AngularVelocity maxAngularVelocity, String logKey) {
    m_controller = new PIDController(headingP, 0.0, 0.0);
    m_maxRadiansPerSecond = maxAngularVelocity.in(RadiansPerSecond);
    m_logKey = logKey;

    // Once, here -- not per call. This is what makes the error wrap, and it is a property of the
    // controller rather than of any one calculation.
    m_controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * The rotation rate that drives {@code current} toward {@code target} by the shorter way round,
   * saturated at the maximum this controller was built with.
   *
   * <p>Proportional only, deliberately. This object outlives any one run of the command using it --
   * a supplier closes over it and a mode factory holds that supplier for the match -- so an integral
   * or derivative term would carry error from the previous run into the first cycle of the next. With
   * P alone the output depends on nothing but the present error, so a command re-entered after a
   * minute of being elsewhere behaves exactly like one freshly scheduled.
   *
   * <p>If I or D is ever wanted, add a {@code reset()} here and build the command with
   * {@code Drive.run(...)} rather than {@code Drive.driveCommand(...)} so the body can call it at the
   * top: a re-scheduled command gets a fresh continuation, so its body does run again from the
   * start. {@code outputDependsOnlyOnThePresentError} is the test that will fail and say so.
   */
  public AngularVelocity calculate(Rotation2d current, Rotation2d target) {
    double omega = m_controller.calculate(current.getRadians(), target.getRadians());
    double clamped = Math.clamp(omega, -m_maxRadiansPerSecond, m_maxRadiansPerSecond);
    m_hasCalculated = true;

    Logger.recordOutput(m_logKey + "/TargetHeading", target);
    Logger.recordOutput(m_logKey + "/HeadingError", getError());
    Logger.recordOutput(m_logKey + "/Saturated", omega != clamped);
    Logger.recordOutput(m_logKey + "/Omega", RadiansPerSecond.of(clamped));

    return RadiansPerSecond.of(clamped);
  }

  /**
   * The wrapped heading error the last {@link #calculate} acted on, which is zero before the first
   * one. Reports the last calculation rather than the present state, so it is only meaningful to a
   * caller that is driving this controller every cycle.
   */
  public Angle getError() {
    return Radians.of(m_controller.getError());
  }

  /** Whether the last {@link #calculate} was within {@code tolerance}; false before the first. */
  public boolean isNearTarget(Angle tolerance) {
    return m_hasCalculated && Math.abs(m_controller.getError()) <= tolerance.in(Radians);
  }
}
