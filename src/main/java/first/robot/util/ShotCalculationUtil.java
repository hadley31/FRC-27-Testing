package first.robot.util;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.lib.util.InterpolatingMeasureTreeMap;

/**
 * Solves for the turret angle, hood angle and flywheel speed that put fuel in a target, accounting
 * for the fact that the robot is usually moving when it shoots.
 *
 * <p>Fuel leaves the turret carrying the turret's field-relative velocity, so over the flight it
 * drifts by {@code velocity * timeOfFlight}. Aiming at a virtual target displaced by the negative of
 * that drift cancels it, and all three lookups are keyed on the range to that virtual target so the
 * hood and flywheel are set up for the range actually being shot.
 *
 * <p>All robot state arrives per call, so a solution is reproducible from a log and testable without
 * a robot. The one thing held as instance state is whether velocity compensation is enabled: binding
 * it once here means no call site can forget to apply it, and injecting it as a supplier keeps the
 * disabled path reachable from a test — reading the dashboard toggle directly would not, since
 * AdvantageKit only refreshes its inputs inside a real robot loop.
 *
 * <p>Latency compensation is <em>not</em> done here — the caller decides which instant the
 * {@link TurretSnapshot} describes.
 */
public final class ShotCalculationUtil {
  private static final Time kTimeOfFlightTolerance = Seconds.of(0.05);
  private static final int kMaximumIterations = 100;

  private final BooleanSupplier m_velocityCompensationEnabled;

  /**
   * @param velocityCompensationEnabled whether to lead the target to cancel the velocity fuel
   *     inherits from a moving turret. On the robot this is {@code Tuning.kVelocityCompensation};
   *     tests pass a literal.
   */
  public ShotCalculationUtil(BooleanSupplier velocityCompensationEnabled) {
    m_velocityCompensationEnabled = velocityCompensationEnabled;
  }

  /**
   * @param turret where the turret is, how it is oriented and how fast it is moving
   * @param target the field-relative point to put fuel into, alliance-flipped by the caller
   * @param profile the tuning tables to solve against
   */
  public ShotParameters calculateShot(
      TurretSnapshot turret, Translation3d target, ShotProfile profile) {
    Translation2d turretGround = turret.position().toTranslation2d();

    boolean compensate = m_velocityCompensationEnabled.getAsBoolean();
    Optional<Translation2d> solvedAimPoint = compensate
        ? getVelocityCompensatedTarget(
            turretGround, target.toTranslation2d(), turret.fieldRelativeVelocity(),
            profile.timeOfFlight())
        : Optional.empty();

    // With compensation off, or if the fixed point did not converge, aim straight at the target. A
    // stationary solution is wrong while moving, but a far better failure mode than a wild aim point.
    Translation2d aimPoint = solvedAimPoint.orElseGet(target::toTranslation2d);

    // Carry the real target's height onto the virtual aim point so the lookups still see the slant
    // range rather than a ground-plane distance.
    Translation3d virtualTarget = new Translation3d(aimPoint.getX(), aimPoint.getY(), target.getZ());
    Distance range = Meters.of(turret.position().getDistance(virtualTarget));

    Logger.recordOutput("ShotCalculation/TurretPosition", turret.position());
    Logger.recordOutput("ShotCalculation/Target", target);
    Logger.recordOutput("ShotCalculation/VirtualTarget", virtualTarget);
    Logger.recordOutput("ShotCalculation/VelocityCompensationEnabled", compensate);
    Logger.recordOutput("ShotCalculation/VelocityCompensationConverged", solvedAimPoint.isPresent());
    Logger.recordOutput("ShotCalculation/Range", range);

    return new ShotParameters(
        calculateTargetTurretAngle(turret.position(), virtualTarget, turret.robotRotation()),
        profile.hoodAngle().get(range),
        profile.flywheelVelocity().get(range));
  }

  /**
   * Solves for the virtual aim point that cancels the velocity fuel inherits from the turret.
   *
   * <p>Displacing the aim point changes the range, which changes the time of flight, which changes
   * the displacement — so this iterates to a fixed point rather than correcting once. All
   * translations are field-relative and in the ground plane.
   *
   * @return the virtual aim point, or empty if it did not converge within
   *     {@link #kMaximumIterations}
   */
  private static Optional<Translation2d> getVelocityCompensatedTarget(
      Translation2d turret,
      Translation2d target,
      ChassisVelocities turretVelocity,
      InterpolatingMeasureTreeMap<Distance, Time> timeOfFlightMap) {
    // Seed the iteration with the time of flight for the uncompensated shot.
    Time timeOfFlight = timeOfFlightMap.get(Meters.of(turret.getDistance(target)));

    for (int iteration = 1; iteration <= kMaximumIterations; iteration++) {
      Translation2d adjustedTarget = target.minus(
          new Translation2d(
              turretVelocity.vx * timeOfFlight.in(Seconds),
              turretVelocity.vy * timeOfFlight.in(Seconds)));

      Time refined = timeOfFlightMap.get(Meters.of(turret.getDistance(adjustedTarget)));

      if (refined.isNear(timeOfFlight, kTimeOfFlightTolerance)) {
        Logger.recordOutput("ShotCalculation/ConvergedIterations", iteration);
        return Optional.of(adjustedTarget);
      }

      timeOfFlight = refined;
    }

    return Optional.empty();
  }

  /**
   * The chassis heading that puts the turret bore on a solved shot's aim point, with the turret
   * parked at {@code fixedTurretAngle}.
   *
   * <p>The inverse of {@link #calculateTargetTurretAngle}, and here beside it because the two share
   * one convention: a turret setpoint is measured in the chassis frame, so adding the chassis heading
   * back recovers the field bearing the solve actually chose. Pointing a bore that sits
   * {@code fixedTurretAngle} off the chassis along that bearing means turning the chassis to the
   * difference.
   *
   * <p>Aiming this way rather than computing a fresh bearing to the target is what buys the chassis
   * every correction the turret already gets: the aim point is the velocity-compensated virtual
   * target, and the bearing was taken from the turret pivot rather than the robot's centre. A chassis
   * aim that recomputed its own bearing would be a second, worse answer to a question
   * {@link #calculateShot} has already answered.
   *
   * <p>One caveat: this is a fixed point rather than a closed-form solution, because rotating the
   * chassis sweeps the pivot, which moves the bearing. The perturbation is bounded by
   * {@code |r| / R} -- the mount offset over the range, about 0.07 at 3 m with
   * {@code kRobotToTurret} -- so a heading controller chasing this converges in a few cycles. It only
   * approaches unity when {@code R} nears the mount offset itself, which is a robot parked on top of
   * the thing it is shooting at.
   *
   * @param turret            the snapshot the shot was solved from, for the chassis heading it was
   *                          solved against. Passing a different one than {@code shot} came from
   *                          mixes two instants and silently biases the heading.
   * @param shot              that solve's parameters
   * @param fixedTurretAngle  where the turret is held while the chassis aims. Zero puts the bore down
   *                          the chassis +X axis, making the answer the bore bearing itself.
   */
  public static Rotation2d chassisHeadingForTurretBore(
      TurretSnapshot turret, ShotParameters shot, Angle fixedTurretAngle) {
    return turret.robotRotation()
        .plus(new Rotation2d(shot.targetTurretAngle()))
        .minus(new Rotation2d(fixedTurretAngle));
  }

  /**
   * Converts a field-relative turret-to-target bearing into the robot-relative angle the turret
   * mechanism is commanded with.
   */
  private static Angle calculateTargetTurretAngle(
      Translation3d turret, Translation3d target, Rotation2d robotRotation) {
    return target.minus(turret).toTranslation2d()
        .getAngle()
        // Degenerate only if the turret is exactly under the target, where any bearing is as good as
        // another. Holding the current heading beats throwing out of the command loop.
        .orElse(robotRotation)
        .minus(robotRotation)
        .getMeasure();
  }
}
