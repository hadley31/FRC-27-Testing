package first.robot.util;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.lib.InterpolatingMeasureTreeMap;

/**
 * Solves for the turret angle, hood angle and flywheel speed that put fuel in a target, accounting
 * for the fact that the robot is usually moving when it shoots.
 *
 * <p>Fuel leaves the turret carrying the turret's field-relative velocity, so over the flight it
 * drifts by {@code velocity * timeOfFlight}. Aiming at a virtual target displaced by the negative of
 * that drift cancels it, and all three lookups are keyed on the range to that virtual target so the
 * hood and flywheel are set up for the range actually being shot.
 *
 * <p>Stateless by design: every input arrives as an argument, so the solution is reproducible from a
 * log and testable without a robot. Latency compensation is <em>not</em> done here — the caller
 * decides which instant the {@link TurretSnapshot} describes.
 */
public final class ShotCalculationUtil {
  private static final Time kTimeOfFlightTolerance = Seconds.of(0.05);
  private static final int kMaximumIterations = 100;

  private ShotCalculationUtil() {
  }

  /**
   * @param turret where the turret is, how it is oriented and how fast it is moving
   * @param target the field-relative point to put fuel into, alliance-flipped by the caller
   * @param profile the tuning tables to solve against
   */
  public static ShotParameters calculateShot(
      TurretSnapshot turret, Translation3d target, ShotProfile profile) {
    Translation2d turretGround = turret.position().toTranslation2d();

    Optional<Translation2d> compensatedTarget = getVelocityCompensatedTarget(
        turretGround, target.toTranslation2d(), turret.fieldRelativeVelocity(),
        profile.timeOfFlight());

    // If the fixed point did not converge, fall back to aiming straight at the target. A stationary
    // solution is wrong while moving, but it is a far better failure mode than a wild aim point.
    Translation2d aimPoint = compensatedTarget.orElseGet(target::toTranslation2d);

    // Carry the real target's height onto the virtual aim point so the lookups still see the slant
    // range rather than a ground-plane distance.
    Translation3d virtualTarget = new Translation3d(aimPoint.getX(), aimPoint.getY(), target.getZ());
    Distance range = Meters.of(turret.position().getDistance(virtualTarget));

    Logger.recordOutput("ShotCalculation/TurretPosition", turret.position());
    Logger.recordOutput("ShotCalculation/Target", target);
    Logger.recordOutput("ShotCalculation/VirtualTarget", virtualTarget);
    Logger.recordOutput("ShotCalculation/Converged", compensatedTarget.isPresent());
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
