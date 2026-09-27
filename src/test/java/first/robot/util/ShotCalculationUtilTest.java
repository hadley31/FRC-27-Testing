package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

import first.lib.InterpolatingMeasureTreeMap;

/**
 * Exercises the shoot-on-the-move solution in {@link ShotCalculationUtil}.
 *
 * <p>The fixtures put the target straight down the +X axis from the turret so the expected bearings
 * are easy to state by hand: a stationary shot aims at 0 degrees, and a shot taken while strafing in
 * +Y must lead against that motion, i.e. turn negative.
 *
 * <p>Ranges are kept inside the interior of {@link ShotProfile#kScoring}'s tables. Their spans are
 * roughly 4-25 ft (flywheel), 4-16 ft (hood) and 6.7-20 ft (time of flight), so the usable overlap is
 * about 2.05 m to 4.85 m. Outside it the interpolator clamps to an endpoint and every comparison
 * below would collapse to an equality.
 */
class ShotCalculationUtilTest {
  private static final double kEpsilonDegrees = 1e-6;

  /** Turret one meter off the floor at the field origin. */
  private static final Translation3d kTurretPosition = new Translation3d(0.0, 0.0, 1.0);

  /** 4 m downrange at turret height: ~13.1 ft, comfortably inside every table. */
  private static final Translation3d kTarget = new Translation3d(4.0, 0.0, 1.0);

  private static ShotParameters shotWith(
      Translation3d turretPosition, Rotation2d robotRotation, ChassisVelocities velocity) {
    return ShotCalculationUtil.calculateShot(
        new TurretSnapshot(turretPosition, robotRotation, velocity), kTarget, ShotProfile.kScoring);
  }

  private static ShotParameters movingAt(ChassisVelocities velocity) {
    return shotWith(kTurretPosition, Rotation2d.ZERO, velocity);
  }

  private static ShotParameters stationary() {
    return movingAt(new ChassisVelocities());
  }

  @Test
  void stationaryTurretAimsDirectlyAtTheTarget() {
    assertEquals(0.0, stationary().targetTurretAngle().in(Degrees), kEpsilonDegrees);
  }

  @Test
  void turretAngleIsRobotRelativeNotFieldRelative() {
    // Chassis yawed 90 degrees CCW. The target is still due +X in the field frame, so the turret has
    // to swing 90 degrees CW to point at it.
    var parameters = shotWith(kTurretPosition, Rotation2d.CCW_90DEG, new ChassisVelocities());

    assertEquals(-90.0, parameters.targetTurretAngle().in(Degrees), kEpsilonDegrees);
  }

  @Test
  void rangeIsMeasuredFromTheTurretPositionItIsGiven() {
    // A turret 1.5 m downrange is closer, so it must call for a slower flywheel. If the position
    // were ignored, these would be identical.
    var near = shotWith(
        new Translation3d(1.5, 0.0, 1.0), Rotation2d.ZERO, new ChassisVelocities());

    assertTrue(
        near.targetFlywheelSpeed().lt(stationary().targetFlywheelSpeed()),
        "closer turret should call for a slower flywheel");
  }

  @Test
  void rangeIsTheSlantRangeNotTheGroundRange() {
    // Turret 2 m above a target on the floor: the slant range must exceed the 4 m ground range, so
    // the flywheel must run faster than for a level shot over the same ground.
    var elevated = ShotCalculationUtil.calculateShot(
        new TurretSnapshot(new Translation3d(0.0, 0.0, 2.0), Rotation2d.ZERO, new ChassisVelocities()),
        new Translation3d(4.0, 0.0, 0.0),
        ShotProfile.kScoring);

    assertTrue(
        elevated.targetFlywheelSpeed().gt(stationary().targetFlywheelSpeed()),
        "slant range should exceed ground range and demand a faster flywheel");
  }

  @Test
  void strafingLeadsTheShotAgainstTheDirectionOfTravel() {
    // Strafing in +Y gives the fuel a +Y velocity, so the aim point must shift to -Y to cancel it.
    var parameters = movingAt(new ChassisVelocities(0.0, 2.0, 0.0));

    assertTrue(
        parameters.targetTurretAngle().in(Degrees) < -1.0,
        "expected a negative lead angle, got " + parameters.targetTurretAngle().in(Degrees));
  }

  @Test
  void strafingLeadMatchesVelocityTimesTimeOfFlight() {
    double speed = 2.0;
    double leadDegrees = movingAt(new ChassisVelocities(0.0, speed, 0.0))
        .targetTurretAngle().in(Degrees);

    // The lead angle is atan2(-speed * tof, range) about a 4 m range. Bracket it with the extremes
    // of the time-of-flight table rather than pinning one interpolated value.
    double lowerBound = Math.toDegrees(Math.atan2(-speed * 1.6, 4.0));
    double upperBound = Math.toDegrees(Math.atan2(-speed * 0.9, 4.0));

    assertTrue(
        leadDegrees > lowerBound && leadDegrees < upperBound,
        "lead " + leadDegrees + " outside the plausible band [" + lowerBound + ", " + upperBound + "]");
  }

  @Test
  void drivingDownrangeShortensTheShotSoTheFlywheelSlows() {
    // Closing on the target means fuel is thrown long unless the range used for the lookups shrinks.
    var closing = movingAt(new ChassisVelocities(2.0, 0.0, 0.0));

    assertTrue(
        closing.targetFlywheelSpeed().lt(stationary().targetFlywheelSpeed()),
        "closing on the target should call for a slower flywheel");
    // Purely radial motion needs no lead, so the bearing should be unchanged.
    assertEquals(
        stationary().targetTurretAngle().in(Degrees),
        closing.targetTurretAngle().in(Degrees),
        kEpsilonDegrees);
  }

  @Test
  void hoodAndFlywheelTrackTheVirtualTargetNotTheRealOne() {
    // Cross-range velocity stretches the slant range to the virtual target, so the hood and flywheel
    // must respond. If the lookups still keyed on the real target, these would be identical.
    var moving = movingAt(new ChassisVelocities(0.0, 2.0, 0.0));

    assertTrue(
        moving.targetFlywheelSpeed().gt(stationary().targetFlywheelSpeed()),
        "longer virtual range should call for a faster flywheel");
    assertTrue(
        moving.targetHoodAngle().gte(stationary().targetHoodAngle()),
        "longer virtual range should not call for a flatter hood");
  }

  @Test
  void compensationIsSymmetricAboutTheDirectionOfTravel() {
    var left = movingAt(new ChassisVelocities(0.0, 2.0, 0.0));
    var right = movingAt(new ChassisVelocities(0.0, -2.0, 0.0));

    assertEquals(
        left.targetTurretAngle().in(Degrees), -right.targetTurretAngle().in(Degrees), 1e-9);
    assertEquals(
        left.targetFlywheelSpeed().baseUnitMagnitude(),
        right.targetFlywheelSpeed().baseUnitMagnitude(),
        1e-9);
  }

  @Test
  void chassisRotationAloneDoesNotLeadTheShot() {
    // The lever-arm contribution is the caller's job to fold into the supplied velocity, so a
    // snapshot carrying omega alone has no linear drift to cancel.
    var spinning = movingAt(new ChassisVelocities(0.0, 0.0, 2.0));

    assertEquals(
        stationary().targetTurretAngle().in(Degrees),
        spinning.targetTurretAngle().in(Degrees),
        kEpsilonDegrees);
  }

  @Test
  void solutionFollowsTheProfileItIsGiven() {
    // A profile whose fuel flies twice as slowly must lead twice as far, without the solver changing.
    var slowProfile = new ShotProfile(
        Seconds.of(0.03),
        InterpolatingMeasureTreeMap.distanceToTime()
            .put(Feet.of(0), Seconds.of(2.8))
            .put(Feet.of(100), Seconds.of(2.8)),
        ShotProfile.kScoring.hoodAngle(),
        ShotProfile.kScoring.flywheelVelocity());

    var fastProfile = new ShotProfile(
        Seconds.of(0.03),
        InterpolatingMeasureTreeMap.distanceToTime()
            .put(Feet.of(0), Seconds.of(1.4))
            .put(Feet.of(100), Seconds.of(1.4)),
        ShotProfile.kScoring.hoodAngle(),
        ShotProfile.kScoring.flywheelVelocity());

    var snapshot = new TurretSnapshot(
        kTurretPosition, Rotation2d.ZERO, new ChassisVelocities(0.0, 2.0, 0.0));

    double slowLead = ShotCalculationUtil.calculateShot(snapshot, kTarget, slowProfile)
        .targetTurretAngle().in(Degrees);
    double fastLead = ShotCalculationUtil.calculateShot(snapshot, kTarget, fastProfile)
        .targetTurretAngle().in(Degrees);

    // atan2 is not linear, so assert ordering and the exact geometry rather than a ratio.
    assertTrue(slowLead < fastLead, "a slower shot should lead further");
    assertEquals(Math.toDegrees(Math.atan2(-2.0 * 2.8, 4.0)), slowLead, 1e-9);
    assertEquals(Math.toDegrees(Math.atan2(-2.0 * 1.4, 4.0)), fastLead, 1e-9);
  }

  @Test
  void flywheelTableIsMonotonicOverTheBandTheFixturesUse() {
    // Sanity check that the flywheel table rises with range over the band these tests exercise, so
    // the comparisons above are testing the compensation rather than a kink in the table.
    var previous = RPM.zero();

    for (double x = 2.0; x >= -0.5; x -= 0.5) {
      var current = shotWith(new Translation3d(x, 0.0, 1.0), Rotation2d.ZERO, new ChassisVelocities())
          .targetFlywheelSpeed();
      assertTrue(current.gte(previous), "flywheel speed should not drop as range grows (x=" + x + ")");
      previous = current;
    }
  }
}
