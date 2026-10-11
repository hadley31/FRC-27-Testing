package first.robot.command;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Pins the gate that decides whether a chassis-aimed shot may feed.
 *
 * <p>Worth its own tests because the failure it exists to prevent is invisible until a match: the
 * turret reports "on target" the instant it reaches its parked angle, so without this the robot
 * feeds fuel while the chassis is still swinging and the shots simply miss.
 */
class ChassisAimGoalTest {
  private static final Angle kTolerance = Degrees.of(2.0);
  private static final AngularVelocity kSettledRate = DegreesPerSecond.of(20);
  private static final AngularVelocity kStill = DegreesPerSecond.zero();

  private static ChassisAimGoal aimingAt(double headingDegrees) {
    var goal = new ChassisAimGoal();
    goal.set(Rotation2d.fromDegrees(headingDegrees), true);
    return goal;
  }

  private static boolean isAimed(ChassisAimGoal goal, double currentDeg, AngularVelocity rate) {
    return goal.isAimed(Rotation2d.fromDegrees(currentDeg), rate, kTolerance, kSettledRate);
  }

  @Test
  void isAimedWhenOnTargetAndSettled() {
    assertTrue(isAimed(aimingAt(30), 30, kStill));
  }

  @Test
  void isAimedWithinTolerance() {
    assertTrue(isAimed(aimingAt(30), 31, kStill));
    assertTrue(isAimed(aimingAt(30), 29, kStill));
  }

  @Test
  void isNotAimedOutsideTolerance() {
    assertFalse(isAimed(aimingAt(30), 35, kStill));
    assertFalse(isAimed(aimingAt(30), 25, kStill));
  }

  @Test
  void isNotAimedWhileTheChassisIsStillTurningFast() {
    // Dead on heading, but swinging through it. This is the mid-swing shot the gate exists to
    // refuse: the shot solution's velocity lead is large and chasing the swing that caused it.
    assertFalse(isAimed(aimingAt(30), 30, DegreesPerSecond.of(90)));
  }

  @Test
  void isNotAimedWhileTurningFastInEitherDirection() {
    assertFalse(isAimed(aimingAt(30), 30, DegreesPerSecond.of(-90)));
  }

  @Test
  void isAimedOnceRotationHasSettled() {
    assertTrue(isAimed(aimingAt(30), 30, DegreesPerSecond.of(5)));
  }

  @Test
  void toleranceIsMeasuredAcrossTheWrap() {
    // Target just clockwise of due south, robot just counter-clockwise of it: one degree apart, not
    // 359. A gate comparing raw degrees would refuse this shot forever.
    assertTrue(isAimed(aimingAt(179.5), -179.5, kStill));
  }

  @Test
  void abstainsWhenTheChassisIsNotTheThingAiming() {
    var goal = new ChassisAimGoal();
    goal.set(Rotation2d.fromDegrees(30), false);

    // Pointed 90 degrees away and spinning hard, yet aimed: the turret is doing the aiming, so this
    // gate has no business holding up the shot.
    assertTrue(isAimed(goal, 120, DegreesPerSecond.of(200)));
  }

  @Test
  void abstainsBeforeAnythingHasBeenSet() {
    assertTrue(isAimed(new ChassisAimGoal(), 90, DegreesPerSecond.of(200)));
  }
}
