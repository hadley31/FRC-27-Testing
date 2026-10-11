package first.robot.mechanism.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Pins the two properties of {@link DriveHeadingController} that are easy to break and hard to
 * notice: that the error wraps, and that the output saturates rather than scaling.
 *
 * <p>Built throughout with an explicit gain of 1.0 and an explicit saturation, so every expected
 * value is the error in radians and can be checked by hand. The real gain is a tuning question and
 * not something a test should assert.
 */
class DriveHeadingControllerTest {
  private static final double kEpsilon = 1e-9;

  /** Well above any error used here, so nothing saturates unless a test means it to. */
  private static final double kGenerousMax = 100.0;

  private static DriveHeadingController controller() {
    return new DriveHeadingController(1.0, RadiansPerSecond.of(kGenerousMax), "Test/Generous");
  }

  private static double omega(DriveHeadingController controller, double currentDeg, double targetDeg) {
    return controller
        .calculate(Rotation2d.fromDegrees(currentDeg), Rotation2d.fromDegrees(targetDeg))
        .in(RadiansPerSecond);
  }

  @Test
  void turnsCounterClockwiseForAPositiveError() {
    // Target 30 degrees to the left of where we point, gain 1.0, so omega is that error in radians.
    assertEquals(Math.toRadians(30), omega(controller(), 0, 30), kEpsilon);
  }

  @Test
  void turnsClockwiseForANegativeError() {
    assertEquals(-Math.toRadians(30), omega(controller(), 0, -30), kEpsilon);
  }

  @Test
  void holdsStillWhenAlreadyOnTarget() {
    assertEquals(0.0, omega(controller(), 45, 45), kEpsilon);
  }

  @Test
  void takesTheShortWayRoundAcrossTheWrap() {
    // From +170 to -170 is 20 degrees counter-clockwise through the wrap, not 340 clockwise. A
    // controller without continuous input unwinds the long way, which reads as a sluggish robot
    // rather than as a bug -- this is the test that catches it.
    double result = omega(controller(), 170, -170);

    assertEquals(Math.toRadians(20), result, kEpsilon);
    assertTrue(result > 0, "the short way round from +170 to -170 is counter-clockwise");
  }

  @Test
  void clampsToTheMaximumAngularVelocity() {
    var clamped = new DriveHeadingController(1.0, RadiansPerSecond.of(2.0), "Test/Clamped");

    // A 180 degree error at gain 1.0 asks for 3.14 rad/s; the chassis only has 2.0.
    assertEquals(2.0, Math.abs(omega(clamped, 0, 179.99)), 1e-3);
  }

  @Test
  void clampsSymmetricallyInBothDirections() {
    var clamped = new DriveHeadingController(1.0, RadiansPerSecond.of(2.0), "Test/ClampedBoth");

    assertEquals(2.0, omega(clamped, 0, 150), kEpsilon);
    assertEquals(-2.0, omega(clamped, 0, -150), kEpsilon);
  }

  @Test
  void isNearTargetIsFalseBeforeTheFirstCalculate() {
    // A fresh PIDController reports zero error, which must not be mistaken for being on target:
    // a feed gate reading this would be told the chassis is aimed before anything aimed it.
    assertFalse(controller().isNearTarget(Degrees.of(1)));
  }

  @Test
  void isNearTargetReflectsTheLastError() {
    var controller = controller();

    omega(controller, 0, 10);
    assertFalse(controller.isNearTarget(Degrees.of(2)));
    assertTrue(controller.isNearTarget(Degrees.of(15)));

    omega(controller, 0, 1);
    assertTrue(controller.isNearTarget(Degrees.of(2)));
  }

  @Test
  void errorIsReportedWrappedAndSigned() {
    var controller = controller();

    omega(controller, 170, -170);

    assertEquals(20.0, controller.getError().in(Degrees), 1e-6);
  }

  @Test
  void outputDependsOnlyOnThePresentError() {
    var controller = controller();

    // Ten cycles of large error. With an integral term these would accumulate; with P alone they
    // leave nothing behind. This pins the property that justifies there being no reset() -- if
    // someone adds I or D, this test fails and tells them what else has to change.
    for (int i = 0; i < 10; i++) {
      omega(controller, 0, 90);
    }

    assertEquals(0.0, omega(controller, 45, 45), kEpsilon);
  }
}
