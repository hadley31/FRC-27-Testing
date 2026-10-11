package first.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Degrees;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Pins {@link GeometryUtil#bearing}, which anything steering toward a point relies on.
 *
 * <p>The arithmetic is WPILib's; what matters here is the direction convention and the answer for
 * two points that coincide, since that is the case a caller cannot handle for itself without
 * knowing what it would have done with a direction.
 */
class GeometryUtilTest {
  private static final double kEpsilonDegrees = 1e-6;

  private static final Rotation2d kFallback = Rotation2d.fromDegrees(123);

  @Test
  void bearingPointsFromTheFirstPointToTheSecond() {
    var bearing = GeometryUtil.bearing(
        new Translation2d(1, 1), new Translation2d(4, 1), kFallback);

    assertEquals(0.0, bearing.getMeasure().in(Degrees), kEpsilonDegrees);
  }

  @Test
  void bearingIsMeasuredCounterClockwiseFromTheFieldXAxis() {
    var bearing = GeometryUtil.bearing(
        new Translation2d(1, 1), new Translation2d(1, 4), kFallback);

    assertEquals(90.0, bearing.getMeasure().in(Degrees), kEpsilonDegrees);
  }

  @Test
  void bearingRunsBackwardsWhenTheTargetIsBehind() {
    var bearing = GeometryUtil.bearing(
        new Translation2d(4, 1), new Translation2d(1, 1), kFallback);

    assertEquals(180.0, Math.abs(bearing.getMeasure().in(Degrees)), kEpsilonDegrees);
  }

  @Test
  void bearingFallsBackWhenThePointsCoincide() {
    var point = new Translation2d(2, 3);

    assertEquals(
        kFallback.getMeasure().in(Degrees),
        GeometryUtil.bearing(point, point, kFallback).getMeasure().in(Degrees),
        kEpsilonDegrees);
  }
}
