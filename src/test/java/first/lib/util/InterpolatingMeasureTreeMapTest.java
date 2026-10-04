package first.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;

import org.junit.jupiter.api.Test;

import first.lib.util.InterpolatingMeasureTreeMap;

/**
 * Pins the parts of {@link InterpolatingMeasureTreeMap} that a live-tuned table depends on.
 *
 * <p>The interpolation itself is WPILib's; what matters here is that a table being rewritten while the
 * robot runs can never be caught half-built, and that an empty table says so rather than surfacing as
 * an NPE from inside a unit conversion.
 */
class InterpolatingMeasureTreeMapTest {
  @Test
  void interpolatesBetweenPointsInWhateverUnitsTheyWereGivenIn() {
    var map = InterpolatingMeasureTreeMap.distanceToAngle()
        .put(Feet.of(10), Degrees.of(0))
        .put(Feet.of(20), Degrees.of(10));

    assertEquals(5.0, map.get(Feet.of(15)).in(Degrees), 1e-9);
  }

  @Test
  void anEmptyTableFailsWithItsOwnDiagnosis() {
    var map = InterpolatingMeasureTreeMap.distanceToAngle();

    // Without the guard this is an NPE unboxing a null Double inside the unit conversion, which reads
    // as a bug in the interpolator rather than as a table nobody filled in.
    var thrown = assertThrows(IllegalStateException.class, () -> map.get(Feet.of(10)));
    assertEquals("Cannot interpolate a table with no points", thrown.getMessage());
  }

  @Test
  void replaceAllSwapsEveryPoint() {
    var map = InterpolatingMeasureTreeMap.distanceToAngle()
        .put(Feet.of(10), Degrees.of(0))
        .put(Feet.of(20), Degrees.of(10));

    map.replaceAll(next -> next
        .put(Feet.of(10), Degrees.of(30))
        .put(Feet.of(20), Degrees.of(40)));

    assertEquals(35.0, map.get(Feet.of(15)).in(Degrees), 1e-9);
  }

  @Test
  void replaceAllDropsPointsTheReplacementOmits() {
    // A dashboard removing a range must actually remove it, rather than leaving the old point behind
    // to keep bending the curve.
    var map = InterpolatingMeasureTreeMap.distanceToAngle()
        .put(Feet.of(10), Degrees.of(0))
        .put(Feet.of(15), Degrees.of(9))
        .put(Feet.of(20), Degrees.of(10));

    map.replaceAll(next -> next
        .put(Feet.of(10), Degrees.of(0))
        .put(Feet.of(20), Degrees.of(10)));

    assertEquals(5.0, map.get(Feet.of(15)).in(Degrees), 1e-9);
  }

  @Test
  void aFailedReplacementLeavesThePreviousTableIntact() {
    // The robot has to keep shooting on the last good table. If the swap were a clear-then-refill, an
    // exception part-way through would leave an empty table and every later lookup would throw.
    var map = InterpolatingMeasureTreeMap.distanceToAngle()
        .put(Feet.of(10), Degrees.of(0))
        .put(Feet.of(20), Degrees.of(10));

    assertThrows(RuntimeException.class, () -> map.replaceAll(next -> {
      next.put(Feet.of(10), Degrees.of(30));
      throw new RuntimeException("dashboard gave us something impossible");
    }));

    assertEquals(5.0, map.get(Feet.of(15)).in(Degrees), 1e-9);
  }

  @Test
  void replaceAllIsVisibleThroughAReferenceTakenBeforeTheSwap() {
    // This is what lets a tuned table be handed out as a plain field: a command holding the map from
    // an earlier cycle sees the new points on its next lookup.
    var map = InterpolatingMeasureTreeMap.distanceToAngle()
        .put(Feet.of(10), Degrees.of(0))
        .put(Feet.of(20), Degrees.of(10));
    var held = map;

    map.replaceAll(next -> next
        .put(Feet.of(10), Degrees.of(30))
        .put(Feet.of(20), Degrees.of(40)));

    assertEquals(35.0, held.get(Feet.of(15)).in(Degrees), 1e-9);
  }
}
