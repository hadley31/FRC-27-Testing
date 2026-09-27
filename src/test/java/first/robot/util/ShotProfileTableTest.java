package first.robot.util;

import static first.robot.util.ShotProfileTable.row;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.regex.Pattern;

import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Distance;

/**
 * Covers the trust boundary between a dashboard and the shooter.
 *
 * <p>Everything a dashboard sends arrives as four loose arrays of doubles, so {@link
 * ShotProfileTable#validate} is the only thing standing between a half-finished edit and a shot
 * solution. Each rejection case below is a way that fails quietly if it is not caught: a row silently
 * disappearing, a lead computed in the wrong direction, a {@code NaN} handed to a mechanism.
 */
class ShotProfileTableTest {
  private static final ShotProfileTable kTable = ShotProfileTable.of(
      row(10.0, 1.0, 5.0, 1000.0),
      row(20.0, 2.0, 15.0, 2000.0));

  private static ShotProfileTable.Result validate(
      double[] distances, double[] flight, double[] hood, double[] flywheel) {
    return ShotProfileTable.validate(distances, flight, hood, flywheel);
  }

  private static ShotProfileTable.Result validateWithDistances(double... distances) {
    int rows = distances.length;
    return validate(distances, new double[rows], new double[rows], new double[rows]);
  }

  // MARK: - Rejections

  @Test
  void columnsOfDifferentLengthsAreRejected() {
    // The four columns are separate NT topics, so a dashboard adding a row is routinely observed
    // part-way through. This has to be a rejection, not a crash and not a silently truncated table.
    var result = validate(
        new double[] {10, 20, 30},
        new double[] {1, 2},
        new double[] {5, 10, 15},
        new double[] {1000, 2000, 3000});

    assertFalse(result.isAccepted());
    assertTrue(
        result.rejection().contains("column lengths disagree"),
        "the reason is shown to whoever is tuning, got: " + result.rejection());
  }

  @Test
  void aTableOfOneRowIsRejected() {
    // One row interpolates to the same shot at every range on the field.
    var result = validateWithDistances(10.0);

    assertFalse(result.isAccepted());
    assertTrue(result.rejection().contains("at least 2 rows"), result.rejection());
  }

  @Test
  void anEmptyTableIsRejected() {
    assertFalse(validateWithDistances().isAccepted());
  }

  @Test
  void duplicateRangesAreRejected() {
    // Two rows at one range collapse into one inside the interpolating map, so a row would vanish
    // from under the dashboard with nothing to show it had.
    var result = validateWithDistances(10.0, 10.0, 20.0);

    assertFalse(result.isAccepted());
    assertTrue(result.rejection().contains("strictly increase"), result.rejection());
  }

  @Test
  void outOfOrderRangesAreRejected() {
    var result = validateWithDistances(10.0, 30.0, 20.0);

    assertFalse(result.isAccepted());
    assertTrue(result.rejection().contains("strictly increase"), result.rejection());
  }

  @Test
  void nonFiniteValuesAreRejected() {
    assertFalse(validateWithDistances(10.0, Double.NaN).isAccepted());
    assertFalse(validateWithDistances(10.0, Double.POSITIVE_INFINITY).isAccepted());
    assertFalse(
        validate(
            new double[] {10, 20},
            new double[] {1, Double.NaN},
            new double[] {5, 10},
            new double[] {1000, 2000})
            .isAccepted(),
        "a NaN flight time would be commanded into the lead calculation");
    assertFalse(
        validate(
            new double[] {10, 20},
            new double[] {1, 2},
            new double[] {5, 10},
            new double[] {1000, Double.NaN})
            .isAccepted(),
        "a NaN flywheel speed would be commanded straight to the mechanism");
  }

  @Test
  void negativeRangesAndFlightTimesAreRejected() {
    assertFalse(validateWithDistances(-5.0, 10.0).isAccepted());

    var result = validate(
        new double[] {10, 20},
        new double[] {1, -2},
        new double[] {5, 10},
        new double[] {1000, 2000});

    assertFalse(result.isAccepted());
    assertTrue(
        result.rejection().contains("negative flight time"),
        "a negative flight time leads the shot the wrong way: " + result.rejection());
  }

  @Test
  void physicallyOddButOrderedValuesAreAccepted() {
    // Deliberately permissive: a hood angle that decreases with range, or a range far past anything
    // the robot can shoot, is a legitimate experiment. Mechanism limits are the mechanisms' job.
    var result = validate(
        new double[] {1, 500},
        new double[] {0, 12},
        new double[] {-45, -90},
        new double[] {9000, 10});

    assertTrue(result.isAccepted(), result.rejection());
  }

  @Test
  void aTableDeclaredInSourceIsHeldToTheSameRules() {
    // A range typed out of order in ShotProfile should fail at class initialization, not produce a
    // table that quietly interpolates the wrong way.
    assertThrows(
        IllegalArgumentException.class,
        () -> ShotProfileTable.of(row(20.0, 1.0, 5.0, 1000.0), row(10.0, 2.0, 15.0, 2000.0)));
  }

  // MARK: - Acceptance

  @Test
  void anAcceptedSnapshotKeepsRowsPairedAndInOrder() {
    var result = validate(
        new double[] {10, 20},
        new double[] {1, 2},
        new double[] {5, 15},
        new double[] {1000, 2000});

    var table = result.table().orElseThrow();

    assertEquals("", result.rejection());
    assertEquals(2, table.rows().size());
    assertEquals(row(10.0, 1.0, 5.0, 1000.0), table.rows().get(0));
    assertEquals(row(20.0, 2.0, 15.0, 2000.0), table.rows().get(1));
  }

  @Test
  void columnsRoundTripThroughATable() {
    // These are exactly the arrays published as the dashboard's defaults, so they have to come back
    // out in the same order they went in.
    assertArrayEqualsExactly(new double[] {10, 20}, kTable.distanceColumn());
    assertArrayEqualsExactly(new double[] {1, 2}, kTable.timeOfFlightColumn());
    assertArrayEqualsExactly(new double[] {5, 15}, kTable.hoodAngleColumn());
    assertArrayEqualsExactly(new double[] {1000, 2000}, kTable.flywheelColumn());
  }

  // MARK: - Conversion to a profile

  @Test
  void toProfileInterpolatesBetweenRows() {
    var profile = kTable.toProfile(Seconds.of(0.03));

    assertEquals(0.03, profile.actuationLatency().in(Seconds), 1e-12);
    assertEquals(1.5, profile.timeOfFlight().get(Feet.of(15)).in(Seconds), 1e-9);
    assertEquals(10.0, profile.hoodAngle().get(Feet.of(15)).in(Degrees), 1e-9);
    assertEquals(1500.0, profile.flywheelVelocity().get(Feet.of(15)).in(RPM), 1e-9);
  }

  @Test
  void toProfileClampsOutsideTheTable() {
    var profile = kTable.toProfile(Seconds.of(0.03));

    assertEquals(1000.0, profile.flywheelVelocity().get(Feet.of(2)).in(RPM), 1e-9);
    assertEquals(2000.0, profile.flywheelVelocity().get(Feet.of(200)).in(RPM), 1e-9);
  }

  @Test
  void applyToRewritesAProfileThroughReferencesTakenBeforehand() {
    // This is the whole mechanism behind live tuning: a command that already holds the profile, or one
    // of its curves, picks up an edited table on its next lookup without being handed a new object.
    var profile = kTable.toProfile(Seconds.of(0.03));
    var heldProfile = profile;
    var heldFlywheelCurve = profile.flywheelVelocity();

    ShotProfileTable.of(
        row(10.0, 1.0, 5.0, 3000.0),
        row(20.0, 2.0, 15.0, 4000.0))
        .applyTo(profile);

    assertEquals(3500.0, heldProfile.flywheelVelocity().get(Feet.of(15)).in(RPM), 1e-9);
    assertEquals(3500.0, heldFlywheelCurve.get(Feet.of(15)).in(RPM), 1e-9);
  }

  @Test
  void applyToRemovesRangesTheNewTableDoesNotHave() {
    var profile = ShotProfileTable.of(
        row(10.0, 1.0, 5.0, 1000.0),
        row(15.0, 1.0, 5.0, 1900.0),
        row(20.0, 2.0, 15.0, 2000.0))
        .toProfile(Seconds.of(0.03));

    kTable.applyTo(profile);

    // 1900 at 15 ft was a real point; dropping that row must put 15 ft back on the straight line.
    assertEquals(1500.0, profile.flywheelVelocity().get(Feet.of(15)).in(RPM), 1e-9);
  }

  // MARK: - Export back into source

  @Test
  void theCommittedTableSurvivesAnExportImportRoundTrip() {
    // The export is how a tuning session ends, so it has to be complete, ordered, and printed at a
    // precision that reads back as the same table.
    assertEquals(ShotProfile.kScoringTable, reimport(ShotProfile.kScoringTable.toJava()));
  }

  @Test
  void theExportNamesTheFactoryItIsPastedInto() {
    assertTrue(ShotProfile.kScoringTable.toJava().startsWith("ShotProfileTable.of("));
    assertTrue(ShotProfile.kScoringTable.toJava().trim().endsWith(");"));
  }

  // MARK: - The committed table

  @Test
  void theCommittedTableStillHoldsTheValuesItWasTunedWith() {
    // kScoring used to be three curves with their own breakpoints. Putting them on one shared range
    // axis was supposed to leave the shooter alone; these are the original hand-tuned points, so if
    // the restructure moved anything, it moved here.
    var profile = ShotProfile.kScoring;

    assertEquals(0.9, flightAt(profile, 6.7), 1e-9);
    assertEquals(1.4, flightAt(profile, 10.4), 1e-9);
    assertEquals(1.4, flightAt(profile, 15.2), 1e-9);
    assertEquals(1.6, flightAt(profile, 20.0), 1e-9);

    assertEquals(0.0, hoodAt(profile, 4.0), 1e-9);
    assertEquals(3.0, hoodAt(profile, 5.0), 1e-9);
    assertEquals(4.5, hoodAt(profile, 6.0), 1e-9);
    assertEquals(7.0, hoodAt(profile, 7.0), 1e-9);
    assertEquals(8.0, hoodAt(profile, 9.0), 1e-9);
    assertEquals(9.0, hoodAt(profile, 10.0), 1e-9);
    assertEquals(10.0, hoodAt(profile, 12.0), 1e-9);
    assertEquals(10.0, hoodAt(profile, 16.0), 1e-9);

    assertEquals(1575.0, flywheelAt(profile, 4.0), 1e-9);
    assertEquals(1625.0, flywheelAt(profile, 6.0), 1e-9);
    assertEquals(1725.0, flywheelAt(profile, 7.0), 1e-9);
    assertEquals(1825.0, flywheelAt(profile, 9.0), 1e-9);
    assertEquals(1900.0, flywheelAt(profile, 11.0), 1e-9);
    assertEquals(2075.0, flywheelAt(profile, 13.0), 1e-9);
    assertEquals(2275.0, flywheelAt(profile, 16.0), 1e-9);
    assertEquals(2675.0, flywheelAt(profile, 25.0), 1e-9);
  }

  @Test
  void theCommittedTableInterpolatesWhereTheOldCurvesDid() {
    // Spot checks between the original breakpoints, where a resample would show up if the shared axis
    // had dropped one. Tolerances are the rounding of the printed table, not a fudge factor.
    var profile = ShotProfile.kScoring;

    // Halfway along the old 6.7 -> 10.4 ft flight-time segment.
    assertEquals(1.15, flightAt(profile, 8.55), 5e-4);
    // Inside the old 16 -> 25 ft flywheel segment.
    assertEquals(2275.0 + 400.0 * 2.0 / 9.0, flywheelAt(profile, 18.0), 0.05);
  }

  private static double flightAt(ShotProfile profile, double feet) {
    return profile.timeOfFlight().get(feet(feet)).in(Seconds);
  }

  private static double hoodAt(ShotProfile profile, double feet) {
    return profile.hoodAngle().get(feet(feet)).in(Degrees);
  }

  private static double flywheelAt(ShotProfile profile, double feet) {
    return profile.flywheelVelocity().get(feet(feet)).in(RPM);
  }

  private static Distance feet(double feet) {
    return Feet.of(feet);
  }

  private static void assertArrayEqualsExactly(double[] expected, double[] actual) {
    assertEquals(expected.length, actual.length);

    for (int i = 0; i < expected.length; i++) {
      assertEquals(expected[i], actual[i], 1e-12, "element " + i);
    }
  }

  /** Reads an export back, the way a person pasting it into {@link ShotProfile} does. */
  private static ShotProfileTable reimport(String java) {
    var pattern = Pattern.compile(
        "row\\(\\s*(-?[\\d.]+),\\s*(-?[\\d.]+),\\s*(-?[\\d.]+),\\s*(-?[\\d.]+)\\)");
    var matcher = pattern.matcher(java);
    var rows = new ArrayList<ShotProfileTable.Row>();

    while (matcher.find()) {
      rows.add(row(
          Double.parseDouble(matcher.group(1)),
          Double.parseDouble(matcher.group(2)),
          Double.parseDouble(matcher.group(3)),
          Double.parseDouble(matcher.group(4))));
    }

    return ShotProfileTable.of(rows.toArray(ShotProfileTable.Row[]::new));
  }
}
