package first.lib.tuning;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import java.nio.file.Files;
import java.util.Arrays;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.AlertSim;
import org.wpilib.units.measure.Angle;

/**
 * Pins the behaviour of {@link LoggedTunable} that fails silently if it regresses: that a value
 * already on the robot beats the code default, that the topic is flagged so the robot actually saves
 * it, that reads are cached rather than live, and that a measure is stored in the unit it was
 * declared in.
 *
 * <p>The fixture is {@code ToggleTest}'s, for the same reason: persistent values land in server
 * storage and need a local subscriber to propagate back out to getters, which is what
 * {@code RobotBase} arranges on a real robot.
 */
class LoggedTunableTest {
  private enum Mode {
    AUTO,
    MANUAL
  }

  private static MultiSubscriber s_subAll;

  @BeforeAll
  static void startServer() throws Exception {
    var inst = NetworkTableInstance.getDefault();
    inst.startServer(Files.createTempFile("nt-logged-tunable-test", ".json").toString(), "", "", 0);
    s_subAll = new MultiSubscriber(inst, new String[] {""});
  }

  @AfterAll
  static void stopServer() {
    s_subAll.close();
    NetworkTableInstance.getDefault().stopServer();
  }

  @AfterEach
  void clearRegistry() {
    // Otherwise the second test to use a path throws on a duplicate, which is the point of that
    // check: on a real robot everything is constructed exactly once.
    LoggedTunables.reset();
  }

  @Test
  void aFreshPathReadsTheCodeDefault() {
    var tunable = LoggedTunableDouble.of("/Tuning/Test/Fresh", 1.5);

    assertEquals(1.5, tunable.getAsDouble());
    assertEquals(1.5, tunable.codeDefault());
    assertTrue(tunable.isDefault());
    assertEquals("/Tuning/Test/Fresh", tunable.path());
  }

  @Test
  void aStoredValueBeatsTheCodeDefault() {
    // The whole point of the feature. Published before construction, so this tests the setDefault
    // semantic directly rather than waiting on a file load.
    var inst = NetworkTableInstance.getDefault();
    try (var publisher = inst.getDoubleTopic("/Tuning/Test/Stored").publish()) {
      publisher.set(3.5);
      inst.flush();

      var tunable = LoggedTunableDouble.of("/Tuning/Test/Stored", 1.0);

      assertEquals(3.5, tunable.getAsDouble(), "setDefault must not stamp over a stored value");
      assertEquals(1.0, tunable.codeDefault());
      assertFalse(tunable.isDefault());
    }
  }

  @Test
  void theTopicIsFlaggedSoTheServerSavesIt() {
    LoggedTunableDouble.of("/Tuning/Test/Persisted", 2.0);
    NetworkTableInstance.getDefault().flush();

    // The ordering under test: the server ignores setPersistent on a topic that has never published
    // a value, and only logs a warning. Together with the test above this is the whole "survives a
    // reboot" claim, split so each half points at one cause when it fails.
    assertTrue(
        NetworkTableInstance.getDefault().getDoubleTopic("/Tuning/Test/Persisted").isPersistent());
  }

  @Test
  void setIsNotVisibleUntilTheNextPeriodic() {
    var tunable = LoggedTunableDouble.of("/Tuning/Test/Cached", 1.0);

    tunable.set(9.0);
    assertEquals(1.0, tunable.getAsDouble(), "get() must be a cached field read, not a live lookup");

    tunable.periodic();
    assertEquals(9.0, tunable.getAsDouble());
  }

  @Test
  void changedThisLoopIsTrueForOneLoopAndDoesNotClearOnRead() {
    var tunable = LoggedTunableDouble.of("/Tuning/Test/Changed", 1.0);

    tunable.periodic();
    assertFalse(tunable.changedThisLoop());

    tunable.set(4.0);
    tunable.periodic();
    // Read twice: this is the org.wpilib.tunable bug pinned shut. A flag cleared by whoever reads it
    // first silently steals the change from every other consumer.
    assertTrue(tunable.changedThisLoop());
    assertTrue(tunable.changedThisLoop());

    tunable.periodic();
    assertFalse(tunable.changedThisLoop(), "a stale flag re-applies configs every loop forever");
  }

  @Test
  void theAlertNamesOffDefaultValuesAndClearsOnReset() {
    var tunable = LoggedTunableDouble.of("/Tuning/Test/Alerting", 1.0);

    LoggedTunables.refresh();
    assertEquals(0, LoggedTunables.nonDefaultPaths().length);
    assertFalse(alertIsActive());

    tunable.set(7.0);
    tunable.periodic();
    LoggedTunables.refresh();

    assertArrayEquals(new String[] {"/Tuning/Test/Alerting"}, LoggedTunables.nonDefaultPaths());
    assertTrue(alertIsActive());
    assertTrue(alertText().contains("/Tuning/Test/Alerting"), "actual: " + alertText());

    LoggedTunables.resetAllToDefaults();
    tunable.periodic();
    LoggedTunables.refresh();

    assertEquals(0, LoggedTunables.nonDefaultPaths().length);
    assertFalse(alertIsActive());
  }

  @Test
  void aMeasureIsStoredInTheUnitItWasDeclaredIn() {
    LoggedTunableMeasure<Angle> tunable =
        LoggedTunableMeasure.of("/Tuning/Test/Angle", Degrees.of(2.5));

    // Degrees, not the 0.0436 radians that base units would have given. The number a person edits
    // has to be the number they declared, which is also why the unit is in the path.
    assertEquals(
        2.5,
        NetworkTableInstance.getDefault().getDoubleTopic(tunable.path()).subscribe(Double.NaN).get());
    assertEquals(2.5, tunable.get().in(Degrees));
    assertEquals("/Tuning/Test/Angle (Degree)", tunable.path());
  }

  @Test
  void aMeasureSetInAnotherUnitIsConvertedIntoThePublishedOne() {
    LoggedTunableMeasure<Angle> tunable =
        LoggedTunableMeasure.of("/Tuning/Test/Converted", Degrees.of(0));

    tunable.set(Rotations.of(0.5));
    tunable.periodic();

    assertEquals(180.0, tunable.get().in(Degrees), 1e-9);
  }

  @Test
  void aMeasureEquivalentToItsDefaultCountsAsDefault() {
    LoggedTunableMeasure<Angle> tunable =
        LoggedTunableMeasure.of("/Tuning/Test/Equivalent", Degrees.of(90));

    tunable.set(Rotations.of(0.25));
    tunable.periodic();

    // Same angle, different unit. Comparing with equals would leave this permanently "overridden"
    // and permanently "changed".
    assertTrue(tunable.isDefault());
    assertFalse(tunable.changedThisLoop());
  }

  @Test
  void anEnumRoundTripsThroughItsName() {
    var tunable = LoggedTunableEnum.of("/Tuning/Test/Mode", Mode.AUTO);

    assertEquals(Mode.AUTO, tunable.get());
    assertEquals(
        "AUTO",
        NetworkTableInstance.getDefault().getStringTopic(tunable.path()).subscribe("").get());

    tunable.set(Mode.MANUAL);
    tunable.periodic();

    assertEquals(Mode.MANUAL, tunable.get());
  }

  @Test
  void anUnrecognisedEnumNameFallsBackToTheCodeDefault() {
    var inst = NetworkTableInstance.getDefault();
    try (var publisher = inst.getStringTopic("/Tuning/Test/BadMode").publish()) {
      publisher.set("SIDEWAYS");
      inst.flush();

      var tunable = LoggedTunableEnum.of("/Tuning/Test/BadMode", Mode.AUTO);

      // Falls back rather than throwing: a renamed constant must not stop the robot from driving.
      assertEquals(Mode.AUTO, tunable.get());
    }
  }

  @Test
  void aBooleanReadsAsASupplier() {
    var tunable = LoggedTunableBoolean.of("/Tuning/Test/Flag", true);

    assertTrue(tunable.getAsBoolean());

    tunable.set(false);
    tunable.periodic();

    assertFalse(tunable.getAsBoolean());
    assertFalse(tunable.isDefault());
  }

  @Test
  void theValueRoundTripsThroughALogTable() {
    // A real replay needs a WPILOGReader fixture. Driving toLog/fromLog directly -- the same methods
    // Logger.processInputs calls -- covers what can actually be wrong here: the log key and the
    // codec's round trip.
    var tunable = LoggedTunableDouble.of("/Tuning/Test/Logged", 1.0);
    tunable.set(6.5);
    tunable.periodic();

    var table = new LogTable(0);
    tunable.toLog(table);

    // The log key mirrors the NT tree with the leading slash dropped, as AdvantageKit's own
    // dashboard inputs do.
    assertEquals(6.5, table.get("Tuning/Test/Logged", 0.0));

    // And the other direction: during replay the log wins over whatever NetworkTables holds.
    var replayTable = new LogTable(0);
    replayTable.put("Tuning/Test/Logged", 11.25);
    tunable.fromLog(replayTable);

    assertEquals(11.25, tunable.getAsDouble());
  }

  @Test
  void pathsMustBeAbsoluteAndNameATopic() {
    // Same rule as Toggle and TunableDoubleArray, which share this package: a relative path would
    // create a topic with no leading slash that dashboards will not nest.
    assertThrows(
        IllegalArgumentException.class, () -> LoggedTunableDouble.of("Tuning/Test/Relative", 1.0));
    assertThrows(
        IllegalArgumentException.class, () -> LoggedTunableDouble.of("/Tuning/Test/Trailing/", 1.0));
    assertThrows(
        IllegalArgumentException.class,
        () -> LoggedTunableMeasure.of("Tuning/Test/RelativeAngle", Degrees.of(1)));
  }

  @Test
  void duplicatePathsAreRejected() {
    LoggedTunableDouble.of("/Tuning/Test/Duplicate", 1.0);

    // Two values on one path would share a topic and a log key, and the second one's code default
    // would be a phantom the alert compares against forever.
    assertThrows(
        IllegalArgumentException.class,
        () -> LoggedTunableDouble.of("/Tuning/Test/Duplicate", 2.0));
  }

  /** The aggregated alert as the driver station sees it, found by its published id. */
  private static AlertSim.AlertInfo alert() {
    return Arrays.stream(AlertSim.getAll())
        .filter(info -> LoggedTunables.kAlertId.equals(info.id))
        .findFirst()
        .orElseThrow(() -> new AssertionError("no alert with id " + LoggedTunables.kAlertId));
  }

  private static boolean alertIsActive() {
    return alert().isActive();
  }

  private static String alertText() {
    return alert().text;
  }
}
