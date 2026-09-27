package first.robot.util;

import static first.robot.util.ShotProfileTable.row;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import java.nio.file.Files;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.units.measure.Time;

/**
 * Exercises live tuning end to end over a real NetworkTables server, since the failure modes worth
 * testing are all about what arrives from a dashboard mid-edit.
 *
 * <p>{@code periodic} is called by hand here. On the robot AdvantageKit calls it at the top of every
 * cycle, before any user code, which is what guarantees a command cannot run against a table mid-swap.
 */
class TunableShotProfileTest {
  private static final Time kLatency = Seconds.of(0.03);

  private static final ShotProfileTable kDefaultTable = ShotProfileTable.of(
      row(10.0, 1.0, 5.0, 1000.0),
      row(20.0, 2.0, 15.0, 2000.0));

  private static final ShotProfileTable kEditedTable = ShotProfileTable.of(
      row(10.0, 1.0, 5.0, 3000.0),
      row(20.0, 2.0, 15.0, 4000.0));

  private static MultiSubscriber s_subAll;

  @BeforeAll
  static void startServer() throws Exception {
    var inst = NetworkTableInstance.getDefault();
    inst.startServer(Files.createTempFile("nt-tunable-shot-profile-test", ".json").toString(),
        "", "", 0);
    // RobotBase does the same: a local subscriber is what propagates published values back out to the
    // getters, including anything restored from persistent storage.
    s_subAll = new MultiSubscriber(inst, new String[] {""});
  }

  @AfterAll
  static void stopServer() {
    s_subAll.close();
    NetworkTableInstance.getDefault().stopServer();
  }

  /** A fresh tunable profile on its own path, so tests cannot see each other's topics. */
  private static TunableShotProfile tunable(String name) {
    return TunableShotProfile.of("/TunableShotProfileTest/" + name, kLatency, kDefaultTable);
  }

  private static void publish(String name, ShotProfileTable table) {
    publish(name, table.distanceColumn(), table.timeOfFlightColumn(),
        table.hoodAngleColumn(), table.flywheelColumn());
  }

  private static void publish(
      String name, double[] distances, double[] flight, double[] hood, double[] flywheel) {
    var inst = NetworkTableInstance.getDefault();
    String path = "/TunableShotProfileTest/" + name;

    inst.getDoubleArrayTopic(path + "/DistancesFeet").publish().set(distances);
    inst.getDoubleArrayTopic(path + "/TimeOfFlightSeconds").publish().set(flight);
    inst.getDoubleArrayTopic(path + "/HoodAngleDegrees").publish().set(hood);
    inst.getDoubleArrayTopic(path + "/FlywheelRpm").publish().set(flywheel);
    inst.flush();
  }

  private static double flywheelAt(TunableShotProfile profile, double feet) {
    return profile.profile().flywheelVelocity().get(Feet.of(feet)).in(RPM);
  }

  @Test
  void aRobotThatHasNeverSeenADashboardUsesTheCommittedTable() {
    var tunable = tunable("Untouched");

    assertTrue(tunable.isDefault());
    assertEquals(kDefaultTable, tunable.table());
    assertEquals(1500.0, flywheelAt(tunable, 15.0), 1e-9);
    assertEquals(0.03, tunable.profile().actuationLatency().in(Seconds), 1e-12);
  }

  @Test
  void anEditedTableReachesTheLiveProfile() {
    var tunable = tunable("Edited");

    publish("Edited", kEditedTable);
    tunable.periodic();

    assertEquals(kEditedTable, tunable.table());
    assertEquals(3500.0, flywheelAt(tunable, 15.0), 1e-9);
    assertFalse(tunable.isDefault(), "a tuned table must be visible as such");
  }

  @Test
  void anEditIsVisibleThroughAProfileFetchedBeforehand() {
    // A command mid-shot is holding the profile from an earlier cycle. It has to pick up the edit, and
    // the profile it holds must stay the same object for that to work.
    var tunable = tunable("HeldProfile");
    var held = tunable.profile();
    var heldFlywheelCurve = held.flywheelVelocity();

    publish("HeldProfile", kEditedTable);
    tunable.periodic();

    assertSame(held, tunable.profile(), "editing the curves must not replace the profile object");
    assertEquals(3500.0, heldFlywheelCurve.get(Feet.of(15)).in(RPM), 1e-9);
  }

  @Test
  void theActuationLatencyIsTheOneItWasBuiltWith() {
    var tunable = tunable("Latency");

    publish("Latency", kEditedTable);
    tunable.periodic();

    assertEquals(0.03, tunable.profile().actuationLatency().in(Seconds), 1e-12,
        "editing the range table must not disturb the mechanism lag");
  }

  @Test
  void aRowAddedToOnlySomeColumnsIsIgnoredUntilTheRestCatchUp() {
    // What a dashboard mid-write looks like: NT makes no promise the four topics update together.
    // Rejecting the torn snapshot costs a cycle; using it would shoot a table nobody authored.
    var tunable = tunable("TornWrite");

    publish("TornWrite", kEditedTable);
    tunable.periodic();

    publish("TornWrite",
        new double[] {10.0, 15.0, 20.0},
        new double[] {1.0, 2.0},
        new double[] {5.0, 15.0},
        new double[] {3000.0, 4000.0});
    tunable.periodic();

    assertEquals(kEditedTable, tunable.table(), "the last good table must stay in effect");
    assertEquals(3500.0, flywheelAt(tunable, 15.0), 1e-9);
  }

  @Test
  void anUnusableTableLeavesTheLastGoodOneInEffect() {
    var tunable = tunable("Unusable");

    publish("Unusable",
        new double[] {10.0, Double.NaN},
        new double[] {1.0, 2.0},
        new double[] {5.0, 15.0},
        new double[] {3000.0, 4000.0});
    tunable.periodic();

    assertEquals(kDefaultTable, tunable.table());
    assertEquals(1500.0, flywheelAt(tunable, 15.0), 1e-9);
    assertTrue(tunable.isDefault());
  }

  @Test
  void addingAndRemovingRangesBothTakeEffect() {
    var tunable = tunable("Resized");

    var withExtraRange = ShotProfileTable.of(
        row(10.0, 1.0, 5.0, 1000.0),
        row(15.0, 1.0, 5.0, 1900.0),
        row(20.0, 2.0, 15.0, 2000.0));

    publish("Resized", withExtraRange);
    tunable.periodic();
    assertEquals(1900.0, flywheelAt(tunable, 15.0), 1e-9);

    publish("Resized", kDefaultTable);
    tunable.periodic();
    assertEquals(1500.0, flywheelAt(tunable, 15.0), 1e-9,
        "removing a range must put the curve back on the straight line");
  }



  @Test
  void resetToDefaultDiscardsAnEdit() {
    var tunable = tunable("Reset");

    publish("Reset", kEditedTable);
    tunable.periodic();
    assertFalse(tunable.isDefault());

    tunable.resetToDefault();
    NetworkTableInstance.getDefault().flush();
    tunable.periodic();

    assertTrue(tunable.isDefault());
    assertEquals(kDefaultTable, tunable.table());
    assertEquals(1500.0, flywheelAt(tunable, 15.0), 1e-9);
  }

  @Test
  void aPersistentProfileFlagsEveryColumnSoTheServerSavesThem() throws Exception {
    // A tuning session has to survive the reboots between matches; a column that is not flagged comes
    // back as the committed default while the others come back tuned, which is the worst outcome.
    TunableShotProfile.persistent(
        "/TunableShotProfileTest/Persisted", kLatency, kDefaultTable);
    NetworkTableInstance.getDefault().flush();

    for (String column : new String[] {
        "DistancesFeet", "TimeOfFlightSeconds", "HoodAngleDegrees", "FlywheelRpm"}) {
      assertTrue(
          NetworkTableInstance.getDefault()
              .getDoubleArrayTopic("/TunableShotProfileTest/Persisted/" + column)
              .isPersistent(),
          column + " must be flagged, or nothing survives a reboot");
    }
  }

  @Test
  void theColumnsArePublishedWhereTheDashboardLooksForThem() {
    tunable("Layout");
    NetworkTableInstance.getDefault().flush();

    var inst = NetworkTableInstance.getDefault();

    assertTrue(inst.getDoubleArrayTopic("/TunableShotProfileTest/Layout/DistancesFeet").exists());
    assertTrue(
        inst.getDoubleArrayTopic("/TunableShotProfileTest/Layout/TimeOfFlightSeconds").exists());
    assertTrue(inst.getDoubleArrayTopic("/TunableShotProfileTest/Layout/HoodAngleDegrees").exists());
    assertTrue(inst.getDoubleArrayTopic("/TunableShotProfileTest/Layout/FlywheelRpm").exists());
  }
}
