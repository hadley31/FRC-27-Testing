package first.lib.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

/**
 * Exercises assembling an auto out of segments against the project's real Choreo paths, since the
 * behaviour worth testing is all about what the drive team's slot choices add up to.
 *
 * <p>Nothing is published here. An unpublished slot selects to its default, which is what
 * {@link SegmentedAutoChooser#setDefaultSequence(String...)} sets, so the defaults stand in for the
 * dashboard and the assembly runs exactly as it would with one attached.
 */
class SegmentedAutoChooserTest {
  /** The trench pass, which ends roughly 2.6 m from where it started. */
  private static final String kPassTrajectory = "LeftTrenchFuelBumpShoot";

  /** The way back, which starts where the pass ends and ends where the pass starts. */
  private static final String kReturnTrajectory = "LeftBumpShootToLeftStartTrench";

  @BeforeAll
  static void initializeHal() {
    // The alert the chooser raises on a bad seam is HAL-backed, and Choreo reports usage when a
    // factory is built. AdvantageKit's conduit segfaults if the driver station has not been
    // refreshed once, which RobotBase.startRobot would have done by now on a robot.
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    DriverStationBackend.refreshData();
  }

  /** A factory with nothing behind it: the assembly never runs, so nothing has to move. */
  private static AutoFactory factory() {
    return new AutoFactory(
        Pose2d::new,
        pose -> {
        },
        sample -> {
        },
        // Flipping off, so the trajectory endpoints resolve without an alliance to flip them for.
        false,
        new Mechanism() {
        },
        (trajectory, isStart) -> {
        });
  }

  /** A unique key per chooser, so each test's slots and seam alert are its own. */
  private static final AtomicInteger s_keys = new AtomicInteger();

  private static SegmentedAutoChooser chooser(int slots) {
    return chooser("SegmentedAutoChooserTest/" + s_keys.incrementAndGet(), slots);
  }

  private static SegmentedAutoChooser chooser(String key, int slots) {
    AutoFactory factory = factory();
    return new SegmentedAutoChooser(key, slots, () -> factory.newRoutine("Test Routine"));
  }

  /** A segment that drives a path and counts how many times it has been built. */
  private static AutoSegment driving(String name, String trajectoryName, AtomicInteger builds) {
    return new AutoSegment(name, routine -> {
      builds.incrementAndGet();
      AutoTrajectory path = routine.trajectory(trajectoryName);
      return AutoSegment.Run.driving(path, path.cmd());
    });
  }

  /** A segment that does not drive, so it can sit anywhere without affecting a seam. */
  private static AutoSegment stationary(String name, AtomicInteger builds) {
    return new AutoSegment(name, routine -> {
      builds.incrementAndGet();
      return AutoSegment.Run.stationary(Command.noRequirements(coroutine -> {
      }).named(name));
    });
  }

  @Test
  void theSlotsAreAssembledInOrder() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    chooser.addSegment(driving("Return", kReturnTrajectory, new AtomicInteger()));
    chooser.setDefaultSequence("Pass", "Return");

    chooser.poll();

    assertEquals("Pass + Return", chooser.getSequenceDescription());
  }

  @Test
  void anEmptySlotInTheMiddleClosesUp() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    chooser.addSegment(driving("Return", kReturnTrajectory, new AtomicInteger()));
    chooser.setDefaultSequence("Pass", "None", "Return");

    chooser.poll();

    assertEquals("Pass + Return", chooser.getSequenceDescription());
  }

  @Test
  void aSegmentPickedTwiceIsBuiltTwice() {
    AtomicInteger passBuilds = new AtomicInteger();
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, passBuilds));
    chooser.addSegment(driving("Return", kReturnTrajectory, new AtomicInteger()));
    chooser.setDefaultSequence("Pass", "Return", "Pass", "Return");

    chooser.poll();

    // Both passes must be their own occurrence. Sharing one would hand the second pass a trajectory
    // that has already finished, and it would fall straight through.
    assertEquals(2, passBuilds.get());
    assertEquals("Pass + Return + Pass + Return", chooser.getSequenceDescription());
  }

  @Test
  void eachOccurrenceGetsItsOwnTrajectory() {
    AutoFactory factory = factory();
    AutoRoutine routine = factory.newRoutine("Test Routine");
    List<AutoTrajectory> built = new ArrayList<>();

    AutoSegment pass = new AutoSegment("Pass", r -> {
      AutoTrajectory path = r.trajectory(kPassTrajectory);
      built.add(path);
      return AutoSegment.Run.driving(path, path.cmd());
    });

    pass.build(routine);
    pass.build(routine);

    assertNotSame(built.get(0), built.get(1));
  }

  @Test
  void aChainThatMeetsRaisesNothing() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    chooser.addSegment(driving("Return", kReturnTrajectory, new AtomicInteger()));
    chooser.setDefaultSequence("Pass", "Return", "Pass", "Return");

    chooser.poll();

    assertEquals(List.of(), chooser.getSeamWarnings());
  }

  @Test
  void twoPassesInARowAreReportedAsASeam() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    chooser.setDefaultSequence("Pass", "Pass");

    chooser.poll();

    // The pass ends over the bump and starts under the trench, so following it with itself asks the
    // robot to teleport back.
    assertEquals(1, chooser.getSeamWarnings().size());
    assertTrue(
        chooser.getSeamWarnings().get(0).contains("'Pass' leaves the robot 2."),
        "unexpected warning: " + chooser.getSeamWarnings().get(0));
  }

  @Test
  void aStationarySegmentDoesNotBreakTheSeamAroundIt() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    chooser.addSegment(driving("Return", kReturnTrajectory, new AtomicInteger()));
    chooser.addSegment(stationary("Shoot", new AtomicInteger()));
    chooser.setDefaultSequence("Pass", "Shoot", "Return");

    chooser.poll();

    assertEquals(List.of(), chooser.getSeamWarnings());
    assertEquals("Pass + Shoot + Return", chooser.getSequenceDescription());
  }

  @Test
  void nothingSelectedAssemblesNothing() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));

    chooser.poll();

    assertEquals("Nothing", chooser.getSequenceDescription());
    assertEquals(List.of(), chooser.getSeamWarnings());
  }

  @Test
  void anUnchangedSelectionIsNotReassembled() {
    AtomicInteger passBuilds = new AtomicInteger();
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, passBuilds));
    chooser.setDefaultSequence("Pass");

    chooser.poll();
    Command first = chooser.selectedCommand();
    chooser.poll();
    chooser.poll();

    // Rebuilding every cycle would rebind the routine's triggers under a running auto, and would
    // read every trajectory off the disk fifty times a second.
    assertEquals(1, passBuilds.get());
    assertSame(first, chooser.selectedCommand());
  }

  @Test
  void aDashboardPickIsWhatGetsAssembled() {
    // A backend of its own, under a prefix nothing else publishes to, standing in for the dashboard
    // the slots are really driven from.
    MockTunableBackend dashboard = new MockTunableBackend();
    String key = "MockDashboard/Auto";
    TunableRegistry.registerBackend("MockDashboard", dashboard);

    var chooser = chooser(key, 4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    chooser.addSegment(driving("Return", kReturnTrajectory, new AtomicInteger()));
    chooser.setDefaultSequence("Pass");
    chooser.publish();
    chooser.poll();

    assertEquals("Pass", chooser.getSequenceDescription());

    dashboard.setValue(key + "/Segment 2/selected", "Return");
    TunableRegistry.update();
    chooser.poll();

    assertEquals("Pass + Return", chooser.getSequenceDescription());

    chooser.close();
  }

  @Test
  void aClosedChooserGivesUpItsKey() {
    // What the framework does to an opmode that owns one of these: closed on disable, rebuilt under
    // the same key on the next cycle. An alert that outlived the chooser would make this throw.
    String key = "SegmentedAutoChooserTest/rebuilt";
    SegmentedAutoChooser first = chooser(key, 4);
    first.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    first.close();

    SegmentedAutoChooser second = chooser(key, 4);
    second.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));
    second.setDefaultSequence("Pass");
    second.poll();

    assertEquals("Pass", second.getSequenceDescription());
    second.close();
  }

  @Test
  void aDefaultSequenceCannotNameAMissingSegment() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));

    assertThrows(IllegalArgumentException.class, () -> chooser.setDefaultSequence("Nonexistent"));
  }

  @Test
  void aDefaultSequenceCannotOutrunTheSlots() {
    var chooser = chooser(2);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));

    assertThrows(
        IllegalArgumentException.class,
        () -> chooser.setDefaultSequence("Pass", "Pass", "Pass"));
  }

  @Test
  void aDuplicateSegmentNameIsRejected() {
    var chooser = chooser(4);
    chooser.addSegment(driving("Pass", kPassTrajectory, new AtomicInteger()));

    assertThrows(
        IllegalArgumentException.class,
        () -> chooser.addSegment(driving("Pass", kReturnTrajectory, new AtomicInteger())));
  }
}
