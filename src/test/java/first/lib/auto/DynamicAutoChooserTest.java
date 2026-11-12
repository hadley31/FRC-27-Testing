package first.lib.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.SequencedMap;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import first.lib.auto.DynamicAutoChooser.ChainTolerance;

/**
 * Exercises the graph the slots are populated from: which segments may open an auto, which may
 * follow which, and what the slots offer as a result.
 *
 * <p>The segments here are built on trajectories made up in the test rather than the project's own
 * paths, so a seam can be made to meet or not meet on purpose. {@link #theProjectsOwnPathsChainAsTheFilesSay()}
 * is the one that holds the real ones to account.
 *
 * <p>The slots are fakes. On the robot they are AdvantageKit choosers, whose only contract that
 * matters here is the one {@link FakeSlot} copies: a pick that is no longer on offer falls back to
 * the default, which is what makes the slots after a changed one clear themselves.
 */
class DynamicAutoChooserTest {
  private static final Pose2d kTrench = new Pose2d(4.0, 7.0, Rotation2d.ZERO);
  private static final Pose2d kShootSpot = new Pose2d(2.0, 5.0, Rotation2d.k180deg);
  private static final double[] kAtRest = {0.0, 0.0, 0.0};
  private static final double[] kRolling = {2.5, 0.0, 0.0};

  @BeforeAll
  static void initializeHal() {
    // The alert the chooser raises is HAL-backed, and Choreo reports usage when a factory is built.
    // AdvantageKit's conduit segfaults if the driver station has not been refreshed once.
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    DriverStationBackend.refreshData();
  }

  /** Leaves the alliance as the rest of these tests expect to find it. */
  @AfterEach
  void clearAlliance() {
    DriverStationSim.setAllianceStationId(AllianceStationID.UNKNOWN);
    DriverStationSim.notifyNewData();
    DriverStationBackend.refreshData();
  }

  /** A slot that behaves like the chooser on the dashboard: an invalid pick reads as the default. */
  private static final class FakeSlot implements ChooserSlot<AutoSegment> {
    private SequencedMap<String, AutoSegment> m_options = ChooserSlot.options();
    private AutoSegment m_default = null;
    private AutoSegment m_pick = null;
    private boolean m_picked = false;

    @Override
    public void setOptions(SequencedMap<String, AutoSegment> options, AutoSegment defaultValue) {
      m_options = options;
      m_default = defaultValue;
    }

    @Override
    public AutoSegment getSelected() {
      if (m_picked && (m_pick == null || m_options.containsValue(m_pick))) {
        return m_pick;
      }
      return m_default;
    }

    /** @param segment the segment to pick, or null for the empty option */
    void pick(AutoSegment segment) {
      m_pick = segment;
      m_picked = true;
    }
  }

  private static List<String> names(List<AutoSegment> segments) {
    return segments.stream().map(AutoSegment::name).toList();
  }

  /**
   * A factory configured the way the robot's is, alliance flipping and all, with nothing behind it:
   * the assembly never runs, so nothing has to move.
   */
  private static AutoFactory factory() {
    return new AutoFactory(
        Pose2d::new,
        pose -> {
        },
        sample -> {
        },
        true,
        new Mechanism() {
        },
        (trajectory, isStart) -> {
        });
  }

  private static SwerveSample sample(double t, Pose2d pose, double[] velocities) {
    return new SwerveSample(
        t,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians(),
        velocities[0],
        velocities[1],
        velocities[2],
        0.0,
        0.0,
        0.0,
        new double[4],
        new double[4]);
  }

  /** A two-sample trajectory that exists only to have the ends asked about. */
  private static Trajectory<SwerveSample> path(
      String name, Pose2d start, double[] startVelocities, Pose2d end, double[] endVelocities) {
    return new Trajectory<>(
        name,
        List.of(sample(0.0, start, startVelocities), sample(1.0, end, endVelocities)),
        List.of(),
        List.of());
  }

  private static AutoSegment driving(
      String name, Trajectory<SwerveSample> trajectory, AtomicInteger builds) {
    return new AutoSegment(name, routine -> {
      builds.incrementAndGet();
      AutoTrajectory path = routine.trajectory(trajectory);
      return AutoSegment.Run.driving(path, path.cmd());
    });
  }

  private static AutoSegment stationary(String name, AtomicInteger builds) {
    return new AutoSegment(name, routine -> {
      builds.incrementAndGet();
      return AutoSegment.Run.stationary(Command.noRequirements(coroutine -> {
      }).named(name));
    });
  }

  /** The fixture most tests use: out and back, both ends at rest, so the two chain either way. */
  private static final class Fixture implements AutoCloseable {
    final List<FakeSlot> slots = new ArrayList<>();
    final DynamicAutoChooser chooser;
    final AtomicInteger outBuilds = new AtomicInteger();
    final AtomicInteger backBuilds = new AtomicInteger();
    final AutoSegment out;
    final AutoSegment back;

    Fixture(int slotCount, ChainTolerance tolerance) {
      AutoFactory factory = factory();
      chooser = new DynamicAutoChooser(
          "DynamicAutoChooserTest",
          slotCount,
          () -> factory.newRoutine("Test Routine"),
          tolerance,
          slot -> {
            slots.add(new FakeSlot());
            return slots.get(slot);
          });

      out = driving("Out", path("out", kTrench, kAtRest, kShootSpot, kAtRest), outBuilds);
      back = driving("Back", path("back", kShootSpot, kAtRest, kTrench, kAtRest), backBuilds);

      chooser.addInitialSegment(out);
      chooser.addSegment(back);
    }

    @Override
    public void close() {
      chooser.close();
    }
  }

  private static Fixture fixture(int slotCount) {
    return new Fixture(slotCount, ChainTolerance.DEFAULT);
  }

  @Test
  void theSlotsFillInWithNoAllianceToFlipFor() {
    // The graph is measured from the trajectories as drawn, so it does not wait on a driver station.
    // Measuring the flipped ends instead left every slot empty until an alliance arrived, which in
    // simulation is never.
    assertTrue(MatchState.getAlliance().isEmpty(), "expected no alliance in a test JVM");

    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();

      assertEquals(List.of("Out"), names(fixture.chooser.getSlotOptions(0)));
      assertEquals(List.of(), fixture.chooser.getProblems());
    }
  }

  @Test
  void onlyTheSegmentsAllowedToOpenAnAutoAreOfferedFirst() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();

      // "Back" starts where "Out" ends, so nothing about its ends disqualifies it. It is kept out of
      // the first slot because nobody said the robot may be placed there.
      assertEquals(List.of("Out"), names(fixture.chooser.getSlotOptions(0)));
      assertEquals(List.of(fixture.out), fixture.chooser.getInitialSegments());
    }
  }

  @Test
  void aSlotOffersWhatCanFollowTheSlotBeforeIt() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();

      assertEquals(List.of("Back"), names(fixture.chooser.getSlotOptions(1)));
      assertEquals(List.of(fixture.back), fixture.chooser.getSuccessors(fixture.out));
      assertEquals(List.of(fixture.out), fixture.chooser.getSuccessors(fixture.back));
    }
  }

  @Test
  void aSlotOffersNothingUntilTheSlotBeforeItIsFilled() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();

      // Slot 2 is fillable, slot 3 is not: there is nothing in slot 2 for it to follow yet.
      assertEquals(List.of("Back"), names(fixture.chooser.getSlotOptions(1)));
      assertEquals(List.of(), names(fixture.chooser.getSlotOptions(2)));

      fixture.slots.get(1).pick(fixture.back);
      fixture.chooser.poll();

      assertEquals(List.of("Out"), names(fixture.chooser.getSlotOptions(2)));
    }
  }

  @Test
  void emptyingASlotClearsEverythingAfterIt() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();
      fixture.slots.get(1).pick(fixture.back);
      fixture.chooser.poll();
      fixture.slots.get(2).pick(fixture.out);
      fixture.chooser.poll();

      assertEquals("Out + Back + Out", fixture.chooser.getSequenceDescription());

      fixture.slots.get(1).pick(null);
      fixture.chooser.poll();

      // The third slot's pick is still "Out", but it is no longer on offer, so it reads as empty.
      assertEquals("Out", fixture.chooser.getSequenceDescription());
      assertEquals(List.of(), names(fixture.chooser.getSlotOptions(2)));
    }
  }

  @Test
  void changingTheFirstSlotInvalidatesWhatFollowedIt() {
    try (Fixture fixture = fixture(4)) {
      // An opening segment that goes somewhere else entirely, so nothing that followed the first one
      // can follow this one.
      AutoSegment side = driving(
          "Side",
          path("side", kTrench, kAtRest, new Pose2d(8.0, 1.0, Rotation2d.ZERO), kAtRest),
          new AtomicInteger());
      fixture.chooser.addInitialSegment(side);

      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();
      fixture.slots.get(1).pick(fixture.back);
      fixture.chooser.poll();

      assertEquals("Out + Back", fixture.chooser.getSequenceDescription());

      fixture.slots.get(0).pick(side);
      fixture.chooser.poll();

      assertEquals("Side", fixture.chooser.getSequenceDescription());
      assertEquals(List.of(), names(fixture.chooser.getSlotOptions(1)));
    }
  }

  @Test
  void aSegmentThatDoesNotDriveFitsAnywhereAndPassesTheSeamThrough() {
    try (Fixture fixture = fixture(4)) {
      AutoSegment shoot = stationary("Shoot", new AtomicInteger());
      fixture.chooser.addSegment(shoot);

      fixture.chooser.poll();

      // Nothing about where the robot is stops a shot, including at the very start.
      assertEquals(List.of("Out", "Shoot"), names(fixture.chooser.getSlotOptions(0)));

      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();
      fixture.slots.get(1).pick(shoot);
      fixture.chooser.poll();

      // The shot did not move the robot, so slot 3 still follows "Out".
      assertEquals(List.of("Back", "Shoot"), names(fixture.chooser.getSlotOptions(2)));

      fixture.slots.get(2).pick(fixture.back);
      fixture.chooser.poll();

      assertEquals("Out + Shoot + Back", fixture.chooser.getSequenceDescription());
    }
  }

  @Test
  void aVelocityStepIsEnoughToBreakAChain() {
    try (Fixture fixture = fixture(4)) {
      // Starts exactly where "Out" ends, but already rolling at 2.5 m/s.
      AutoSegment rollingBack = driving(
          "Rolling Back",
          path("rolling", kShootSpot, kRolling, kTrench, kAtRest),
          new AtomicInteger());
      fixture.chooser.addSegment(rollingBack);

      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();

      assertEquals(List.of("Back"), names(fixture.chooser.getSlotOptions(1)));
      assertTrue(
          fixture.chooser.getProblems().stream()
              .anyMatch(problem -> problem.startsWith("'Rolling Back' cannot follow anything")),
          "expected an unreachable-segment problem, got " + fixture.chooser.getProblems());
    }
  }

  @Test
  void ignoringVelocitiesLetsThatSameChainThrough() {
    try (Fixture fixture = new Fixture(4, ChainTolerance.POSE_ONLY)) {
      AutoSegment rollingBack = driving(
          "Rolling Back",
          path("rolling", kShootSpot, kRolling, kTrench, kAtRest),
          new AtomicInteger());
      fixture.chooser.addSegment(rollingBack);

      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();

      assertEquals(List.of("Back", "Rolling Back"), names(fixture.chooser.getSlotOptions(1)));
      assertEquals(List.of(), fixture.chooser.getProblems());
    }
  }

  @Test
  void aSegmentPickedTwiceIsBuiltOncePerSlot() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.slots.get(1).pick(fixture.back);
      fixture.slots.get(2).pick(fixture.out);
      fixture.slots.get(3).pick(fixture.back);
      fixture.chooser.poll();
      fixture.chooser.poll();

      assertEquals("Out + Back + Out + Back", fixture.chooser.getSequenceDescription());

      // Two occurrences in the assembly, plus the one the graph was measured from. Sharing an
      // occurrence would hand the second pass a trajectory that has already finished, and it would
      // fall straight through.
      assertEquals(3, fixture.outBuilds.get());
      assertEquals(3, fixture.backBuilds.get());
    }
  }

  @Test
  void eachOccurrenceGetsItsOwnTrajectory() {
    AutoFactory factory = factory();
    AutoRoutine routine = factory.newRoutine("Test Routine");
    List<AutoTrajectory> built = new ArrayList<>();
    Trajectory<SwerveSample> out = path("out", kTrench, kAtRest, kShootSpot, kAtRest);

    AutoSegment segment = new AutoSegment("Out", r -> {
      AutoTrajectory path = r.trajectory(out);
      built.add(path);
      return AutoSegment.Run.driving(path, path.cmd());
    });

    segment.build(routine);
    segment.build(routine);

    assertNotSame(built.get(0), built.get(1));
  }

  @Test
  void anUnchangedSelectionIsNotReassembled() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.poll();
      fixture.slots.get(0).pick(fixture.out);
      fixture.chooser.poll();

      Command first = fixture.chooser.selectedCommand();
      int builds = fixture.outBuilds.get();
      fixture.chooser.poll();
      fixture.chooser.poll();

      // Reassembling every cycle would rebind the routine's triggers under a running auto, and would
      // read every trajectory off the disk fifty times a second.
      assertEquals(builds, fixture.outBuilds.get());
      assertSame(first, fixture.chooser.selectedCommand());
    }
  }

  @Test
  void theDefaultSequenceIsWhatTheSlotsHoldUntouched() {
    try (Fixture fixture = fixture(4)) {
      fixture.chooser.setDefaultSequence(fixture.out, fixture.back);

      fixture.chooser.poll();

      assertEquals("Out + Back", fixture.chooser.getSequenceDescription());
      assertEquals(List.of(), fixture.chooser.getProblems());
    }
  }

  @Test
  void anIllegalDefaultSequenceIsReported() {
    try (Fixture fixture = fixture(4)) {
      // "Back" cannot open an auto, so this default would quietly assemble nothing at all.
      fixture.chooser.setDefaultSequence(fixture.back, fixture.out);

      fixture.chooser.poll();

      assertEquals("Nothing", fixture.chooser.getSequenceDescription());
      assertTrue(
          fixture.chooser.getProblems().stream()
              .anyMatch(problem -> problem.contains("slot 1 ('Back') cannot follow the start")),
          "expected an illegal-default problem, got " + fixture.chooser.getProblems());
    }
  }

  @Test
  void theProjectsOwnPathsChainAsTheFilesSay() {
    AutoFactory factory = factory();
    List<FakeSlot> slots = new ArrayList<>();
    try (DynamicAutoChooser chooser = new DynamicAutoChooser(
        "DynamicAutoChooserTest/RealPaths",
        4,
        () -> factory.newRoutine("Real Routine"),
        ChainTolerance.DEFAULT,
        slot -> {
          slots.add(new FakeSlot());
          return slots.get(slot);
        })) {

      AtomicInteger builds = new AtomicInteger();
      AutoSegment pass = trajectorySegment("Pass", "LeftTrenchFuelBumpShoot", builds);
      AutoSegment secondPass =
          trajectorySegment("Second Pass", "LeftTrenchFuelBumpShootSecondPass", builds);
      AutoSegment back = trajectorySegment("Return", "LeftBumpShootToLeftStartTrench", builds);

      chooser.addInitialSegment(pass);
      chooser.addSegment(secondPass);
      chooser.addSegment(back);

      chooser.poll();

      // This is the canary for an edit in Choreo: the whole two-pass chain holds together only
      // because each pair agrees at the waypoint they share. The pass and the return meet at rest.
      // The return leaves the trench at 1.99 m/s and the second pass picks it up at 2.00 m/s, which
      // is what makes that handoff legal -- and why the return cannot be followed by the first pass,
      // which starts from a standstill.
      assertEquals(List.of(back), chooser.getSuccessors(pass));
      assertEquals(List.of(secondPass), chooser.getSuccessors(back));
      assertEquals(List.of(back), chooser.getSuccessors(secondPass));
      assertEquals(List.of(pass), chooser.getInitialSegments());
      assertEquals(List.of(), chooser.getProblems());

      // The sequence CompetitionAutoMode ships as its default, held to the same graph the drive
      // team's picks are held to. A default that stopped chaining would quietly run a shorter auto.
      chooser.setDefaultSequence(pass, back, secondPass, back);
      chooser.poll();

      assertEquals(List.of(), chooser.getProblems());
      assertEquals("Pass + Return + Second Pass + Return", chooser.getSequenceDescription());
    }
  }

  @Test
  void ignoringVelocitiesAdmitsAHandoffTheVelocityRuleRefuses() {
    // The same three paths with only the velocity half of the rule relaxed. The return then leads
    // to the first pass as well, which is the handoff the velocity rule is there to refuse: the
    // return arrives at the trench at 1.99 m/s and that pass expects to start from a standstill.
    AutoFactory factory = factory();
    List<FakeSlot> slots = new ArrayList<>();

    try (DynamicAutoChooser chooser = new DynamicAutoChooser(
        "DynamicAutoChooserTest/PoseOnlyRealPaths",
        4,
        () -> factory.newRoutine("Real Routine"),
        ChainTolerance.POSE_ONLY,
        slot -> {
          slots.add(new FakeSlot());
          return slots.get(slot);
        })) {

      AtomicInteger builds = new AtomicInteger();
      AutoSegment pass = trajectorySegment("Pass", "LeftTrenchFuelBumpShoot", builds);
      AutoSegment secondPass =
          trajectorySegment("Second Pass", "LeftTrenchFuelBumpShootSecondPass", builds);
      AutoSegment back = trajectorySegment("Return", "LeftBumpShootToLeftStartTrench", builds);

      chooser.addInitialSegment(pass);
      chooser.addSegment(secondPass);
      chooser.addSegment(back);

      chooser.poll();

      // Both passes start where the return ends, so on pose alone the return leads to either.
      assertEquals(List.of(pass, secondPass), chooser.getSuccessors(back));
      assertEquals(List.of(), chooser.getProblems());
    }
  }

  private static AutoSegment trajectorySegment(
      String name, String trajectoryName, AtomicInteger builds) {
    return new AutoSegment(name, routine -> {
      builds.incrementAndGet();
      AutoTrajectory path = routine.trajectory(trajectoryName);
      return AutoSegment.Run.driving(path, path.cmd());
    });
  }

  @Test
  void aDefaultSequenceCannotNameASegmentThatWasNeverAdded() {
    try (Fixture fixture = fixture(4)) {
      AutoSegment stranger =
          driving("Stranger", path("stranger", kTrench, kAtRest, kTrench, kAtRest), new AtomicInteger());

      assertThrows(
          IllegalArgumentException.class, () -> fixture.chooser.setDefaultSequence(stranger));
    }
  }

  @Test
  void aDefaultSequenceCannotOutrunTheSlots() {
    try (Fixture fixture = fixture(2)) {
      assertThrows(
          IllegalArgumentException.class,
          () -> fixture.chooser.setDefaultSequence(fixture.out, fixture.back, fixture.out));
    }
  }

  @Test
  void aDuplicateSegmentNameIsRejected() {
    try (Fixture fixture = fixture(4)) {
      assertThrows(
          IllegalArgumentException.class,
          () -> fixture.chooser.addSegment(
              driving("Out", path("other", kTrench, kAtRest, kTrench, kAtRest), new AtomicInteger())));
    }
  }

  @Test
  void theNetworkBackedSlotsWorkAndAreSharedPerKey() {
    // The production slots, which the rest of these tests fake out. AdvantageKit cannot unregister
    // a dashboard input, so asking twice for one key has to hand back the same slot rather than a
    // second publisher on the same topics.
    AutoSegment out = driving("Out", path("out", kTrench, kAtRest, kShootSpot, kAtRest),
        new AtomicInteger());
    ChooserSlot<AutoSegment> first =
        ChooserSlot.networkBacked("DynamicAutoChooserTest/Network/Segment 1");
    ChooserSlot<AutoSegment> again =
        ChooserSlot.networkBacked("DynamicAutoChooserTest/Network/Segment 1");

    assertSame(first, again);

    SequencedMap<String, AutoSegment> options = ChooserSlot.options();
    options.put("None", null);
    options.put("Out", out);
    first.setOptions(options, out);

    assertSame(out, first.getSelected());
  }

  @Test
  void theProductionSlotsPublishWhereTheDashboardHasToLook() {
    // The robot's own configuration, down to the alliance being set: AdvantageKit choosers, real
    // paths, flipping on. What this pins down is the NT topics the drive team's widgets bind to.
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    DriverStationBackend.refreshData();
    assertEquals(Optional.of(Alliance.RED), MatchState.getAlliance(), "alliance did not take");

    AutoFactory factory = factory();
    String key = "DynamicAutoChooserTest/Published";

    try (DynamicAutoChooser chooser = new DynamicAutoChooser(
        key, 6, () -> factory.newRoutine("Published Routine"))) {
      AtomicInteger builds = new AtomicInteger();
      AutoSegment pass = trajectorySegment("Pass", "LeftTrenchFuelBumpShoot", builds);
      AutoSegment back = trajectorySegment("Return", "LeftBumpShootToLeftStartTrench", builds);

      chooser.addInitialSegment(pass);
      chooser.addSegment(back);
      chooser.setDefaultSequence(pass, back);

      chooser.poll();

      assertEquals(List.of(), chooser.getProblems());
      assertEquals("Pass + Return", chooser.getSequenceDescription());

      var inst = NetworkTableInstance.getDefault();
      String slotOne = "/" + key + "/Segment 1";

      assertEquals(
          "Selectable",
          inst.getStringTopic(slotOne + "/.type").subscribe("").get(),
          "the dashboard finds a chooser by its type");
      assertEquals(
          List.of("None", "Pass"),
          List.of(inst.getStringArrayTopic(slotOne + "/options").subscribe(new String[0]).get()));
      assertEquals("Pass", inst.getStringTopic(slotOne + "/default").subscribe("").get());
      assertEquals(
          List.of("None", "Return"),
          List.of(inst.getStringArrayTopic("/" + key + "/Segment 2/options")
              .subscribe(new String[0]).get()));
    }
  }

  @Test
  void aClosedChooserGivesUpItsKey() {
    // What the framework does to an opmode that owns one of these: closed on disable, rebuilt under
    // the same key on the next cycle. An alert that outlived the chooser would make this throw.
    try (Fixture first = fixture(4)) {
      first.chooser.poll();
    }

    try (Fixture second = fixture(4)) {
      second.chooser.setDefaultSequence(second.out);
      second.chooser.poll();

      assertEquals("Out", second.chooser.getSequenceDescription());
    }
  }
}
