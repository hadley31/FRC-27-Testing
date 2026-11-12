package first.lib.auto;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.SequencedMap;
import java.util.Set;
import java.util.function.IntFunction;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

/**
 * An auto the drive team assembles out of {@link AutoSegment}s, one per slot, from the dashboard.
 *
 * <p>The segments form a graph: an edge from one to another means the second one picks the robot up
 * where the first one put it, in both pose and velocity. A slot only offers the segments that can
 * follow what the slots before it add up to, so an illegal auto cannot be assembled in the first
 * place rather than being caught after the fact. Which segments may go first is not something the
 * graph can work out -- nothing precedes them -- so {@link #addInitialSegment(AutoSegment)} is how
 * the places the robot may be placed get named.
 *
 * <p>An empty slot ends the auto, and the slots after it offer nothing until it is filled in. That
 * falls out of the same rule: a slot's options are the successors of the segment before it, and
 * after an empty slot there is no segment before it to have successors.
 *
 * <p>Whatever the slots add up to is built into a single {@link AutoRoutine}: the segments'
 * trajectories all come from that one routine, which is what lets a segment bind triggers against
 * its own path and still be polled by the auto that contains it. The first segment that drives also
 * places the robot, by resetting odometry to that path's start before anything else runs, wherever
 * in the slots it sits.
 *
 * <p>{@link #poll()} belongs in an opmode's {@code disabledPeriodic()}. It is where the graph is
 * built, the options are rewritten, and the auto is reassembled -- all of it only when something has
 * actually changed, so the trajectories are read off the disk as the drive team picks rather than at
 * the moment auto starts.
 *
 * <p>The graph is measured from the trajectories as they were drawn, before alliance flipping, so
 * the slots fill in whether or not a driver station has said which alliance the robot is on. That
 * costs nothing in accuracy: flipping moves both ends of a seam the same way, so two ends that meet
 * unflipped meet flipped.
 *
 * <p>What the graph cannot vouch for is reported through an alert: a segment whose trajectory never
 * loaded, and a default sequence that is not a legal chain. The second one matters because
 * {@link #setDefaultSequence(AutoSegment...)} is written by a programmer rather than picked from the
 * offered options, so it is the one way an illegal chain can still be asked for -- and it would show
 * up as an auto that quietly stops early.
 *
 * <p>This class only builds the routine. What runs it is somebody else's decision:
 * {@link #selectedRoutine()} is a supplier shaped for a chooser of whole autos, Choreo's own
 * included, and {@link #selectedCommand()} is the same routine ready to schedule.
 */
public class DynamicAutoChooser implements AutoCloseable {
  /** The slot option standing for "nothing here", which every slot carries. */
  private static final String kNoneName = "None";

  /** What {@link #getSequenceDescription()} says when the selection adds up to nothing. */
  private static final String kNothingDescription = "Nothing";

  /**
   * How closely two segments have to agree about where the robot is and what it is doing before one
   * is allowed to follow the other.
   *
   * <p>The velocity half is the part that decides whether paths may hand off at speed. Two paths
   * that both start and end at rest agree trivially; two that were drawn without each other in mind
   * usually do not, and a tolerance loose enough to let them through is a tolerance that is not
   * checking anything. {@link #POSE_ONLY} says that out loud instead.
   *
   * @param position        how far apart the two poses may be
   * @param rotation        how far out of line the two headings may be
   * @param velocity        how much the translational velocity may step, per axis
   * @param angularVelocity how much the rotational velocity may step
   */
  public record ChainTolerance(
      Distance position, Angle rotation, LinearVelocity velocity, AngularVelocity angularVelocity) {
    /** Tight enough that only paths actually drawn to meet each other will. */
    public static final ChainTolerance DEFAULT = new ChainTolerance(
        Meters.of(0.1), Degrees.of(5), MetersPerSecond.of(0.5), DegreesPerSecond.of(45));

    /**
     * Poses must meet, velocities are ignored.
     *
     * <p>For paths whose ends were never matched to each other: the chain is geometrically sound and
     * the robot takes whatever velocity step the seam asks of it.
     */
    public static final ChainTolerance POSE_ONLY = new ChainTolerance(
        Meters.of(0.1),
        Degrees.of(5),
        MetersPerSecond.of(Double.POSITIVE_INFINITY),
        DegreesPerSecond.of(Double.POSITIVE_INFINITY));
  }

  /**
   * Where a segment leaves the robot, or where it needs the robot to be.
   *
   * @param pose       the field-relative pose
   * @param velocities the velocities as the trajectory reports them at that end
   */
  public record SegmentState(Pose2d pose, ChassisVelocities velocities) {
  }

  /** The two ends of a segment that drives. */
  private record SegmentEndpoints(SegmentState entry, SegmentState exit) {
  }

  private final String m_key;
  private final Supplier<AutoRoutine> m_routineFactory;
  private final ChainTolerance m_tolerance;
  private final List<ChooserSlot<AutoSegment>> m_slots = new ArrayList<>();
  private final List<List<AutoSegment>> m_slotOptions = new ArrayList<>();
  private final AutoSegment[] m_slotDefaults;

  /** Every segment on offer, in the order the dashboard lists them. */
  private final List<AutoSegment> m_catalog = new ArrayList<>();
  private final Set<AutoSegment> m_initialSegments = new LinkedHashSet<>();
  private final Alert m_problemAlert;

  /** The two ends of every segment that drives. Empty until the graph is built. */
  private final Map<AutoSegment, SegmentEndpoints> m_endpoints = new LinkedHashMap<>();

  /** Which segments may follow each driving segment. The graph itself. */
  private final Map<AutoSegment, List<AutoSegment>> m_successors = new LinkedHashMap<>();

  /** Segments that do not drive, and so cannot be entered from the wrong place. */
  private final Set<AutoSegment> m_stationarySegments = new LinkedHashSet<>();

  private boolean m_graphBuilt = false;
  private List<String> m_problems = List.of();

  private List<AutoSegment> m_selectionAtGeneration = null;
  private Optional<Alliance> m_allianceAtGeneration = Optional.empty();
  private AutoRoutine m_generatedRoutine = null;
  private Command m_generatedCommand = doNothing(kNothingDescription);
  private String m_sequenceDescription = kNothingDescription;

  /**
   * @param key            where the slots are published, one chooser per slot underneath it
   * @param slots          how many segments the drive team can chain. Every slot is published
   *                       whether or not it is used, so this is as much a dashboard layout decision
   *                       as a limit.
   * @param routineFactory builds the empty routine an assembly fills. Called once per assembly and
   *                       once to measure the segments, since a routine cannot be rebuilt.
   */
  public DynamicAutoChooser(String key, int slots, Supplier<AutoRoutine> routineFactory) {
    this(key, slots, routineFactory, ChainTolerance.DEFAULT);
  }

  /**
   * @param key            where the slots are published, one chooser per slot underneath it
   * @param slots          how many segments the drive team can chain
   * @param routineFactory builds the empty routine an assembly fills
   * @param tolerance      how closely segments have to meet to be chained
   */
  public DynamicAutoChooser(
      String key, int slots, Supplier<AutoRoutine> routineFactory, ChainTolerance tolerance) {
    this(
        key,
        slots,
        routineFactory,
        tolerance,
        slot -> ChooserSlot.networkBacked(key + "/Segment " + (slot + 1)));
  }

  /**
   * @param key            where the slots are published, and the log key the assembled sequence is
   *                       recorded under
   * @param slots          how many segments the drive team can chain
   * @param routineFactory builds the empty routine an assembly fills
   * @param tolerance      how closely segments have to meet to be chained
   * @param slotFactory    builds the slot for each index, counting from zero
   */
  public DynamicAutoChooser(
      String key,
      int slots,
      Supplier<AutoRoutine> routineFactory,
      ChainTolerance tolerance,
      IntFunction<ChooserSlot<AutoSegment>> slotFactory) {
    m_key = key;
    m_routineFactory = routineFactory;
    m_tolerance = tolerance;
    m_slotDefaults = new AutoSegment[slots];

    for (int slot = 0; slot < slots; slot++) {
      m_slots.add(slotFactory.apply(slot));
      m_slotOptions.add(List.of());
    }

    // Keyed by the chooser's path, so a robot with more than one of these does not have them
    // fighting over one alert.
    m_problemAlert = new Alert(key + ": auto segments unusable", "", Level.MEDIUM);
  }

  /**
   * Adds a segment the drive team can pick, anywhere the graph allows it.
   *
   * @param segment the segment to offer
   * @return this chooser, so calls can be chained
   * @throws IllegalArgumentException if a segment of the same name was already added
   */
  public DynamicAutoChooser addSegment(AutoSegment segment) {
    if (m_catalog.stream().anyMatch(added -> added.name().equals(segment.name()))) {
      // Two segments under one name would be one option on the dashboard, and whichever the drive
      // team picked would be a coin toss.
      throw new IllegalArgumentException("Duplicate auto segment name: " + segment.name());
    }

    m_catalog.add(segment);
    m_graphBuilt = false;

    return this;
  }

  /**
   * Adds a segment that may also go first, because the drive team is allowed to place the robot
   * where it starts.
   *
   * <p>Nothing precedes the first slot, so this is the one part of the graph that cannot be
   * measured: a segment is only offered there because someone said the robot may start there.
   *
   * @param segment the segment to offer
   * @return this chooser, so calls can be chained
   * @throws IllegalArgumentException if a segment of the same name was already added
   */
  public DynamicAutoChooser addInitialSegment(AutoSegment segment) {
    addSegment(segment);
    m_initialSegments.add(segment);

    return this;
  }

  /**
   * Sets what the slots hold before anyone touches the dashboard. Slots past the end of the
   * sequence are left empty.
   *
   * <p>This is how a known-good auto stays one that nobody has to assemble: the defaults are the
   * routine the team would have hard-coded, and the slots are there to depart from it. Unlike a
   * pick from a dropdown it is not checked as it is made, so the chain is checked against the graph
   * once that is built, and anything illegal is reported through {@link #getProblems()}.
   *
   * @param segmentNames the segments to put in the first slots, in order
   * @return this chooser, so calls can be chained
   * @throws IllegalArgumentException if a name was never added, or there are more names than slots
   */
  public DynamicAutoChooser setDefaultSequence(AutoSegment... segments) {
    if (segments.length > m_slots.size()) {
      throw new IllegalArgumentException(
          "Default sequence is " + segments.length + " segments long, but there are only "
              + m_slots.size() + " slots");
    }

    for (int slot = 0; slot < segments.length; slot++) {
      requireAdded(segments[slot]);
      m_slotDefaults[slot] = segments[slot];
    }

    // The options already pushed to the slots carry the old defaults with them.
    m_slotOptions.replaceAll(options -> List.of());

    return this;
  }

  /**
   * Builds the graph if it is not built, rewrites any slot whose options have changed, and
   * reassembles the auto if the selection has moved. Cheap when nothing has changed, so it belongs
   * in a periodic callback that runs while the robot is disabled.
   */
  public void poll() {
    buildGraphIfNeeded();

    List<AutoSegment> selection = updateSlots();

    if (isStale(selection)) {
      regenerate(selection);
    }
  }

  /**
   * The assembled routine, brought up to date with the dashboard first.
   *
   * <p>Shaped for a chooser of whole autos to take as a supplier, Choreo's {@code AutoChooser}
   * included. Note that such a chooser asks for the routine when its own selection changes, which
   * is not when the slots change: a chooser that holds on to what it was given will go stale if the
   * slots move afterwards.
   *
   * @return the routine the slots add up to, which is a routine that does nothing when they add up
   *     to nothing.
   */
  public AutoRoutine selectedRoutine() {
    poll();

    return m_generatedRoutine;
  }

  /**
   * The assembled auto as a command, brought up to date with the dashboard first.
   *
   * @return {@link AutoRoutine#cmd()} for {@link #selectedRoutine()}, the same instance until the
   *     selection changes.
   */
  public Command selectedCommand() {
    poll();

    return m_generatedCommand;
  }

  /**
   * What the slots currently add up to.
   *
   * @return the assembled segment names in order.
   */
  public String getSequenceDescription() {
    return m_sequenceDescription;
  }

  /**
   * What the graph could not vouch for, in the words the alert carries.
   *
   * @return one entry per unusable segment and per illegal default, empty when there are none.
   */
  public List<String> getProblems() {
    return m_problems;
  }

  /**
   * What a slot is currently offering, in the order the dashboard shows them.
   *
   * @param slot the slot index, counting from zero
   * @return the segments on offer. The empty option every slot carries is not one of them.
   */
  public List<AutoSegment> getSlotOptions(int slot) {
    return m_slotOptions.get(slot);
  }

  /**
   * Which segments can follow a given one, as the graph has it.
   *
   * @param segment the segment to look up
   * @return the segments that may follow it, in catalog order. Empty for a segment that does not
   *     drive, whose successors depend on what came before it instead.
   */
  public List<AutoSegment> getSuccessors(AutoSegment segment) {
    return m_successors.getOrDefault(segment, List.of());
  }

  /**
   * Which segments may go first.
   *
   * @return the segments added as initial ones, minus any whose trajectory never loaded, plus the
   *     ones that do not drive and so can go anywhere.
   */
  public List<AutoSegment> getInitialSegments() {
    return allowedAfter(null);
  }

  /** Gives up the alert's name. The slots outlive this object and are left alone. */
  @Override
  public void close() {
    m_problemAlert.close();
  }

  private void buildGraphIfNeeded() {
    if (m_graphBuilt) {
      return;
    }

    // One throwaway routine to measure against: a segment's ends are only knowable by building it,
    // and the commands that come back are discarded.
    AutoRoutine probe = m_routineFactory.get();
    List<AutoSegment> unusable = new ArrayList<>();

    m_endpoints.clear();
    m_stationarySegments.clear();
    m_successors.clear();

    for (AutoSegment segment : m_catalog) {
      AutoSegment.Run run = segment.build(probe);

      if (run.entryPath().isEmpty()) {
        m_stationarySegments.add(segment);
        continue;
      }

      AutoTrajectory entryPath = run.entryPath().get();
      Optional<SegmentState> entry = entryState(entryPath);
      Optional<SegmentState> exit = exitState(run.exitPath().orElse(entryPath));

      if (entry.isEmpty() || exit.isEmpty()) {
        // The only way an unflipped end is unknown is a trajectory with no samples in it, which is
        // what a .traj that failed to load leaves behind.
        unusable.add(segment);
        continue;
      }

      m_endpoints.put(segment, new SegmentEndpoints(entry.get(), exit.get()));
    }

    for (var from : m_endpoints.entrySet()) {
      m_successors.put(
          from.getKey(),
          m_endpoints.entrySet().stream()
              .filter(to -> canFollow(from.getValue().exit(), to.getValue().entry()))
              .map(Map.Entry::getKey)
              .toList());
    }

    m_graphBuilt = true;

    List<String> problems = new ArrayList<>();
    for (AutoSegment segment : unusable) {
      problems.add("'" + segment.name() + "' has no usable trajectory, so it is not offered");
    }
    problems.addAll(unreachableSegments());
    problems.addAll(illegalDefaults());
    reportProblems(problems);
  }

  private Optional<SegmentState> entryState(AutoTrajectory path) {
    return path.getUnflippedInitialPose().flatMap(
        pose -> path.getUnflippedInitialChassisVelocities()
            .map(velocities -> new SegmentState(pose, velocities)));
  }

  private Optional<SegmentState> exitState(AutoTrajectory path) {
    return path.getUnflippedFinalPose().flatMap(
        pose -> path.getUnflippedFinalChassisVelocities()
            .map(velocities -> new SegmentState(pose, velocities)));
  }

  private boolean canFollow(SegmentState exit, SegmentState entry) {
    double gapMeters = exit.pose().getTranslation().getDistance(entry.pose().getTranslation());
    double turnDegrees = Math.abs(
        exit.pose().getRotation().minus(entry.pose().getRotation()).getDegrees());

    return gapMeters <= m_tolerance.position().in(Meters)
        && turnDegrees <= m_tolerance.rotation().in(Degrees)
        && Math.abs(exit.velocities().vx - entry.velocities().vx) <= m_tolerance.velocity().in(MetersPerSecond)
        && Math.abs(exit.velocities().vy - entry.velocities().vy) <= m_tolerance.velocity().in(MetersPerSecond)
        && Math.abs(exit.velocities().omega - entry.velocities().omega)
            <= m_tolerance.angularVelocity().in(RadiansPerSecond);
  }

  /**
   * The segments that may follow a given driving segment, or that may go first when given null.
   * Catalog order, so the dashboard's lists are stable.
   */
  private List<AutoSegment> allowedAfter(AutoSegment lastDriving) {
    Set<AutoSegment> allowed = new LinkedHashSet<>();

    if (lastDriving == null) {
      // Nothing has moved the robot yet, so the only paths that may run are the ones someone said
      // the robot can be placed at the start of.
      m_initialSegments.stream().filter(m_endpoints::containsKey).forEach(allowed::add);
    } else {
      allowed.addAll(m_successors.getOrDefault(lastDriving, List.of()));
    }

    allowed.addAll(m_stationarySegments);

    return m_catalog.stream().filter(allowed::contains).toList();
  }

  /** Rewrites the slots' options and reports back what they now hold. */
  private List<AutoSegment> updateSlots() {
    List<AutoSegment> selection = new ArrayList<>();
    AutoSegment lastDriving = null;
    boolean ended = false;

    for (int slot = 0; slot < m_slots.size(); slot++) {
      List<AutoSegment> allowed = ended ? List.of() : allowedAfter(lastDriving);
      applyOptions(slot, allowed);

      AutoSegment chosen = m_slots.get(slot).getSelected();

      if (chosen == null || !allowed.contains(chosen)) {
        // An empty slot ends the auto. Carry on through the rest of the slots to clear them, since
        // there is no longer anything for them to follow.
        ended = true;
        continue;
      }

      selection.add(chosen);
      if (m_endpoints.containsKey(chosen)) {
        lastDriving = chosen;
      }
    }

    return selection;
  }

  private void applyOptions(int slot, List<AutoSegment> allowed) {
    if (allowed.equals(m_slotOptions.get(slot))) {
      return;
    }

    m_slotOptions.set(slot, List.copyOf(allowed));

    // The empty option stands for no segment at all, so it carries no segment: a slot reading back
    // null is a slot holding nothing.
    SequencedMap<String, AutoSegment> options = ChooserSlot.options();
    options.put(kNoneName, null);
    allowed.forEach(segment -> options.put(segment.name(), segment));

    AutoSegment configured = m_slotDefaults[slot];
    m_slots.get(slot).setOptions(
        options, configured != null && allowed.contains(configured) ? configured : null);
  }

  /**
   * Walks the graph from the initial segments and names the ones it never arrives at.
   *
   * <p>A segment nothing can precede and that may not go first is a segment the drive team will
   * never be offered, which is worth saying out loud: it is usually a pair of paths whose ends were
   * meant to meet and do not.
   */
  private List<String> unreachableSegments() {
    Deque<AutoSegment> queue = new ArrayDeque<>(allowedAfter(null));
    Set<AutoSegment> reached = new LinkedHashSet<>(queue);

    while (!queue.isEmpty()) {
      for (AutoSegment successor : m_successors.getOrDefault(queue.poll(), List.of())) {
        if (reached.add(successor)) {
          queue.add(successor);
        }
      }
    }

    return m_endpoints.keySet().stream()
        .filter(segment -> !reached.contains(segment))
        .map(segment -> "'" + segment.name() + "' cannot follow anything the robot can reach, so it"
            + " can never be picked")
        .toList();
  }

  /** Walks the configured defaults as if they were picked, and says where that would go wrong. */
  private List<String> illegalDefaults() {
    List<String> problems = new ArrayList<>();
    AutoSegment lastDriving = null;
    boolean ended = false;

    for (int slot = 0; slot < m_slotDefaults.length; slot++) {
      AutoSegment segment = m_slotDefaults[slot];
      if (segment == null) {
        ended = true;
        continue;
      }

      if (ended) {
        problems.add(
            "the default for slot " + (slot + 1) + " ('" + segment.name() + "') follows an empty"
                + " slot, so it will be dropped");
        continue;
      }

      if (!allowedAfter(lastDriving).contains(segment)) {
        problems.add(
            "the default for slot " + (slot + 1) + " ('" + segment.name() + "') cannot follow "
                + (lastDriving == null ? "the start of the auto" : "'" + lastDriving.name() + "'")
                + ", so it and everything after it will be dropped");
        ended = true;
        continue;
      }

      if (m_endpoints.containsKey(segment)) {
        lastDriving = segment;
      }
    }

    return problems;
  }

  private AutoSegment requireAdded(AutoSegment segment) {
    if (!m_catalog.contains(segment)) {
      throw new IllegalArgumentException(
          "Auto segment '" + segment.name() + "' was never added to this chooser");
    }

    return segment;
  }

  private void reportProblems(List<String> problems) {
    m_problems = List.copyOf(problems);
    // Logged as well as alerted, so an empty dashboard can be explained from the log afterwards
    // rather than only by someone who was watching at the time.
    Logger.recordOutput(m_key + "/Problems", m_problems.toArray(new String[0]));

    if (m_problems.isEmpty()) {
      m_problemAlert.set(false);
    } else {
      m_problemAlert.setText(String.join("; ", m_problems));
      m_problemAlert.set(true);
    }
  }

  private boolean isStale(List<AutoSegment> selection) {
    if (m_selectionAtGeneration == null) {
      return true;
    }

    // The alliance is part of the selection, not just an input to it: a routine's trajectories are
    // flipped around the field it was built for.
    if (!m_allianceAtGeneration.equals(MatchState.getAlliance())) {
      return true;
    }

    return !m_selectionAtGeneration.equals(selection);
  }

  private void regenerate(List<AutoSegment> selection) {
    m_selectionAtGeneration = List.copyOf(selection);
    m_allianceAtGeneration = MatchState.getAlliance();
    m_sequenceDescription = selection.isEmpty()
        ? kNothingDescription
        : selection.stream().map(AutoSegment::name).collect(Collectors.joining(" + "));
    Logger.recordOutput(m_key + "/Sequence", m_sequenceDescription);

    AutoRoutine routine = m_routineFactory.get();
    m_generatedRoutine = routine;

    if (selection.isEmpty()) {
      // Still a routine, so that a chooser of whole autos always has one to take, and so that
      // picking nothing is an auto that does nothing rather than an absent one.
      m_generatedCommand = routine.cmd();
      return;
    }

    List<Command> commands = new ArrayList<>();
    AutoTrajectory startingPath = null;

    for (AutoSegment segment : selection) {
      AutoSegment.Run run = segment.build(routine);
      commands.add(run.command());

      if (startingPath == null && run.entryPath().isPresent()) {
        startingPath = run.entryPath().get();
      }
    }

    final Command resetOdometry = startingPath == null ? null : startingPath.resetOdometry();
    final List<Command> sequence = List.copyOf(commands);

    Command assembled = Command.noRequirements(coroutine -> {
      if (resetOdometry != null) {
        coroutine.await(resetOdometry);
      }
      for (Command command : sequence) {
        coroutine.await(command);
      }
    }).named(m_sequenceDescription);

    routine.active().onTrue(assembled);
    m_generatedCommand = routine.cmd();
  }

  private static Command doNothing(String name) {
    return Command.noRequirements(coroutine -> {
    }).named(name);
  }

}
