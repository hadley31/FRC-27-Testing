package first.lib.auto;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableOption;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

/**
 * An auto the drive team assembles out of {@link AutoSegment}s, one per slot, from the dashboard.
 *
 * <p>Each slot is published as its own selector, so the dashboard shows a column of drop-downs
 * rather than one list of whole autos. Whatever they add up to is built into a single
 * {@link AutoRoutine}: the segments' trajectories all come from that one routine, which is what
 * lets a segment bind triggers against its own path and still be polled by the auto that contains
 * it.
 *
 * <p>Slots are read in order and {@code None} is skipped, so leaving a hole in the middle closes up
 * rather than ending the auto. The first segment that drives also places the robot: the assembled
 * command resets odometry to that path's start before anything else runs, wherever in the slots it
 * sits.
 *
 * <p>Assembly happens when the selection changes, not when the auto starts -- {@link #poll()}
 * belongs in an opmode's {@code disabledPeriodic()}. Trajectories are read off the disk as the
 * drive team picks, which is both where the time is free and where a path that fails to load still
 * has someone looking at the dashboard. {@link #selectedCommand()} reassembles first if the
 * selection moved since, so what runs is always what the dashboard shows.
 *
 * <p>Segments that do not meet raise an alert naming the seam. The check is a warning rather than a
 * refusal: a gap of a few centimetres between two paths is routine, and the drive team is better
 * placed than this class to decide whether a metre and a half of it was deliberate.
 */
public class SegmentedAutoChooser implements AutoCloseable {
  /** The slot option for "nothing here", which every slot carries and starts on. */
  private static final String kNoneName = "None";

  /** How far apart two paths' ends can be before the seam between them is called out. */
  private static final Distance kSeamPositionTolerance = Meters.of(0.1);

  /** How far out of line two paths' ends can be before the seam between them is called out. */
  private static final Angle kSeamRotationTolerance = Degrees.of(5);

  /**
   * A placeholder for an empty slot. Filtered out before assembly, so its command never runs; it
   * exists so that a slot always selects to something rather than to null.
   */
  private static final AutoSegment kNone = new AutoSegment(
      kNoneName, routine -> AutoSegment.Run.stationary(doNothing(kNoneName)));

  private final String m_key;
  private final Supplier<AutoRoutine> m_routineFactory;
  private final List<Selectable<AutoSegment>> m_slots = new ArrayList<>();
  private final Map<String, AutoSegment> m_catalog = new LinkedHashMap<>();
  private final Alert m_seamAlert;

  private List<String> m_seamWarnings = List.of();
  private List<String> m_namesAtGeneration = null;
  private Optional<Alliance> m_allianceAtGeneration = Optional.empty();
  private Command m_generatedCommand = doNothing("No Auto");
  private String m_sequenceDescription = "Nothing";

  /**
   * @param key            where the slots are published, one table per slot underneath it
   * @param slots          how many segments the drive team can chain. Every slot is published
   *                       whether or not it is used, so this is as much a dashboard layout decision
   *                       as a limit.
   * @param routineFactory builds the empty routine each assembly fills. Called once per assembly,
   *                       since a routine cannot be rebuilt.
   */
  public SegmentedAutoChooser(String key, int slots, Supplier<AutoRoutine> routineFactory) {
    m_key = key;
    m_routineFactory = routineFactory;
    m_catalog.put(kNoneName, kNone);
    // Keyed by the chooser's path, so a robot with more than one of these does not have them
    // fighting over one alert.
    m_seamAlert = new Alert(key + ": auto segments do not meet", "", Level.MEDIUM);

    for (int slot = 0; slot < slots; slot++) {
      Selectable<AutoSegment> selectable = new Selectable<>();
      selectable.addDefault(kNoneName, kNone);
      m_slots.add(selectable);
    }
  }

  /**
   * Adds a segment to every slot's options.
   *
   * @param segment the segment to offer
   * @return this chooser, so calls can be chained
   * @throws IllegalArgumentException if a segment of the same name was already added
   */
  public SegmentedAutoChooser addSegment(AutoSegment segment) {
    if (m_catalog.containsKey(segment.name())) {
      throw new IllegalArgumentException("Duplicate auto segment name: " + segment.name());
    }

    m_catalog.put(segment.name(), segment);
    m_slots.forEach(slot -> slot.add(segment.name(), segment));

    return this;
  }

  /**
   * Sets what the slots hold before anyone touches the dashboard. Slots past the end of the
   * sequence are left on {@code None}.
   *
   * <p>This is how a known-good auto stays one that nobody has to assemble: the defaults are the
   * routine the team would have hard-coded, and the slots are there to depart from it.
   *
   * @param segmentNames the segments to put in the first slots, in order
   * @return this chooser, so calls can be chained
   * @throws IllegalArgumentException if a name was never added, or there are more names than slots
   */
  public SegmentedAutoChooser setDefaultSequence(String... segmentNames) {
    if (segmentNames.length > m_slots.size()) {
      throw new IllegalArgumentException(
          "Default sequence is " + segmentNames.length + " segments long, but there are only "
              + m_slots.size() + " slots");
    }

    for (int slot = 0; slot < segmentNames.length; slot++) {
      if (!m_catalog.containsKey(segmentNames[slot])) {
        throw new IllegalArgumentException("Unknown auto segment name: " + segmentNames[slot]);
      }
      m_slots.get(slot).setDefault(segmentNames[slot]);
    }

    return this;
  }

  /** Publishes every slot, and a read-only readout of what they currently add up to. */
  public void publish() {
    for (int slot = 0; slot < m_slots.size(); slot++) {
      Tunables.publish(slotKey(slot), m_slots.get(slot));
    }

    Tunables.publishValue(
        m_key + "/Sequence",
        () -> m_sequenceDescription,
        null,
        String.class,
        TunableConfig.of(TunableOption.IMMUTABLE));
  }

  /**
   * Takes the slots back off the dashboard and gives up the seam alert's name.
   *
   * <p>Both halves matter to a chooser that lives in an opmode, since the framework builds a new
   * instance under the same key every time the opmode is selected: a publication left behind leaves
   * the registry pointed at dead selectors, and an alert left behind makes the next instance's
   * constructor throw on the duplicate name.
   */
  @Override
  public void close() {
    Tunables.remove(m_key);
    m_seamAlert.close();
  }

  /**
   * Reassembles the auto if the selection has moved since the last one. Cheap when it has not, so
   * it belongs in a periodic callback that runs while the robot is disabled.
   */
  public void poll() {
    if (isStale()) {
      regenerate();
    }
  }

  /**
   * The assembled auto, reassembled first if the selection moved since the last {@link #poll()}.
   *
   * @return the command to schedule when autonomous starts.
   */
  public Command selectedCommand() {
    if (isStale()) {
      regenerate();
    }

    return m_generatedCommand;
  }

  /**
   * What the slots currently add up to, as the dashboard readout shows it.
   *
   * @return the assembled segment names in order.
   */
  public String getSequenceDescription() {
    return m_sequenceDescription;
  }

  /**
   * What is wrong with the seams of the current assembly, in the words the alert carries.
   *
   * @return one entry per pair of consecutive segments that do not meet, empty when they all do.
   */
  public List<String> getSeamWarnings() {
    return m_seamWarnings;
  }

  private String slotKey(int slot) {
    return m_key + "/Segment " + (slot + 1);
  }

  private List<AutoSegment> selectedSegments() {
    return m_slots.stream()
        .map(Selectable::getSelected)
        .filter(segment -> segment != null && segment != kNone)
        .toList();
  }

  private boolean isStale() {
    if (m_namesAtGeneration == null) {
      return true;
    }

    // The alliance is part of the selection, not just an input to it: a routine's trajectories are
    // flipped around the field it was built for.
    if (!m_allianceAtGeneration.equals(MatchState.getAlliance())) {
      return true;
    }

    return !m_namesAtGeneration.equals(selectedSegments().stream().map(AutoSegment::name).toList());
  }

  private void regenerate() {
    List<AutoSegment> selection = selectedSegments();
    m_namesAtGeneration = selection.stream().map(AutoSegment::name).toList();
    m_allianceAtGeneration = MatchState.getAlliance();
    m_sequenceDescription = m_namesAtGeneration.isEmpty()
        ? "Nothing"
        : String.join(" + ", m_namesAtGeneration);

    if (selection.isEmpty()) {
      m_generatedCommand = doNothing("No Auto");
      m_seamWarnings = List.of();
      m_seamAlert.set(false);
      return;
    }

    AutoRoutine routine = m_routineFactory.get();
    List<Command> commands = new ArrayList<>();
    List<String> seams = new ArrayList<>();
    AutoTrajectory startingPath = null;
    AutoTrajectory previousExit = null;
    String previousName = null;

    for (AutoSegment segment : selection) {
      AutoSegment.Run run = segment.build(routine);
      commands.add(run.command());

      if (run.entryPath().isEmpty()) {
        // A segment that does not drive cannot be entered from the wrong place, and leaves the
        // robot wherever the last driving segment left it.
        continue;
      }

      AutoTrajectory entry = run.entryPath().get();
      if (previousExit == null) {
        startingPath = entry;
      } else {
        checkSeam(previousName, previousExit, segment.name(), entry, seams);
      }

      previousExit = run.exitPath().orElse(entry);
      previousName = segment.name();
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

    m_seamWarnings = List.copyOf(seams);

    if (seams.isEmpty()) {
      m_seamAlert.set(false);
    } else {
      m_seamAlert.setText("Auto segments do not meet: " + String.join("; ", seams));
      m_seamAlert.set(true);
    }
  }

  private static void checkSeam(
      String fromName,
      AutoTrajectory from,
      String toName,
      AutoTrajectory to,
      List<String> seams) {
    Optional<Pose2d> end = from.getFinalPose();
    Optional<Pose2d> start = to.getInitialPose();

    // Either pose is absent when the alliance is unknown or the trajectory failed to load. Neither
    // is this check's to report: the first fixes itself when the driver station connects, and
    // Choreo already alerts on the second.
    if (end.isEmpty() || start.isEmpty()) {
      return;
    }

    double gapMeters = end.get().getTranslation().getDistance(start.get().getTranslation());
    double turnDegrees = Math.abs(
        end.get().getRotation().minus(start.get().getRotation()).getDegrees());

    if (gapMeters > kSeamPositionTolerance.in(Meters)
        || turnDegrees > kSeamRotationTolerance.in(Degrees)) {
      seams.add(String.format(
          "'%s' leaves the robot %.2f m and %.0f deg from where '%s' starts",
          fromName, gapMeters, turnDegrees, toName));
    }
  }

  private static Command doNothing(String name) {
    return Command.noRequirements(coroutine -> {
    }).named(name);
  }
}
