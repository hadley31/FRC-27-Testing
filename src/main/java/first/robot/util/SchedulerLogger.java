package first.robot.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;

import org.littletonrobotics.junction.Logger;

/**
 * Logs command scheduler telemetry through AdvantageKit, so it lands in the log file and is
 * regenerated during replay rather than only being published live to NetworkTables.
 *
 * <p>The scheduler's {@link Scheduler#proto} serializer is deliberately not used: AdvantageScope
 * does not render {@code ProtobufScheduler}, so it would only show up as an opaque blob. Everything
 * the protobuf carries is available from public {@link Scheduler} accessors, so it is flattened here
 * into primitives and string arrays that AdvantageScope can graph and tabulate.
 *
 * <h2>Why both a snapshot and events</h2>
 *
 * <p>State is sampled from the scheduler after each {@link Scheduler#run()} and is treated as ground
 * truth, so no amount of event mishandling can make the logged state drift over a match. But a
 * snapshot alone cannot see a command that schedules, runs and finishes inside a single {@code
 * run()}, and cannot say <i>why</i> a command left the running set.
 *
 * <p>{@link Scheduler#addEventListener} fills both gaps. {@code Mounted} and {@code Yielded} are the
 * events that mean a command body actually executed, so together they mark every tick a command ran
 * -- including single-tick commands the snapshot misses entirely, which show up here as a one-tick
 * pulse on {@code Active}.
 *
 * <p>Timing is latched from events rather than read at termination, because the scheduler drops a
 * command from its running set <i>before</i> emitting {@code Canceled} and {@code
 * CompletedWithError} (though after {@code Completed}), at which point {@link
 * Scheduler#totalRuntimeMs} returns -1 and {@link Scheduler#runId} returns 0. Latching only while
 * the values are still real means terminal events keep the last good numbers.
 */
public class SchedulerLogger {
  private static final String kRoot = "Scheduler";

  private static final Set<Scheduler> m_attached =
      Collections.newSetFromMap(new IdentityHashMap<>());

  /** What the scheduler reported during the current tick. Reset by each {@link #refresh}. */
  private static final class Tick {
    /** Human-readable lifecycle events, in the order they were emitted. */
    final List<String> events = new ArrayList<>();

    /** Commands whose body executed during this tick. */
    final Set<String> ran = new LinkedHashSet<>();

    void clear() {
      events.clear();
      ran.clear();
    }
  }

  /** The last state seen for a command while the scheduler still had real values for it. */
  private record CommandInfo(
      int runId,
      String parent,
      int priority,
      String[] requirements,
      double lastTimeMs,
      double totalTimeMs) {}

  private static final Tick m_tick = new Tick();
  private static final Map<String, CommandInfo> m_latched = new HashMap<>();

  /**
   * Every command key written so far. AdvantageKit's log table retains keys across cycles, so a
   * command that stops running has to be explicitly marked inactive rather than simply omitted.
   * Bounded by the number of distinct command names in the program.
   */
  private static final Set<String> m_knownCommands = new HashSet<>();

  private SchedulerLogger() {}

  /**
   * Records the scheduler's state for this tick. Call once per loop, after {@link Scheduler#run()}.
   *
   * @param scheduler the scheduler to log.
   */
  public static void refresh(Scheduler scheduler) {
    if (m_attached.add(scheduler)) {
      scheduler.addEventListener(event -> onEvent(scheduler, event));
    }

    var running = scheduler.getRunningCommands();
    var queued = scheduler.getQueuedCommands();

    // Live values override whatever the events latched. Sorted ascending so that when two commands
    // share a name the longest-running one is written last and wins, keeping output deterministic
    // for replay comparisons.
    running.stream()
        .sorted(Comparator.comparingDouble(scheduler::totalRuntimeMs))
        .forEach(command -> latch(scheduler, command));

    Logger.recordOutput(kRoot + "/RunTimeMS", scheduler.lastRuntimeMs());
    Logger.recordOutput(kRoot + "/Running", identify(scheduler, running));
    Logger.recordOutput(kRoot + "/Queued", identify(scheduler, queued));
    Logger.recordOutput(kRoot + "/RunningCount", running.size());
    Logger.recordOutput(kRoot + "/QueuedCount", queued.size());
    Logger.recordOutput(kRoot + "/Events", m_tick.events.toArray(String[]::new));

    // Anything that ran this tick, plus anything the scheduler still considers running. The second
    // term should be a subset of the first, and is included so the snapshot stays authoritative.
    Set<String> active = new LinkedHashSet<>(m_tick.ran);
    for (Command command : running) {
      active.add(key(command));
    }

    for (String stale : m_knownCommands) {
      if (!active.contains(stale)) {
        Logger.recordOutput(path(stale) + "/Active", false);
      }
    }

    for (String commandKey : active) {
      m_knownCommands.add(commandKey);
      String path = path(commandKey);
      Logger.recordOutput(path + "/Active", true);

      CommandInfo info = m_latched.get(commandKey);
      if (info != null) {
        Logger.recordOutput(path + "/RunId", info.runId());
        Logger.recordOutput(path + "/Parent", info.parent());
        Logger.recordOutput(path + "/Priority", info.priority());
        Logger.recordOutput(path + "/Requirements", info.requirements());
        Logger.recordOutput(path + "/LastTimeMS", info.lastTimeMs());
        Logger.recordOutput(path + "/TotalTimeMS", info.totalTimeMs());
      }
    }

    m_tick.clear();
  }

  private static void onEvent(Scheduler scheduler, SchedulerEvent event) {
    Command command = commandOf(event);
    if (command != null) {
      if (event instanceof SchedulerEvent.Mounted || event instanceof SchedulerEvent.Yielded) {
        m_tick.ran.add(key(command));
      }
      // A run id of 0 means the scheduler has already dropped the command, so its timing and
      // parentage are no longer available; keep the previously latched values in that case.
      if (scheduler.runId(command) != 0) {
        latch(scheduler, command);
      }
    }

    // Yielded fires for every running command every tick. It drives the "ran this tick" set above,
    // but it is left out of the event list, where it would drown out everything else.
    if (!(event instanceof SchedulerEvent.Yielded)) {
      m_tick.events.add(describe(event));
    }
  }

  private static void latch(Scheduler scheduler, Command command) {
    Command parent = scheduler.getParentOf(command);
    m_latched.put(
        key(command),
        new CommandInfo(
            scheduler.runId(command),
            parent == null ? "" : parent.name(),
            command.priority(),
            command.requirements().stream().map(Mechanism::getName).sorted().toArray(String[]::new),
            scheduler.lastCommandRuntimeMs(command),
            scheduler.totalRuntimeMs(command)));
  }

  private static Command commandOf(SchedulerEvent event) {
    return switch (event) {
      case SchedulerEvent.Scheduled e -> e.command();
      case SchedulerEvent.Mounted e -> e.command();
      case SchedulerEvent.Yielded e -> e.command();
      case SchedulerEvent.Completed e -> e.command();
      case SchedulerEvent.Canceled e -> e.command();
      case SchedulerEvent.CompletedWithError e -> e.command();
      case SchedulerEvent.Interrupted e -> e.command();
      default -> null;
    };
  }

  private static String describe(SchedulerEvent event) {
    return switch (event) {
      case SchedulerEvent.Scheduled e -> "Scheduled: " + e.command().name();
      case SchedulerEvent.Mounted e -> "Mounted: " + e.command().name();
      case SchedulerEvent.Yielded e -> "Yielded: " + e.command().name();
      case SchedulerEvent.Completed e -> "Completed: " + e.command().name();
      case SchedulerEvent.Canceled e -> "Canceled: " + e.command().name();
      case SchedulerEvent.CompletedWithError e ->
        "CompletedWithError: " + e.command().name() + " (" + e.error() + ")";
      case SchedulerEvent.Interrupted e ->
        "Interrupted: " + e.command().name() + " by " + e.interrupter().name();
      // Event types added by later WPILib releases (ForkFailure, as of alpha-7 + 49) land here
      // rather than breaking the build.
      default -> event.getClass().getSimpleName();
    };
  }

  /** Command names paired with their run ids, so repeated names stay distinguishable. */
  private static String[] identify(Scheduler scheduler, Iterable<Command> commands) {
    List<String> names = new ArrayList<>();
    for (Command command : commands) {
      names.add(command.name() + "#" + scheduler.runId(command));
    }
    return names.toArray(String[]::new);
  }

  /** Slashes would split a command name across log table subtables, so they are replaced. */
  private static String key(Command command) {
    return command.name().replace('/', '_');
  }

  private static String path(String commandKey) {
    return kRoot + "/Commands/" + commandKey;
  }
}
