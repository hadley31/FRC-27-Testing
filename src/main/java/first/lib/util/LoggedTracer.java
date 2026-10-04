// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.lib.util;

import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Time;

/**
 * Utility class for logging code execution times.
 *
 * <p>Two ways of measuring are on offer. A span -- {@link #startTrace}/{@link #endTrace}, or
 * {@link #traced} when the work fits in a lambda -- times a bracketed piece of work and nests
 * inside whatever span it was opened within, so the durations come out of the log shaped like the
 * call tree that produced them. {@link #lap} instead carves a stretch of straight-line code into
 * consecutive stretches, each measured from where the last one ended.
 *
 * <p>Everything is logged beneath {@code LoggedTracer/}, and spans and laps measured on different
 * threads never mix: the nesting is per-thread, because that is what a call stack is.
 */
public class LoggedTracer {
  private LoggedTracer() {
  }

  private static Time startTime = Seconds.of(-1);

  /**
   * One span that has been started but not yet ended.
   *
   * @param name the name passed to {@link #startTrace(String)}
   * @param path that name joined to the names of every span enclosing it, which is what the
   *        duration is logged under
   * @param startTime monotonic time at which the span was started
   */
  private record Trace(String name, String path, Time startTime) {
  }

  /**
   * The spans open on the calling thread, innermost first.
   *
   * <p>A stack rather than a flat map is what produces the nested log keys: a span's path is built
   * from everything already on the stack when it starts, so a span opened inside {@code Drive}
   * lands under {@code LoggedTracer/Drive/...} and is grouped with its parent in the log viewer. It
   * also makes the pairing checkable -- the span being ended should be the one on top -- which a
   * map keyed by name cannot see.
   *
   * <p>Per-thread because traces can be started from the odometry thread as well as the main loop,
   * and those are separate call stacks: nesting one inside the other would be a lie, and sharing
   * one stack between them would need a lock on every start and end. Each thread therefore also
   * reconciles its own leftovers; see {@link #clearTraces()}.
   */
  private static final ThreadLocal<Deque<Trace>> activeTraces = ThreadLocal.withInitial(ArrayDeque::new);

  /** Reset the clock and drop any spans still open on this thread. */
  public static void reset() {
    activeTraces.get().clear();
    startTime = RobotController.getMeasureMonotonicTime();
  }

  /**
   * Save the time elapsed since the last {@link #reset} or lap, as
   * {@code LoggedTracer/<path>MS}, and start the next stretch here.
   *
   * <p>For splitting straight-line code into consecutive stretches, where a span per stretch would
   * only be a start immediately following an end. Unlike a span, the clock is shared and global:
   * one lap's measurement begins wherever the previous one left off, so a lap on another thread
   * will cut this one short. The path is prefixed by any spans open on this thread, so lapping
   * inside a span files the stretches beneath it.
   */
  public static void lap(String epochName) {
    Time now = RobotController.getMeasureMonotonicTime();
    Logger.recordOutput("LoggedTracer/" + pathOf(epochName) + "MS",
        now.minus(startTime).in(Milliseconds));
    startTime = now;
  }

  /**
   * Times {@code runnable} as a span named {@code epochName}, which is
   * {@link #startTrace}/{@link #endTrace} around work that fits in a lambda and cannot then be
   * returned out of or left open by accident.
   *
   * <p>No {@code finally}: a throw propagates with the span still open, and the span is reported by
   * {@link #clearTraces()} at the end of the loop rather than logging a duration that stopped at an
   * exception.
   */
  public static void traced(String epochName, Runnable runnable) {
    startTrace(epochName);
    runnable.run();
    endTrace(epochName);
  }

  /**
   * Begin timing a named span. Pair with {@link #endTrace(String)}.
   *
   * <p>The span nests inside whichever spans are already open on this thread, so its duration is
   * logged under their names; starting {@code "Odometry"} inside {@code "Drive"} logs to
   * {@code LoggedTracer/Drive/Odometry/ElapsedTimeMS}.
   *
   * <p>An early return or a thrown exception between the two calls leaves the span open;
   * {@link #endTrace(String)} and {@link #clearTraces()} report that rather than letting it corrupt
   * another span's measurement.
   */
  public static void startTrace(String name) {
    Deque<Trace> traces = activeTraces.get();
    traces.push(new Trace(name, pathOf(name), RobotController.getMeasureMonotonicTime()));
  }

  /**
   * Finish timing a named span and log its duration to
   * {@code LoggedTracer/<path>/ElapsedTimeMS}, where the path is the span's name prefixed by the
   * names of the spans enclosing it.
   *
   * <p>A mismatch degrades a measurement rather than the robot. Ending a name that is not open on
   * this thread logs nothing and records the name under {@code LoggedTracer/UnstartedTraces}.
   * Ending an outer span while inner ones are still open closes the outer one anyway -- so one
   * leak does not cascade into every enclosing span -- and records the abandoned inner paths under
   * {@code LoggedTracer/MismatchedTraces}.
   */
  public static void endTrace(String name) {
    Time now = RobotController.getMeasureMonotonicTime();
    Deque<Trace> traces = activeTraces.get();

    // Walking the stack rather than only checking the top lets an out-of-order end still close the
    // span it names, which is what makes a single leak recoverable.
    if (traces.stream().noneMatch(trace -> trace.name().equals(name))) {
      Logger.recordOutput("LoggedTracer/UnstartedTraces", name);
      return;
    }

    List<String> abandoned = new ArrayList<>();
    while (!traces.peek().name().equals(name)) {
      abandoned.add(traces.pop().path());
    }
    Trace trace = traces.pop();

    if (!abandoned.isEmpty()) {
      Logger.recordOutput("LoggedTracer/MismatchedTraces", abandoned.toArray(String[]::new));
      DriverStationErrors.reportWarning(
          "Traces still open when " + trace.path() + " ended: " + abandoned, false);
    }

    Logger.recordOutput("LoggedTracer/" + trace.path() + "/ElapsedTimeMS",
        now.minus(trace.startTime()).in(Milliseconds));
  }

  /**
   * The paths of the spans open on this thread, outermost first. Empty between loops.
   *
   * @return the active traces
   */
  public static List<String> getActiveTraces() {
    Deque<Trace> traces = activeTraces.get();
    List<String> paths = new ArrayList<>(traces.size());
    traces.forEach(trace -> paths.add(0, trace.path()));
    return paths;
  }

  /**
   * Drop every span still open on this thread and report them.
   *
   * <p>A span left open means its {@code endTrace} never ran, which would otherwise leak a stack
   * entry and nest everything measured afterwards underneath it. Call this once per loop, from the
   * thread that traces, after everything that traces has run: the leftover paths go to
   * {@code LoggedTracer/UnfinishedTraces}, empty when all is well.
   *
   * @return the paths that were still open, innermost first
   */
  public static Set<String> clearTraces() {
    // Copy before resetting: reset() clears the same stack these paths are read from.
    Set<String> unfinished = new LinkedHashSet<>();
    activeTraces.get().forEach(trace -> unfinished.add(trace.path()));
    reset();
    Logger.recordOutput("LoggedTracer/UnfinishedTraces", unfinished.toArray(String[]::new));
    if (!unfinished.isEmpty()) {
      DriverStationErrors.reportWarning("Unfinished traces: " + unfinished, false);
    }
    return unfinished;
  }

  /** {@code name} prefixed by the paths of the spans already open on this thread. */
  private static String pathOf(String name) {
    Trace parent = activeTraces.get().peek();
    return parent == null ? name : parent.path() + "/" + name;
  }
}
