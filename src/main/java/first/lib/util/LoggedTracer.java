// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.lib.util;

import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Seconds;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Time;

/** Utility class for logging code execution times. */
public class LoggedTracer {
  private LoggedTracer() {
  }

  private static Time startTime = Seconds.of(-1);

  /**
   * Monotonic start time, in nanoseconds, of every trace that has been started but not yet ended.
   *
   * <p>Keying by name rather than by a single stopwatch is what makes nesting safe: an inner trace
   * gets its own entry and removing it leaves the outer trace's start time untouched. Concurrent
   * because traces can be started from the odometry thread as well as the main loop.
   */
  private static final Map<String, Time> openTraces = new ConcurrentHashMap<>();

  /** Reset the clock. */
  public static void reset() {
    openTraces.clear();
    startTime = RobotController.getMeasureMonotonicTime();
  }

  /** Save the time elapsed since the last reset or record. */
  public static void record(String epochName) {
    Time now = RobotController.getMeasureMonotonicTime();
    Logger.recordOutput("LoggedTracer/" + epochName + "MS", now.minus(startTime).in(Milliseconds));
    startTime = now;
  }

  /**
   * Begin timing a named span. Pair with {@link #endTrace(String)}.
   *
   * <p>Starting a name that is already open overwrites its start time, so recursive or cross-thread
   * reuse of a single name will not measure what you expect. An early return or a thrown exception
   * between the two calls leaves the span open; {@link #clearTraces()} reports it rather than
   * letting it corrupt the next measurement.
   */
  public static void startTrace(String name) {
    openTraces.put(name, RobotController.getMeasureMonotonicTime());
  }

  /**
   * Finish timing a named span and log its duration to {@code LoggedTracer/<name>MS}.
   *
   * <p>Ending a name that was never started logs nothing and records the name under
   * {@code LoggedTracer/UnstartedTraces} instead of throwing, so a mismatch degrades a single
   * measurement rather than the robot.
   */
  public static void endTrace(String name) {
    Time now = RobotController.getMeasureMonotonicTime();
    Time start = openTraces.remove(name);
    if (start == null) {
      Logger.recordOutput("LoggedTracer/UnstartedTraces", name);
      return;
    }
    Logger.recordOutput("LoggedTracer/" + name + "MS", now.minus(start).in(Milliseconds));
  }

  /**
   * Drop every trace still open and report them.
   *
   * <p>A trace left open means its {@code endTrace} never ran, which would otherwise leak a map
   * entry and make the next run of that name measure from a stale start. Call this once per loop
   * (after everything that traces has run) to both clean up and surface the mistake: the leftover
   * names go to {@code LoggedTracer/UnfinishedTraces}, empty when all is well.
   *
   * @return the names that were still open
   */
  public static Set<String> clearTraces() {
    // Copy before clearing: keySet() is a live view of the map, so the returned set would empty
    // itself out from under the caller.
    Set<String> unfinished = Set.copyOf(openTraces.keySet());
    reset();
    Logger.recordOutput("LoggedTracer/UnfinishedTraces", unfinished.toArray(String[]::new));
    if (!unfinished.isEmpty()) {
      DriverStationErrors.reportWarning("Unfinished traces: " + unfinished, false);
    }
    return unfinished;
  }
}
