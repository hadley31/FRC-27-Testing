package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

/**
 * Verifies that {@link SchedulerLogger} captures a command that schedules, runs and finishes inside
 * a single {@link Scheduler#run()} -- which the scheduler's state snapshot cannot see at all.
 */
class SchedulerLoggerTest {
  private static final String kPrefix = "RealOutputs/Scheduler";

  /** Captures the log entry flushed at the end of each simulated tick. */
  private static final class RecordingReceiver implements LogDataReceiver {
    final List<LogTable> entries = new ArrayList<>();

    @Override
    public void putTable(LogTable table) {
      entries.add(LogTable.clone(table));
    }
  }

  private static boolean active(LogTable entry, String command) {
    return entry.get(kPrefix + "/Commands/" + command + "/Active", false);
  }

  private static List<String> events(LogTable entry) {
    return Arrays.asList(entry.get(kPrefix + "/Events", new String[0]));
  }

  private static List<String> running(LogTable entry) {
    return Arrays.asList(entry.get(kPrefix + "/Running", new String[0]));
  }

  @Test
  void singleTickCommandIsVisibleEvenThoughTheSnapshotMissesIt() {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    // AdvantageKit's conduit reads system telemetry over the system server's NetworkTables
    // instance, and segfaults if it is touched before the driver station has been refreshed once.
    // RobotBase.startRobot does this before constructing the robot; a test has to do it itself.
    DriverStationBackend.refreshData();

    RecordingReceiver receiver = new RecordingReceiver();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.addDataReceiver(receiver);
    Logger.start();

    try {
      Scheduler scheduler = Scheduler.createIndependentScheduler();

      // Never calls yield(), so it mounts, runs and completes inside one run() call.
      Command oneShot = Command.noRequirements(coroutine -> {}).named("OneShot");

      // Yields twice, so it survives across ticks and the snapshot can see it.
      Command multiTick = Command.noRequirements(coroutine -> {
        coroutine.yield();
        coroutine.yield();
      }).named("MultiTick");

      tick(scheduler); // baseline, nothing scheduled

      scheduler.schedule(oneShot);
      scheduler.schedule(multiTick);
      tick(scheduler); // the tick both commands are scheduled in

      tick(scheduler);
      tick(scheduler);
      tick(scheduler); // let MultiTick finish and settle
    } finally {
      // Entries are handed to receivers on a separate thread; end() joins it, so every entry has
      // been delivered by the time the assertions below run.
      Logger.end();
    }

    List<LogTable> entries = receiver.entries;
    int index = -1;
    for (int i = 0; i < entries.size(); i++) {
      if (events(entries.get(i)).contains("Scheduled: OneShot")) {
        index = i;
        break;
      }
    }
    assertTrue(index >= 0, "no entry recorded the one-shot being scheduled");
    assertTrue(index + 1 < entries.size(), "no entry after the one-shot's tick");

    LogTable ran = entries.get(index);
    LogTable after = entries.get(index + 1);

    // The one-shot's whole lifecycle is in the events...
    assertTrue(events(ran).contains("Mounted: OneShot"), events(ran).toString());
    assertTrue(events(ran).contains("Completed: OneShot"), events(ran).toString());

    // ...and it is genuinely absent from the state snapshot, which is why the events are needed.
    assertTrue(running(ran).stream().noneMatch(name -> name.startsWith("OneShot")),
        "snapshot unexpectedly saw the one-shot: " + running(ran));

    // The events still give it a one-tick Active pulse, which then falls back to false.
    assertTrue(active(ran, "OneShot"), "one-shot was not marked active in its tick");
    assertFalse(active(after, "OneShot"), "one-shot stayed active after its tick");

    // The multi-tick command is visible to the snapshot, as a contrast.
    assertTrue(active(ran, "MultiTick"), "multi-tick command was not active");
    assertTrue(running(ran).stream().anyMatch(name -> name.startsWith("MultiTick")),
        "snapshot did not see the multi-tick command: " + running(ran));
    assertFalse(active(entries.get(entries.size() - 1), "MultiTick"),
        "multi-tick command stayed active after finishing");
  }

  /** Runs one simulated robot cycle, bracketed the way LoggedOpModeRobot brackets a real one. */
  private static void tick(Scheduler scheduler) {
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    scheduler.run();
    SchedulerLogger.refresh(scheduler);
    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
  }
}
