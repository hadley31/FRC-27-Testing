package first.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.SimHooks;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

/**
 * Verifies that {@link LoggedOpModeRobot}'s logging hooks bracket a whole loop cycle, including
 * callbacks registered on OpModeRobot's queue after the hooks themselves (which is how the selected
 * opmode's {@code periodic()} is registered).
 */
class LoggedOpModeRobotTest {
  private static final int CYCLES = 6;

  /** Captures every log entry that the logger flushes. */
  private static final class RecordingReceiver implements LogDataReceiver {
    final List<int[]> markers = new CopyOnWriteArrayList<>();

    @Override
    public void putTable(LogTable table) {
      markers.add(new int[] {
          table.get("RealOutputs/Marker/RobotPeriodic", -1),
          table.get("RealOutputs/Marker/LateCallback", -1)
      });
    }
  }

  private static final class TestRobot extends LoggedOpModeRobot {
    private int m_cycle;

    TestRobot(RecordingReceiver receiver) {
      Logger.addDataReceiver(receiver);
      Logger.start();

      // Registered after the hooks, so it gets a higher queue id than either of them -- the same
      // position the selected opmode's periodic() callback ends up in.
      addPeriodic(this::lateCallback, getPeriod());
    }

    @Override
    public void robotPeriodic() {
      m_cycle++;
      Logger.recordOutput("Marker/RobotPeriodic", m_cycle);
    }

    private void lateCallback() {
      Logger.recordOutput("Marker/LateCallback", m_cycle);
    }
  }

  @Test
  void hooksBracketEveryCycle() throws InterruptedException {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    SimHooks.pauseTiming();
    try {
      RecordingReceiver receiver = new RecordingReceiver();
      TestRobot robot = new TestRobot(receiver);

      Thread robotThread = new Thread(robot::startCompetition, "TestRobot");
      robotThread.setDaemon(true);
      robotThread.start();

      // Let the robot thread reach its first notifier alarm before driving the clock.
      Thread.sleep(500);

      for (int i = 0; i < CYCLES; i++) {
        SimHooks.stepTiming(robot.getPeriod());
      }
      Thread.sleep(500);

      robot.endCompetition();
      Logger.end();

      // Drop the entry for the construction cycle, which has no markers yet.
      List<int[]> cycles = new ArrayList<>(receiver.markers);
      cycles.removeIf(marker -> marker[0] < 1);
      assertTrue(cycles.size() >= CYCLES - 1,
          "expected at least " + (CYCLES - 1) + " logged cycles, got " + cycles.size());

      for (int i = 0; i < cycles.size(); i++) {
        int[] marker = cycles.get(i);
        // One entry per cycle, in order: the "before" hook opened a fresh entry each cycle.
        assertEquals(i + 1, marker[0], "robotPeriodic marker for logged cycle " + i);
        // Same cycle number: the "after" hook flushed only once both callbacks had run, so a
        // callback registered behind the hook is still inside the logging window.
        assertEquals(marker[0], marker[1], "late callback landed in the wrong entry at cycle " + i);
      }
    } finally {
      SimHooks.resumeTiming();
    }
  }
}
