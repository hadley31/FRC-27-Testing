package first.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

/**
 * Verifies the span API: that durations reach the log under keys that mirror how the spans nest,
 * that an enclosing span is not truncated by the ones inside it, and that an unbalanced or
 * out-of-order call degrades into a report rather than a wrong number.
 */
class LoggedTracerTest {
  private static final String kPrefix = "RealOutputs/LoggedTracer/";

  /** Captures the log entry flushed at the end of each simulated tick. */
  private static final class RecordingReceiver implements LogDataReceiver {
    final List<LogTable> entries = new ArrayList<>();

    @Override
    public void putTable(LogTable table) {
      entries.add(LogTable.clone(table));
    }
  }

  /** Burns enough monotonic time that the measured duration is unambiguously positive. */
  private static void spin() {
    long deadline = System.nanoTime() + 200_000L;
    while (System.nanoTime() < deadline) {
      Thread.onSpinWait();
    }
  }

  /** The duration {@link LoggedTracer#endTrace} logs for a span, by the path it nests at. */
  private static double duration(LogTable entry, String path) {
    return entry.get(kPrefix + path + "/ElapsedTimeMS", -1.0);
  }

  @Test
  void spansAreTimedNestedAndReconciled() {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    // AdvantageKit's conduit reads system telemetry over the system server's NetworkTables
    // instance, and segfaults if it is touched before the driver station has been refreshed once.
    DriverStationBackend.refreshData();

    RecordingReceiver receiver = new RecordingReceiver();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.addDataReceiver(receiver);
    Logger.start();

    try {
      Logger.AdvancedHooks.invokePeriodicBeforeUser();

      // A plain span.
      LoggedTracer.startTrace("Simple");
      spin();
      LoggedTracer.endTrace("Simple");

      // Nested spans: the outer one brackets the inner rather than being truncated by it, and the
      // inner one is logged under the outer one's name.
      LoggedTracer.startTrace("Outer");
      spin();
      LoggedTracer.startTrace("Inner");
      spin();
      assertEquals(List.of("Outer", "Outer/Inner"), LoggedTracer.getActiveTraces());
      LoggedTracer.endTrace("Inner");
      assertEquals(List.of("Outer"), LoggedTracer.getActiveTraces());
      LoggedTracer.endTrace("Outer");
      assertEquals(List.of(), LoggedTracer.getActiveTraces());

      // Ending a name that was never started is reported, not thrown.
      LoggedTracer.endTrace("NeverStarted");

      // Closing an enclosing span while one inside it is still open: the enclosing span is still
      // measured, and the abandoned inner path is reported.
      LoggedTracer.startTrace("Enclosing");
      spin();
      LoggedTracer.startTrace("Abandoned");
      LoggedTracer.endTrace("Enclosing");
      assertEquals(List.of(), LoggedTracer.getActiveTraces(),
          "mismatched end left the stack dirty");

      // A span whose endTrace never runs -- an early return or a throw in real code.
      LoggedTracer.startTrace("Leaked");

      assertEquals(Set.of("Leaked"), LoggedTracer.clearTraces());
      Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
    } finally {
      // Entries are handed to receivers on a separate thread; end() joins it.
      Logger.end();
    }

    assertEquals(1, receiver.entries.size(), "expected exactly one flushed entry");
    LogTable entry = receiver.entries.get(0);

    assertTrue(duration(entry, "Simple") > 0.0, "no duration logged for a plain span");

    // The inner span is logged beneath its parent, not at the top level.
    double inner = duration(entry, "Outer/Inner");
    double outer = duration(entry, "Outer");
    assertTrue(inner > 0.0, "inner span was not logged under its parent");
    assertEquals(-1.0, duration(entry, "Inner"), "inner span was also logged at the top level");
    assertTrue(outer > inner,
        "outer span (" + outer + "ms) did not outlast the inner one (" + inner + "ms)");

    // An unmatched end logs no duration, only its name.
    assertEquals(-1.0, duration(entry, "NeverStarted"), "an unstarted span logged a duration");
    assertEquals("NeverStarted", entry.get(kPrefix + "UnstartedTraces", ""));

    // An out-of-order end still measures the span it names and reports what it discarded.
    assertTrue(duration(entry, "Enclosing") > 0.0, "a span closed out of order was not logged");
    assertEquals(List.of("Enclosing/Abandoned"),
        List.of(entry.get(kPrefix + "MismatchedTraces", new String[0])));

    // A span nothing ever closed is reported rather than silently leaked.
    assertEquals(List.of("Leaked"),
        List.of(entry.get(kPrefix + "UnfinishedTraces", new String[0])));
  }
}
