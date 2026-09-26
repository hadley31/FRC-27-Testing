package first.lib;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Field;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.internal.PeriodicPriorityQueue;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.RobotController;
import org.wpilib.util.UsageReporting;

/**
 * An {@link OpModeRobot} that drives the AdvantageKit logging framework, providing the same
 * functionality that {@code LoggedRobot} provides for {@code IterativeRobotBase} subclasses.
 *
 * <p>Subclasses configure the logger in their constructor exactly as they would with
 * {@code LoggedRobot} (record metadata, add data receivers, set a replay source, then call
 * {@link Logger#start()}), and everything else -- deterministic timestamps, conduit capture,
 * {@code @AutoLogOutput} fields, console capture, alerts and GC stats -- is handled here.
 *
 * <h2>How this works</h2>
 *
 * <p>AdvantageKit needs to bracket <i>every</i> loop cycle: {@code periodicBeforeUser()} latches the
 * cycle timestamp (or, during replay, pulls the next log entry and injects the recorded driver
 * station state), and {@code periodicAfterUser()} captures conduit data and flushes the completed
 * entry to the data receivers. {@code LoggedRobot} gets that by implementing
 * {@code startCompetition()} itself, which {@link OpModeRobot} declares {@code final}.
 *
 * <p>Instead, this class registers the two hooks directly on OpModeRobot's callback queue, offset
 * {@value #HOOK_OFFSET_SECONDS} seconds either side of the main loop. Because the queue only runs
 * callbacks whose expiration time has already passed, each hook lands on its own notifier wake-up:
 * the "before" hook fires, then the next wake-up runs the entire user-code batch
 * ({@code loopFunc()}, the selected opmode's {@code periodic()}, and anything registered through
 * {@link #addPeriodic}), then the "after" hook fires. Ordering is what matters, not the sub-cycle
 * offset -- {@link Logger} latches the timestamp itself.
 *
 * <p>This requires reaching two private {@link OpModeRobot} fields reflectively, because
 * {@link #addPeriodic} offers no offset and ties in the queue are broken by registration order,
 * which always places user-registered callbacks after {@code loopFunc()} and before the selected
 * opmode's {@code periodic()}. If a future WPILib release renames those fields, construction fails
 * with an explanatory error rather than logging silently-misordered data.
 */
public abstract class LoggedOpModeRobot extends OpModeRobot {
  /** Offset of each logging hook from the main loop, in seconds. */
  public static final double HOOK_OFFSET_SECONDS = 500e-6;

  private final PeriodicPriorityQueue m_callbackQueue;
  private final GcStatsCollector m_gcStatsCollector = new GcStatsCollector();
  private final long m_initStartNs = RobotController.getMonotonicTime();

  private boolean m_initCycleClosed;
  private long m_beforeUserStartNs;
  private long m_userCodeStartNs;
  private boolean m_useTiming = true;

  /** Constructor for LoggedOpModeRobot, using the default 20 ms period. */
  protected LoggedOpModeRobot() {
    this(OpModeRobot.DEFAULT_PERIOD);
  }

  /**
   * Constructor for LoggedOpModeRobot.
   *
   * @param period the period at which to run the robot and opmode periodic callbacks, in seconds.
   */
  @SuppressWarnings("this-escape")
  protected LoggedOpModeRobot(double period) {
    super(period);

    m_callbackQueue = readField("m_callbacks", PeriodicPriorityQueue.class);
    long frameworkStartTimeNs = readField("m_startTimeNs", Long.class);

    // Scheduled against OpModeRobot's own start time so the hooks stay phase-locked to loopFunc()
    // even if this constructor (and the subclass constructor) take longer than one period.
    m_callbackQueue.add(new PeriodicPriorityQueue.Callback(
        this::periodicBeforeUser, frameworkStartTimeNs, period, -HOOK_OFFSET_SECONDS));
    m_callbackQueue.add(new PeriodicPriorityQueue.Callback(
        this::periodicAfterUser, frameworkStartTimeNs, period, HOOK_OFFSET_SECONDS));

    // Logger.start() otherwise scans the stack for LoggedRobot and exits the program.
    Logger.AdvancedHooks.disableRobotBaseCheck();

    // Flush whatever is still queued if the program exits or crashes. Both calls are no-ops when
    // the logger was never started or has already been ended.
    Runtime.getRuntime().addShutdownHook(new Thread(() -> {
      Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
      Logger.end();
    }, "AdvantageKit Shutdown"));

    UsageReporting.reportUsage("LoggingFramework", "AdvantageKit");
  }

  /**
   * Sets whether to use standard timing or run as fast as possible. Pass false during replay so the
   * log is consumed at the speed of the CPU rather than in real time. Only supported in simulation,
   * and only before the main loop starts (i.e. from the subclass constructor).
   *
   * <p>{@link OpModeRobot} waits on a HAL notifier that this class cannot replace, so instead of
   * removing the wait this pauses the simulated clock and advances it from a helper thread as
   * quickly as the main loop acknowledges each wake-up.
   *
   * @param useTiming if true, use standard timing. If false, run as fast as possible.
   */
  public void setUseTiming(boolean useTiming) {
    if (useTiming == m_useTiming) {
      return;
    }
    if (useTiming) {
      throw new UnsupportedOperationException("Standard timing cannot be re-enabled once disabled");
    }
    if (isReal()) {
      DriverStationErrors.reportWarning(
          "[AdvantageKit] setUseTiming(false) is only supported in simulation, ignoring.", false);
      return;
    }
    m_useTiming = false;

    SimHooks.pauseTiming();
    Thread clock = new Thread(() -> {
      while (true) {
        // Advances at most one period per call, stopping at each notifier alarm along the way and
        // blocking until the main loop acknowledges it by arming the next one.
        SimHooks.stepTiming(getPeriod());
      }
    }, "AdvantageKit Replay Clock");
    clock.setDaemon(true);
    clock.start();
  }

  /**
   * Returns whether standard timing is in use.
   *
   * @return false if loop cycles are running as fast as possible, true otherwise.
   */
  public boolean getUseTiming() {
    return m_useTiming;
  }

  /** Opens the log entry for the cycle that is about to run. */
  private void periodicBeforeUser() {
    if (!m_initCycleClosed) {
      // Close out the cycle that Logger.start() opened during construction, matching what
      // LoggedRobot does at the top of startCompetition().
      m_initCycleClosed = true;
      AutoLogOutputManager.addObject(this);
      Logger.AdvancedHooks.invokePeriodicAfterUser(
          RobotController.getMonotonicTime() - m_initStartNs, 0);
    }

    m_beforeUserStartNs = RobotController.getMonotonicTime();
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    m_userCodeStartNs = RobotController.getMonotonicTime();
  }

  /** Closes the log entry for the cycle that just ran and hands it to the data receivers. */
  private void periodicAfterUser() {
    if (!m_initCycleClosed) {
      // Depending on how long construction took, this hook can come due before the first "before"
      // hook. There is no cycle to close yet, so skip it; the pair is in phase from here on.
      return;
    }

    long userCodeEndNs = RobotController.getMonotonicTime();
    m_gcStatsCollector.update();
    Logger.AdvancedHooks.invokePeriodicAfterUser(
        userCodeEndNs - m_userCodeStartNs, m_userCodeStartNs - m_beforeUserStartNs);
  }

  private <T> T readField(String name, Class<T> type) {
    try {
      Field field = OpModeRobot.class.getDeclaredField(name);
      field.setAccessible(true);
      return type.cast(field.get(this));
    } catch (ReflectiveOperationException | RuntimeException e) {
      throw new IllegalStateException(
          "LoggedOpModeRobot could not read OpModeRobot." + name + ", which it needs in order to "
              + "bracket each loop cycle with AdvantageKit's logging hooks. This usually means "
              + "WPILib changed OpModeRobot's internals; check LoggedOpModeRobot against the "
              + "current OpModeRobot source.",
          e);
    }
  }

  /** Records time spent in garbage collection, matching LoggedRobot's output. */
  private static final class GcStatsCollector {
    private final List<GarbageCollectorMXBean> m_gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] m_lastTimes = new long[m_gcBeans.size()];
    private final long[] m_lastCounts = new long[m_gcBeans.size()];

    void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < m_gcBeans.size(); i++) {
        long gcTime = m_gcBeans.get(i).getCollectionTime();
        long gcCount = m_gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - m_lastTimes[i];
        accumCounts += gcCount - m_lastCounts[i];

        m_lastTimes[i] = gcTime;
        m_lastCounts[i] = gcCount;
      }

      Logger.recordOutput("LoggedRobot/GCTimeMS", (double) accumTime);
      Logger.recordOutput("LoggedRobot/GCCounts", (double) accumCounts);
    }
  }
}
