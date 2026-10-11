package first.lib.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.Tunables;

/**
 * Pins the contract a dashboard reads a command through, because nothing in commands v3 enforces it:
 * the framework has no dashboard integration of its own, so the topic names and the {@code "Command"}
 * type string below are the whole reason a button appears in Glass rather than a bare checkbox.
 *
 * <p>Runs against a private NetworkTables instance with the real tunable backend, so these are the
 * topics the robot would actually publish.
 */
class TunableCommandTest {
  private static final String kPath = "/Tunables/Tuning/Thing";

  private NetworkTableInstance m_inst;
  private Scheduler m_scheduler;
  private Command m_command;

  @BeforeEach
  void setUp() {
    m_inst = NetworkTableInstance.create();
    TunableRegistry.reset();
    TunableRegistry.registerBackend("", new NetworkTablesTunableBackend(m_inst, "/Tunables"));

    m_scheduler = Scheduler.createIndependentScheduler();
    // Requires nothing, so the scheduler will not cancel it for wanting a mechanism that cannot be
    // controlled while disabled -- the driver station is not simulated here.
    m_command = Command.noRequirements(Coroutine::park).named("Tune Thing");

    Tunables.publish("Tuning/Thing", new TunableCommand(m_command, m_scheduler));
  }

  @AfterEach
  void tearDown() {
    TunableRegistry.reset();
    m_inst.close();
  }

  @Test
  void thePublishedShapeIsTheOneGlassLooksForInACommand() {
    assertEquals("Command", m_inst.getStringTopic(kPath + "/.type").subscribe("").get());
    assertEquals("Tune Thing", m_inst.getStringTopic(kPath + "/name").subscribe("").get());
    assertFalse(m_inst.getBooleanTopic(kPath + "/running").subscribe(true).get());

    // Glass only offers the button if the topic it writes advertises itself as mutable, and only
    // ever reads the name.
    assertEquals("true", m_inst.getTopic(kPath + "/running").getProperty("mutable"));
    assertEquals("false", m_inst.getTopic(kPath + "/name").getProperty("mutable"));
  }

  @Test
  void pressingTheButtonSchedulesTheCommandAndPressingItAgainCancelsIt() {
    press(true);
    assertTrue(m_scheduler.isScheduledOrRunning(m_command), "the command did not get scheduled");

    m_scheduler.run();
    assertTrue(m_scheduler.isRunning(m_command), "the command did not start running");
    assertTrue(reportedRunning(), "the dashboard was not told the command is running");

    press(false);
    assertFalse(m_scheduler.isScheduledOrRunning(m_command), "the command did not get cancelled");
    assertFalse(reportedRunning(), "the dashboard was not told the command had stopped");
  }

  @Test
  void theSameCommandCanBeRunAgainAfterItEnds() {
    press(true);
    m_scheduler.run();
    press(false);

    press(true);
    m_scheduler.run();
    assertTrue(m_scheduler.isRunning(m_command), "the command could not be run a second time");
  }

  /** Writes to the {@code running} topic the way a dashboard does, then lets the robot see it. */
  private void press(boolean running) {
    try (BooleanPublisher publisher = m_inst.getBooleanTopic(kPath + "/running").publish()) {
      publisher.set(running);
      m_inst.flush();
      // What IterativeRobotBase.loopFunc does every cycle: hand incoming values to their setters and
      // publish the current local ones back out.
      TunableRegistry.update();
    }
  }

  /** The value of {@code running} as a dashboard would read it back. */
  private boolean reportedRunning() {
    TunableRegistry.update();
    return m_inst.getBooleanTopic(kPath + "/running").subscribe(false).get();
  }
}
