package first.lib.command3;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;

/**
 * Covers the difference between the two ways a command can start another one on a condition, which
 * is why {@code RobotCommandFactory.autoAimAndShoot} forks its feed instead of binding a trigger to
 * it.
 *
 * <p>A trigger bound from inside a command body schedules against the scheduler's own event loop.
 * That loop is polled outside command execution, so the bound command is queued rather than started,
 * and the binding's scope records the enclosing command as its parent. Lose the enclosing command in
 * the window between the queue and the promotion -- one mechanism conflict is enough -- and the
 * queued command is promoted with a parent that is already gone. The scheduler only looks for
 * orphans among running commands, so nothing cancels it, and it goes on holding its mechanism until
 * the trigger next polls.
 *
 * <p>A forked command is started from inside the body, as a direct child, and is cancelled with its
 * parent. That is the property these tests hold onto.
 */
class CommandParentingTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    DriverStationBackend.refreshData();
  }

  private static Mechanism mechanism(Scheduler scheduler, String name) {
    return new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return scheduler;
      }

      @Override
      public String getName() {
        return name;
      }
    };
  }

  @Test
  void aForkedChildIsCancelledWithTheCommandThatForkedIt() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism chassis = mechanism(scheduler, "Chassis");
    Mechanism feeder = mechanism(scheduler, "Feeder");

    Command feed = feeder.run(coroutine -> coroutine.park()).named("Feed");
    Command shot = chassis.run(coroutine -> {
      coroutine.fork(feed);
      coroutine.park();
    }).named("Shot");

    scheduler.schedule(shot);
    scheduler.run();

    assertTrue(scheduler.isRunning(feed), "the fork should be running alongside its parent");

    // Something else takes the chassis, which is what ends the shot mid-cycle.
    Command interloper = chassis.run(coroutine -> coroutine.park()).named("Interloper");
    scheduler.schedule(interloper);
    scheduler.run();

    assertFalse(scheduler.isRunning(shot), "the shot should have lost the chassis");
    assertFalse(
        scheduler.isRunning(feed),
        "the fork should have gone with its parent rather than keeping the feeder");
  }

  @Test
  void aTriggerBoundInsideACommandCanLeaveItsChildBehind() {
    // The pattern this test exists to warn about, pinned down so the reason for forking does not
    // have to be taken on faith. If a WPILib release makes this pass differently, the fork is still
    // correct; this is the failure it avoids.
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism chassis = mechanism(scheduler, "Chassis");
    Mechanism feeder = mechanism(scheduler, "Feeder");

    boolean[] interlope = {false};
    boolean[] ready = {false};

    Command interloper = chassis.run(coroutine -> coroutine.park()).named("Interloper");
    new Trigger(scheduler, () -> interlope[0]).onTrue(interloper);

    Command feed = feeder.run(coroutine -> coroutine.park()).named("Feed");
    Command shot = chassis.run(coroutine -> {
      new Trigger(scheduler, () -> ready[0]).whileTrue(feed);
      coroutine.park();
    }).named("Shot");

    scheduler.schedule(shot);
    scheduler.run();

    // Both conditions come true in the same cycle: the feed is queued against the shot, and the
    // interloper takes the chassis out from under the shot before the queue is promoted.
    interlope[0] = true;
    ready[0] = true;
    scheduler.run();

    assertFalse(scheduler.isRunning(shot));
    assertTrue(scheduler.isRunning(feed), "the feed outlived the command it was bound inside");

    // The orphan: still holding the feeder, still pointing at a parent that is no longer running.
    // Every later schedule that conflicts with it walks that broken chain.
    assertSame(shot, scheduler.getParentOf(feed));
    assertFalse(scheduler.isRunning(scheduler.getParentOf(feed)));
  }
}
