package first.robot.command;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;

import first.robot.Robot;
import first.robot.util.ShotCalculationUtil;

public class RobotCommands {
  private final Robot m_robot;
  private final ShotCalculationUtil m_shotCalculationUtil;

  public RobotCommands(Robot robot) {
    m_robot = robot;
    m_shotCalculationUtil = new ShotCalculationUtil(
        null, null, null);
  }

  public Command stowAllMechanisms() {
    // No requirements here: the forked children already claim these mechanisms themselves.
    // Requiring them on the parent too would make the fork below self-conflict and fail.
    return Command.noRequirements(coroutine -> {
      var forkResult = coroutine.fork(
          m_robot.feeder.haltCommand(),
          m_robot.flywheel.haltCommand(),
          m_robot.turret.haltCommand(),
          m_robot.hood.haltCommand());

      if (forkResult.successful()) {
        forkResult.awaitCompletion();
      }
    }).named("Stow All Mechanisms");
  }

  public Command haltAllMechanisms() {
    return Command
        .requiring(m_robot.feeder, m_robot.flywheel, m_robot.turret, m_robot.hood)
        .executing(coroutine -> {
          m_robot.feeder.halt();
          m_robot.flywheel.halt();
          m_robot.turret.halt();
          m_robot.hood.halt();
        })
        .named("Halt All Mechanisms");
  }

  public Command feedFuel() {
    return m_robot.feeder
        .setTargetCommand(RPM.of(1000))
        .whenCanceled(m_robot.feeder::halt)
        .named("Feed Fuel");
  }

  public Command autoAim() {
    return Command
        .requiring(m_robot.flywheel, m_robot.turret, m_robot.hood)
        .executing(coroutine -> {
          while (true) {
            var parameters = m_shotCalculationUtil.calculateShot();

            m_robot.flywheel.setTarget(parameters.targetFlywheelSpeed());
            m_robot.turret.setTarget(parameters.targetTurretAngle());
            m_robot.hood.setTarget(parameters.targetHoodAngle());

            coroutine.yield();
          }
        })
        .named("Auto Aim");
  }

  public Command autoAimAndShoot() {
    return Command.noRequirements(coroutine -> {
      Trigger shouldFeedFuel = autoShootShouldFeedFuel();

      shouldFeedFuel.whileTrue(feedFuel());

      coroutine.await(autoAim());
    }).named("Auto Shoot");
  }

  private Trigger autoShootShouldFeedFuel() {
    return new Trigger(() -> {
      return m_robot.turret.isNearTarget()
          && m_robot.flywheel.isNearTarget()
          && m_robot.hood.isNearTarget();
    }).debounce(Seconds.of(0.1), DebounceType.FALLING);
  }
}
