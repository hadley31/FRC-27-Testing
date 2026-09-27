package first.robot.command;

import static org.wpilib.units.Units.RPM;

import org.wpilib.command3.Command;

import first.robot.Robot;
import first.robot.util.FieldConstants;
import first.robot.util.ShotCalculationUtil;
import first.robot.util.ShotParameters;
import first.robot.util.ShotProfile;

public class RobotCommandFactory {
  private static final ShotProfile kShotProfile = ShotProfile.kScoring;

  private final Robot m_robot;

  public RobotCommandFactory(Robot robot) {
    m_robot = robot;
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
            ShotParameters parameters = calculateShot();

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
      m_robot.state.isPreparedToShootTrigger().whileTrue(feedFuel());

      coroutine.await(autoAim());
    }).named("Auto Shoot");
  }

  private ShotParameters calculateShot() {
    return ShotCalculationUtil.calculateShot(
        m_robot.state.getTurretSnapshot(kShotProfile.actuationLatency()),
        FieldConstants.getHubPosition3d(),
        kShotProfile);
  }
}
