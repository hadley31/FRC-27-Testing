package first.robot.command;

import static org.wpilib.units.Units.RPM;

import org.wpilib.command3.Command;

import first.robot.Robot;
import first.robot.util.FieldConstants;
import first.robot.util.ShotCalculationUtil;
import first.robot.util.ShotParameters;
import first.robot.util.Tuning;

public class RobotCommandFactory {
  private final Robot m_robot;
  private final ShotCalculationUtil m_shotCalculationUtil =
      new ShotCalculationUtil(Tuning.kVelocityCompensation);

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
    // Fetched fresh every solution rather than held in a field: the dashboard can retune this table
    // while the robot is running, and tuning the actuation latency replaces the profile object.
    var profile = Tuning.kScoringShotProfile.profile();

    return m_shotCalculationUtil.calculateShot(
        m_robot.state.getTurretSnapshot(profile.actuationLatency()),
        FieldConstants.getHubPosition3d(),
        profile);
  }
}
