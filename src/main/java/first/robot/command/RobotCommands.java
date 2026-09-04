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
  private final ShotCalculationUtil m_shotCalculationUtil = new ShotCalculationUtil();

  public RobotCommands(Robot robot) {
    m_robot = robot;
  }

  public Command feedFuel() {
    return m_robot.feeder.setTargetCommand(RPM.of(1000));
  }

  public Command autoShoot() {
    return Command
        .requiring(m_robot.flywheel, m_robot.turret, m_robot.hood)
        .executing(coroutine -> {
          Trigger shouldFeedFuel = autoShootShouldFeedFuel();

          shouldFeedFuel.whileTrue(feedFuel());

          while (true) {
            var parameters = m_shotCalculationUtil.calculateShot();

            m_robot.flywheel.setTarget(parameters.m_targetFlywheelSpeed());
            m_robot.turret.setTarget(parameters.m_targetTurretAngle());
            m_robot.hood.setTarget(parameters.m_targetHoodAngle());

            coroutine.yield();
          }
        })
        .named("Auto Shoot");
  }

  private Trigger autoShootShouldFeedFuel() {
    return new Trigger(() -> {
      return m_robot.turret.isNearTarget()
          && m_robot.flywheel.isNearTarget()
          && m_robot.hood.isNearTarget();
    }).debounce(Seconds.of(0.1), DebounceType.FALLING);
  }
}
