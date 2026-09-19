package first.robot.opmode.autonomous;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import first.robot.Robot;
import first.robot.command.RobotCommands;

@Autonomous(name = "Two Piece Auto")
public class TwoPieceAutoOpMode extends PeriodicOpMode {
  private final Robot m_robot;
  private final RobotCommands m_commands;
  private final AutoFactory m_factory;

  private Command m_autoCommand;

  public TwoPieceAutoOpMode(Robot robot) {
    m_robot = robot;
    m_commands = new RobotCommands(robot);
    m_factory = new AutoFactory(
        m_robot.drive::getPose,
        m_robot.drive::resetPose,
        m_robot.drive::followSample,
        false,
        m_robot.drive);
  }

  private AutoRoutine twoPieceRoutine() {
    AutoRoutine routine = m_factory.newRoutine("Two Piece Auto");
    AutoTrajectory grabSecondPiece = routine.trajectory("TwoPieceAuto");

    routine.active().onTrue(
        Command.noRequirements(coroutine -> {
          coroutine.await(grabSecondPiece.resetOdometry());
          coroutine.await(m_commands.autoAimAndShoot().withTimeout(Seconds.of(1.5)));
          coroutine.await(grabSecondPiece.cmd());
          coroutine.await(m_commands.autoAimAndShoot().withTimeout(Seconds.of(1.5)));
        }).named("Two Piece Auto Sequence"));

    return routine;
  }

  @Override
  public void start() {
    m_autoCommand = twoPieceRoutine().cmd();
    Scheduler.getDefault().schedule(m_autoCommand);
  }

  @Override
  public void end() {
    if (m_autoCommand != null) {
      Scheduler.getDefault().cancel(m_autoCommand);
    }
  }
}
