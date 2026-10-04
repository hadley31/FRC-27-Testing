package first.robot.mode;

import static org.wpilib.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.tunable.Tunables;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import first.robot.Robot;
import first.robot.command.RobotCommandFactory;
import first.robot.mechanism.drive.DrivePathController;

/**
 * Every autonomous routine the robot can run, sharing one {@link AutoFactory}.
 *
 * <p>Each routine is a factory method rather than a field: a Choreo {@link AutoRoutine} carries
 * per-run state (trigger bindings and trajectory progress), so one has to be built for the run that
 * is about to start rather than reused. {@link #createAutoChooser()} wires the methods into a
 * chooser, which only calls them once the routine is actually selected.
 */
public class CompetitionAutoFactory {
  private static final String AUTO_TRAJECTORY_LOG_KEY = "RobotState/Odometry/Trajectory/TrajectoryPoses";
  private final Robot m_robot;
  private final RobotCommandFactory m_commands;
  private final AutoFactory m_factory;

  public CompetitionAutoFactory(Robot robot) {
    m_robot = robot;
    m_commands = new RobotCommandFactory(robot);

    DrivePathController pathController = new DrivePathController(robot.state::getRobotPose, robot.drive::drive);

    m_factory = new AutoFactory(
        m_robot.state::getRobotPose,
        m_robot.state::resetPose,
        pathController::followSample,
        true,
        m_robot.drive,
        (trajectory, isStart) -> {
          boolean shouldFlip = MatchState.getAlliance().orElse(Alliance.BLUE).equals(Alliance.RED);
          var traj = shouldFlip ? trajectory.flipped() : trajectory;
          if (isStart) {
            Logger.recordOutput(AUTO_TRAJECTORY_LOG_KEY, Pose2d.struct, traj.getPoses());
          } else {
            Logger.recordOutput(AUTO_TRAJECTORY_LOG_KEY, Pose2d.struct, new Pose2d[0]);
          }
        });
  }

  /**
   * Builds a chooser over every routine in this class and publishes it for the dashboard.
   *
   * @return the chooser, whose {@link AutoChooser#selectedCommand()} is what {@link Robot} schedules
   *     at the start of autonomous.
   */
  public AutoChooser createAutoChooser() {
    AutoChooser chooser = new AutoChooser();

    chooser.addRoutine("Two Piece", this::twoPiece);

    Tunables.publish("Choosers/Auto", chooser);

    return chooser;
  }

  /**
   * Shoots the preload, drives out for a second piece, and shoots that.
   *
   * @return a newly built routine.
   */
  public AutoRoutine twoPiece() {
    AutoRoutine routine = m_factory.newRoutine("Two Piece Auto");
    AutoTrajectory grabSecondPiece = routine.trajectory("TwoPieceAuto");

    routine.active().onTrue(
        Command.noRequirements(coroutine -> {
          coroutine.await(grabSecondPiece.resetOdometry());
          coroutine.await(m_commands.autoAimAndShoot().withTimeout(Seconds.of(1.5)));
          coroutine.await(grabSecondPiece.cmd());
          coroutine.await(m_commands.autoAimAndShoot());
        }).named("Two Piece Auto Sequence"));

    return routine;
  }
}
