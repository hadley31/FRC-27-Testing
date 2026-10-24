package first.robot.mode;

import static org.wpilib.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.tunable.Tunables;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import first.robot.Robot;
import first.robot.command.RobotCommandFactory;
import first.robot.mechanism.drive.DriveHeadingController;
import first.robot.mechanism.drive.DrivePathController;
import first.robot.util.Constants;
import first.robot.util.SwerveInputStream;

/**
 * Every autonomous routine the robot can run, sharing one {@link AutoFactory}, with a chooser
 * deciding which of them this opmode runs.
 *
 * <p>Each routine is a factory method rather than a field: a Choreo {@link AutoRoutine} carries
 * per-run state (trigger bindings and trajectory progress), so one has to be built for the run that
 * is about to start rather than reused. {@link #createAutoChooser()} wires the methods into a
 * chooser, which only calls them once the routine is actually selected -- and only while the robot
 * is disabled, so the trajectories are read off the disk before the match rather than at the
 * moment auto starts.
 *
 * <p>The chooser is published from the constructor and removed in {@link #close()}, because the
 * framework builds a new instance of this class every time the opmode is selected. Leaving the old
 * publication in place would leave the tunable registry pointed at a dead chooser: the dashboard
 * would be driving the selection of a chooser that no longer runs anything, while this instance sat
 * on its default.
 */
@Autonomous(name = "Competition")
public class CompetitionAutoMode extends PeriodicOpMode {
  private static final String AUTO_CHOOSER_KEY = "Choosers/Auto";
  private static final String AUTO_TRAJECTORY_LOG_KEY = "RobotState/Odometry/Trajectory/TrajectoryPoses";
  private final Robot m_robot;
  private final RobotCommandFactory m_commands;
  private final AutoFactory m_factory;
  private final AutoChooser m_chooser;
  private Command m_autoCommand;

  public CompetitionAutoMode(Robot robot) {
    m_robot = robot;

    // A stream with no sticks behind it: in auto there is no driver to blend translation with, so a
    // command that aims the chassis rotates in place.
    m_commands = new RobotCommandFactory(robot, SwerveInputStream.of(
        robot.state::getRobotPose,
        new DriveHeadingController(
            Constants.DriveConstants.kMaxAngularVelocity, "SwerveInputStream/Auto")));

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

    m_chooser = createAutoChooser();
  }

  /** Runs whichever routine the chooser is holding when the robot is enabled. */
  @Override
  public void start() {
    m_autoCommand = m_chooser.selectedCommand();
    Scheduler.getDefault().schedule(m_autoCommand);
  }

  /**
   * Stops the routine.
   *
   * <p>The command outlives the opmode's scope -- the next instance is created under the same
   * opmode id -- so it has to be cancelled by hand rather than left to the scheduler.
   */
  @Override
  public void end() {
    if (m_autoCommand != null) {
      Scheduler.getDefault().cancel(m_autoCommand);
      m_autoCommand = null;
    }
  }

  /** Takes the chooser off the dashboard, so the next instance's chooser is what gets published. */
  @Override
  public void close() {
    Tunables.remove(AUTO_CHOOSER_KEY);
  }

  /**
   * Builds a chooser over every routine in this class and publishes it for the dashboard.
   *
   * @return the chooser, whose {@link AutoChooser#selectedCommand()} is what {@link #start()}
   *     schedules.
   */
  private AutoChooser createAutoChooser() {
    AutoChooser chooser = new AutoChooser();

    chooser.addRoutine("Left Side Two Pass", this::leftSideTwoPass);

    Tunables.publish(AUTO_CHOOSER_KEY, chooser);

    return chooser;
  }

  /**
   * Shoots the preload, drives out for a second piece, and shoots that.
   *
   * @return a newly built routine.
   */
  public AutoRoutine leftSideTwoPass() {
    AutoRoutine routine = m_factory.newRoutine("Left Trench Two Pass");
    AutoTrajectory leftTrenchFuelBumpShoot = routine.trajectory("LeftTrenchFuelBumpShoot");
    AutoTrajectory leftBumpShootToLeftStartTrench = routine.trajectory("LeftBumpShootToLeftStartTrench");
    AutoTrajectory leftTrenchFuelBumpShootSecondPass = routine.trajectory("LeftTrenchFuelBumpShootSecondPass");

    Command shootCommand = m_commands.autoAimAndShoot().withTimeout(Seconds.of(5));

    leftTrenchFuelBumpShoot.active().onTrue(m_commands.intakeFuel());
    leftTrenchFuelBumpShootSecondPass.active().onTrue(m_commands.intakeFuel());
    leftBumpShootToLeftStartTrench.active().onTrue(m_commands.stowAllMechanisms());

    routine.active().onTrue(
        Command.noRequirements(coroutine -> {
          coroutine.await(leftTrenchFuelBumpShoot.resetOdometry());
          coroutine.await(leftTrenchFuelBumpShoot.cmd());
          coroutine.await(shootCommand);
          coroutine.await(leftBumpShootToLeftStartTrench.cmd());
          coroutine.await(leftTrenchFuelBumpShootSecondPass.cmd());
          coroutine.await(shootCommand);
          coroutine.await(leftBumpShootToLeftStartTrench.cmd());
        }).named("Two Piece Auto Sequence"));

    return routine;
  }
}
