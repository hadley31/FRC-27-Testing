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
import org.wpilib.units.measure.Time;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import first.lib.auto.AutoSegment;
import first.lib.auto.SegmentedAutoChooser;
import first.robot.Robot;
import first.robot.command.RobotCommandFactory;
import first.robot.mechanism.drive.DriveHeadingController;
import first.robot.mechanism.drive.DrivePathController;
import first.robot.util.Constants;
import first.robot.util.SwerveInputStream;

/**
 * Every piece of autonomous the robot can run, offered to the drive team as segments they chain
 * from the dashboard rather than as whole routines.
 *
 * <p>Each segment is a factory method rather than a field: a Choreo {@link AutoTrajectory} carries
 * per-run state, so one has to be built for the run that is about to start rather than reused, and
 * a segment the drive team picked twice needs two of them. {@link SegmentedAutoChooser} is what
 * calls those methods, once per occurrence, each time the selection changes while the robot is
 * disabled.
 *
 * <p>The slots start on the two-pass auto this class used to run as a single routine, so the
 * out-of-the-box behaviour is unchanged and the slots are there to depart from it.
 *
 * <p>The chooser is published from the constructor and removed in {@link #close()}, because the
 * framework builds a new instance of this class every time the opmode is selected. Leaving the old
 * publication in place would leave the tunable registry pointed at dead selectors: the dashboard
 * would be driving the selection of a chooser that no longer assembles anything, while this
 * instance sat on its defaults.
 */
@Autonomous(name = "Competition")
public class CompetitionAutoMode extends PeriodicOpMode {
  private static final String AUTO_CHOOSER_KEY = "Choosers/Auto";
  private static final String AUTO_TRAJECTORY_LOG_KEY = "RobotState/Odometry/Trajectory/TrajectoryPoses";

  /** How many segments the drive team can chain. Every slot shows on the dashboard. */
  private static final int kSegmentSlots = 6;

  /** How long a segment keeps trying to put its fuel through the hub before giving up on it. */
  private static final Time kShotTimeout = Seconds.of(5);

  private final Robot m_robot;
  private final RobotCommandFactory m_commands;
  private final AutoFactory m_factory;
  private final SegmentedAutoChooser m_chooser;
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

  /**
   * Assembles the auto as the drive team picks it, so the trajectories are read and the segments
   * are checked against each other before the match rather than at the moment auto starts.
   */
  @Override
  public void disabledPeriodic() {
    m_chooser.poll();
  }

  /** Runs whatever the slots add up to when the robot is enabled. */
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

  /** Takes the slots off the dashboard, so the next instance's are what gets published. */
  @Override
  public void close() {
    m_chooser.close();
  }

  /**
   * Builds the chooser over every segment in this class and publishes its slots for the dashboard.
   *
   * @return the chooser, whose {@link SegmentedAutoChooser#selectedCommand()} is what
   *     {@link #start()} schedules.
   */
  private SegmentedAutoChooser createAutoChooser() {
    SegmentedAutoChooser chooser = new SegmentedAutoChooser(
        AUTO_CHOOSER_KEY, kSegmentSlots, () -> m_factory.newRoutine("Segmented Auto"));

    chooser.addSegment(fuelPass("Trench Fuel Pass", "LeftTrenchFuelBumpShoot"));
    chooser.addSegment(fuelPass("Trench Fuel Pass (Second)", "LeftTrenchFuelBumpShootSecondPass"));
    chooser.addSegment(returnToTrench());
    chooser.addSegment(shoot());

    chooser.setDefaultSequence(
        "Trench Fuel Pass",
        "Return To Trench",
        "Trench Fuel Pass (Second)",
        "Return To Trench");

    chooser.publish();

    return chooser;
  }

  /**
   * Starts under the trench, gathers fuel on the way out over the bump, and shoots what it picked
   * up from where the path leaves it.
   *
   * <p>The intake is bound to the path rather than awaited before it, so it comes down as the robot
   * starts moving and goes back up when the path ends; the shot is awaited after, because it is the
   * one part of the segment that has to finish before the next segment may have the chassis.
   *
   * @param name           what the dashboard calls this pass
   * @param trajectoryName the Choreo path it drives
   * @return the segment.
   */
  private AutoSegment fuelPass(String name, String trajectoryName) {
    return new AutoSegment(name, routine -> {
      AutoTrajectory path = routine.trajectory(trajectoryName);
      path.active().onTrue(m_commands.intakeFuel());

      return AutoSegment.Run.driving(path, Command.noRequirements(coroutine -> {
        coroutine.await(path.cmd());
        coroutine.await(m_commands.autoAimAndShoot().withTimeout(kShotTimeout));
      }).named(name));
    });
  }

  /**
   * Drives back over the bump to where a pass starts, stowing on the way.
   *
   * @return the segment.
   */
  private AutoSegment returnToTrench() {
    return new AutoSegment("Return To Trench", routine -> {
      AutoTrajectory path = routine.trajectory("LeftBumpShootToLeftStartTrench");
      path.active().onTrue(m_commands.stowAllMechanisms());

      return AutoSegment.Run.driving(path, path.cmd());
    });
  }

  /**
   * Shoots from wherever the robot is standing, aiming the chassis if the turret cannot get there
   * on its own.
   *
   * <p>Drives nothing, so it can go in any slot -- a preload shot before the first pass, or another
   * attempt after one.
   *
   * @return the segment.
   */
  private AutoSegment shoot() {
    return new AutoSegment(
        "Shoot", routine -> AutoSegment.Run.stationary(
            m_commands.autoAimAndShoot().withTimeout(kShotTimeout)));
  }
}
