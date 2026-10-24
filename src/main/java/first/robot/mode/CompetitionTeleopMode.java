package first.robot.mode;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;
import first.robot.command.RobotCommandFactory;
import first.robot.mechanism.drive.DriveHeadingController;
import first.robot.oi.CompetitionDriverControls;
import first.robot.util.Constants;
import first.robot.util.SwerveInputStream;

/**
 * The teleoperated driver experience: controller bindings, the drive default command, and the state
 * machine that moves the robot between superstructure modes.
 *
 * <p>Constructed by the framework when the driver station selects this opmode, which is also when
 * the default commands below are registered: everything an opmode wires up is scoped to that
 * opmode, so the scheduler drops it on its own once another opmode is selected.
 *
 * <p>The one thing that scope does not cover is a re-run. The framework closes an opmode when the
 * robot disables and immediately builds a fresh instance under the <i>same</i> opmode id, so a
 * binding made from this constructor is still live when the next instance makes its own. Default
 * commands survive that -- the newest binding for a mechanism wins and the scheduler cancels the
 * rest -- but a second {@code onTrue} on the same button would fire twice, which for a toggle means
 * not at all. So the button bindings are made from inside {@link #m_teleopCommand} instead, where
 * they are scoped to that command's run and dropped with it.
 */
@Teleop(name = "Competition")
public class CompetitionTeleopMode extends PeriodicOpMode {
  private final Robot m_robot;
  private final CompetitionDriverControls m_driverControls;
  private final RobotCommandFactory m_robotCommands;
  private final StateMachine m_stateMachine;
  private final Command m_teleopCommand;

  public CompetitionTeleopMode(Robot robot) {
    m_robot = robot;
    m_driverControls = new CompetitionDriverControls(0);

    SwerveInputStream swerveInputStream = SwerveInputStream.of(
        robot.state::getRobotPose,
        new DriveHeadingController(
            Constants.DriveConstants.kMaxAngularVelocity, "SwerveInputStream/Teleop"),
        m_driverControls::getDriveForward,
        m_driverControls::getDriveLeft,
        m_driverControls::getDriveRotate);

    // Built from the driver's stream, so a command that aims the chassis for the robot still leaves
    // translation under the driver's hand rather than freezing them in place while it turns.
    m_robotCommands = new RobotCommandFactory(robot, swerveInputStream);

    Command joystickDriveCommand = robot.drive.driveCommand(
        swerveInputStream.getNormalDriveSupplier(), "Joystick Drive");

    robot.drive.setDefaultCommand(joystickDriveCommand);
    robot.intakePivot.setDefaultCommand(m_robotCommands.stowIntakePivot());
    robot.intakeRollers.setDefaultCommand(m_robotCommands.haltIntakeRollers());
    robot.feeder.setDefaultCommand(m_robotCommands.haltFeeder());
    robot.turret.setDefaultCommand(m_robotCommands.haltTurret());
    robot.hood.setDefaultCommand(m_robotCommands.stowHood());
    robot.flywheel.setDefaultCommand(m_robotCommands.haltFlywheel());

    m_stateMachine = buildStateMachine();
    m_teleopCommand = buildTeleopCommand();
  }

  /** Binds the driver's buttons and starts the state machine for the length of the enabled run. */
  @Override
  public void start() {
    Scheduler.getDefault().schedule(m_teleopCommand);
  }

  /**
   * Stops driving the robot.
   *
   * <p>The command outlives the opmode's scope -- the next instance is created under the same
   * opmode id -- so it has to be cancelled by hand rather than left to the scheduler.
   */
  @Override
  public void end() {
    Scheduler.getDefault().cancel(m_teleopCommand);
  }

  private Command buildTeleopCommand() {
    return Command.noRequirements(coroutine -> {
      // Bound from inside the command, so the binding is scoped to this run of teleop rather than
      // to the opmode, and is gone by the time the next instance binds the same button.
      //
      // A toggle rather than a hold: switching which joint aims is a decision about the rest of the
      // match, not something to keep a thumb on. Auto-aim reads it every cycle, so it takes effect
      // whether or not the robot is currently shooting.
      m_driverControls.toggleFixedTurretMode().onTrue(toggleFixedTurretMode());

      coroutine.await(m_stateMachine);
    }).named("Competition Teleop");
  }

  private Command toggleFixedTurretMode() {
    return Command.noRequirements(
        coroutine -> m_robot.state.setFixedTurretMode(!m_robot.state.isFixedTurretModeEnabled()))
        .named("Toggle Fixed Turret Mode");
  }

  private StateMachine buildStateMachine() {
    var stateMachine = new StateMachine("Driver Controls State Machine");

    /*
     * DEFINE STATES
     */
    var stowState = stateMachine.addState(m_robotCommands.stowAllMechanisms());
    var intakeState = stateMachine.addState(m_robotCommands.intakeFuel());
    var revFlywheelState = stateMachine.addState(m_robotCommands.autoAim());
    var autoAimAndShootState = stateMachine.addState(m_robotCommands.autoAimAndShoot());

    /*
     * DEFINE TRANSITIONS
    */
    stateMachine.switchFromAny().to(stowState).when(m_driverControls.setModeToStow());
    stowState.switchTo(intakeState).when(m_driverControls.setModeToIntake());
    stowState.switchTo(revFlywheelState).when(m_driverControls.setModeToScore());
    revFlywheelState.switchTo(autoAimAndShootState).when(m_driverControls.setModeToScore());

    /*
     * DEFINE INITIAL STATE
     */
    stateMachine.setInitialState(stowState);

    return stateMachine;
  }
}
