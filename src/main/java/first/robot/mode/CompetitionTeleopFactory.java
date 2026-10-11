package first.robot.mode;

import org.wpilib.command3.Command;
import org.wpilib.command3.StateMachine;

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
 * <p>Constructed once from {@link Robot}'s constructor, which is where the default command belongs
 * so that it is registered in commands v3's global binding scope rather than a per-mode one. The
 * state machine itself is handed back through {@link #getTeleopCommand()} and scheduled by
 * {@link Robot#teleopInit()}, so it only runs while the robot is in teleop.
 */
public class CompetitionTeleopFactory {
  private final CompetitionDriverControls m_driverControls;
  private final RobotCommandFactory m_robotCommands;
  private final StateMachine m_stateMachine;

  public CompetitionTeleopFactory(Robot robot) {
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

    // A toggle rather than a hold: switching which joint aims is a decision about the rest of the
    // match, not something to keep a thumb on. Auto-aim reads it every cycle, so it takes effect
    // whether or not the robot is currently shooting.
    m_driverControls.toggleFixedTurretMode()
        .onTrue(Command.noRequirements(
            coroutine -> robot.state.setFixedTurretMode(!robot.state.isFixedTurretModeEnabled()))
            .named("Toggle Fixed Turret Mode"));

    m_stateMachine = buildStateMachine();
  }

  /**
   * Returns the command to run for the duration of teleop.
   *
   * @return the driver controls state machine.
   */
  public Command getTeleopCommand() {
    return m_stateMachine;
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
