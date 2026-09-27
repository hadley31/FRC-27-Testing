package first.robot.mode;

import org.wpilib.command3.Command;
import org.wpilib.command3.StateMachine;

import first.robot.Robot;
import first.robot.command.RobotCommandFactory;
import first.robot.oi.CompetitionDriverControls;
import first.robot.util.CommandUtil;
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
    m_robotCommands = new RobotCommandFactory(robot);

    SwerveInputStream swerveInputStream = SwerveInputStream.of(
        m_driverControls::getDriveForward,
        m_driverControls::getDriveLeft,
        m_driverControls::getDriveRotate);

    Command joystickDriveCommand = robot.drive.driveCommand(swerveInputStream.getNormalDriveSupplier());

    robot.drive.setDefaultCommand(joystickDriveCommand);

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
    var printState = stateMachine.addState(CommandUtil.print("The cake is a lie."));
    var revFlywheelState = stateMachine.addState(m_robotCommands.autoAim());
    var autoAimAndShootState = stateMachine.addState(m_robotCommands.autoAimAndShoot());

    /*
     * DEFINE TRANSITIONS
    */
    stateMachine.switchFromAny().to(stowState).when(m_driverControls.setModeToStow());
    printState.switchTo(stowState).whenComplete();
    stowState.switchTo(revFlywheelState).when(m_driverControls.setModeToScore());
    revFlywheelState.switchTo(autoAimAndShootState).when(m_driverControls.setModeToScore());

    /*
     * DEFINE INITIAL STATE
     */
    stateMachine.setInitialState(printState);

    return stateMachine;
  }
}
