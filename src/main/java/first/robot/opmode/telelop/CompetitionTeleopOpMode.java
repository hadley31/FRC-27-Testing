package first.robot.opmode.telelop;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;
import first.robot.command.RobotCommands;
import first.robot.oi.CompetitionDriverControls;
import first.robot.util.CommandUtil;
import first.robot.util.DriveInputUtil;

@Teleop(name = "Competition Teleop")
public class CompetitionTeleopOpMode extends PeriodicOpMode {
  private final CompetitionDriverControls m_driverControls;
  private final Robot m_robot;
  private final RobotCommands m_robotCommands;

  public CompetitionTeleopOpMode(Robot robot) {
    m_driverControls = new CompetitionDriverControls(0);
    m_robot = robot;
    m_robotCommands = new RobotCommands(robot);

    Supplier<ChassisVelocities> chassisVelocitiesSupplier = DriveInputUtil.getChassisVelocitiesSupplier(
        m_driverControls::getDriveForward,
        m_driverControls::getDriveLeft,
        m_driverControls::getDriveRotate);

    Command joystickDriveCommand = robot.drive.driveCommand(chassisVelocitiesSupplier);

    robot.drive.setDefaultCommand(joystickDriveCommand.withTimeout(null));
  }

  @Override
  public void start() {
    var result = Scheduler.getDefault().schedule(getCompetitionStateMachine());

    System.out.println(result.successful());
  }

  private StateMachine getCompetitionStateMachine() {
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
