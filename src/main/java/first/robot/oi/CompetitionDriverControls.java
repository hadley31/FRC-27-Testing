package first.robot.oi;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandXboxController;

import first.robot.util.Constants;
import first.robot.util.DriveInputUtil;

public class CompetitionDriverControls implements DriverControls {
  private final CommandXboxController m_controller;

  public CompetitionDriverControls(CommandXboxController controller) {
    m_controller = controller;
  }

  public CompetitionDriverControls(int port) {
    this(new CommandXboxController(port));
  }

  public double getDriveForward() {
    return DriveInputUtil.applyControllerStickMapping(m_controller.getLeftY(), Constants.kControllerDeadband);
  }

  public double getDriveLeft() {
    return DriveInputUtil.applyControllerStickMapping(m_controller.getLeftX(), Constants.kControllerDeadband);
  }

  public double getDriveRotate() {
    return DriveInputUtil.applyControllerStickMapping(m_controller.getRightX(), Constants.kControllerDeadband);
  }

  @Override
  public Trigger setModeToScore() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger setModeToStow() {
    return m_controller.leftBumper();
  }
}
