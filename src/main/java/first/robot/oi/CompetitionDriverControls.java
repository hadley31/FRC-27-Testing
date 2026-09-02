package first.robot.oi;

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
}
