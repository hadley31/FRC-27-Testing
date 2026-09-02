package first.robot.opmode.telelop;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;
import first.robot.oi.CompetitionDriverControls;
import first.robot.util.DriveInputUtil;

@Teleop
public class CompetitionTeleopOpMode extends PeriodicOpMode {
    private final CompetitionDriverControls m_driverControls;

    public CompetitionTeleopOpMode(Robot robot) {
        m_driverControls = new CompetitionDriverControls(0);

        Supplier<ChassisVelocities> chassisVelocitiesSupplier = DriveInputUtil.getChassisVelocitiesSupplier(
            m_driverControls::getDriveForward,
            m_driverControls::getDriveLeft,
            m_driverControls::getDriveRotate
        );

        Command joystickDriveCommand = robot.drive.driveCommand(chassisVelocitiesSupplier);

        robot.drive.setDefaultCommand(joystickDriveCommand);
    }
}
