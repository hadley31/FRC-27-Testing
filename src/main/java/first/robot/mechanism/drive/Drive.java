package first.robot.mechanism.drive;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Drive implements Mechanism {
    private void drive(ChassisVelocities speeds) {
        // no-op
    }

    public Command driveCommand(Supplier<ChassisVelocities> speeds) {
        return run((c) -> drive(speeds.get())).named("Drive");
    }
}
