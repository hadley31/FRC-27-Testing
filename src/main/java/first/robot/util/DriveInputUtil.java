package first.robot.util;
import java.util.function.Supplier;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

public class DriveInputUtil {
    public static double applyControllerStickMapping(double value, double deadband) {
        return MathUtil.applyDeadband(-value * Math.abs(value), deadband);
    }

    public static Supplier<ChassisVelocities> getChassisVelocitiesSupplier(Supplier<Double> forwardSupplier, Supplier<Double> leftSupplier, Supplier<Double> rotateSupplier) {
        return () -> new ChassisVelocities(
            Constants.kMaxDriveLinearVelocity.times(forwardSupplier.get()),
            Constants.kMaxDriveLinearVelocity.times(leftSupplier.get()),
            Constants.kMaxDriveAngularVelocity.times(rotateSupplier.get())
        );
    }
}
