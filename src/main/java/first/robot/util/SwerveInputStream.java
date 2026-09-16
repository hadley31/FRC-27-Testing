package first.robot.util;

import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

public class SwerveInputStream {
  private final DoubleSupplier m_forwardSupplier;
  private final DoubleSupplier m_leftSupplier;
  private final DoubleSupplier m_rotateSupplier;

  private SwerveInputStream(DoubleSupplier forwardSupplier, DoubleSupplier leftSupplier,
      DoubleSupplier rotateSupplier) {
    m_forwardSupplier = forwardSupplier;
    m_leftSupplier = leftSupplier;
    m_rotateSupplier = rotateSupplier;
  }

  public static SwerveInputStream of(DoubleSupplier forwardSupplier,
      DoubleSupplier leftSupplier,
      DoubleSupplier rotateSupplier) {
    return new SwerveInputStream(forwardSupplier, leftSupplier, rotateSupplier);
  }

  public static double applyControllerStickMapping(double value, double deadband) {
    return MathUtil.applyDeadband(-value * Math.abs(value), deadband);
  }

  public Supplier<ChassisVelocities> getNormalDriveSupplier() {
    return () -> new ChassisVelocities(
        getForwardVelocitySupplier().get(),
        getLeftVelocitySupplier().get(),
        getRotateVelocitySupplier().get());
  }

  public Supplier<ChassisVelocities> getTargetAngleSupplier(Supplier<Rotation2d> targetAngleSupplier) {
    return () -> new ChassisVelocities(
        getForwardVelocitySupplier().get(),
        getLeftVelocitySupplier().get(),
        RotationsPerSecond.zero());
  }

  private Supplier<LinearVelocity> getForwardVelocitySupplier() {
    return () -> Constants.kMaxDriveLinearVelocity.times(m_forwardSupplier.getAsDouble());
  }

  private Supplier<LinearVelocity> getLeftVelocitySupplier() {
    return () -> Constants.kMaxDriveLinearVelocity.times(m_leftSupplier.getAsDouble());
  }

  private Supplier<AngularVelocity> getRotateVelocitySupplier() {
    return () -> Constants.kMaxDriveAngularVelocity.times(m_rotateSupplier.getAsDouble());
  }
}
