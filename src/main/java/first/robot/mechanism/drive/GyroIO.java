package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;

import first.lib.mechanism.LoggedComponentIO;

/**
 * The drivetrain's heading sensor.
 *
 * <p>{@link #updateInputs} is a no-op by default, so a robot without a gyro — or one running in
 * simulation — can pass {@code new GyroIO() {}} and leave {@code connected} false. {@link Drive}
 * then reports no yaw to the pose estimator, which falls back to the heading the wheels imply.
 */
public interface GyroIO extends LoggedComponentIO<GyroIOInputsAutoLogged> {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Angle yaw = Degrees.of(0);
    public Angle pitch = Degrees.of(0);
    public Angle roll = Degrees.of(0);

    /**
     * Yaw as sampled by {@link PhoenixOdometryThread} since the last cycle, oldest first. Entry
     * {@code i} was taken at the same instant as entry {@code i} of every module's samples.
     */
    public Rotation2d[] odometryYawPositions = new Rotation2d[0];
  }

  @Override
  public default void updateInputs(GyroIOInputsAutoLogged inputs) {
  }
}
