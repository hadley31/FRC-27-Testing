package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import first.lib.mechanism.LoggedComponentIO;

/**
 * One swerve module: a wheel that spins and an azimuth that points it.
 *
 * <p>Drive quantities are reported and commanded at the wheel's contact patch rather than at the
 * motor, because the wheel radius and gearing belong to whichever hardware this IO wraps.
 * {@link SwerveModule} and {@link Drive} above it then deal only in metres.
 */
public interface SwerveModuleIO extends LoggedComponentIO<SwerveModuleIOInputsAutoLogged> {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public Distance drivePosition = Meters.of(0);
    public LinearVelocity driveVelocity = MetersPerSecond.of(0);
    public Voltage driveAppliedVoltage = Volts.of(0);
    public Current driveCurrent = Amps.of(0);

    public boolean steerConnected = false;
    public boolean steerEncoderConnected = false;
    /** Azimuth angle as the steer controller sees it, which is what closed loop runs against. */
    public Rotation2d steerPosition = Rotation2d.ZERO;
    /** Azimuth angle straight off the absolute encoder, for checking the fused reading against. */
    public Rotation2d steerAbsolutePosition = Rotation2d.ZERO;
    public AngularVelocity steerVelocity = RadiansPerSecond.of(0);
    public Voltage steerAppliedVoltage = Volts.of(0);
    public Current steerCurrent = Amps.of(0);

    /**
     * Wheel travel and azimuth as sampled by {@link PhoenixOdometryThread} since the last cycle,
     * oldest first, alongside the timestamp each sample was taken at.
     *
     * <p>These are plain arrays rather than measures because the logger has no representation for an
     * array of measures. Travel is in metres and timestamps are in seconds on the same clock as
     * {@code Timer.getTimestamp()}.
     */
    public double[] odometryTimestampSeconds = new double[0];

    public SwerveModulePosition[] odometryPositions = new SwerveModulePosition[0];
  }

  /** Runs the wheel at {@code velocity} measured at the contact patch. */
  public void setDriveVelocity(LinearVelocity velocity);

  /** Points the azimuth at {@code position}, taking the shorter way round. */
  public void setSteerPosition(Rotation2d position);

  /**
   * Releases both motors to neutral.
   *
   * <p>Phoenix holds the last control request it was given, so without this a setpoint from the
   * previous enable would be reapplied the instant the robot is enabled again.
   */
  public void halt();
}
