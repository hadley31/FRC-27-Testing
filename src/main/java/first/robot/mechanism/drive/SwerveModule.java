package first.robot.mechanism.drive;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import first.lib.mechanism.LoggedComponent;

/**
 * One corner of the drivetrain, holding the module's inputs and the small amount of geometry that
 * turns them into the kinematics types {@link Drive} works in.
 */
public class SwerveModule implements LoggedComponent<SwerveModuleIO, SwerveModuleIOInputsAutoLogged> {
  private final SwerveModuleIO m_io;
  private final SwerveModuleIOInputsAutoLogged m_inputs = new SwerveModuleIOInputsAutoLogged();
  private final String m_logName;

  public SwerveModule(SwerveModuleIO io, String name) {
    m_io = io;
    m_logName = "Drive/" + name;
  }

  @Override
  public SwerveModuleIO getIO() {
    return m_io;
  }

  @Override
  public SwerveModuleIOInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public String getLogName() {
    return m_logName;
  }

  /**
   * Points the module at {@code setpoint} and spins the wheel at its velocity.
   *
   * <p>The setpoint is reversed when that leaves a shorter turn for the azimuth, and scaled down by
   * how far the azimuth still has to travel, so that a module changing direction does not drag the
   * robot sideways on its way round.
   *
   * @return the setpoint that was actually applied, which is what belongs in the log
   */
  public SwerveModuleVelocity setVelocity(SwerveModuleVelocity setpoint) {
    SwerveModuleVelocity applied = setpoint.optimize(getAngle()).cosineScale(getAngle());

    m_io.setDriveVelocity(MetersPerSecond.of(applied.velocity));
    m_io.setSteerPosition(applied.angle);

    return applied;
  }

  /** Releases both motors to neutral. See {@link SwerveModuleIO#halt()}. */
  public void halt() {
    m_io.halt();
  }

  public Rotation2d getAngle() {
    return m_inputs.steerPosition;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_inputs.drivePosition, getAngle());
  }

  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(m_inputs.driveVelocity, getAngle());
  }

  /** The positions sampled since the last cycle, oldest first. */
  public SwerveModulePosition[] getOdometryPositions() {
    return m_inputs.odometryPositions;
  }

  /** When each of {@link #getOdometryPositions()} was taken, in seconds. */
  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestampSeconds;
  }
}
