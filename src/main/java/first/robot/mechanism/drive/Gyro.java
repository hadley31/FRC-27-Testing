package first.robot.mechanism.drive;

import org.wpilib.math.geometry.Rotation2d;

import first.lib.mechanism.LoggedComponent;

/**
 * The drivetrain's gyro, wrapped as its own logged component so that {@link Drive} reads heading the
 * same way it reads a module: out of replayable inputs rather than off the hardware.
 *
 * <p>Logged as {@code Drive/Gyro}, which is the drivetrain's name and this class's own rather than a
 * path written down here; see {@link LoggedComponent#getLogPrefix()}.
 */
public class Gyro implements LoggedComponent<GyroIO, GyroIOInputsAutoLogged> {
  private final Drive m_drive;
  private final GyroIO m_io;
  private final GyroIOInputsAutoLogged m_inputs = new GyroIOInputsAutoLogged();

  /**
   * @param drive the drivetrain this gyro belongs to, which is what its inputs are logged beneath.
   *              A gyro is constructed by its drivetrain for this reason.
   * @param io    how to read the gyro
   */
  public Gyro(Drive drive, GyroIO io) {
    m_drive = drive;
    m_io = io;
  }

  @Override
  public GyroIO getIO() {
    return m_io;
  }

  @Override
  public GyroIOInputsAutoLogged getInputs() {
    return m_inputs;
  }

  @Override
  public Drive getMechanism() {
    return m_drive;
  }

  /**
   * Whether the gyro answered this cycle. When false its readings are stale and every consumer
   * should ignore them rather than trusting a frozen heading.
   */
  public boolean isConnected() {
    return m_inputs.connected;
  }

  public Rotation2d getYaw() {
    return new Rotation2d(m_inputs.yaw);
  }

  public Rotation2d getPitch() {
    return new Rotation2d(m_inputs.pitch);
  }

  public Rotation2d getRoll() {
    return new Rotation2d(m_inputs.roll);
  }

  /** The yaw samples taken since the last cycle, one per odometry sample, oldest first. */
  public Rotation2d[] getOdometryYawPositions() {
    return m_inputs.odometryYawPositions;
  }
}
