package first.robot.mechanism.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Newtons;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Force;

import first.lib.mechanism.LoggedComponent;

/**
 * One corner of the drivetrain, holding the module's inputs and the small amount of geometry that
 * turns them into the kinematics types {@link Drive} works in.
 *
 * <p>Logged as {@code Drive/<name>}: unlike most components, a module cannot take its log name from
 * its class, because all four are this class and would share a key. The corner's name is supplied
 * at construction instead and returned from {@link #getName()}.
 */
public class SwerveModule implements LoggedComponent<SwerveModuleIO, SwerveModuleIOInputsAutoLogged> {
  private final Drive m_drive;
  private final SwerveModuleIO m_io;
  private final SwerveModuleIOInputsAutoLogged m_inputs = new SwerveModuleIOInputsAutoLogged();
  private final String m_name;

  /**
   * @param drive the drivetrain this module belongs to, which is what its inputs are logged
   *              beneath. A module is constructed by its drivetrain for this reason.
   * @param io    how to read and command this corner
   * @param name  which corner this is, as it appears in the log beneath the drivetrain -- and so
   *              what selects this module's recorded inputs during replay. Must be distinct from
   *              the other modules'.
   */
  public SwerveModule(Drive drive, SwerveModuleIO io, String name) {
    m_drive = drive;
    m_io = io;
    m_name = name;
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
  public Drive getMechanism() {
    return m_drive;
  }

  @Override
  public String getName() {
    return m_name;
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
    return setVelocity(setpoint, Translation2d.ZERO);
  }

  /**
   * As {@link #setVelocity(SwerveModuleVelocity)}, and additionally feeds forward the force this
   * corner of the robot is supposed to be pushing with.
   *
   * @param setpoint           where to point and how fast to spin
   * @param robotRelativeForce the force this module should be exerting on the robot, as a
   *                           robot-relative vector in newtons. A trajectory supplies these; zero is
   *                           the right value when nothing does.
   * @return the setpoint that was actually applied, which is what belongs in the log
   */
  public SwerveModuleVelocity setVelocity(
      SwerveModuleVelocity setpoint, Translation2d robotRelativeForce) {
    Rotation2d angle = getAngle();
    SwerveModuleVelocity applied = setpoint.optimize(angle).cosineScale(angle);

    // Only the component along the wheel's rolling direction can be produced by driving the wheel.
    // Whatever is left over is held by the tyre gripping sideways and is no business of this motor,
    // which is why a module being pushed around a corner correctly asks for almost nothing: its
    // force is nearly all perpendicular to the way it rolls.
    //
    // Projected onto where the azimuth actually is rather than where it is being sent, for two
    // reasons. It is the honest answer about what this wheel can presently exert, and it is what
    // makes the sign come out right after the setpoint has been optimized: a module the optimizer
    // reversed gets a negated velocity, and projecting onto the unchanged physical angle negates
    // the force to match.
    Force traction = Newtons.of(robotRelativeForce.rotateBy(angle.unaryMinus()).getX());

    m_io.setDriveVelocity(MetersPerSecond.of(applied.velocity), traction);
    m_io.setSteerPosition(applied.angle);

    Logger.recordOutput(getLogPrefix() + "/DriveFeedforwardForce", traction);

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
