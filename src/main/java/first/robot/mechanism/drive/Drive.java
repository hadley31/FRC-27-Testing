package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Time;

import first.lib.mechanism.LoggedComponent;
import first.lib.mechanism.LoggedMultiComponentMechanism;
import first.lib.util.LoggedTracer;
import first.robot.util.PoseEstimator.OdometryObservation;

/**
 * The swerve drivetrain: four modules, a gyro, and the kinematics that tie a chassis velocity to
 * them.
 *
 * <p>This mechanism deliberately stops at "make the robot move at this velocity, and report what the
 * wheels did". It holds no pose. Odometry samples come out through
 * {@link #getOdometryObservations()} and are handed to the pose estimator from outside, the same way
 * vision observations are, so that the estimator has exactly one owner and the drivetrain has no
 * opinion about where it is. Path following lives in {@link DrivePathController}.
 */
public class Drive implements LoggedMultiComponentMechanism {
  /** The period module setpoints are discretized over, which is the rate the scheduler runs at. */
  static final Time kLoopPeriod = Seconds.of(0.02);

  /** For driving on velocity alone, with no trajectory to say what force that ought to take. */
  private static final Translation2d[] kNoModuleForces = new Translation2d[0];

  private final Gyro m_gyro;
  private final SwerveModule[] m_modules;
  private final List<LoggedComponent<?, ?>> m_ioContainers = new ArrayList<>();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      TunerConstants.kModuleTranslations);

  private List<OdometryObservation> m_odometryObservations = List.of();

  /**
   * Takes the IO for each piece of hardware and builds the components around them here, rather than
   * being handed the components: a component is logged beneath the mechanism that owns it and so has
   * to be told which one that is, which cannot happen before this constructor runs. What varies
   * between a real robot, simulation and replay therefore stops at the IO layer -- see
   * {@code Robot.createGyroIO} and {@code Robot.createSwerveModuleIO} -- and the four corners'
   * names, which are the keys their inputs replay from, are fixed here.
   */
  public Drive(
      GyroIO gyroIO,
      SwerveModuleIO frontLeftIO,
      SwerveModuleIO frontRightIO,
      SwerveModuleIO backLeftIO,
      SwerveModuleIO backRightIO) {
    m_gyro = new Gyro(this, gyroIO);
    m_modules = new SwerveModule[] {
        new SwerveModule(this, frontLeftIO, "FrontLeft"),
        new SwerveModule(this, frontRightIO, "FrontRight"),
        new SwerveModule(this, backLeftIO, "BackLeft"),
        new SwerveModule(this, backRightIO, "BackRight"),
    };

    m_ioContainers.add(m_gyro);
    m_ioContainers.addAll(Arrays.asList(m_modules));

    PhoenixOdometryThread.getInstance().start();

    getRegisteredScheduler().addPeriodic(() -> LoggedTracer.traced(getName(), this::periodic));
  }

  @Override
  public List<? extends LoggedComponent<?, ?>> getComponents() {
    return m_ioContainers;
  }

  private void periodic() {
    // The odometry thread fills the IO queues from another thread, so hold its lock while they are
    // drained: without it a pass of that thread could land between two modules' reads and leave
    // their sample counts out of step.
    var odometryLock = PhoenixOdometryThread.getLock();
    odometryLock.lock();
    try {
      updateComponents();
    } finally {
      odometryLock.unlock();
    }

    m_odometryObservations = collectOdometryObservations();

    // A motor controller holds its last request, so a setpoint left over from the previous enable
    // would be reapplied the instant the robot is enabled again.
    if (RobotState.isDisabled()) {
      for (SwerveModule module : m_modules) {
        module.halt();
      }
    }

    Logger.recordOutput("Drive/ModuleVelocities/Measured", getModuleVelocities());
    Logger.recordOutput("Drive/ChassisVelocities/Measured", getRobotRelativeSpeeds());
  }

  /**
   * Drives the robot at the given robot-relative velocity.
   *
   * @param velocities the velocity to hold until the next call
   */
  public void drive(ChassisVelocities velocities) {
    drive(velocities, kNoModuleForces);
  }

  /**
   * Drives the robot at the given robot-relative velocity, feeding forward the force each module is
   * expected to be producing.
   *
   * <p>A velocity alone leaves the module controllers to discover the force needed to achieve it
   * from the error it has already cost them. A trajectory planned against the robot's mass knows
   * that force in advance, so handing it over turns the module controllers from the thing that
   * produces the acceleration into the thing that corrects it.
   *
   * @param velocities                 the velocity to hold until the next call
   * @param robotRelativeModuleForces  the force each module should exert on the robot, in newtons,
   *                                   in module order. Shorter than the module count, including
   *                                   empty, means the rest get nothing fed forward.
   */
  public void drive(ChassisVelocities velocities, Translation2d[] robotRelativeModuleForces) {
    // Holding a velocity for a whole loop while also rotating traces an arc, not the straight line
    // the naive inverse kinematics assume; discretizing corrects for the difference.
    ChassisVelocities setpoint = velocities.discretize(kLoopPeriod.in(Seconds));

    // Note that the forces are deliberately not scaled alongside a desaturated velocity. Force is
    // not proportional to velocity, so there is no factor that would be correct, and a path that
    // saturates the modules is one the planner thought was feasible and was wrong about -- a
    // slightly optimistic feedforward is not what has gone wrong in that moment.
    SwerveModuleVelocity[] setpoints = SwerveDriveKinematics.desaturateWheelVelocities(
        m_kinematics.toSwerveModuleVelocities(setpoint), TunerConstants.kSpeedAt12Volts);

    SwerveModuleVelocity[] applied = new SwerveModuleVelocity[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      Translation2d force = i < robotRelativeModuleForces.length
          ? robotRelativeModuleForces[i]
          : Translation2d.ZERO;

      applied[i] = m_modules[i].setVelocity(setpoints[i], force);
    }

    Logger.recordOutput("Drive/ChassisVelocities/Setpoint", setpoint);
    Logger.recordOutput("Drive/ModuleVelocities/Setpoints", setpoints);
    Logger.recordOutput("Drive/ModuleVelocities/SetpointsOptimized", applied);
    Logger.recordOutput("Drive/ModuleForces/Setpoints", robotRelativeModuleForces);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * The gyro's own heading, with no field-frame offset applied. Only useful for establishing that
   * offset, which is what {@code PoseEstimator.resetPose} does with it.
   */
  public Rotation2d getRawGyroAngle() {
    return m_gyro.getYaw();
  }

  public SwerveModulePosition[] getModulePositions() {
    return Stream.of(m_modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  public SwerveModuleVelocity[] getModuleVelocities() {
    return Stream.of(m_modules)
        .map(SwerveModule::getVelocity)
        .toArray(SwerveModuleVelocity[]::new);
  }

  public ChassisVelocities getRobotRelativeSpeeds() {
    return m_kinematics.toChassisVelocities(getModuleVelocities());
  }

  /**
   * The odometry samples taken since the last cycle, oldest first.
   *
   * <p>Read once per loop and fed to the pose estimator from outside this mechanism. The list is
   * replaced each cycle rather than drained, so reading it twice is harmless but feeding it to the
   * estimator twice would double-count the motion.
   */
  public List<OdometryObservation> getOdometryObservations() {
    return m_odometryObservations;
  }

  public Command driveCommand(Supplier<ChassisVelocities> speeds) {
    return run(coroutine -> {
      while (true) {
        drive(speeds.get());
        coroutine.yield();
      }
    }).named("Drive");
  }

  /**
   * Turns this cycle's high-frequency samples into observations for the pose estimator.
   *
   * <p>One pass of the odometry thread appends to every queue, so entry {@code i} of each belongs to
   * the same instant. A queue that overflowed because the main loop stalled breaks that, so the
   * shortest queue sets the count and the extra samples are dropped rather than paired up wrongly.
   */
  private List<OdometryObservation> collectOdometryObservations() {
    // Tilt is not sampled at odometry frequency, so every observation this cycle shares the latest
    // reading. It only scales how far the wheels are believed to have carried the robot, and the
    // chassis cannot tip appreciably within one loop.
    Optional<Rotation2d> pitch = m_gyro.isConnected()
        ? Optional.of(m_gyro.getPitch())
        : Optional.empty();
    Optional<Rotation2d> roll = m_gyro.isConnected()
        ? Optional.of(m_gyro.getRoll())
        : Optional.empty();

    double[] timestamps = m_modules[0].getOdometryTimestamps();
    Rotation2d[] yaws = m_gyro.getOdometryYawPositions();

    int sampleCount = timestamps.length;
    for (SwerveModule module : m_modules) {
      sampleCount = Math.min(sampleCount, module.getOdometryPositions().length);
    }
    if (m_gyro.isConnected()) {
      sampleCount = Math.min(sampleCount, yaws.length);
    }

    var observations = new ArrayList<OdometryObservation>(sampleCount);
    for (int i = 0; i < sampleCount; i++) {
      var wheelPositions = new SwerveModulePosition[m_modules.length];
      for (int module = 0; module < m_modules.length; module++) {
        wheelPositions[module] = m_modules[module].getOdometryPositions()[i];
      }

      // Reporting no yaw when the gyro is absent is what makes the estimator fall back to the
      // heading the wheels imply, rather than trusting a frozen reading.
      observations.add(new OdometryObservation(
          Seconds.of(timestamps[i]),
          wheelPositions,
          pitch,
          roll,
          m_gyro.isConnected() ? Optional.of(yaws[i]) : Optional.empty()));
    }

    return observations;
  }
}
