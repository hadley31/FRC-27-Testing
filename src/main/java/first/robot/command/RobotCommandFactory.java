package first.robot.command;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.util.GeometryUtil;
import first.robot.Robot;
import first.robot.util.FieldConstants;
import first.robot.util.ShotCalculationUtil;
import first.robot.util.ShotParameters;
import first.robot.util.SwerveInputStream;
import first.robot.util.Tuning;
import first.robot.util.TurretSnapshot;

public class RobotCommandFactory {
  /**
   * Where the turret parks while the chassis does the aiming, in fixed-turret mode.
   *
   * <p>Zero points the bore down the chassis +X axis, which makes the chassis heading the bore
   * bearing itself. Any value works -- {@link ShotCalculationUtil#chassisHeadingForTurretBore}
   * subtracts it -- so this can move without touching the arithmetic.
   *
   * <p>CONFIRM AGAINST MECHANICAL: nothing in the code defines the turret's parked or hard-stop
   * angle, and a wrong value here is silently wrong. The chassis would aim off by exactly this
   * angle, with nothing pointing at the cause.
   */
  private static final Angle kFixedTurretAngle = Degrees.zero();

  /** How close the chassis heading must be, in fixed-turret mode, before fuel is fed. */
  private static final Angle kChassisAimTolerance = Degrees.of(2.0);

  /**
   * How slowly the chassis must be turning before its aim is believed.
   *
   * <p>A heading within tolerance in passing is not an aim. It matters more than usual here because
   * the shot solution leads for the turret pivot's velocity, and the pivot rides a lever arm: while
   * the chassis is swinging hard the lead is large and chasing its own cause, so the setpoint is only
   * trustworthy once the rotation has settled.
   */
  private static final AngularVelocity kChassisAimSettledRate = DegreesPerSecond.of(20);

  private final Robot m_robot;
  private final ShotCalculationUtil m_shotCalculationUtil = new ShotCalculationUtil(Tuning.kVelocityCompensation);
  private final SwerveInputStream m_driveInputs;

  /**
   * @param driveInputs the translation the robot's own chassis-aim commands rotate on top of.
   *                    Teleop passes the driver's sticks, so aiming does not freeze them in place;
   *                    auto passes a stream with no sticks behind it, which rotates in place. That
   *                    this is a constructor parameter is why no command in here needs to know which
   *                    mode it is running in.
   */
  public RobotCommandFactory(Robot robot, SwerveInputStream driveInputs) {
    m_robot = robot;
    m_driveInputs = driveInputs;
  }

  public Command haltAllMechanisms() {
    return Command
        .requiring(m_robot.feeder, m_robot.flywheel, m_robot.turret, m_robot.hood)
        .executing(coroutine -> {
          m_robot.feeder.halt();
          m_robot.flywheel.halt();
          m_robot.turret.halt();
          m_robot.hood.halt();
        })
        .named("Halt All Mechanisms");
  }

  public Command stowAllMechanisms() {
    return Command.parallel(
        stowIntake(),
        stowHood())
        .named("Stow All Mechanisms");
  }

  public Command feedFuel() {
    return m_robot.feeder
        .setTargetCommand(RPM.of(1000))
        .whenCanceled(m_robot.feeder::halt)
        .named("Feed Fuel");
  }

  // MARK: - Chassis aiming

  /**
   * Rotates the chassis until its front faces {@code target}, and keeps it facing there.
   *
   * <p>Aims the chassis +X axis at a literal point, which is the right thing for driver aim-assist
   * and for facing a fixed feature. It is <em>not</em> what {@link #autoAim} uses: putting fuel in
   * the hub is a question about where the turret bore points, and the shot solution answers it
   * better than a bearing from the robot's centre can. See {@link #holdChassisHeading}.
   *
   * <p>Runs until canceled, like every other {@code driveCommand}. A heading is held while something
   * else decides it is done -- a shot, or the driver letting go of a button -- rather than finishing
   * on tolerance, which would hand the chassis back at exactly the moment it needs to stay aimed.
   *
   * @param target a field-relative point to face, re-read every cycle: the direction to even a fixed
   *               point changes as the robot drives past it
   */
  public Command pointChassisAtPoint(Supplier<Translation2d> target) {
    return holdChassisHeading(
        () -> {
          var robotPose = m_robot.state.getRobotPose();
          return GeometryUtil.bearing(
              robotPose.getTranslation(), target.get(), robotPose.getRotation());
        },
        "Point Chassis At Point");
  }

  /**
   * Holds a chassis heading that something else recomputes every cycle.
   *
   * <p>The one place a chassis-aim command is built. Everything that aims the chassis differs only in
   * which heading it supplies, so that is the only thing passed in: the controller that turns a
   * heading into a rotation rate belongs to the drive layer, and deciding which heading is worth
   * holding belongs here.
   */
  private Command holdChassisHeading(Supplier<Rotation2d> heading, String name) {
    return m_robot.drive.driveCommand(
        m_driveInputs.getHeadingLockedDriveSupplier(heading), name);
  }

  public Command autoAim() {
    return autoAim(new ChassisAimGoal());
  }

  /**
   * Aims everything that can be aimed at the hub, including the chassis when
   * {@link Tuning#kFixedTurretMode} says the chassis is what aims.
   *
   * <p>One solution per cycle feeds every consumer. The turret and hood take it directly; the
   * chassis takes it through {@link ShotCalculationUtil#chassisHeadingForTurretBore}, which is the
   * same solve read differently rather than a second one -- two solves would take their snapshots at
   * different instants, aim at marginally different virtual targets, and overwrite each other's
   * {@code ShotCalculation/*} log keys within one loop.
   *
   * @param goal where the chassis heading is published for the forked child and the feed gate
   */
  private Command autoAim(ChassisAimGoal goal) {
    return Command
        .requiring(m_robot.flywheel, m_robot.turret, m_robot.hood)
        .executing(coroutine -> {
          Command chassisAim = holdChassisHeading(goal::heading, "Point Chassis At Hub");
          boolean aiming = false;

          while (true) {
            boolean fixedTurret = m_robot.state.isFixedTurretModeEnabled();

            // Taken once and reused, so the turret setpoint and the chassis heading describe the
            // same instant.
            TurretSnapshot snapshot = currentTurretSnapshot();
            ShotParameters parameters = calculateShot(snapshot);

            m_robot.flywheel.setTarget(parameters.targetFlywheelSpeed());
            m_robot.hood.setTarget(parameters.targetHoodAngle());
            m_robot.turret.setTarget(
                fixedTurret ? kFixedTurretAngle : parameters.targetTurretAngle());

            goal.set(
                ShotCalculationUtil.chassisHeadingForTurretBore(
                    snapshot, parameters, kFixedTurretAngle),
                fixedTurret);

            Logger.recordOutput("AutoAim/FixedTurretMode", fixedTurret);
            Logger.recordOutput("AutoAim/DesiredChassisHeading", goal.heading());

            // Edge-triggered rather than polling whether the child is running. "Not running" also
            // means "something else took the drivetrain", so a level-triggered version would re-fork
            // and steal it back every cycle -- and at equal priority it would win, fighting a path
            // follower for the chassis indefinitely. Losing it once and staying lost is the right
            // answer: whoever took it wanted it.
            if (fixedTurret && !aiming) {
              coroutine.fork(chassisAim);
              aiming = true;
            } else if (!fixedTurret && aiming) {
              coroutine.scheduler().cancel(chassisAim);
              aiming = false;
            }

            coroutine.yield();
          }
        })
        .named("Auto Aim");
  }

  public Command autoAimAndShoot() {
    var goal = new ChassisAimGoal();

    return Command.noRequirements(coroutine -> {
      // The chassis term is what keeps fixed-turret mode from feeding mid-swing: the turret reports
      // "on target" the instant it reaches its parked angle, which says nothing at all about where
      // the robot is pointed.
      m_robot.state.isPreparedToShootTrigger()
          .and(() -> goal.isAimed(
              m_robot.state.getRobotPose().getRotation(),
              // Robot-relative, since a rotation rate is the same number in either frame and the
              // field-relative accessor would rotate two translations to arrive at it.
              DegreesPerSecond.of(Math.toDegrees(m_robot.state.getRobotRelativeSpeeds().omega)),
              kChassisAimTolerance,
              kChassisAimSettledRate))
          .whileTrue(feedFuel());

      coroutine.await(autoAim(goal));
    }).named("Auto Shoot");
  }

  // MARK: - Turret

  /**
   * The turret's default command: tracks the hub while {@link Tuning#kAutoAim} is set, and holds
   * whatever it was last told when it is not.
   *
   * <p>One long-running command that reads the toggle each cycle, rather than a command gated by the
   * toggle. A default command's binding is scheduled continuously while its mechanism is free, so a
   * command that ended when the toggle went false would be re-scheduled and end again every single
   * loop. Branching inside the body is what makes the toggle flippable at no cost.
   *
   * <p>Priority sits below {@link Command#DEFAULT_PRIORITY} so that anything else wanting the turret
   * -- {@link #autoAim}, a tuning routine -- simply takes it, and gets it back on its own when that
   * command ends. It also requires nothing but the turret, which a default command may not do
   * anyway: that is why this tracks only the turret and leaves the hood and flywheel to
   * {@code autoAim}.
   */
  public Command trackHubWithTurret() {
    return m_robot.turret.run(coroutine -> {
      while (true) {
        if (m_robot.state.isAutoAimEnabled()) {
          m_robot.turret.setTarget(calculateShot(currentTurretSnapshot()).targetTurretAngle());
        }

        coroutine.yield();
      }
    })
        .withPriority(Command.DEFAULT_PRIORITY - 1)
        .named("Track Hub With Turret");
  }

  public Command haltTurret() {
    return m_robot.turret.haltCommand();
  }

  // MARK: - Feeder

  public Command haltFeeder() {
    return m_robot.feeder.haltCommand();
  }

  // MARK: - Flywheel

  public Command haltFlywheel() {
    return m_robot.flywheel.haltCommand();
  }

  // MARK: - Hood

  public Command stowHood() {
    return m_robot.hood.setTargetCommand(Degrees.zero()).named("Stow Hood");
  }

  // MARK: - Intake

  public Command extendIntakePivot() {
    return m_robot.intakePivot.setTargetCommand(Degrees.zero()).named("Extend Intake Pivot");
  }

  public Command stowIntakePivot() {
    return m_robot.intakePivot.setTargetCommand(Degrees.of(60)).named("Stow Intake Pivot");
  }

  public Command haltIntakeRollers() {
    return m_robot.intakeRollers.haltCommand();
  }

  public Command runIntakeRollersIn() {
    return m_robot.intakeRollers.setTargetCommand(RPM.of(1000)).named("Run Intake Rollers In");
  }

  public Command runIntakeRollersOut() {
    return m_robot.intakeRollers.setTargetCommand(RPM.of(1000)).named("Run Intake Rollers Out");
  }

  public Command intakeFuel() {
    // A parallel group, not a fork-and-return: a body that forks and falls off the end completes on
    // the cycle it starts, which cancels the children with it and -- when this is a state machine
    // state -- exits the whole machine before any transition out of it is ever polled.
    return Command.parallel(
        extendIntakePivot(),
        runIntakeRollersIn())
        .named("Intake Fuel");
  }

  public Command stowIntake() {
    return Command.parallel(
        stowIntakePivot(),
        haltIntakeRollers())
        .named("Stow Intake");
  }

  /**
   * @param turret the instant to solve for. Taken by the caller rather than fetched here, so that a
   *               caller consuming the solution more than once -- the turret and the chassis both
   *               aiming from it -- gets one instant rather than two.
   */
  private ShotParameters calculateShot(TurretSnapshot turret) {
    // Fetched fresh every solution rather than held in a field: the dashboard can retune this table
    // while the robot is running, and tuning the actuation latency replaces the profile object.
    var profile = Tuning.kScoringShotProfile.profile();

    return m_shotCalculationUtil.calculateShot(
        turret, FieldConstants.getHubPosition3d(), profile);
  }

  /** The snapshot a shot should be solved from now, projected forward by the actuation latency. */
  private TurretSnapshot currentTurretSnapshot() {
    return m_robot.state.getTurretSnapshot(
        Tuning.kScoringShotProfile.profile().actuationLatency());
  }
}
