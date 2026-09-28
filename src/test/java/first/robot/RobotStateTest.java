package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

import first.lib.mechanism.TunableGains;
import first.lib.mechanism.angle.AngleMechanismIO;
import first.lib.mechanism.angle.AngleMechanismInputsAutoLogged;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismIO;
import first.lib.mechanism.angularvelocity.AngularVelocityMechanismInputsAutoLogged;
import first.robot.mechanism.drive.Drive;
import first.robot.mechanism.drive.GyroIO;
import first.robot.mechanism.drive.SwerveModuleIO;
import first.robot.mechanism.drive.SwerveModuleIOInputsAutoLogged;
import first.robot.mechanism.flywheel.Flywheel;
import first.robot.mechanism.hood.Hood;
import first.robot.mechanism.turret.Turret;
import first.robot.mechanism.vision.AprilTagVision;
import first.robot.util.Constants.RobotGeometryConstants;
import first.robot.util.FieldConstants;
import first.robot.util.PoseEstimator;
import first.robot.util.TurretSnapshot;

/**
 * Covers the field-frame composition in {@link RobotState#getTurretSnapshot}: turning a chassis pose
 * and a robot-relative mounting offset into the field-relative turret state the shot solver consumes.
 *
 * <p>These are the frames that are easy to mix up — a robot-relative offset compared against a
 * field-relative target reads as a plausible number and aims nowhere near the target.
 */
class RobotStateTest {
  private static final double kEpsilon = 1e-9;

  /**
   * A {@link Drive} with stubbed IO, whose reported velocity the test can pose. The pose itself now
   * lives in the {@link PoseEstimator}, so tests set it through {@link RobotState#resetPose}.
   */
  private static final class FakeDrive extends Drive {
    private ChassisVelocities m_robotRelativeSpeeds = new ChassisVelocities();

    private FakeDrive() {
      super(new GyroIO() {
      }, new StubModuleIO(), new StubModuleIO(), new StubModuleIO(), new StubModuleIO());
    }

    @Override
    public ChassisVelocities getRobotRelativeSpeeds() {
      return m_robotRelativeSpeeds;
    }
  }

  /** A module that never moves: stopped, pointed straight ahead, and reporting no odometry. */
  private static final class StubModuleIO implements SwerveModuleIO {
    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    }

    @Override
    public void setDriveVelocity(org.wpilib.units.measure.LinearVelocity velocity) {
    }

    @Override
    public void setSteerPosition(Rotation2d position) {
    }

    @Override
    public void halt() {
    }
  }

  /** Inputs are mutated directly; nothing here drives real hardware. */
  private static final class StubAngleIO implements AngleMechanismIO {
    @Override
    public void updateInputs(AngleMechanismInputsAutoLogged inputs) {
    }

    @Override
    public void setTargetAngle(org.wpilib.units.measure.Angle angle) {
    }

    @Override
    public void setGains(TunableGains gains) {
    }

    @Override
    public void halt() {
    }
  }

  private static final class StubVelocityIO implements AngularVelocityMechanismIO {
    @Override
    public void updateInputs(AngularVelocityMechanismInputsAutoLogged inputs) {
    }

    @Override
    public void setTargetAngularVelocity(org.wpilib.units.measure.AngularVelocity velocity) {
    }

    @Override
    public void setGains(TunableGains gains) {
    }

    @Override
    public void halt() {
    }
  }

  private FakeDrive m_drive;
  private Turret m_turret;
  private RobotState m_state;

  @BeforeAll
  static void initializeHal() {
    // RobotState publishes tunables and network booleans, so the HAL has to be up. AdvantageKit's
    // conduit reads telemetry over the system server's NetworkTables instance and segfaults if it is
    // touched before the driver station has been refreshed once.
    HAL.initialize();
    DriverStationBackend.refreshData();
  }

  private RobotState newState() {
    m_drive = new FakeDrive();
    m_turret = new Turret(new StubAngleIO());

    // Tilt compensation and the wall clamp are off so that a posed pose comes back unchanged.
    var poseEstimator = new PoseEstimator(m_drive.getKinematics(), () -> false, () -> false);

    m_state = new RobotState(
        m_drive, m_turret, new Hood(new StubAngleIO()), new Flywheel(new StubVelocityIO()),
        new AprilTagVision(), poseEstimator);
    return m_state;
  }

  /** The mounting offset rotated into the field frame for a given chassis heading. */
  private static Translation2d expectedOffset(Rotation2d robotRotation) {
    return RobotGeometryConstants.kRobotToTurret.rotateBy(robotRotation);
  }

  @Test
  void turretPositionIsFieldRelative() {
    var state = newState();
    state.resetPose(new Pose2d(3.0, 2.0, Rotation2d.ZERO));

    TurretSnapshot snapshot = state.getTurretSnapshot(Seconds.zero());
    Translation2d expected = new Translation2d(3.0, 2.0).plus(expectedOffset(Rotation2d.ZERO));

    assertEquals(expected.getX(), snapshot.position().getX(), kEpsilon);
    assertEquals(expected.getY(), snapshot.position().getY(), kEpsilon);
  }

  @Test
  void turretPositionCarriesTheMountingHeight() {
    var state = newState();

    assertEquals(
        RobotGeometryConstants.kRobotToTurret3d.getZ(),
        state.getTurretSnapshot(Seconds.zero()).position().getZ(),
        kEpsilon);
  }

  @Test
  void mountingOffsetRotatesWithTheChassis() {
    var state = newState();
    state.resetPose(new Pose2d(0.0, 0.0, Rotation2d.PI));

    TurretSnapshot snapshot = state.getTurretSnapshot(Seconds.zero());
    Translation2d expected = expectedOffset(Rotation2d.PI);

    // A 180 degree yaw must flip the offset, not leave it in the robot frame.
    assertEquals(expected.getX(), snapshot.position().getX(), kEpsilon);
    assertEquals(expected.getY(), snapshot.position().getY(), kEpsilon);
    assertEquals(
        -RobotGeometryConstants.kRobotToTurret.getX(), snapshot.position().getX(), kEpsilon);
  }

  @Test
  void snapshotCarriesTheChassisHeadingNotTheTurretHeading() {
    var state = newState();
    state.resetPose(new Pose2d(0.0, 0.0, Rotation2d.CCW_90DEG));
    m_turret.getInputs().currentAngle = Degrees.of(45);

    // The turret setpoint is measured in the robot frame, so the snapshot must expose the chassis
    // heading alone. Folding the turret's own angle in here would double-count it.
    assertEquals(
        90.0,
        state.getTurretSnapshot(Seconds.zero()).robotRotation().getMeasure().in(Degrees),
        1e-9);
  }

  @Test
  void turretVelocityAddsTheLeverArmTermFromChassisRotation() {
    var state = newState();
    double omega = 1.5;
    m_drive.m_robotRelativeSpeeds = new ChassisVelocities(0.0, 0.0, omega);

    ChassisVelocities velocity = state.getTurretSnapshot(Seconds.zero()).fieldRelativeVelocity();

    // v = omega x r. Spinning in place still sweeps the turret, which is what shoot-on-the-move has
    // to cancel; treating the chassis velocity as the turret's would miss it entirely.
    Translation2d leverArm = expectedOffset(Rotation2d.ZERO);
    assertEquals(-leverArm.getY() * omega, velocity.vx, kEpsilon);
    assertEquals(leverArm.getX() * omega, velocity.vy, kEpsilon);
  }

  @Test
  void turretVelocityIsTheChassisVelocityWhenNotRotating() {
    var state = newState();
    m_drive.m_robotRelativeSpeeds = new ChassisVelocities(1.0, -2.0, 0.0);

    ChassisVelocities velocity = state.getTurretSnapshot(Seconds.zero()).fieldRelativeVelocity();

    assertEquals(1.0, velocity.vx, kEpsilon);
    assertEquals(-2.0, velocity.vy, kEpsilon);
  }

  @Test
  void latencyProjectsTheSnapshotForward() {
    var state = newState();
    m_drive.m_robotRelativeSpeeds = new ChassisVelocities(2.0, 0.0, 0.0);

    double baseline = state.getTurretSnapshot(Seconds.zero()).position().getX();
    double projected = state.getTurretSnapshot(Seconds.of(0.1)).position().getX();

    // 2 m/s for 100 ms with no rotation is a straight 0.2 m of travel.
    assertEquals(baseline + 0.2, projected, 1e-9);
  }

  @Test
  void zeroLatencySnapshotMatchesTheCurrentTurretPose() {
    var state = newState();
    state.resetPose(new Pose2d(1.0, 1.0, Rotation2d.CCW_90DEG));
    m_drive.m_robotRelativeSpeeds = new ChassisVelocities(3.0, 4.0, 1.0);

    Translation2d live = state.getTurretPose().getTranslation();
    var snapshot = state.getTurretSnapshot(Seconds.zero()).position();

    assertEquals(live.getX(), snapshot.getX(), kEpsilon);
    assertEquals(live.getY(), snapshot.getY(), kEpsilon);
  }

  @Test
  void fieldBoundsClampKeepsTheRobotFootprintInside() {
    // Guards the geometry that moved from PoseEstimator into FieldConstants.
    var clamped = FieldConstants.clampToFieldBounds(
        new Pose2d(-5.0, -5.0, Rotation2d.ZERO),
        RobotGeometryConstants.kRobotLengthWithBumpers,
        RobotGeometryConstants.kRobotWidthWithBumpers);

    double halfLength = RobotGeometryConstants.kRobotLengthWithBumpers.in(Meters) / 2.0;
    double halfWidth = RobotGeometryConstants.kRobotWidthWithBumpers.in(Meters) / 2.0;

    assertEquals(halfLength, clamped.getX(), kEpsilon);
    assertEquals(halfWidth, clamped.getY(), kEpsilon);
  }
}
