package first.robot;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;

import first.lib.util.GeometryUtil;
import first.robot.mechanism.intake.IntakePivot;
import first.robot.util.Constants.VisionConstants;

/**
 * Publishes the robot's geometry the way AdvantageScope's 3D field wants to read it, so the
 * articulated parts of the robot model move with the mechanisms instead of standing still.
 *
 * <p>Nothing here is used by the robot to decide anything. It exists so that a log can be watched
 * rather than read: a hood that never leaves its hard stop or a turret that lags the chassis is
 * obvious in the model and invisible in a column of numbers. It is therefore safe to add keys to,
 * and safe for everything in it to be wrong without the robot misbehaving -- which is also why
 * none of these numbers are shared with the control code.
 *
 * <h2>Wiring it up in AdvantageScope</h2>
 *
 * <p>The {@code Robot_G3-2026} asset must be installed, by copying
 * {@code advantagescope/Robot_G3-2026} into AdvantageScope's user assets folder (<em>Help &gt; Show
 * Assets Folder</em>). Then, in a 3D Field tab:
 *
 * <ol>
 * <li>drag {@code RobotGeometryViz/RobotPose} in and set the object to <em>Robot</em>, choosing the
 * {@code G3-2026} model;
 * <li>drag {@code RobotGeometryViz/ComponentPoses} onto that same robot object, which offers it as
 * <em>Robot Components</em>;
 * <li>optionally drag {@code RobotGeometryViz/TurretPose} in as <em>Axes</em>, and
 * {@code RobotGeometryViz/CameraPoses} as <em>Camera</em> or <em>Axes</em>.
 * </ol>
 */
public class RobotGeometryViz {
  /**
   * Where the asset's articulated components pivot, in the robot frame, in the order the asset
   * numbers them: {@code model_0} is the turret, {@code model_1} the intake, {@code model_2} the
   * hood.
   *
   * <p>Each is the negation of the matching {@code zeroedPosition} in the asset's
   * {@code config.json}. That field tells AdvantageScope how far to shove a component mesh so its
   * pivot lands on the model origin; undoing that shift is what puts the pivot back where it
   * belongs on the robot, which is the pose this class has to publish. So the two move together: if
   * {@code config.json} is regenerated from a new CAD export, these change sign for sign with it.
   *
   * <p>They describe the model, not this robot, and are deliberately not read from
   * {@code RobotGeometryConstants} even where the robot has a constant for the same joint -- the
   * turret pivot here is where the G3 mesh's turret sits, which is not the same number as the
   * {@code kRobotToTurret3d} the shot math uses.
   */
  private static final Translation3d kTurretPivot = new Translation3d(-0.1, 0.2, 0.5);

  private static final Translation3d kIntakePivot = new Translation3d(0.09, 0.0, 0.19);

  private static final Translation3d kHoodPivot = new Translation3d(0.005, 0.2, 0.525);

  /**
   * The hood pivot measured from the turret pivot, with the turret at zero.
   *
   * <p>The hood mesh is bolted to the turret and sweeps with it, so its pose is built by riding the
   * turret's rather than being placed in the robot frame directly. Taking the offset as the
   * difference of the two pivots, instead of writing it out as its own measurement, is what keeps
   * the hood landing exactly on its own {@code zeroedPosition} when the turret is pointed straight
   * ahead.
   */
  private static final Translation3d kTurretToHood = kHoodPivot.minus(kTurretPivot);

  /** One identity pose per component, for checking the asset lines up. See {@link #periodic}. */
  private static final Pose3d[] kZeroedComponentPoses = {
      new Pose3d(kTurretPivot, Rotation3d.ZERO),
      new Pose3d(kIntakePivot, Rotation3d.ZERO),
      new Pose3d(kHoodPivot, Rotation3d.ZERO),
  };

  private final RobotState m_state;
  private final IntakePivot m_intakePivot;

  /**
   * @param intakePivot read directly rather than through {@code state}, because the intake is not
   *                    part of any geometry the robot computes: it is drawn and nothing more, so
   *                    teaching {@link RobotState} about it would add a dependency that only the
   *                    renderer ever uses
   */
  public RobotGeometryViz(RobotState state, IntakePivot intakePivot) {
    m_state = state;
    m_intakePivot = intakePivot;
  }

  public void periodic() {
    Pose3d robotPose = new Pose3d(m_state.getRobotPose());

    Logger.recordOutput("RobotGeometryViz/RobotPose", robotPose);
    Logger.recordOutput("RobotGeometryViz/ComponentPoses", getComponentPoses());

    // Published every loop rather than once, so that it can be dragged onto a second robot object
    // and watched beside the live one. A component drawn at its zeroed pose should sit exactly
    // where the one-piece model draws it, so anything that does not line up is a disagreement
    // between the pivot constants above and the asset -- which is worth seeing as a static model
    // rather than chasing as a mechanism that moves to slightly the wrong place.
    Logger.recordOutput("RobotGeometryViz/ZeroedComponentPoses", kZeroedComponentPoses);

    logTurret(robotPose);
  }

  /**
   * The component poses AdvantageScope applies to the model, robot-relative and index-matched to the
   * asset's {@code model_<n>} files.
   *
   * <p>The turret takes yaw and the other two pitch, each applied in the direction the asset's
   * meshes were exported to swing: positive, so a mechanism reading a larger angle deploys the
   * intake further or raises the hood higher. If a mechanism's encoder zero is ever moved off its
   * hard stop, the offset belongs here, not in the mechanism -- the mechanism's zero is whatever
   * the control code finds convenient, and this is the only place that needs it to agree with how a
   * CAD model happens to have been exported.
   */
  private Pose3d[] getComponentPoses() {
    Pose3d turretPose =
        new Pose3d(kTurretPivot, GeometryUtil.rotation3dFromYaw(m_state.getTurretAngle()));

    return new Pose3d[] {
        turretPose,
        new Pose3d(kIntakePivot, pitch(m_intakePivot.getCurrentMeasurement())),
        turretPose.transformBy(new Transform3d(kTurretToHood, pitch(m_state.getHoodAngle()))),
    };
  }

  /**
   * The turret pivot and the camera riding on it, in field coordinates.
   *
   * <p>Overlapping the turret component on purpose: this one is drawn from the constants the shot
   * math actually uses, so the camera pose beside it is what makes a bad {@code kTurretToCamera}
   * visible -- a camera that drifts off the turret as it sweeps is mounted wrong in the constants.
   * The component above answers a different question, whether the model is being driven correctly,
   * and so is pinned to the asset instead.
   */
  private void logTurret(Pose3d robotPose) {
    Logger.recordOutput(
        "RobotGeometryViz/TurretPose",
        robotPose.transformBy(m_state.getRobotToTurretTransform3d()));
    Logger.recordOutput(
        "RobotGeometryViz/TurretCameraPose",
        robotPose.transformBy(m_state.getRobotToTurretCamera()));

    // The chassis-mounted cameras, from the same list the vision code actually uses, so a camera
    // added to the robot shows up here without this class being touched.
    Logger.recordOutput(
        "RobotGeometryViz/CameraPoses",
        VisionConstants.kCameras.stream()
            .map(camera -> robotPose.transformBy(camera.robotToCamera()))
            .toArray(Pose3d[]::new));
  }

  private static Rotation3d pitch(Angle angle) {
    return GeometryUtil.rotation3dFromPitch(angle);
  }
}
