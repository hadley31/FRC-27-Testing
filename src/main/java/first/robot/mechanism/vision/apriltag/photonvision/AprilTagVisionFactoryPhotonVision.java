package first.robot.mechanism.vision.apriltag.photonvision;

import java.util.Optional;
import java.util.function.Supplier;

import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;

import first.robot.mechanism.vision.apriltag.AprilTagCameraConfig;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIO;
import first.robot.mechanism.vision.apriltag.AprilTagVision;
import first.robot.mechanism.vision.apriltag.AprilTagVisionFactory;
import first.robot.mechanism.vision.apriltag.AprilTagVisionSim;
import first.robot.mechanism.vision.apriltag.RobotHeadingSource;

/**
 * Cameras read from PhotonVision, on a real robot or against a simulated field.
 *
 * <p>One class covers both because there is nothing for a second one to implement. PhotonVision
 * simulates the camera, not the reading of it: a simulated camera publishes to the same
 * NetworkTables topics a coprocessor would, and {@link AprilTagCameraIOPhotonVision} reads them with
 * the same code either way. The only difference is whether a {@link AprilTagVisionSimPhotonVision}
 * exists for the cameras to be registered into, which is what {@link #real} and {@link #simulated}
 * decide.
 *
 * <p>Simulating with this does not commit the real robot to it — see {@link AprilTagVisionFactory} —
 * so a robot on some other vendor's hardware can still be simulated here.
 */
public class AprilTagVisionFactoryPhotonVision implements AprilTagVisionFactory {
  private final Field m_field;
  private final RobotHeadingSource m_headingSource;
  private final Optional<AprilTagVisionSimPhotonVision> m_visionSim;

  private AprilTagVisionFactoryPhotonVision(
      Field field,
      RobotHeadingSource headingSource,
      Optional<AprilTagVisionSimPhotonVision> visionSim) {
    m_field = field;
    m_headingSource = headingSource;
    m_visionSim = visionSim;
  }

  /**
   * Cameras read from PhotonVision coprocessors, looking at the real world.
   *
   * @param field         the tag layout the coprocessors solve against, which must be the layout
   *                      they were given: a filter judging observations by a layout the cameras
   *                      never used is judging them by the wrong field
   * @param headingSource the robot heading single-tag solves need; see {@link RobotHeadingSource}
   *                      for what it must be
   */
  public static AprilTagVisionFactoryPhotonVision real(
      Field field, RobotHeadingSource headingSource) {
    return new AprilTagVisionFactoryPhotonVision(field, headingSource, Optional.empty());
  }

  /**
   * The same cameras, with a simulated field rendered by PhotonVision's simulation tooling behind
   * them.
   *
   * <p>Builds the field here, but schedules nothing: when it gets rendered, and when it hears about
   * a pose reset, are both settled by {@link #createSim} handing it to the thing that reads the
   * cameras.
   *
   * @param field                   the tag layout to render, and to solve the rendered frames
   *                                against
   * @param headingSource           the robot heading single-tag solves need; see
   *                                {@link RobotHeadingSource}
   * @param groundTruthPoseSupplier where the robot really is, read every loop to pose the simulated
   *                                cameras. This must be a pose no vision observation has corrected;
   *                                see {@link AprilTagVisionSim#update}.
   */
  public static AprilTagVisionFactoryPhotonVision simulated(
      Field field,
      RobotHeadingSource headingSource,
      Supplier<Pose2d> groundTruthPoseSupplier) {
    return new AprilTagVisionFactoryPhotonVision(
        field,
        headingSource,
        Optional.of(new AprilTagVisionSimPhotonVision(field, groundTruthPoseSupplier)));
  }

  @Override
  public AprilTagCameraIO createCameraIO(AprilTagCameraConfig config) {
    return m_visionSim
        .map(sim -> AprilTagCameraIOPhotonVision.simulated(config, m_field, m_headingSource, sim))
        .orElseGet(() -> new AprilTagCameraIOPhotonVision(config, m_field, m_headingSource));
  }

  /**
   * The field the cameras built above are registered into, and empty on a real robot, where what a
   * camera sees is the world and nothing has to put it there.
   *
   * <p>Handed over for {@link AprilTagVision} to drive rather than driven from a scheduler here,
   * which is what makes "rendered before the cameras are read" a property of the code instead of a
   * property of what got constructed first.
   */
  @Override
  public Optional<AprilTagVisionSim> createSim() {
    // Cast only to widen Optional's element type; there is no narrowing here.
    return m_visionSim.map(sim -> (AprilTagVisionSim) sim);
  }
}
