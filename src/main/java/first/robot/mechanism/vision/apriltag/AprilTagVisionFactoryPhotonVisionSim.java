package first.robot.mechanism.vision.apriltag;

import java.util.Optional;
import java.util.function.Supplier;

import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;

/**
 * Cameras looking at a simulated field, rendered by PhotonVision's simulation tooling.
 *
 * <p>Simulation shares the real robot's IO rather than getting one of its own, because there is
 * nothing for a second IO to implement: PhotonVision simulates the camera, not the reading of it, so
 * a simulated camera publishes to the same NetworkTables topics a coprocessor would and
 * {@link AprilTagCameraIOPhotonVision} reads them with the same code. Only the construction differs,
 * and that is what this class is.
 *
 * <p>Usable whatever the real robot reads its cameras with — see {@link AprilTagVisionFactory} — so
 * a robot on some other vendor's hardware can still be simulated here.
 */
public class AprilTagVisionFactoryPhotonVisionSim implements AprilTagVisionFactory {
  private final Field m_field;
  private final RobotHeadingSource m_headingSource;
  private final PhotonAprilTagVisionSim m_visionSim;

  /**
   * Builds the simulated field. Nothing is scheduled here: when it gets rendered, and when it hears
   * about a pose reset, are both settled by {@link #createSim} handing it to the thing that reads
   * the cameras.
   *
   * @param field                   the tag layout to render, and to solve the rendered frames
   *                                against
   * @param headingSource           the robot heading single-tag solves need; see
   *                                {@link RobotHeadingSource}
   * @param groundTruthPoseSupplier where the robot really is, read every loop to pose the simulated
   *                                cameras. This must be a pose no vision observation has corrected;
   *                                see {@link AprilTagVisionSim#update}.
   */
  public AprilTagVisionFactoryPhotonVisionSim(
      Field field, RobotHeadingSource headingSource, Supplier<Pose2d> groundTruthPoseSupplier) {
    m_field = field;
    m_headingSource = headingSource;
    m_visionSim = new PhotonAprilTagVisionSim(field, groundTruthPoseSupplier);
  }

  @Override
  public AprilTagCameraIO createCameraIO(AprilTagCameraConfig config) {
    return AprilTagCameraIOPhotonVision.simulated(config, m_field, m_headingSource, m_visionSim);
  }

  /**
   * The field the cameras built above are registered into.
   *
   * <p>Handed over for {@link AprilTagVision} to drive rather than driven from a scheduler here,
   * which is what makes "rendered before the cameras are read" a property of the code instead of a
   * property of what got constructed first.
   */
  @Override
  public Optional<AprilTagVisionSim> createSim() {
    return Optional.of(m_visionSim);
  }
}
