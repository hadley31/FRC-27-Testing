package first.robot.mechanism.vision.apriltag;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * An {@link AprilTagVisionSim} rendered by PhotonVision's simulation tooling.
 *
 * <p>This exists as its own object, rather than living inside {@link AprilTagCameraIOPhotonVision},
 * because PhotonVision's simulation is scoped to the robot and not to a camera: one instance holds
 * the tag layout and the robot's true pose, and every camera registers into it. What a camera does
 * own — its lens, its noise, its frame rate — stays in the IO layer with the camera it describes.
 *
 * <p>Nothing above this class has to know it exists. A simulated camera writes to the same
 * NetworkTables topics a real coprocessor would, so {@link AprilTagCameraIOPhotonVision} reads
 * results in simulation through exactly the code it uses on the robot; only the construction
 * differs.
 */
public class PhotonAprilTagVisionSim implements AprilTagVisionSim {
  private final VisionSystemSim m_sim;
  private final Supplier<Pose2d> m_groundTruthPoseSupplier;

  /**
   * @param field                   the tag layout to render
   * @param groundTruthPoseSupplier where the robot really is, read on every {@link #update}. This
   *                                must be a pose that no vision observation has corrected —
   *                                {@code PoseEstimator#getOdometryPose} rather than the estimate.
   *                                Rendering sightings from a vision-corrected pose makes them agree
   *                                with the correction by construction, and the filter above can
   *                                never be wrong, or right.
   */
  public PhotonAprilTagVisionSim(Field field, Supplier<Pose2d> groundTruthPoseSupplier) {
    m_sim = new VisionSystemSim("main");
    m_sim.addAprilTags(field);
    m_groundTruthPoseSupplier = groundTruthPoseSupplier;
  }

  @Override
  public void update() {
    m_sim.update(m_groundTruthPoseSupplier.get());
  }

  /**
   * {@inheritDoc}
   *
   * <p>{@link VisionSystemSim} is the reason this method is on the interface at all. It keeps its own
   * buffer of where the robot has been and renders each frame from the pose at that frame's capture
   * time rather than from the pose it was last handed, which is how it models exposure latency. That
   * buffer is what a reset has to clear, and clearing it is all this does.
   */
  @Override
  public void resetRobotPose(Pose2d robotPose) {
    m_sim.resetRobotPose(robotPose);
  }

  /**
   * Places a simulated camera on the robot. Package-private, and absent from
   * {@link AprilTagVisionSim}, because the only thing that should be building a
   * {@link PhotonCameraSim} is the PhotonVision IO layer, which owns the {@code PhotonCamera} it has
   * to wrap.
   */
  void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
    m_sim.addCamera(cameraSim, robotToCamera);
  }
}
