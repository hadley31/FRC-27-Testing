package first.robot.mechanism.vision.apriltag;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The simulated field every simulated PhotonVision camera looks at.
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
public class PhotonVisionSim {
  private final VisionSystemSim m_sim;

  /**
   * @param field the tag layout to render
   */
  public PhotonVisionSim(Field field) {
    m_sim = new VisionSystemSim("main");
    m_sim.addAprilTags(field);
  }

  /**
   * Moves the robot on the simulated field and renders what every camera on it can now see, which
   * is what puts results on the NetworkTables topics the IO layer reads. Call once per loop,
   * before the cameras are read.
   *
   * @param groundTruthRobotPose where the robot really is. This must be a pose that no vision
   *                             observation has corrected — {@code PoseEstimator#getOdometryPose}
   *                             rather than the estimate. Rendering sightings from a
   *                             vision-corrected pose makes them agree with the correction by
   *                             construction, and the filter above can never be wrong, or right.
   */
  public void update(Pose2d groundTruthRobotPose) {
    m_sim.update(groundTruthRobotPose);
  }

  /**
   * Places a simulated camera on the robot. Package-private because the only thing that should be
   * building a {@link PhotonCameraSim} is the PhotonVision IO layer, which owns the
   * {@code PhotonCamera} it has to wrap.
   */
  void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
    m_sim.addCamera(cameraSim, robotToCamera);
  }
}
