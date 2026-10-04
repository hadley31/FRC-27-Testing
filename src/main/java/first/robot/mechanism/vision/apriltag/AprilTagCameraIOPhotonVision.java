package first.robot.mechanism.vision.apriltag;

import static org.wpilib.units.Units.Seconds;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

public class AprilTagCameraIOPhotonVision implements AprilTagCameraIO {
  /**
   * Published where a solve reports no usable quality number, matching PhotonVision's own
   * convention for the same thing.
   *
   * <p>Deliberately not the worst case. An absent ambiguity or reprojection error means the solve
   * had no use for one, not that it was bad, so publishing 1.0 here would have the filter downstream
   * discard every single-tag frame on the strength of a number the single-tag strategy never
   * computes.
   */
  private static final double NOT_REPORTED = -1.0;

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final RobotHeadingSource m_headingSource;

  /**
   * @param config        the camera to read
   * @param field         the tag layout to solve against
   * @param headingSource the robot heading single-tag solves need; see {@link RobotHeadingSource}
   */
  public AprilTagCameraIOPhotonVision(
      AprilTagCameraConfig config, Field field, RobotHeadingSource headingSource) {
    m_camera = new PhotonCamera(config.name());
    m_poseEstimator = new PhotonPoseEstimator(field, config.robotToCamera());
    m_headingSource = headingSource;
  }

  /**
   * The same camera, with a simulated one behind it feeding the NetworkTables topics a coprocessor
   * would normally publish.
   *
   * <p>Simulation gets a factory here rather than an IO class of its own because there is nothing
   * for a second IO to implement: PhotonVision simulates the camera, not the reading of it, so
   * {@link #updateInputs} below is the same code either way. Only the wiring differs, and this is
   * the wiring.
   *
   * @param visionSim the simulated field to place this camera on
   */
  public static AprilTagCameraIOPhotonVision simulated(
      AprilTagCameraConfig config,
      Field field,
      RobotHeadingSource headingSource,
      PhotonAprilTagVisionSim visionSim) {
    AprilTagCameraIOPhotonVision io = new AprilTagCameraIOPhotonVision(config, field, headingSource);
    visionSim.addCamera(
        new PhotonCameraSim(io.m_camera, simulatedCameraProperties(), field), config.robotToCamera());
    return io;
  }

  /**
   * A global-shutter camera of the kind these cameras usually are, with enough calibration error
   * and latency that the filtering above this layer has something to do. Built fresh per camera
   * because each carries its own noise generator.
   */
  private static SimCameraProperties simulatedCameraProperties() {
    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(1600, 1304, Rotation2d.fromDegrees(80.0));
    properties.setCalibError(0.25, 0.08);
    properties.setFPS(15.0);
    properties.setAvgLatencyMs(35.0);
    properties.setLatencyStdDevMs(5.0);
    return properties;
  }

  /**
   * Drains the camera's result queue and turns every frame in it into an estimate.
   *
   * <p>PhotonVision buffers the results it has published since the last read and hands back all of
   * them at once, so this must run exactly once per loop: reading twice splits the frames between
   * two callers, and not reading often enough drops the oldest ones once the queue (20 deep) fills.
   * Keeping every frame rather than only the newest is what lets the pose estimator replay each
   * measurement at the time it was actually captured.
   */
  @Override
  public void updateInputs(AprilTagCameraIOInputsAutoLogged inputs) {
    // Single-tag solves take the robot's heading as given and only solve for position, so they need
    // the heading as it was when the frame was captured, not as it is now: the frames read below are
    // already tens of milliseconds old. Sampling every loop, before the frames are read, is what
    // leaves the estimator's one-second heading buffer able to interpolate back to a capture time.
    // Without it estimatePnpDistanceTrigSolvePose finds no heading and returns empty for every
    // single-tag frame -- silently, and indistinguishably from a camera that sees nothing.
    m_poseEstimator.addHeadingData(
        RobotController.getMeasureTime().in(Seconds), m_headingSource.getRobotHeading());

    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

    // Sized to the whole queue, which is an upper bound: a frame that fails to solve contributes
    // nothing. The arrays are filled together and trimmed together, because everything above
    // this layer reads them by a shared index.
    double[] timestampsSeconds = new double[results.size()];
    Pose3d[] observedRobotPoses = new Pose3d[results.size()];
    double[] ambiguities = new double[results.size()];
    double[] reprojectionErrorsPixels = new double[results.size()];
    int[][] tagIds = new int[results.size()][];
    double[][] tagRangesMeters = new double[results.size()][];
    int count = 0;

    for (PhotonPipelineResult result : results) {
      Optional<MultiTargetPNPResult> multiTagResult = result.getMultiTagResult();

      // The coprocessor only publishes a multi-tag solve when it saw enough tags to run one; when
      // it did, it is the better estimate, and when it did not, the tag's distance and the robot's
      // own heading are all there is to go on.
      Optional<EstimatedRobotPose> estimate = multiTagResult.isPresent()
          ? m_poseEstimator.estimateCoprocMultiTagPose(result)
          : m_poseEstimator.estimatePnpDistanceTrigSolvePose(result);

      if (estimate.isEmpty()) {
        continue;
      }

      timestampsSeconds[count] = estimate.get().timestampSeconds;
      observedRobotPoses[count] = estimate.get().estimatedPose;

      ambiguities[count] = multiTagResult
          .map(r -> r.estimatedPose.ambiguity)
          .orElseGet(() -> ambiguityOf(estimate.get()));
      reprojectionErrorsPixels[count] = multiTagResult
          .map(r -> r.estimatedPose.bestReprojErr)
          .orElse(NOT_REPORTED);
      tagIds[count] = multiTagResult
          .map(r -> r.fiducialIDsUsed.stream().mapToInt(Short::intValue).toArray())
          .orElseGet(() -> tagIdsOf(estimate.get()));
      tagRangesMeters[count] = measuredRangesOf(result.getTargets(), tagIds[count]);
      count++;
    }

    inputs.connected = m_camera.isConnected();
    inputs.enabled = m_camera.isConnected() && m_camera.getEnabled();
    inputs.timestampsSeconds = Arrays.copyOf(timestampsSeconds, count);
    inputs.observedRobotPoses = Arrays.copyOf(observedRobotPoses, count);
    inputs.ambiguities = Arrays.copyOf(ambiguities, count);
    inputs.reprojectionErrorsPixels = Arrays.copyOf(reprojectionErrorsPixels, count);
    inputs.tagIds = Arrays.copyOf(tagIds, count);
    inputs.tagRangesMeters = Arrays.copyOf(tagRangesMeters, count);
    inputs.pipelineId = m_camera.getPipelineIndex();
  }

  private static double ambiguityOf(EstimatedRobotPose estimate) {
    return estimate.targetsUsed.stream()
        .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
        .filter(ambiguity -> ambiguity >= 0.0)
        .min()
        .orElse(NOT_REPORTED);
  }

  /**
   * How far the camera measured each of {@code tagIds} to be, leaving out any whose transform it
   * could not supply.
   *
   * <p>A target with no 3D data comes back carrying an identity transform, whose norm is zero. Zero
   * is the most dangerous possible value here -- it would tell the filter the robot is standing on
   * the tag, and be believed -- so it is dropped rather than published, and a solve none of whose
   * tags could be measured publishes an empty row for the filter to notice.
   */
  private static double[] measuredRangesOf(List<PhotonTrackedTarget> targets, int[] tagIds) {
    Set<Integer> used = Arrays.stream(tagIds).boxed().collect(Collectors.toSet());

    return targets.stream()
        .filter(target -> used.contains(target.getFiducialId()))
        .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
        .filter(range -> range > 0.0)
        .toArray();
  }

  private static int[] tagIdsOf(EstimatedRobotPose estimate) {
    return estimate.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
  }

  @Override
  public int getPipelineIndex() {
    return m_camera.getPipelineIndex();
  }

  @Override
  public void setPipelineIndex(int index) {
    m_camera.setPipelineIndex(index);
  }
}
