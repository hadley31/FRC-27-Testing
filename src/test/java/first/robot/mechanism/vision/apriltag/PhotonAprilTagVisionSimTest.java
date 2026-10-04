package first.robot.mechanism.vision.apriltag;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;


/**
 * Drives the simulated camera end to end: place the robot somewhere it can see a tag, render the
 * field, and read the result back out through the same IO the real robot uses.
 *
 * <p>This is the only check that the simulation is wired to the thing it is meant to feed. The
 * pieces each look right in isolation — the camera publishes, the IO subscribes — but they only
 * meet over NetworkTables, where a wrong camera name or an unregistered camera fails silently and
 * looks exactly like a camera that can see no tags.
 */
public class PhotonAprilTagVisionSimTest {
  private static final Field FIELD = Fields.DEFAULT_FIELD.loadField();

  /** A camera at the robot's origin looking straight ahead, level. */
  private static final Transform3d ROBOT_TO_CAMERA = new Transform3d();

  @BeforeAll
  static void initializeHal() {
    // The simulated camera publishes a video stream and its results over NetworkTables, so the HAL
    // has to be up, and the driver station refreshed before AdvantageKit's conduit is touched.
    HAL.initialize();
    DriverStationBackend.refreshData();
  }

  /** A pose a couple of metres in front of a tag, facing it. */
  private static Pose2d facing(Pose3d tagPose) {
    Rotation2d tagYaw = tagPose.getRotation().toRotation2d();
    Translation2d standoff = new Translation2d(2.0, tagYaw);

    return new Pose2d(
        tagPose.getTranslation().toTranslation2d().plus(standoff),
        tagYaw.plus(Rotation2d.k180deg));
  }

  @Test
  void aCameraLookingAtATagProducesAnEstimateNearTheRobotsTruePose() {
    Pose3d tagPose = FIELD.getTagPose(FIELD.getTags().get(0).getID()).orElseThrow();
    Pose2d truePose = facing(tagPose);

    PhotonAprilTagVisionSim visionSim = new PhotonAprilTagVisionSim(FIELD, () -> truePose);
    AprilTagCameraIO io = AprilTagCameraIOPhotonVision.simulated(
        new AprilTagCameraConfig("SimTestCamera", ROBOT_TO_CAMERA),
        FIELD,
        truePose::getRotation,
        visionSim);

    AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

    // The camera runs at a frame rate with latency, and results reach the IO over NetworkTables, so
    // a frame is not guaranteed to be readable on the first pass. Pump until one arrives.
    for (int i = 0; i < 50 && inputs.observedRobotPoses.length == 0; i++) {
      visionSim.update();
      io.updateInputs(inputs);
    }

    assertEquals(1, inputs.observedRobotPoses.length, "expected the simulated camera to report a frame");
    assertTrue(inputs.tagIds[0].length > 0, "expected the estimate to name the tags behind it");

    double error = inputs.observedRobotPoses[0].toPose2d().getTranslation().getDistance(truePose.getTranslation());
    assertTrue(error < 0.5, "estimate was %.3f m from the true pose".formatted(error));
  }


  // MARK: - Pose resets

  /** How often the simulation is rendered while the teleport test is waiting on the camera. */
  private static final long PUMP_INTERVAL_MILLIS = 2;

  /** Enough pumping to cover more than one frame period at any plausible frame rate. */
  private static final int PUMP_LIMIT = 150;

  /**
   * A robot told it has been teleported is rendered where it has been put — by the simulation these
   * cameras are actually looking at.
   *
   * <p>Which is the whole job of {@code resetRobotPose}. The simulation renders each frame from the
   * pose at that frame's capture time, so it keeps a trail of recent poses, and a teleport leaves a
   * discontinuity in that trail for the renderer to interpolate across. What this pins down is that
   * clearing the trail reaches the instance the cameras render from, which is the half of the job
   * that is ours to get wrong; what goes wrong when the trail is not cleared is PhotonVision's
   * behaviour, and is described on {@link AprilTagVisionSim#resetRobotPose} rather than asserted
   * here.
   */
  @Test
  void aTeleportTheSimulationIsToldAboutIsRenderedAtTheNewPose() throws InterruptedException {
    Pose3d tagPose = FIELD.getTagPose(FIELD.getTags().get(0).getID()).orElseThrow();
    Pose2d oldPose = facing(tagPose);

    // Sideways rather than along the camera's bore because that is the axis a tag pins down best,
    // so the two candidate poses are as far apart as the camera can tell; a metre also keeps the
    // tag inside the lens.
    Pose2d newPose = oldPose.plus(new Transform2d(0.0, 1.0, Rotation2d.ZERO));

    // The simulation pulls the robot's true pose every render, the way it does on the robot, so
    // moving the robot here means moving what this holds.
    AtomicReference<Pose2d> groundTruth = new AtomicReference<>(oldPose);

    PhotonAprilTagVisionSim visionSim = new PhotonAprilTagVisionSim(FIELD, groundTruth::get);
    AprilTagCameraIO io = AprilTagCameraIOPhotonVision.simulated(
        new AprilTagCameraConfig("ResetTestCamera", ROBOT_TO_CAMERA),
        FIELD,
        newPose::getRotation,
        visionSim);

    AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

    // A frame of history at the old pose, so there is a trail for the reset to clear and the
    // assertion below would have something to catch if it were not cleared. Frames are read on the
    // loop they arrive rather than left to queue, so that what gets examined is a frame rendered
    // after the move.
    pumpForFrame(visionSim, io, inputs)
        .orElseThrow(() -> new AssertionError("expected a frame from the simulated camera"));

    groundTruth.set(newPose);
    visionSim.resetRobotPose(newPose);

    // Which frame arrives first does not matter here, whatever the camera's shutter is doing: the
    // reset leaves the pose buffer holding the new pose and nothing else, so every exposure after
    // it renders from the new pose whether it is stamped before or after the move.
    Pose3d observed = pumpForFrame(visionSim, io, inputs)
        .orElseThrow(() -> new AssertionError("expected a frame after the move"));

    Translation2d reported = observed.toPose2d().getTranslation();
    double fromNewPose = reported.getDistance(newPose.getTranslation());
    double fromOldPose = reported.getDistance(oldPose.getTranslation());

    assertTrue(fromNewPose < fromOldPose,
        "the first frame after the reset should describe the new pose: %.3f m from it against %.3f m from the old one"
            .formatted(fromNewPose, fromOldPose));
    assertTrue(fromNewPose < 0.5, "estimate was %.3f m from the new pose".formatted(fromNewPose));
  }

  /**
   * Renders the simulation until a frame reaches the IO, and hands back the pose it was solved to.
   *
   * @return the pose, or empty if the camera produced no frame within the budget
   */
  private static Optional<Pose3d> pumpForFrame(
      PhotonAprilTagVisionSim visionSim,
      AprilTagCameraIO io,
      AprilTagCameraIOInputsAutoLogged inputs) throws InterruptedException {
    for (int i = 0; i < PUMP_LIMIT; i++) {
      visionSim.update();
      io.updateInputs(inputs);

      // Index zero is the oldest frame in the batch, which is the first one the camera produced
      // since the last read -- the one closest to whatever just happened.
      if (inputs.observedRobotPoses.length > 0) {
        return Optional.of(inputs.observedRobotPoses[0]);
      }

      Thread.sleep(PUMP_INTERVAL_MILLIS);
    }

    return Optional.empty();
  }
}
