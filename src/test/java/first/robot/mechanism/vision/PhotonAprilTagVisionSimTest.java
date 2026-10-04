package first.robot.mechanism.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

import first.robot.mechanism.vision.apriltag.AprilTagCameraConfig;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIO;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOInputsAutoLogged;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOPhotonVision;
import first.robot.mechanism.vision.apriltag.PhotonAprilTagVisionSim;

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

  /**
   * A robot told it has been teleported is rendered where it has been put.
   *
   * <p>Which is the whole job of {@code resetRobotPose}, and the paired test below is what shows it
   * is doing it: the simulation renders each frame from the pose at that frame's capture time, so
   * without being told it keeps a trail of poses leading back to where the robot came from and
   * renders across the jump.
   */
  @Test
  void aTeleportTheSimulationIsToldAboutIsRenderedAtTheNewPose() throws InterruptedException {
    Teleport teleport = teleportAfterBuildingHistory(true);

    assertTrue(teleport.distanceFromNewPose() < teleport.distanceFromOldPose(),
        "the first frame after the reset should describe the new pose: %.3f m from it against %.3f m from the old one"
            .formatted(teleport.distanceFromNewPose(), teleport.distanceFromOldPose()));
    assertTrue(teleport.distanceFromNewPose() < 0.5,
        "estimate was %.3f m from the new pose".formatted(teleport.distanceFromNewPose()));
  }

  /**
   * The failure {@code resetRobotPose} exists to prevent, pinned down so that the test above cannot
   * pass for the wrong reason.
   *
   * <p>Not a test of our own code but a characterisation of PhotonVision's: moving the pose the
   * simulation is handed, without clearing its history, leaves it rendering the robot back where it
   * was — on frames stamped after the move, which is what makes them undetectable downstream. If
   * this ever starts failing, PhotonVision has changed and {@code resetRobotPose} may no longer be
   * needed.
   */
  @Test
  void aTeleportTheSimulationIsNotToldAboutIsStillRenderedAtTheOldPose() throws InterruptedException {
    Teleport teleport = teleportAfterBuildingHistory(false);

    assertTrue(teleport.distanceFromOldPose() < teleport.distanceFromNewPose(),
        "an untold simulation should still be rendering the old pose: %.3f m from it against %.3f m from the new one"
            .formatted(teleport.distanceFromOldPose(), teleport.distanceFromNewPose()));
  }

  /** Where the first frame after a teleport put the robot, relative to the two candidate poses. */
  private record Teleport(double distanceFromOldPose, double distanceFromNewPose) {
  }

  /**
   * Builds up a pose history, moves the robot a metre sideways, and reports where the next frame
   * placed it.
   *
   * <p>Sideways rather than along the camera's bore because that is the axis a tag pins down best,
   * so the two candidate poses are as far apart as the camera can tell; a metre also keeps the tag
   * inside the lens.
   *
   * @param tellTheSimulation whether to call {@code resetRobotPose}, which is the one thing the two
   *                          tests above differ by
   */
  private static Teleport teleportAfterBuildingHistory(boolean tellTheSimulation)
      throws InterruptedException {
    Pose3d tagPose = FIELD.getTagPose(FIELD.getTags().get(0).getID()).orElseThrow();
    Pose2d oldPose = facing(tagPose);
    Pose2d newPose = oldPose.plus(new Transform2d(0.0, 1.0, Rotation2d.ZERO));

    // The simulation pulls the robot's true pose every render, the way it does on the robot, so
    // moving the robot here means moving what this holds.
    AtomicReference<Pose2d> groundTruth = new AtomicReference<>(oldPose);

    PhotonAprilTagVisionSim visionSim = new PhotonAprilTagVisionSim(FIELD, groundTruth::get);
    AprilTagCameraIO io = AprilTagCameraIOPhotonVision.simulated(
        new AprilTagCameraConfig("ResetTestCamera" + tellTheSimulation, ROBOT_TO_CAMERA),
        FIELD,
        newPose::getRotation,
        visionSim);

    AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

    // Long enough to span the camera's latency, so there is a history to be wrongly interpolated
    // across. Frames are read and discarded rather than left to queue, so that what gets examined
    // below is a frame rendered after the move.
    for (int i = 0; i < 20; i++) {
      visionSim.update();
      io.updateInputs(inputs);
      Thread.sleep(5);
    }

    groundTruth.set(newPose);
    if (tellTheSimulation) {
      visionSim.resetRobotPose(newPose);
    }

    inputs.observedRobotPoses = new Pose3d[0];
    for (int i = 0; i < 60 && inputs.observedRobotPoses.length == 0; i++) {
      visionSim.update();
      io.updateInputs(inputs);
      Thread.sleep(5);
    }

    assertTrue(inputs.observedRobotPoses.length > 0, "expected a frame after the move");

    Translation2d reported = inputs.observedRobotPoses[0].toPose2d().getTranslation();

    return new Teleport(
        reported.getDistance(oldPose.getTranslation()),
        reported.getDistance(newPose.getTranslation()));
  }
}
