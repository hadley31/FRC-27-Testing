package first.robot.mechanism.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

import first.robot.mechanism.vision.apriltag.AprilTagCameraConfig;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOInputsAutoLogged;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIO;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOPhotonVision;
import first.robot.mechanism.vision.apriltag.PhotonVisionSim;

/**
 * Drives the simulated camera end to end: place the robot somewhere it can see a tag, render the
 * field, and read the result back out through the same IO the real robot uses.
 *
 * <p>This is the only check that the simulation is wired to the thing it is meant to feed. The
 * pieces each look right in isolation — the camera publishes, the IO subscribes — but they only
 * meet over NetworkTables, where a wrong camera name or an unregistered camera fails silently and
 * looks exactly like a camera that can see no tags.
 */
public class PhotonVisionSimTest {
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

    PhotonVisionSim visionSim = new PhotonVisionSim(FIELD);
    AprilTagCameraIO io = AprilTagCameraIOPhotonVision.simulated(
        new AprilTagCameraConfig("SimTestCamera", ROBOT_TO_CAMERA),
        FIELD,
        truePose::getRotation,
        visionSim);

    AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

    // The camera runs at 30fps with latency, and results reach the IO over NetworkTables, so a
    // frame is not guaranteed to be readable on the first pass. Pump until one arrives.
    for (int i = 0; i < 50 && inputs.observedRobotPoses.length == 0; i++) {
      visionSim.update(truePose);
      io.updateInputs(inputs);
    }

    assertEquals(1, inputs.observedRobotPoses.length, "expected the simulated camera to report a frame");
    assertTrue(inputs.tagIds[0].length > 0, "expected the estimate to name the tags behind it");

    double error = inputs.observedRobotPoses[0].toPose2d().getTranslation().getDistance(truePose.getTranslation());
    assertTrue(error < 0.5, "estimate was %.3f m from the true pose".formatted(error));
  }
}
