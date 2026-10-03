package first.robot.mechanism.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

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
import first.robot.mechanism.vision.apriltag.AprilTagCameraIO;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOInputsAutoLogged;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOPhotonVision;
import first.robot.mechanism.vision.apriltag.PhotonVisionSim;
import first.robot.mechanism.vision.apriltag.RobotHeadingSource;

/**
 * Covers the single-tag path, which is the one that needs something from outside the camera.
 *
 * <p>One tag fixes only the camera's distance and bearing; the robot's own heading is what turns
 * those into a field position, so {@link AprilTagCameraIOPhotonVision} has to keep PhotonVision's
 * heading buffer fed or every single-tag frame is silently dropped. Both halves of that are worth
 * pinning down, because both fail quietly: an unfed buffer looks exactly like a camera that sees
 * nothing, and a buffer fed the wrong rotation looks exactly like a camera that works.
 */
public class AprilTagSingleTagSolveTest {
  /** How far in front of a tag to stand. Close enough that only the one tag is in frame. */
  private static final double STANDOFF_METERS = 1.0;

  private static final Field FIELD = Fields.DEFAULT_FIELD.loadField();

  /** A camera at the robot's origin looking straight ahead, level. */
  private static final Transform3d ROBOT_TO_CAMERA = new Transform3d();

  /** A pose from which exactly one tag is visible, found once and shared by the tests below. */
  private static Pose2d singleTagPose;

  @BeforeAll
  static void initializeHal() {
    // The simulated camera publishes its results over NetworkTables, so the HAL has to be up, and
    // the driver station refreshed before AdvantageKit's conduit is touched.
    HAL.initialize();
    DriverStationBackend.refreshData();

    singleTagPose = findSingleTagPose().orElseThrow(
        () -> new IllegalStateException(
            "no tag on %s is isolated enough to be seen alone from %.1f m"
                .formatted(Fields.DEFAULT_FIELD, STANDOFF_METERS)));
  }

  /**
   * Hunts for a viewpoint that yields a one-tag frame rather than hard-coding one, since which tags
   * stand alone is a property of whichever field happens to be current.
   */
  private static Optional<Pose2d> findSingleTagPose() {
    for (var tag : FIELD.getTags()) {
      Pose2d candidate = facing(FIELD.getTagPose(tag.getID()).orElseThrow());
      var inputs = observe("TagSearch" + tag.getID(), candidate, candidate::getRotation);

      if (inputs.tagIds.length == 1 && inputs.tagIds[0].length == 1) {
        return Optional.of(candidate);
      }
    }

    return Optional.empty();
  }

  /** A pose {@link #STANDOFF_METERS} in front of a tag, facing it. */
  private static Pose2d facing(Pose3d tagPose) {
    Rotation2d tagYaw = tagPose.getRotation().toRotation2d();
    Translation2d standoff = new Translation2d(STANDOFF_METERS, tagYaw);

    return new Pose2d(
        tagPose.getTranslation().toTranslation2d().plus(standoff),
        tagYaw.plus(Rotation2d.k180deg));
  }

  /**
   * Puts a fresh simulated camera at {@code truePose} and reads it until it reports a frame.
   *
   * @param cameraName  must be unique per call, since the cameras share one NetworkTables instance
   * @param truePose    where the robot really is, which is what the simulation renders from
   * @param headingSource what the camera is told the robot's heading is, which is the thing under
   *                      test and so is not necessarily {@code truePose}'s rotation
   */
  private static AprilTagCameraIOInputsAutoLogged observe(
      String cameraName, Pose2d truePose, RobotHeadingSource headingSource) {
    PhotonVisionSim visionSim = new PhotonVisionSim(FIELD);
    AprilTagCameraIO io = AprilTagCameraIOPhotonVision.simulated(
        new AprilTagCameraConfig(cameraName, ROBOT_TO_CAMERA), FIELD, headingSource, visionSim);

    var inputs = new AprilTagCameraIOInputsAutoLogged();

    // The camera runs with latency and its results reach the IO over NetworkTables, so a frame is
    // not guaranteed to be readable on the first pass. Pump until one arrives.
    for (int i = 0; i < 60 && inputs.observedRobotPoses.length == 0; i++) {
      visionSim.update(truePose);
      io.updateInputs(inputs);
    }

    return inputs;
  }

  @Test
  void aOneTagFrameIsSolvedWhenTheHeadingIsFed() {
    var inputs = observe("SingleTagCorrectHeading", singleTagPose, singleTagPose::getRotation);

    assertEquals(1, inputs.observedRobotPoses.length, "expected the one-tag frame to be solved");
    assertEquals(1, inputs.tagIds[0].length, "expected the solve to have used a single tag");

    double error = inputs.observedRobotPoses[0].toPose2d().getTranslation()
        .getDistance(singleTagPose.getTranslation());
    assertTrue(error < 0.2, "estimate was %.3f m from the true pose".formatted(error));
  }

  @Test
  void theSuppliedHeadingIsWhatTheOneTagSolveIsBuiltOn() {
    // A heading a quarter turn off. The camera still sees the tag at the same distance and bearing,
    // so if the reported pose barely moves, the solve is not actually using what it was handed and
    // the wiring being tested is decorative.
    RobotHeadingSource wrongHeading = () -> singleTagPose.getRotation().plus(Rotation2d.fromDegrees(90.0));
    var inputs = observe("SingleTagWrongHeading", singleTagPose, wrongHeading);

    assertEquals(1, inputs.observedRobotPoses.length, "expected the one-tag frame to be solved");

    Pose2d estimate = inputs.observedRobotPoses[0].toPose2d();
    assertEquals(
        wrongHeading.getRobotHeading().getRadians(),
        estimate.getRotation().getRadians(),
        1e-6,
        "the solve should report back the heading it was given");

    double error = estimate.getTranslation().getDistance(singleTagPose.getTranslation());
    assertTrue(error > 0.5,
        "a quarter-turn heading error moved the estimate only %.3f m, so the heading is not reaching the solve"
            .formatted(error));
  }
}
