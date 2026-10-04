package first.robot.mechanism.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.RobotController;

import first.robot.mechanism.vision.apriltag.AprilTagCameraConfig;
import first.robot.mechanism.vision.apriltag.AprilTagPoseObservation;
import first.robot.mechanism.vision.apriltag.AprilTagVisionProcessor;
import first.robot.util.PoseEstimator.VisionObservation;

/**
 * Pins the shape of the vision log, which is load-bearing for a reason that is easy to miss: the
 * coefficients in the error model are meant to be fitted from this log, so a log that misrepresents
 * what happened is a model fitted to the wrong data.
 *
 * <p>The log describes one observation per camera per loop, because a camera delivering several
 * frames in a loop is rare. Two things make that thinning safe rather than merely cheap, and neither
 * is visible from the call site, which is why these read the log back: the counts have to stay honest
 * about how many frames there really were, and the frame chosen to be described has to be the one
 * that represents what the filter did -- never a rejected frame on a loop where a different frame was
 * accepted.
 *
 * <p>One test method covering five simulated loops, rather than five methods: AdvantageKit's
 * {@link Logger} is a static singleton whose receiver thread cannot be restarted once ended, so a
 * JVM gets one logging session and the build already forks one JVM per test class to suit.
 */
class AprilTagVisionProcessorLoggingTest {
  private static final String kPrefix = "RealOutputs/Vision/";

  /** A tag on a wall ahead of the robot faces back along -x. */
  private static final Rotation3d FACING_BACK = new Rotation3d(0.0, 0.0, Math.PI);

  private static final AprilTagCameraConfig CAMERA = camera("FakeCamera");
  private static final AprilTagCameraConfig OTHER_CAMERA = camera("OtherCamera");

  /**
   * An observation whose camera supplied no measured tag ranges, so that the model falls back to the
   * distances the solved pose implies and these tests can place tags at known distances.
   */
  private static final double[] NO_MEASURED_RANGES = new double[0];

  /** A pose a metre off the floor, which no gate lets through. */
  private static final Pose3d AIRBORNE =
      new Pose3d(new Translation3d(0.0, 0.0, 1.0), new Rotation3d());

  /** Captures the log entry flushed at the end of each simulated loop. */
  private static final class RecordingReceiver implements LogDataReceiver {
    final List<LogTable> entries = new ArrayList<>();

    @Override
    public void putTable(LogTable table) {
      entries.add(LogTable.clone(table));
    }
  }

  @Test
  void theLoggedObservationRepresentsTheLoopItCameFrom() {
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    // AdvantageKit's conduit reads system telemetry over the system server's NetworkTables
    // instance, and segfaults if it is touched before the driver station has been refreshed once.
    DriverStationBackend.refreshData();

    // Tags at 2, 3 and 4 m, so the range logged for a loop says which of its frames was described,
    // and all three are inside the distance gate so that nothing here is rejected for being far.
    Field field = fieldWithTagsAt(
        new Translation3d(2.0, 0.0, 0.0),
        new Translation3d(3.0, 0.0, 0.0),
        new Translation3d(4.0, 0.0, 0.0));

    List<VisionObservation> accepted = new ArrayList<>();
    AprilTagVisionProcessor processor =
        new AprilTagVisionProcessor(field, ChassisVelocities::new, () -> true, accepted::add);

    RecordingReceiver receiver = new RecordingReceiver();
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.addDataReceiver(receiver);
    Logger.start();

    try {
      // A backlog of three good frames, oldest first, as getAllUnreadResults hands them over.
      loop(() -> processor.process(List.of(
          observation(CAMERA, Pose3d.ZERO, Set.of(1), now() - 0.06),
          observation(CAMERA, Pose3d.ZERO, Set.of(2), now() - 0.03),
          observation(CAMERA, Pose3d.ZERO, Set.of(3), now()))));

      // An accepted frame followed by a rejected one, so that the described frame and the verdict
      // for the loop disagree and the counts have to be what resolves them.
      loop(() -> processor.process(List.of(
          observation(CAMERA, Pose3d.ZERO, Set.of(1), now() - 0.02),
          observation(CAMERA, AIRBORNE, Set.of(2), now()))));

      // Two frames, both rejected, this time for staleness so that their poses stay put and the
      // range still says which one was described.
      loop(() -> processor.process(List.of(
          observation(CAMERA, Pose3d.ZERO, Set.of(1), now() - 5.0),
          observation(CAMERA, Pose3d.ZERO, Set.of(2), now() - 4.0))));

      // Nothing at all, from a camera unplugged or simply not finished with a frame.
      loop(() -> processor.process(List.of()));

      // Two cameras at once, contributing different frames.
      loop(() -> processor.process(List.of(
          observation(CAMERA, Pose3d.ZERO, Set.of(1), now()),
          observation(OTHER_CAMERA, Pose3d.ZERO, Set.of(3), now()))));
    } finally {
      // Entries are handed to receivers on a separate thread; end() joins it.
      Logger.end();
    }

    assertEquals(5, receiver.entries.size(), "expected one flushed entry per simulated loop");

    // Three from the backlog, one of the two mixed verdicts, none from the two stale ones, none from
    // the silent loop, two from the two cameras. Thinning the log never thins what the filter gets.
    assertEquals(6, accepted.size(), "every frame that passed the gates should have been submitted");

    LogTable backlog = receiver.entries.get(0);
    LogTable mixedVerdicts = receiver.entries.get(1);
    LogTable allRejected = receiver.entries.get(2);
    LogTable silent = receiver.entries.get(3);
    LogTable twoCameras = receiver.entries.get(4);

    // MARK: - A backlog is counted in full and described by its newest frame

    assertEquals(3, backlog.get(kPrefix + "FakeCamera/ObservationCount", -1),
        "a backlog must be counted in full even though one frame is described");
    assertEquals(3, backlog.get(kPrefix + "FakeCamera/AcceptedCount", -1));
    // The 4 m tag, which the newest frame saw; ranges are from the lens, 0.35 m ahead of the origin.
    assertEquals(3.65, number(backlog, "FakeCamera/MostRecent/RangeDistance"), 1.0e-6,
        "the newest of three equally good frames should be the one described");
    assertEquals(3, backlog.get(kPrefix + "FakeCamera/AcceptedRobotPoses", new Pose3d[0]).length,
        "every pose is still logged, since that is what a field view reads");
    // The tag the newest frame saw, from the layout, so the described frame's tags and its range
    // agree about which frame is being described.
    assertArrayEquals(
        new Pose3d[] {new Pose3d(new Translation3d(4.0, 0.0, 0.0), FACING_BACK)},
        backlog.get(kPrefix + "FakeCamera/MostRecent/TagPoses", new Pose3d[0]),
        "the described frame's tags should be the ones it was solved from");
    assertEquals(0, backlog.get(kPrefix + "FakeCamera/RejectedRobotPoses", new Pose3d[0]).length,
        "nothing in this loop was rejected");

    // MARK: - The newest frame is described, and the counts carry the loop's verdict

    assertEquals(2, mixedVerdicts.get(kPrefix + "FakeCamera/ObservationCount", -1));

    // The described frame is the newest, which here is the rejected one. That is why the counts
    // exist: on their own the fields below would read as a loop where vision contributed nothing,
    // and this one corrected the estimate.
    assertFalse(mixedVerdicts.get(kPrefix + "FakeCamera/MostRecent/Accepted", true),
        "the newest frame was rejected, so the described frame should say so");
    assertEquals(1, mixedVerdicts.get(kPrefix + "FakeCamera/AcceptedCount", -1),
        "the count is what records that a frame did get through");

    // Both poses are logged, split by verdict so a field view can draw them apart.
    assertArrayEquals(new Pose3d[] {Pose3d.ZERO},
        mixedVerdicts.get(kPrefix + "FakeCamera/AcceptedRobotPoses", new Pose3d[0]),
        "the accepted pose is the one that moved the estimate");
    assertArrayEquals(new Pose3d[] {AIRBORNE},
        mixedVerdicts.get(kPrefix + "FakeCamera/RejectedRobotPoses", new Pose3d[0]),
        "the rejected pose is still worth seeing, to work out why it was rejected");

    // MARK: - With nothing accepted, the newest rejection is described

    assertEquals(2, allRejected.get(kPrefix + "FakeCamera/ObservationCount", -1));
    assertEquals(0, allRejected.get(kPrefix + "FakeCamera/AcceptedCount", -1));
    assertFalse(allRejected.get(kPrefix + "FakeCamera/MostRecent/Accepted", true));
    assertTrue(allRejected.get(kPrefix + "FakeCamera/MostRecent/RejectionReason", "").startsWith("Observation too stale"),
        "a loop that accepted nothing should still say why");
    // The 3 m tag, which the newer of the two stale frames saw.
    assertEquals(2.65, number(allRejected, "FakeCamera/MostRecent/RangeDistance"), 1.0e-6,
        "the newest rejection should be the one described");
    assertTrue(Double.isNaN(number(allRejected, "FakeCamera/MostRecent/StdDevs/X")),
        "a rejected observation's standard deviations should be absent, not zero");
    assertEquals(0, allRejected.get(kPrefix + "FakeCamera/AcceptedRobotPoses", new Pose3d[0]).length);
    assertArrayEquals(
        new Pose3d[] {new Pose3d(new Translation3d(3.0, 0.0, 0.0), FACING_BACK)},
        allRejected.get(kPrefix + "FakeCamera/MostRecent/TagPoses", new Pose3d[0]),
        "a rejected frame still says which tags it came from");
    assertEquals(2, allRejected.get(kPrefix + "FakeCamera/RejectedRobotPoses", new Pose3d[0]).length,
        "a loop that accepted nothing still shows where its frames thought the robot was");

    // MARK: - A silent camera says so rather than saying nothing

    assertEquals(0, silent.get(kPrefix + "FakeCamera/ObservationCount", -1),
        "a camera that reported nothing must say so");
    assertEquals(0, silent.get(kPrefix + "FakeCamera/AcceptedRobotPoses", new Pose3d[0]).length,
        "a silent camera draws nothing on the field");
    assertEquals(0, silent.get(kPrefix + "FakeCamera/RejectedRobotPoses", new Pose3d[0]).length,
        "a silent camera draws nothing on the field");
    assertEquals(-1, silent.get(kPrefix + "FakeCamera/MostRecent/TagCount", 0));
    assertEquals(0, silent.get(kPrefix + "FakeCamera/MostRecent/TagPoses", new Pose3d[0]).length,
        "a silent camera was solved from no tags, rather than still from its last ones");

    for (String key : List.of(
        "MostRecent/RangeDistance",
        "MostRecent/BearingDistance",
        "MostRecent/NearestTagDistance",
        "MostRecent/IncidenceCosine",
        "MostRecent/TagSpread",
        "MostRecent/TagBearingRadians",
        "MostRecent/HeightError",
        "MostRecent/TiltError",
        "MostRecent/Ambiguity",
        "MostRecent/ReprojectionErrorPixels",
        "MostRecent/LatencySeconds",
        "MostRecent/StdDevs/X",
        "MostRecent/StdDevs/Y",
        "MostRecent/StdDevs/Theta")) {
      assertTrue(Double.isNaN(number(silent, "FakeCamera/" + key)),
          key + " should be absent for a silent camera, not left holding the last frame");
    }

    // MARK: - Cameras are logged apart

    assertEquals(1.65, number(twoCameras, "FakeCamera/MostRecent/RangeDistance"), 1.0e-6);
    assertEquals(3.65, number(twoCameras, "OtherCamera/MostRecent/RangeDistance"), 1.0e-6,
        "the second camera's frame should be its own, not the first camera's");
  }

  // MARK: - Fixtures

  /** Runs one simulated robot loop, flushing exactly one entry to the receiver. */
  private static void loop(Runnable body) {
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    body.run();
    Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
  }

  /**
   * The time the processor will measure staleness against, which is the logger's rather than the
   * wall clock's: {@link Logger#start} takes over {@link RobotController}'s time source.
   */
  private static double now() {
    return RobotController.getMeasureTime().in(Seconds);
  }

  private static double number(LogTable entry, String key) {
    return entry.get(kPrefix + key, 0.0);
  }

  private static AprilTagPoseObservation observation(
      AprilTagCameraConfig camera, Pose3d robotPose, Set<Integer> tags, double timestampSeconds) {
    return new AprilTagPoseObservation(
        camera, robotPose, Seconds.of(timestampSeconds), tags, 0.0, -1.0, NO_MEASURED_RANGES);
  }

  /** Tags at floor level on a wall ahead of the robot, numbered from one in the order given. */
  private static Field fieldWithTagsAt(Translation3d... positions) {
    List<FieldTag> tags = new ArrayList<>(positions.length);

    for (int i = 0; i < positions.length; i++) {
      tags.add(new FieldTag(i + 1, new Pose3d(positions[i], FACING_BACK)));
    }

    return new Field("Test", "2027", "Test", null, 16.0, 8.0, "FRC", tags);
  }

  /**
   * A named camera a third of a metre in front of the robot origin, far enough out that measuring
   * tag range from the robot origin instead would be visibly wrong.
   */
  private static AprilTagCameraConfig camera(String name) {
    return new AprilTagCameraConfig(
        name, new Transform3d(new Translation3d(0.35, 0.0, 0.0), new Rotation3d()));
  }
}
