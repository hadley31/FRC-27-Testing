package first.robot.mechanism.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Seconds;

import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Time;

import first.robot.mechanism.vision.apriltag.AprilTagCamera;
import first.robot.mechanism.vision.apriltag.AprilTagCameraConfig;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIOInputsAutoLogged;
import first.robot.mechanism.vision.apriltag.AprilTagCameraIO;
import first.robot.mechanism.vision.apriltag.AprilTagPoseObservation;

/**
 * Covers the part of the multi-estimate path that only breaks at runtime: one estimate is spread
 * across several arrays, and nothing in the type system says those arrays still line up after a trip
 * through a log. These tests take that trip and check that they do.
 */
public class AprilTagCameraTest {
  private static final Pose3d POSE_A = new Pose3d(new Translation3d(1, 2, 0), new Rotation3d());
  private static final Pose3d POSE_B = new Pose3d(new Translation3d(3, 4, 0), new Rotation3d());

  /** An IO that reports a fixed set of estimates, standing in for a camera. */
  private static final class FakeCameraIO implements AprilTagCameraIO {
    private final double[] m_timestampsSeconds;
    private final Pose3d[] m_observedRobotPoses;
    private final double[] m_ambiguities;
    private final double[] m_reprojectionErrorsPixels;
    private final int[][] m_tagIds;
    private final double[][] m_tagRangesMeters;

    FakeCameraIO(double[] timestampsSeconds, Pose3d[] observedRobotPoses, double[] ambiguities,
        double[] reprojectionErrorsPixels, int[][] tagIds, double[][] tagRangesMeters) {
      m_timestampsSeconds = timestampsSeconds;
      m_observedRobotPoses = observedRobotPoses;
      m_ambiguities = ambiguities;
      m_reprojectionErrorsPixels = reprojectionErrorsPixels;
      m_tagIds = tagIds;
      m_tagRangesMeters = tagRangesMeters;
    }

    @Override
    public void updateInputs(AprilTagCameraIOInputsAutoLogged inputs) {
      inputs.timestampsSeconds = m_timestampsSeconds;
      inputs.observedRobotPoses = m_observedRobotPoses;
      inputs.ambiguities = m_ambiguities;
      inputs.reprojectionErrorsPixels = m_reprojectionErrorsPixels;
      inputs.tagIds = m_tagIds;
      inputs.tagRangesMeters = m_tagRangesMeters;
    }

    @Override
    public int getPipelineIndex() {
      return 0;
    }

    @Override
    public void setPipelineIndex(int pipelineId) {
    }
  }

  /** Fills inputs from the IO, then round-trips them through a log table the way replay does. */
  private static AprilTagCamera replayedCamera(double[] timestampsSeconds, Pose3d[] observedRobotPoses,
      double[] ambiguities, double[] reprojectionErrorsPixels, int[][] tagIds,
      double[][] tagRangesMeters) {
    AprilTagCamera camera = new AprilTagCamera(
        new AprilTagCameraConfig("FakeCamera", Transform3d.ZERO),
        new FakeCameraIO(timestampsSeconds, observedRobotPoses, ambiguities,
            reprojectionErrorsPixels, tagIds, tagRangesMeters));
    camera.getIO().updateInputs(camera.getInputs());

    LogTable table = new LogTable(0);
    camera.getInputs().toLog(table);
    camera.getInputs().fromLog(table);

    return camera;
  }

  /** Two estimates from one cycle, the shape the rest of these tests read back. */
  private static AprilTagCamera twoEstimateCamera() {
    return replayedCamera(
        new double[] { 10.0, 10.02 },
        new Pose3d[] { POSE_A, POSE_B },
        new double[] { 0.05, 0.2 },
        new double[] { 1.5, -1.0 },
        new int[][] { { 1, 2 }, { 7 } },
        new double[][] { { 2.5, 3.5 }, { 4.5 } });
  }

  @Test
  void everyEstimateFromOneCycleBecomesAnObservation() {
    List<AprilTagPoseObservation> observations = twoEstimateCamera().getObservations();

    assertEquals(2, observations.size());
  }

  @Test
  void anEstimateKeepsTheTagsItWasSolvedFrom() {
    List<AprilTagPoseObservation> observations = twoEstimateCamera().getObservations();

    assertEquals(Set.of(1, 2), observations.get(0).tags());
    assertEquals(Set.of(7), observations.get(1).tags());
  }

  /**
   * The measured tag ranges are the one input to the error model that is a measurement rather than a
   * consequence of the solve, and they are ragged: a row per estimate, of a length nothing else
   * fixes. Two tags' worth for one estimate and one tag's worth for the next has to come back that
   * way rather than squared off or flattened together.
   */
  @Test
  void aReplayedEstimateKeepsTheRangesItsCameraMeasured() {
    List<AprilTagPoseObservation> observations = twoEstimateCamera().getObservations();

    assertArrayEquals(new double[] { 2.5, 3.5 }, observations.get(0).measuredTagRanges());
    assertArrayEquals(new double[] { 4.5 }, observations.get(1).measuredTagRanges());
  }

  @Test
  void anEstimateKeepsItsOwnCaptureTimeAndPose() {
    List<AprilTagPoseObservation> observations = twoEstimateCamera().getObservations();

    assertEquals(Seconds.of(10.02), observations.get(1).timestamp());
    assertEquals(POSE_B, observations.get(1).observedRobotPose());
    assertEquals(0.2, observations.get(1).ambiguity());
    assertEquals(-1.0, observations.get(1).reprojectionErrorPixels());
  }

  /**
   * A timestamp is logged as a bare number of seconds and made a measure on the way out, because
   * AdvantageKit cannot pack the unit of a measure nested inside a record: store one and it comes
   * back from the log carrying a null unit, which blows up on the first arithmetic that touches
   * it. This pins the unit rather than the magnitude, which is the half that goes missing.
   */
  @Test
  void aReplayedTimestampKeepsItsUnit() {
    Time timestamp = twoEstimateCamera().getObservations().get(0).timestamp();

    assertEquals(Seconds, timestamp.unit());
    assertEquals(10.0, timestamp.in(Seconds));
  }

  @Test
  void aCycleWithNoNewFramesProducesNoObservations() {
    AprilTagCamera camera = replayedCamera(
        new double[0], new Pose3d[0], new double[0], new double[0], new int[0][], new double[0][]);

    assertEquals(List.of(), camera.getObservations());
  }

  @Test
  void anObservationIsAttributedToTheCameraThatMadeIt() {
    AprilTagCamera camera = replayedCamera(
        new double[] { 10.0 },
        new Pose3d[] { POSE_A },
        new double[] { 0.05 },
        new double[] { 1.5 },
        new int[][] { { 1 } },
        new double[][] { { 2.5 } });

    assertEquals("FakeCamera", camera.getObservations().get(0).camera().name());
  }

  /**
   * Nothing enforces that the arrays are the same length, and a log written by an older
   * version of the IO could disagree. Reading only the prefix that every array covers is what
   * keeps that from throwing part way through a loop.
   */
  @Test
  void arraysOfUnequalLengthYieldOnlyTheFullyPopulatedPrefix() {
    AprilTagCamera camera = replayedCamera(
        new double[] { 10.0, 10.02 },
        new Pose3d[] { POSE_A, POSE_B },
        new double[] { 0.05 },
        new double[] { 1.5, 1.5 },
        new int[][] { { 1, 2 }, { 7 } },
        new double[][] { { 2.5, 3.5 }, { 4.5 } });

    List<AprilTagPoseObservation> observations = camera.getObservations();

    assertEquals(1, observations.size());
    assertEquals(POSE_A, observations.get(0).observedRobotPose());
  }
}
