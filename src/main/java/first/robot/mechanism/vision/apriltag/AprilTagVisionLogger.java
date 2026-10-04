package first.robot.mechanism.vision.apriltag;

import static org.wpilib.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import java.util.function.ToDoubleFunction;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose3d;

import first.robot.mechanism.vision.apriltag.AprilTagVisionProcessor.Evaluation;

/**
 * Publishes what the vision filter decided, one camera at a time.
 *
 * <p>Separate from {@link AprilTagVisionProcessor} because the two change for unrelated reasons.
 * What an observation is worth is a model to be tuned against logs; what gets written about it is a
 * presentation to be read from a dashboard, and renaming a key or publishing one more regressor
 * should not touch the filter, nor a retune touch the log. They had already grown separate test
 * suites before they grew separate files.
 *
 * <p>Stateless, and deliberately so. Which cameras exist and which of them went quiet is the
 * filter's business, because it is the thing that knows what it was handed; this class is told what
 * to write, including that there was nothing, and writes it.
 */
class AprilTagVisionLogger {
  /**
   * Publishes one camera's work for this loop: how much of it there was, where it put the robot, and
   * one observation in full.
   *
   * <p>Only the newest one, because a camera delivering several frames in a loop is rare enough that
   * detailing all of them would cost more than it returns. Arrays would make every number here an indexed
   * child whose length changes from loop to loop, which is awkward to graph and worse to read at a
   * glance, and that cost would be paid on every loop to serve the few percent that carry a backlog.
   * The counts are what keep the thinning honest: a backlog is always visible as one, even
   * though only one of its frames is described, so a model fitted from this log can never be fitted
   * from a silently truncated sample.
   *
   * <p>The poses stay arrays regardless. That is the type a field view wants, and an empty one draws
   * nothing, which is the right rendering for a camera that saw nothing and is not a thing a single
   * pose can express. Every pose is published, accepted or not, since a backlog's poses are cheap
   * and seeing where a rejected solve thought the robot was is most of working out why it was wrong.
   */
  void log(AprilTagCameraConfig camera, List<Evaluation> evaluations) {
    String prefix = "Vision/%s".formatted(camera.name());

    Logger.recordOutput(prefix + "/ObservationCount", evaluations.size());
    Logger.recordOutput(
        prefix + "/AcceptedCount",
        (int) evaluations.stream().filter(evaluation -> evaluation.result().accepted()).count());

    Logger.recordOutput(
        prefix + "/RejectedRobotPoses",
        evaluations.stream()
            .filter(evaluation -> !evaluation.result().accepted())
            .map(evaluation -> evaluation.observation().observedRobotPose())
            .toArray(Pose3d[]::new));

    // Split by verdict rather than published as one array, so a field view can draw the two apart:
    // the accepted poses are what moved the estimate, and the rejected ones are only worth seeing to
    // work out why they were not.
    Logger.recordOutput(
        prefix + "/AcceptedRobotPoses",
        evaluations.stream()
            .filter(evaluation -> evaluation.result().accepted())
            .map(evaluation -> evaluation.observation().observedRobotPose())
            .toArray(Pose3d[]::new));

    logMostRecent(
        prefix + "/MostRecent",
        // The newest frame, with no preference for whether it was accepted. Choosing the accepted
        // one in a mixed loop would read better in that rare case, but it would also skew the
        // described frames toward accepted ones, and these keys are the sample the error model gets
        // fitted from. The counts above already carry the verdict for the loop as a whole, so
        // nothing is lost by letting this one be an unbiased draw.
        evaluations.isEmpty() ? Optional.empty() : Optional.of(evaluations.getLast()));
  }

  /**
   * Describes one observation, or describes the absence of one.
   *
   * <p>This is the newest frame of the loop, not necessarily the one the estimator acted on: on a
   * loop where one frame was accepted and a later one rejected, {@code Accepted} here reads false
   * while the estimate was in fact corrected. {@code AcceptedCount} is the field that answers
   * whether this camera contributed, and these describe one frame rather than the loop.
   *
   * <p>A camera that reported nothing writes here too, rather than leaving the keys alone. An
   * unwritten key keeps whatever it last held, so skipping a silent camera would make one that has
   * gone blind indistinguishable from one still watching the tag it saw a minute ago -- and a camera
   * dropping out is the failure this log most needs to show. NaN is what it writes, because NaN does
   * not plot, which is the honest rendering of a measurement that was never taken.
   */
  private static void logMostRecent(String prefix, Optional<Evaluation> latestEvaluation) {
    Logger.recordOutput(
        prefix + "/Accepted",
        latestEvaluation.map(evaluation -> evaluation.result().accepted()).orElse(false));
    Logger.recordOutput(prefix + "/RejectionReason", latestEvaluation.map(Evaluation::reason).orElse(""));
    Logger.recordOutput(
        prefix + "/LatencySeconds",
        scalarOf(latestEvaluation, evaluation -> evaluation.observation().latency().in(Seconds)));

    Logger.recordOutput(prefix + "/StdDevs/X", scalarOf(latestEvaluation, evaluation -> evaluation.stdDev(0)));
    Logger.recordOutput(prefix + "/StdDevs/Y", scalarOf(latestEvaluation, evaluation -> evaluation.stdDev(1)));
    Logger.recordOutput(prefix + "/StdDevs/Theta", scalarOf(latestEvaluation, evaluation -> evaluation.stdDev(2)));

    // The regressors the coefficients in this class are fit against. Logged for a rejected
    // observation as readily as an accepted one: a log of only what survived would be a sample
    // biased against exactly the geometry the model is least sure about.
    Logger.recordOutput(
        prefix + "/TagCount",
        latestEvaluation.map(evaluation -> evaluation.geometry().tagCount()).orElse(-1));

    // Where the tags the newest frame was solved from actually are, so a field view can draw them
    // beside the pose they produced. Which is most of reading a bad frame: a solve pulled off to one
    // side is one thing when the tag it came from is the far one of a pair and another when it is the
    // tag a metre ahead, and the count alone cannot tell those apart.
    //
    // These are the layout's poses, not the camera's, and only for tags the layout recognised. So a
    // frame that claimed more tags than appear here is one whose camera is solving against a
    // different field, which is visible as the array being shorter than the solve's tag list.
    Logger.recordOutput(
        prefix + "/TagPoses",
        latestEvaluation
            .map(evaluation -> evaluation.geometry().tagPoses().toArray(Pose3d[]::new))
            .orElseGet(() -> new Pose3d[0]));
    Logger.recordOutput(
        prefix + "/RangeDistance",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().rangeDistanceMeters()));
    Logger.recordOutput(
        prefix + "/BearingDistance",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().bearingDistanceMeters()));
    Logger.recordOutput(
        prefix + "/NearestTagDistance",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().nearestRangeMeters()));
    Logger.recordOutput(
        prefix + "/IncidenceCosine",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().incidenceCosine()));
    Logger.recordOutput(
        prefix + "/TagSpread",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().tagSpreadMeters()));
    Logger.recordOutput(
        prefix + "/TagBearingRadians",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().tagBearing().getRadians()));
    Logger.recordOutput(
        prefix + "/HeightError",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().heightErrorMeters()));
    Logger.recordOutput(
        prefix + "/TiltError",
        scalarOf(latestEvaluation, evaluation -> evaluation.geometry().tiltErrorRadians()));
    Logger.recordOutput(
        prefix + "/Ambiguity",
        scalarOf(latestEvaluation, evaluation -> evaluation.observation().ambiguity()));
    Logger.recordOutput(
        prefix + "/ReprojectionErrorPixels",
        scalarOf(latestEvaluation, evaluation -> evaluation.observation().reprojectionErrorPixels()));
  }

  private static double scalarOf(
      Optional<Evaluation> latestEvaluation, ToDoubleFunction<Evaluation> field) {
    return latestEvaluation.isPresent() ? field.applyAsDouble(latestEvaluation.get()) : Double.NaN;
  }
}
