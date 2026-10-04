package first.robot.mechanism.vision.apriltag;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.robot.util.PoseEstimator.VisionObservation;

/**
 * Decides what each AprilTag observation is worth, and tells the pose estimator.
 *
 * <p>Two jobs, and the second is the one that matters. The gates below throw away observations that
 * are impossible rather than merely poor — a robot solved into the air, a frame whose timestamp
 * makes no sense — and there are deliberately few of them, because a hard threshold is a cliff and
 * an observation that deweights itself needs no cliff. The standard deviation model is what actually
 * arbitrates: it converts the geometry of a solve into how far the estimate should move toward it.
 */
public class AprilTagVisionProcessor {
  // MARK: - Gates
  //
  // These mostly reject the impossible rather than the imprecise: anything a solve merely does badly
  // is priced by the model below instead, and an observation that deweights itself needs no cliff.
  // The distance gate is the exception, and the reason is in its own note -- past a point the error
  // stops following the smooth curve the model prices it with, and a standard deviation stops being
  // an honest answer.

  /**
   * Measured to the nearest tag: one close tag makes an observation good however far the others are.
   *
   * <p>Fifteen feet rather than most of the field. The range term below grows as {@code d^2} and so
   * already distrusts a distant tag, but distrusted is not ignored: a solve seven metres out still
   * arrives with a standard deviation small enough to move the estimate, and out there the solve is
   * wrong in ways this model cannot see. A tag twenty pixels wide turns half a pixel of corner noise
   * into a large fraction of its apparent width, and the layout's own survey error stops being small
   * compared to what is being measured, so the error is no longer the smooth {@code d^2} the
   * coefficients describe. Past this the cheapest correct answer is to keep the odometry and wait
   * for a closer look.
   */
  private static final Distance MAX_NEAREST_TAG_DISTANCE = Feet.of(15.0);

  /** The robot is on the floor, so a solve that puts it this far above the floor is not a solve. */
  private static final Distance MAX_DISTANCE_ABOVE_GROUND = Feet.of(1.0);

  /**
   * A frame older than this is treated as a broken clock rather than a slow one.
   *
   * <p>Generous because staleness alone is harmless: the estimator replays each observation at the
   * time it was captured, against a two second odometry buffer, so an old frame is corrected in the
   * past and carried forward rather than applied as though it were current. What this catches is a
   * coprocessor whose time sync has failed, which produces timestamps that are wrong rather than
   * late.
   */
  private static final Time MAX_LATENCY = Seconds.of(0.25);

  /**
   * Pixels of residual a multi-tag solve may leave behind.
   *
   * <p>This replaces the ambiguity gate that used to sit here, which could not work: a multi-tag
   * solve has no alternate solution to be ambiguous against and always reports zero ambiguity, and a
   * single-tag trig solve reads only the tag's range and bearing and so never touches the
   * orientation solve that ambiguity describes. The gate therefore never fired where it could have
   * helped and discarded usable single-tag frames where it could not. Reprojection error is the
   * number that actually measures a multi-tag fit, and observations without one are not gated here
   * at all.
   */
  private static final double MAX_REPROJECTION_ERROR_PIXELS = 8.0;

  // MARK: - Error model
  //
  // The error of a tag solve is not circular, and treating it as circular is the single largest
  // avoidable loss here. A tag pins down its bearing superbly, because half a pixel of corner noise
  // is milliradians, and its range badly, because range is inferred from apparent size and the
  // sensitivity of apparent size to range falls off as 1/d^2. So bearing error grows linearly with
  // distance while range error grows quadratically, and by four metres the error ellipse is an order
  // of magnitude longer along the camera bore than across it. One number for both axes has to either
  // distrust the bearing or over-trust the range; these two coefficients are the whole model, and
  // everything after them either inflates or rotates the ellipse they describe.

  /** Range error over distance squared, in 1/m. Fit this from logs; see the class-level tuning note. */
  private static final double RANGE_COEFFICIENT = 0.005;

  /** Bearing error over distance, dimensionless. Fit this from logs alongside the coefficient above. */
  private static final double BEARING_COEFFICIENT = 0.005;

  /**
   * Error no amount of tags or proximity can remove, added in quadrature.
   *
   * <p>Field tags are surveyed to a tolerance, and on a field assembled by volunteers to rather
   * worse than that; the transform from the robot origin to each lens is measured by hand. Neither
   * shrinks as the robot gets closer, so without this term the model happily reports a centimetre of
   * uncertainty at point blank range and the estimate snaps to vision and jitters — which is
   * precisely where a pose jump is most visible and least welcome.
   */
  private static final Distance SYSTEMATIC_FLOOR = Meters.of(0.025);

  /**
   * How wrong a capture timestamp can be, which matters in proportion to how fast the lens is
   * moving.
   *
   * <p>Bulk latency is already handled, since the estimator replays observations into the past.
   * Jitter is not, and it bites hardest while turning: a lens a third of a metre off centre on a
   * robot spinning at eight radians per second is travelling nearly three metres a second no matter
   * what the chassis is doing, so ten milliseconds of timestamp error is three centimetres of
   * position error that no amount of tag quality can remove.
   */
  private static final Time TIMESTAMP_STD_DEV = Seconds.of(0.01);

  /**
   * What a single-tag solve's heading is worth: nothing.
   *
   * <p>PhotonVision's trig solve ends by constructing its pose as {@code new Pose2d(translation,
   * headingSample)} — the rotation it returns is the gyro heading this class's camera handed it,
   * echoed back. Feeding it to the estimator as a heading measurement would be feeding the gyro to
   * the filter as though it were independent evidence of itself, which at close range came out to
   * roughly a third of the residual applied per frame. Large enough that the Kalman gain rounds to
   * zero, and finite because the estimator squares it.
   */
  private static final double NO_HEADING_INFORMATION = 1.0e4;

  /**
   * How much less a one-tag solve is worth than its geometry alone says, on both axes.
   *
   * <p>{@link #effectiveDistance} already hands a second tag the root-N it earns, so this is charged
   * on top of that, and what it prices is the part of a single-tag solve that is not about how far
   * the tag was. One tag has no redundancy: nothing contradicts a misread corner or a tag matched to
   * the wrong id, where a second tag would leave a residual the reprojection term could charge for.
   * Its range comes from one apparent width rather than a fit over several. And the trig solve rests
   * on the gyro heading being right, so a heading error swings the whole translation about the tag
   * rather than perturbing it. None of that varies with {@code d}, so none of it is priced above.
   */
  private static final double SINGLE_TAG_STD_DEV_SCALE = 2.5;

  /** Below this, two tags constrain rotation no better than one does. */
  private static final Distance MIN_TAG_SPREAD = Meters.of(0.15);

  // The solve tells on itself, and these are the scales at which it doing so doubles the standard
  // deviations. The robot is on the floor and level, so every centimetre of height and every degree
  // of tilt a solve reports is pure error, measured for free against a ground truth of exactly zero
  // — and a solve wrong about its height is wrong about x and y as well.
  private static final Distance RESIDUAL_HEIGHT_SCALE = Meters.of(0.15);
  private static final Angle RESIDUAL_TILT_SCALE = Degrees.of(10.0);
  private static final double RESIDUAL_REPROJECTION_SCALE = 4.0;
  private static final double AMBIGUITY_SCALE = 0.2;

  /**
   * Cap on the {@code 1/cos} blowup from an obliquely viewed tag, so an edge-on tag is heavily
   * distrusted rather than unboundedly so.
   */
  private static final double MIN_INCIDENCE_COSINE = 0.2;

  /**
   * The powers the per-tag variances are combined at: {@code d^4} for range and {@code d^2} for
   * bearing, matching the two error growth rates the coefficients above describe.
   */
  private static final double RANGE_VARIANCE_POWER = 4.0;
  private static final double BEARING_VARIANCE_POWER = 2.0;

  private final Field m_field;
  private final Supplier<ChassisVelocities> m_robotVelocitiesSupplier;
  private final BooleanSupplier m_singleTagEstimationEnabled;
  private final Consumer<VisionObservation> m_observationConsumer;

  /** Every camera that has reported at least once, so that a silent one still gets logged. */
  private final Set<AprilTagCameraIO> m_camerasSeen = new LinkedHashSet<>();

  /**
   * @param field                   the tag layout to measure observations against, which must be the
   *                                layout the cameras solved against
   * @param robotVelocitiesSupplier how fast the robot is moving, robot-relative. Read from the
   *                                drivetrain rather than from the pose estimate, so that the
   *                                weight given to an observation cannot depend on the estimate the
   *                                observation is about to correct.
   * @param singleTagEstimationEnabled whether one-tag solves are used at all. Injected rather than
   *                                read from a dashboard here, so this class stays testable without
   *                                NetworkTables; on the robot pass {@code
   *                                Tuning.kSingleTagEstimation}. Off, every one-tag frame is
   *                                rejected and the robot runs on multi-tag frames and odometry,
   *                                which is the configuration to reach for when the estimate is
   *                                being pulled about by a camera that can only ever see one tag.
   * @param observationConsumer     where accepted observations go
   */
  public AprilTagVisionProcessor(
      Field field,
      Supplier<ChassisVelocities> robotVelocitiesSupplier,
      BooleanSupplier singleTagEstimationEnabled,
      Consumer<VisionObservation> observationConsumer) {
    m_field = field;
    m_robotVelocitiesSupplier = robotVelocitiesSupplier;
    m_singleTagEstimationEnabled = singleTagEstimationEnabled;
    m_observationConsumer = observationConsumer;
  }

  /**
   * Weighs every observation from this loop and submits the ones worth having.
   *
   * <p>The observations arrive mixed together from every camera, and one camera can contribute
   * several in a loop, so they are grouped before being logged: a camera's log keys are shared by
   * all of its observations, and writing them one observation at a time would leave only whichever
   * happened to be written last.
   */
  public void process(List<AprilTagPoseObservation> observations) {
    Map<AprilTagCameraIO, List<Evaluation>> byCamera = new LinkedHashMap<>();

    for (AprilTagPoseObservation observation : observations) {
      SolveGeometry geometry = SolveGeometry.of(m_field, observation);
      Result result = processObservation(observation, geometry);

      if (result instanceof Accepted accepted) {
        submitAcceptedObservation(accepted);
      }

      byCamera.computeIfAbsent(observation.cameraIO(), cameraIO -> new ArrayList<>())
          .add(new Evaluation(result, geometry));
    }

    m_camerasSeen.addAll(byCamera.keySet());

    // Cameras that reported nothing this loop are logged as having reported nothing, rather than
    // skipped. A key that goes unwritten keeps whatever it last held, so skipping them would make a
    // camera that has gone blind indistinguishable from one still looking at the tag it saw a minute
    // ago -- and a camera dropping out is the failure this log most needs to show.
    //
    // Which cameras exist is learned from the observations rather than declared, since a camera that
    // has never reported since boot is already visible as a disconnected one in its own inputs.
    for (AprilTagCameraIO cameraIO : m_camerasSeen) {
      log(cameraIO, byCamera.getOrDefault(cameraIO, List.of()));
    }
  }

  private Result processObservation(AprilTagPoseObservation observation, SolveGeometry geometry) {
    if (geometry.tagCount() == 0) {
      return new UnknownTagRejection(observation);
    }

    if (geometry.tagCount() < 2 && !m_singleTagEstimationEnabled.getAsBoolean()) {
      return new SingleTagDisabledRejection(observation);
    }

    Distance heightAboveGround = observation.observedRobotPose().getMeasureZ();
    if (heightAboveGround.gt(MAX_DISTANCE_ABOVE_GROUND)) {
      return new PoseHeightRejection(observation, heightAboveGround);
    }

    Distance nearestTagDistance = Meters.of(geometry.nearestRangeMeters());
    if (nearestTagDistance.gt(MAX_NEAREST_TAG_DISTANCE)) {
      return new TagDistanceRejection(observation, nearestTagDistance);
    }

    if (observation.hasReprojectionError()
        && observation.reprojectionErrorPixels() > MAX_REPROJECTION_ERROR_PIXELS) {
      return new ReprojectionErrorRejection(observation, observation.reprojectionErrorPixels());
    }

    Time latency = latencyOf(observation);
    if (latency.gt(MAX_LATENCY)) {
      return new LatencyRejection(observation, latency);
    }

    return new Acceptance(observation, standardDeviations(observation, geometry));
  }

  /**
   * The error ellipse this solve earns, rotated into the field frame and reduced to the diagonal the
   * pose estimator can carry.
   */
  private Vector<N3> standardDeviations(
      AprilTagPoseObservation observation, SolveGeometry geometry) {
    double rangeDistance = geometry.rangeDistanceMeters();

    double sigmaRange = RANGE_COEFFICIENT * rangeDistance * rangeDistance / geometry.incidenceCosine();
    double sigmaBearing = BEARING_COEFFICIENT * geometry.bearingDistanceMeters();

    double penalty = qualityPenalty(observation, geometry);
    if (geometry.tagCount() < 2) {
      penalty *= SINGLE_TAG_STD_DEV_SCALE;
    }

    sigmaRange *= penalty;
    sigmaBearing *= penalty;

    double irreducible = Math.hypot(motionStandardDeviation(observation), SYSTEMATIC_FLOOR.in(Meters));
    sigmaRange = Math.hypot(sigmaRange, irreducible);
    sigmaBearing = Math.hypot(sigmaBearing, irreducible);

    // Rotate the ellipse out of (range, bearing) and into the field frame: this is the diagonal of
    // R * Sigma * R^T, taken about the bearing from the robot to the tags. The off-diagonal term the
    // estimator's interface cannot carry is dropped rather than approximated, which loses the
    // correlation between x and y and so errs toward distrust rather than toward confidence.
    //
    // Strictly this is exact for one tag and conservative for several spread widely apart, where the
    // joint solve's error is no longer aligned with the camera bore and the true ellipse is rounder
    // than this one. Being too round in the range axis would be the dangerous direction; being too
    // long, as here, is not.
    double cos = geometry.tagBearing().getCos();
    double sin = geometry.tagBearing().getSin();
    double varRange = sigmaRange * sigmaRange;
    double varBearing = sigmaBearing * sigmaBearing;

    return VecBuilder.fill(
        Math.sqrt(varRange * cos * cos + varBearing * sin * sin),
        Math.sqrt(varRange * sin * sin + varBearing * cos * cos),
        headingStandardDeviation(geometry, sigmaBearing));
  }

  /**
   * How much to inflate both axes for the things a solve reports about its own quality.
   *
   * <p>All four terms are free: the height and tilt have a known ground truth of zero, and the other
   * two are numbers the camera already published. They are summed rather than multiplied so that one
   * bad signal inflates rather than annihilates.
   */
  private static double qualityPenalty(
      AprilTagPoseObservation observation, SolveGeometry geometry) {
    double penalty = 1.0
        + geometry.heightErrorMeters() / RESIDUAL_HEIGHT_SCALE.in(Meters)
        + geometry.tiltErrorRadians() / RESIDUAL_TILT_SCALE.in(Radians);

    // Weak evidence rather than no evidence. Ambiguity does not describe what the trig solve
    // computes, so it cannot justify a gate, but a tag detected badly enough to be ambiguous was
    // also detected badly enough for its corners to be noisy, and that does move the range.
    if (observation.hasAmbiguity()) {
      penalty += observation.ambiguity() / AMBIGUITY_SCALE;
    }

    if (observation.hasReprojectionError()) {
      penalty += observation.reprojectionErrorPixels() / RESIDUAL_REPROJECTION_SCALE;
    }

    return penalty;
  }

  /** The position error a mistimed frame buys, which grows with how fast the lens is travelling. */
  private double motionStandardDeviation(AprilTagPoseObservation observation) {
    ChassisVelocities velocities = m_robotVelocitiesSupplier.get();

    double cameraRadius = observation.cameraIO().getRobotToCamera()
        .getTranslation().toTranslation2d().getNorm();
    double cameraSpeed = Math.hypot(velocities.vx, velocities.vy) + Math.abs(velocities.omega) * cameraRadius;

    return TIMESTAMP_STD_DEV.in(Seconds) * cameraSpeed;
  }

  /**
   * What the solve's heading is worth, in radians.
   *
   * <p>Rotational error is a lateral error swung about a lever arm, so the bearing standard
   * deviation divided by how far apart the tags are is already an angle — which is why this has no
   * coefficient of its own to tune and falls out of the two above. It also distrusts exactly the
   * right case: two tags a handspan apart get the clamp, and two tags a handspan apart are the
   * geometry that produces the planar-PnP flip.
   */
  private static double headingStandardDeviation(SolveGeometry geometry, double sigmaBearing) {
    if (geometry.tagCount() < 2) {
      return NO_HEADING_INFORMATION;
    }

    return sigmaBearing / Math.max(geometry.tagSpreadMeters(), MIN_TAG_SPREAD.in(Meters));
  }

  private static Time latencyOf(AprilTagPoseObservation observation) {
    return RobotController.getMeasureTime().minus(observation.timestamp());
  }

  private void submitAcceptedObservation(Accepted acceptedResult) {
    VisionObservation visionObservation = new VisionObservation(
        acceptedResult.observation().timestamp(),
        acceptedResult.observation().observedRobotPose(),
        acceptedResult.stdDevs());
    m_observationConsumer.accept(visionObservation);
  }

  /**
   * Publishes one camera's work for this loop: how much of it there was, where it put the robot, and
   * one observation in full.
   *
   * <p>Only the newest one, because a camera delivering several frames in a loop is rare enough that
   * detailing all of them would cost more than it returns. Arrays would make every number here an indexed
   * child whose length changes from loop to loop, which is awkward to graph and worse to read at a
   * glance, and that cost would be paid on every loop to serve the few percent that carry a backlog.
   * The counts above are what keep the thinning honest: a backlog is always visible as one, even
   * though only one of its frames is described, so a model fitted from this log can never be fitted
   * from a silently truncated sample.
   *
   * <p>The poses stay arrays regardless. That is the type a field view wants, and an empty one draws
   * nothing, which is the right rendering for a camera that saw nothing and is not a thing a single
   * pose can express. Every pose is published, accepted or not, since a backlog's poses are cheap
   * and seeing where a rejected solve thought the robot was is most of working out why it was wrong.
   */
  private void log(AprilTagCameraIO cameraIO, List<Evaluation> evaluations) {
    String prefix = "Vision/%s".formatted(cameraIO.getName());

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
  private void logMostRecent(String prefix, Optional<Evaluation> latestEvaluation) {
    Logger.recordOutput(
        prefix + "/Accepted",
        latestEvaluation.map(evaluation -> evaluation.result().accepted()).orElse(false));
    Logger.recordOutput(prefix + "/RejectionReason", latestEvaluation.map(Evaluation::reason).orElse(""));
    Logger.recordOutput(
        prefix + "/LatencySeconds",
        scalarOf(latestEvaluation, evaluation -> latencyOf(evaluation.observation()).in(Seconds)));

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
      Optional<Evaluation> representative, ToDoubleFunction<Evaluation> field) {
    return representative.isPresent() ? field.applyAsDouble(representative.get()) : Double.NaN;
  }

  /**
   * One observation's verdict together with the geometry it was reached from, held only long enough
   * for its camera's loop to be logged.
   */
  private record Evaluation(Result result, SolveGeometry geometry) {
    AprilTagPoseObservation observation() {
      return result.observation();
    }

    String reason() {
      return result instanceof Rejected rejected ? rejected.reason() : "";
    }

    /**
     * One component of the standard deviations, or NaN if the observation was rejected and never
     * given any -- not zero, which would read as an observation trusted absolutely.
     */
    double stdDev(int row) {
      return result instanceof Accepted accepted ? accepted.stdDevs().get(row) : Double.NaN;
    }
  }

  // MARK: - Geometry

  /**
   * Everything about where the tags were that the error model and the gates both need, derived once
   * per observation.
   *
   * <p>Computed whether or not the observation survives, because these are the quantities the model
   * is tuned against and a log of only the accepted ones is a biased sample.
   *
   * @param tagPoses             where the layout puts each tag the solve used, in the order the
   *                             observation listed them, and only the ones the layout knows -- which
   *                             can be fewer than the solve claimed to use. Every scalar below is
   *                             derived from these, and {@code tagCount} is their number, so the two
   *                             cannot drift apart.
   * @param nearestRangeMeters   how far away the closest of them was
   * @param rangeDistanceMeters  the effective distance for range error; see
   *                             {@link #effectiveDistance}
   * @param bearingDistanceMeters the effective distance for bearing error
   * @param incidenceCosine      how square-on the tags were, 1 face-on, clamped from below
   * @param tagSpreadMeters      the widest separation between any two tags used
   * @param tagBearing           field-relative direction from the robot to the tags, the axis the
   *                             error ellipse is aligned with
   * @param heightErrorMeters    how far above the floor the solve put the robot
   * @param tiltErrorRadians     how far from level the solve put the robot
   */
  private record SolveGeometry(
      List<Pose3d> tagPoses,
      double nearestRangeMeters,
      double rangeDistanceMeters,
      double bearingDistanceMeters,
      double incidenceCosine,
      double tagSpreadMeters,
      Rotation2d tagBearing,
      double heightErrorMeters,
      double tiltErrorRadians) {

    int tagCount() {
      return tagPoses.size();
    }

    static SolveGeometry of(Field field, AprilTagPoseObservation observation) {
      Pose3d robotPose = observation.observedRobotPose();

      // Where the lens is, for the fallback ranges below and for how square-on the tags were. Both
      // want the lens rather than the robot origin: the model squares a range, and at close quarters
      // the camera's offset is a large fraction of one.
      Pose3d cameraPose = robotPose.plus(observation.cameraIO().getRobotToCamera());

      Rotation3d rotation = robotPose.getRotation();
      double heightError = Math.abs(robotPose.getZ());
      double tiltError = Math.hypot(rotation.getX(), rotation.getY());

      List<Pose3d> tagPoses = observation.tags().stream()
          .map(field::getTagPose)
          .flatMap(Optional::stream)
          .toList();

      if (tagPoses.isEmpty()) {
        return new SolveGeometry(
            List.of(),
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            1.0,
            0.0,
            Rotation2d.ZERO,
            heightError,
            tiltError);
      }

      // What the camera measured, if it could. Falling back to the distances the solved pose implies
      // is a last resort rather than the normal path: those agree with the pose they came from
      // however wrong it is, so a solve that has gone astray prices itself as though it had not --
      // and a flipped one, landing nearer a tag than it really is, would be rewarded for it.
      double[] ranges = observation.measuredTagRanges().length > 0
          ? observation.measuredTagRanges()
          : tagPoses.stream()
              .mapToDouble(tag -> tag.getTranslation().getDistance(cameraPose.getTranslation()))
              .toArray();

      return new SolveGeometry(
          tagPoses,
          Arrays.stream(ranges).min().orElse(Double.POSITIVE_INFINITY),
          effectiveDistance(ranges, RANGE_VARIANCE_POWER),
          effectiveDistance(ranges, BEARING_VARIANCE_POWER),
          AprilTagVisionProcessor.incidenceCosine(tagPoses, cameraPose),
          tagSpread(tagPoses),
          bearingTo(tagPoses, robotPose),
          heightError,
          tiltError);
    }
  }

  /**
   * The distance one tag would have to sit at to carry as much information as these tags carry
   * together, for an error growing as {@code d^(power/2)}.
   *
   * <p>This is what replaces averaging the tag distances, and it is the difference between counting
   * tags and weighing them. Each tag contributes information — the reciprocal of its variance — and
   * information adds, so combining the variances rather than the distances yields two properties an
   * average cannot. For N tags at equal distance it returns {@code d / N^(1/power)}, which hands the
   * model the {@code 1/sqrt(N)} that independent observations actually earn, continuously and with no
   * cliff between one tag and two. And it is dominated by the nearest tag, so a distant tag can only
   * ever help: under an average, a far fourth tag drags the distance up and throws away an estimate
   * three close tags had already pinned down.
   */
  private static double effectiveDistance(double[] ranges, double power) {
    double information = 0.0;
    for (double range : ranges) {
      information += Math.pow(Math.max(range, 1e-6), -power);
    }

    return information == 0.0 ? Double.POSITIVE_INFINITY : Math.pow(information, -1.0 / power);
  }

  /**
   * How square-on the tags were, 1 for face-on down to {@link #MIN_INCIDENCE_COSINE}.
   *
   * <p>This is the part of a solve's quality that distance alone cannot see. Range is inferred from
   * apparent width, and an obliquely viewed tag has most of its apparent width foreshortened away,
   * so a tag two metres off to the side is worth much less than a tag two metres straight ahead
   * even though both are two metres away.
   *
   * <p>Taken from the layout and the solved camera position rather than from the camera's measured
   * transform, even though the range beside it is measured. The two are not alike: a single-tag
   * solve determines a tag's translation well and its rotation badly -- that ambiguity is the whole
   * reason the single-tag strategy ignores the rotation -- and incidence is a question about the
   * tag's facing. The layout knows that exactly, and needs only a roughly right camera position to
   * turn it into an angle.
   */
  private static double incidenceCosine(List<Pose3d> tagPoses, Pose3d cameraPose) {
    double total = 0.0;

    for (Pose3d tag : tagPoses) {
      // A tag's pose faces along its own x axis. Taking the absolute value of the dot product makes
      // this indifferent to which way along that axis the layout considers the front, since a tag
      // seen from behind is not a case that arises and a sign error here would be silent.
      Translation3d normal = new Translation3d(1.0, 0.0, 0.0).rotateBy(tag.getRotation());
      Translation3d toCamera = cameraPose.getTranslation().minus(tag.getTranslation());
      double norm = toCamera.getNorm();

      total += norm < 1e-6 ? 1.0 : Math.abs(normal.dot(toCamera) / norm);
    }

    return Math.max(total / tagPoses.size(), MIN_INCIDENCE_COSINE);
  }

  /**
   * The widest separation between any two tags used, which is the lever arm the solve's rotation
   * rests on. Quadratic in the tag count, which is at most a handful.
   */
  private static double tagSpread(List<Pose3d> tagPoses) {
    double spread = 0.0;

    for (int i = 0; i < tagPoses.size(); i++) {
      for (int j = i + 1; j < tagPoses.size(); j++) {
        spread = Math.max(
            spread,
            tagPoses.get(i).getTranslation().getDistance(tagPoses.get(j).getTranslation()));
      }
    }

    return spread;
  }

  /**
   * The field-relative direction from the robot to the centre of the tags it was solved from.
   *
   * <p>Undefined if the solve put the robot exactly on that centre, in which case the axis the
   * ellipse is aligned with does not matter: the robot is inside the tag, every range is at the
   * systematic floor, and the ellipse is round.
   */
  private static Rotation2d bearingTo(List<Pose3d> tagPoses, Pose3d robotPose) {
    Translation2d centroid = tagPoses.stream()
        .map(tag -> tag.getTranslation().toTranslation2d())
        .reduce(Translation2d.ZERO, Translation2d::plus)
        .div(tagPoses.size());

    return centroid.minus(robotPose.getTranslation().toTranslation2d())
        .getAngle()
        .orElse(Rotation2d.ZERO);
  }

  // MARK: - Result types

  /**
   * The verdict on one observation: accepted, carrying the standard deviations the pose estimator
   * should weight it by, or rejected, carrying the rule that turned it away. These live here rather
   * than beside the observation because this class is the only thing that produces or reads them.
   */
  public sealed interface Result {
    public AprilTagPoseObservation observation();

    public boolean accepted();
  }

  sealed interface Accepted extends Result {
    Vector<N3> stdDevs();

    @Override
    default boolean accepted() {
      return true;
    }
  }

  sealed interface Rejected extends Result {
    String reason();

    @Override
    default boolean accepted() {
      return false;
    }
  }

  public final record Acceptance(AprilTagPoseObservation observation, Vector<N3> stdDevs)
      implements Accepted {
  }

  public final record TagDistanceRejection(AprilTagPoseObservation observation,
      Distance nearestTagDistance)
      implements Rejected {
    @Override
    public String reason() {
      return "Nearest tag too far away: " + nearestTagDistance.toShortString();
    }
  }

  public final record PoseHeightRejection(AprilTagPoseObservation observation,
      Distance distanceAboveGround)
      implements Rejected {
    @Override
    public String reason() {
      return "Observed position too elevated: " + distanceAboveGround.toShortString();
    }
  }

  public final record ReprojectionErrorRejection(AprilTagPoseObservation observation,
      double reprojectionErrorPixels)
      implements Rejected {
    @Override
    public String reason() {
      return "Solve fits its tags too poorly: " + reprojectionErrorPixels + " px";
    }
  }

  /**
   * A one-tag solve arrived with single-tag estimation switched off.
   *
   * <p>Worth its own rejection rather than a silent drop: this is a configuration choice, and a
   * robot ignoring most of its frames on purpose should not look in the log like one whose cameras
   * have gone bad.
   */
  public final record SingleTagDisabledRejection(AprilTagPoseObservation observation)
      implements Rejected {
    @Override
    public String reason() {
      return "Single-tag estimation is disabled";
    }
  }

  public final record LatencyRejection(AprilTagPoseObservation observation, Time latency)
      implements Rejected {
    @Override
    public String reason() {
      return "Observation too stale: " + latency.toShortString();
    }
  }

  /**
   * Every tag the solve used is absent from the layout the filter measures against, so there is
   * nothing to judge the observation by. Worth its own rejection rather than being folded into a
   * distance rejection, because it means the cameras and this class disagree about what field they
   * are on.
   */
  public final record UnknownTagRejection(AprilTagPoseObservation observation)
      implements Rejected {
    @Override
    public String reason() {
      return "No tag in this solve is in the field layout: " + observation.tags();
    }
  }
}
