package first.robot.mechanism.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
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
 * Covers what an observation is judged to be worth, which is the part of the vision path that fails
 * silently.
 *
 * <p>A wrong gate announces itself: the pose stops following the robot. A wrong standard deviation
 * does not. It shows up a season later as a pose that jitters while scoring, or one that takes a
 * second too long to believe a tag, and by then nobody is looking at the filter. So these tests pin
 * the shape of the model rather than its tuning: that error is longer along the camera bore than
 * across it, that extra tags help by root-N and never hurt, that a single tag's heading is worth
 * nothing, and that no geometry can talk the filter into trusting an observation more than the field
 * is surveyed. The coefficients are expected to move; the relationships are not.
 *
 * <p>The field here is synthetic, with tags at floor level, so that ranges come out to round numbers
 * and each test can vary one thing.
 */
public class AprilTagVisionProcessorTest {
  /** A tag on a wall ahead of the robot faces back along -x, which is a yaw of half a turn. */
  private static final Rotation3d FACING_BACK = new Rotation3d(0.0, 0.0, Math.PI);

  private static final long NOW_NANOS = 60_000_000_000L;
  private static final double NOW_SECONDS = 60.0;

  /**
   * An observation whose camera supplied no measured tag ranges, so that the model falls back to the
   * distances the solved pose implies and these tests can place tags at known distances.
   */
  private static final double[] NO_MEASURED_RANGES = new double[0];

  /** Where the standard deviations bottom out, from the processor's systematic floor. */
  private static final double FLOOR_METERS = 0.025;

  /** The flat inflation the processor charges a one-tag solve, on top of its geometry. */
  private static final double SINGLE_TAG_SCALE = 2.5;

  private final List<VisionObservation> m_accepted = new ArrayList<>();
  private ChassisVelocities m_velocities = new ChassisVelocities();
  private boolean m_singleTagEstimationEnabled = true;

  @BeforeEach
  void freezeTheClock() {
    // The processor measures latency against RobotController's time source, which AdvantageKit
    // replaces on a real robot. Replacing it here too makes staleness a value the test sets rather
    // than a race with the wall clock, and keeps the HAL out of it.
    RobotController.setTimeSource(() -> NOW_NANOS);
  }

  @AfterEach
  void restoreTheClock() {
    RobotController.setTimeSource(RobotController::getMonotonicTime);
  }

  // MARK: - Heading

  /**
   * The single-tag strategy solves for position given a heading, and returns the heading it was
   * given. Handing that back to the estimator as a heading measurement would be handing it the gyro
   * as evidence about the gyro.
   */
  @Test
  void aSingleTagObservationCarriesNoHeadingWeight() {
    VisionObservation observation = submit(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1)));

    assertTrue(
        observation.stdDevs().get(2, 0) > 1.0e3,
        "a single tag's heading must carry no weight, got " + observation.stdDevs().get(2, 0));
  }

  /** Two tags do solve for heading, so theirs is worth something. */
  @Test
  void aMultiTagObservationCarriesUsableHeadingWeight() {
    VisionObservation observation = submit(
        fieldWithTagsAt(new Translation3d(3.0, -1.0, 0.0), new Translation3d(3.0, 1.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2)));

    double stdDevTheta = observation.stdDevs().get(2, 0);

    assertTrue(stdDevTheta > 0.0, "heading weight must be finite and positive");
    assertTrue(stdDevTheta < 0.2, "two tags two metres apart should fix heading, got " + stdDevTheta);
  }

  /**
   * Rotation rests on the separation between the tags, so two tags almost on top of each other
   * constrain it barely better than one does -- which is also the geometry that produces the
   * planar-PnP flip.
   */
  @Test
  void tagsCloseTogetherFixHeadingLessWellThanTagsFarApart() {
    double narrow = submit(
        fieldWithTagsAt(new Translation3d(3.0, -0.05, 0.0), new Translation3d(3.0, 0.05, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2)))
        .stdDevs().get(2, 0);

    double wide = submit(
        fieldWithTagsAt(new Translation3d(3.0, -2.0, 0.0), new Translation3d(3.0, 2.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2)))
        .stdDevs().get(2, 0);

    assertTrue(narrow > wide * 2.0,
        "a narrow tag pair should be much worse for heading: %f vs %f".formatted(narrow, wide));
  }

  // MARK: - Anisotropy

  /**
   * The whole reason the model has two coefficients. Range comes from apparent size and is
   * imprecise; bearing comes from where the corners land and is not. So the error ellipse is long
   * along the line to the tag and narrow across it, and which field axis that falls on depends on
   * where the tag is.
   */
  @Test
  void errorIsLongTowardTheTagAndNarrowAcrossIt() {
    VisionObservation ahead = submit(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1)));

    assertTrue(
        ahead.stdDevs().get(0, 0) > ahead.stdDevs().get(1, 0),
        "a tag straight ahead should leave x less certain than y");

    // The same tag moved to the robot's left, which swaps which axis the long side lands on. If this
    // fails while the test above passes, the ellipse is not being rotated into the field frame.
    VisionObservation beside = submit(
        fieldWithTagsAt(new Translation3d(0.0, 4.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1)));

    assertTrue(
        beside.stdDevs().get(1, 0) > beside.stdDevs().get(0, 0),
        "a tag off to the side should leave y less certain than x");
  }

  // MARK: - Combining tags

  /**
   * The property that averaging the tag distances cannot have, and the reason this model combines
   * variances instead. Under an average, a far fourth tag drags the distance up and throws away an
   * estimate the close tags had already pinned down; information only ever adds.
   */
  @Test
  void aDistantTagNeverWorsensWhatACloseTagPinnedDown() {
    Field field = fieldWithTagsAt(new Translation3d(2.0, 0.0, 0.0), new Translation3d(8.0, 0.0, 0.0));

    double closeAlone = submit(field, observationAt(Pose3d.ZERO, Set.of(1))).stdDevs().get(0, 0);
    double closeAndFar = submit(field, observationAt(Pose3d.ZERO, Set.of(1, 2))).stdDevs().get(0, 0);

    assertTrue(closeAndFar <= closeAlone,
        "adding a distant tag must not cost accuracy: %f became %f".formatted(closeAlone, closeAndFar));

    // An average would have put the pair at 5 m and so trusted them far less than the near tag
    // alone. This is the size of the mistake being avoided, not a tuning assertion.
    assertTrue(closeAndFar < closeAlone * 2.0, "the pair should be priced by its near tag");
  }

  /**
   * Two independent looks at the same thing halve the variance, so they divide the standard
   * deviation by root two -- not by ten, which is what a discrete multi-tag bonus amounts to.
   *
   * <p>{@link #SINGLE_TAG_SCALE} is divided back out, because that charge is deliberately not
   * root-N: it prices what a lone tag lacks regardless of its range, and
   * {@link #aLoneTagIsChargedBeyondWhatRootNExplains} is the test for it. What is being checked here
   * is that the rest of the gap is the root two independent measurements earn, continuously, rather
   * than a bonus awarded for crossing from one tag to two.
   */
  @Test
  void aSecondTagAtTheSameRangeImprovesErrorByRootTwo() {
    // Placed symmetrically about the axis to the robot, so the pair's bearing and incidence match
    // the lone tag's and the only thing that changes is how many there are.
    double oneTag = geometricPart(submit(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1)))
        .stdDevs().get(0, 0)) / SINGLE_TAG_SCALE;

    double twoTags = geometricPart(submit(
        fieldWithTagsAt(new Translation3d(4.0, -0.3, 0.0), new Translation3d(4.0, 0.3, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2)))
        .stdDevs().get(0, 0));

    assertEquals(1.0 / Math.sqrt(2.0), twoTags / oneTag, 0.05,
        "two tags should be worth root two, got %f from %f and %f"
            .formatted(twoTags / oneTag, oneTag, twoTags));
  }

  /**
   * A lone tag is charged for more than being one sample of a d^2 error, because what it is missing
   * is not distance. Nothing contradicts a misread corner or a tag matched to the wrong id, and the
   * trig solve rests on the gyro heading, so the ways a one-tag frame goes badly wrong leave no
   * trace in the geometry the model above can see. The pair here is the same two tags as the root-N
   * test, so the gap this asserts is over and above the root two of that.
   */
  @Test
  void aLoneTagIsChargedBeyondWhatRootNExplains() {
    double oneTag = submit(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1)))
        .stdDevs().get(0, 0);

    double twoTags = submit(
        fieldWithTagsAt(new Translation3d(4.0, -0.3, 0.0), new Translation3d(4.0, 0.3, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2)))
        .stdDevs().get(0, 0);

    assertTrue(oneTag / twoTags > Math.sqrt(2.0) * 1.5,
        "a one-tag frame should be distrusted well past root N: %f vs %f".formatted(oneTag, twoTags));
  }

  // MARK: - Quality signals

  /**
   * Range is read off a tag's apparent width, and an obliquely viewed tag has most of its width
   * foreshortened away. This is the part of a solve's quality that distance alone cannot see: both
   * tags here are exactly four metres out.
   */
  @Test
  void anObliqueTagIsTrustedLessThanASquareOnTagAtTheSameRange() {
    double squareOn = submit(
        fieldWithTags(new FieldTag(1, new Pose3d(new Translation3d(4.0, 0.0, 0.0), FACING_BACK))),
        observationAt(Pose3d.ZERO, Set.of(1)))
        .stdDevs().get(0, 0);

    double oblique = submit(
        fieldWithTags(new FieldTag(1, new Pose3d(
            new Translation3d(4.0, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Math.PI - Math.toRadians(70.0))))),
        observationAt(Pose3d.ZERO, Set.of(1)))
        .stdDevs().get(0, 0);

    assertTrue(oblique > squareOn * 2.0,
        "an edge-on tag should be much less trusted: %f vs %f".formatted(oblique, squareOn));
  }

  /**
   * The robot is on the floor, so any height a solve reports is error it measured about itself for
   * free -- and one that is wrong about its height is wrong about x and y too. The old code only
   * ever used this as a cliff.
   */
  @Test
  void aSlightlyElevatedSolveIsDistrustedRatherThanDiscarded() {
    Field field = fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0));

    double level = submit(field, observationAt(Pose3d.ZERO, Set.of(1))).stdDevs().get(0, 0);
    double elevated = submit(
        field,
        observationAt(new Pose3d(new Translation3d(0.0, 0.0, 0.1), new Rotation3d()), Set.of(1)))
        .stdDevs().get(0, 0);

    assertTrue(elevated > level,
        "a solve reporting itself airborne should be trusted less: %f vs %f"
            .formatted(elevated, level));
  }

  /**
   * The estimator replays observations into the past, so latency is handled; timestamp jitter is
   * not, and it costs most while turning, when the lens is moving fast however slowly the chassis
   * is.
   */
  @Test
  void spinningInflatesErrorEvenWithATagThatHasNotMoved() {
    Field field = fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0));
    AprilTagPoseObservation observation = observationAt(Pose3d.ZERO, Set.of(1));

    m_velocities = new ChassisVelocities();
    double stationary = submit(field, observation).stdDevs().get(0, 0);

    m_velocities = new ChassisVelocities(0.0, 0.0, 10.0);
    double spinning = submit(field, observation).stdDevs().get(0, 0);

    assertTrue(spinning > stationary,
        "a lens swinging on a spinning robot should be trusted less: %f vs %f"
            .formatted(spinning, stationary));
  }

  /**
   * Field tags are surveyed to a tolerance and the transform to each lens is measured by hand, and
   * neither improves as the robot gets closer. Without this the model reports millimetres at point
   * blank range, which is exactly where the estimate snapping to vision is most visible.
   */
  @Test
  void errorNeverFallsBelowWhatTheFieldIsSurveyedTo() {
    VisionObservation observation = submit(
        fieldWithTagsAt(
            new Translation3d(0.3, -0.1, 0.0),
            new Translation3d(0.3, 0.0, 0.0),
            new Translation3d(0.3, 0.1, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2, 3)));

    assertTrue(observation.stdDevs().get(0, 0) >= FLOOR_METERS * 0.99,
        "three tags at arm's length must not be trusted absolutely, got "
            + observation.stdDevs().get(0, 0));
    assertTrue(observation.stdDevs().get(1, 0) >= FLOOR_METERS * 0.99,
        "three tags at arm's length must not be trusted absolutely, got "
            + observation.stdDevs().get(1, 0));
  }

  // MARK: - Gates

  /**
   * Ambiguity describes the orientation solve, and the single-tag strategy reads only a tag's range
   * and bearing. Gating on it discarded usable frames for a number the solve never consulted.
   */
  @Test
  void anAmbiguousSingleTagObservationIsStillUsed() {
    Optional<VisionObservation> observation = process(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observation(Pose3d.ZERO, Set.of(1), 0.9, -1.0, NOW_SECONDS));

    assertTrue(observation.isPresent(), "a highly ambiguous single-tag frame is still a measurement");
  }

  /** What replaced the ambiguity gate: the residual a multi-tag fit actually leaves behind. */
  @Test
  void aSolveThatFitsItsTagsPoorlyIsRejected() {
    Optional<VisionObservation> observation = process(
        fieldWithTagsAt(new Translation3d(3.0, -1.0, 0.0), new Translation3d(3.0, 1.0, 0.0)),
        observation(Pose3d.ZERO, Set.of(1, 2), 0.0, 20.0, NOW_SECONDS));

    assertFalse(observation.isPresent(), "a twenty pixel residual is a broken solve");
  }

  @Test
  void aSolveThatPutsTheRobotInTheAirIsRejected() {
    Optional<VisionObservation> observation = process(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observationAt(new Pose3d(new Translation3d(0.0, 0.0, 1.0), new Rotation3d()), Set.of(1)));

    assertFalse(observation.isPresent(), "the robot cannot be a metre off the floor");
  }

  /**
   * Not folded into the distance rejection, because it means the cameras and the filter disagree
   * about which field they are on.
   */
  @Test
  void anObservationWhoseTagsAreNotOnTheFieldIsRejected() {
    Optional<VisionObservation> observation = process(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(999)));

    assertFalse(observation.isPresent(), "a tag the layout has never heard of judges nothing");
  }

  @Test
  void anObservationWithANonsensicalTimestampIsRejected() {
    Optional<VisionObservation> observation = process(
        fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)),
        observation(Pose3d.ZERO, Set.of(1), 0.0, -1.0, NOW_SECONDS - 1.0));

    assertFalse(observation.isPresent(), "a frame a second old means the clock, not the camera");
  }

  /**
   * The gate is fifteen feet to the nearest tag, so a solve whose only tag is past that is refused
   * rather than weighed. Out there the error stops being the smooth d^2 the coefficients describe --
   * half a pixel of corner noise is a large fraction of a tag twenty pixels wide -- so a standard
   * deviation is no longer an honest price for it.
   */
  @Test
  void anObservationWhoseOnlyTagIsBeyondFifteenFeetIsRejected() {
    assertFalse(
        process(fieldWithTagsAt(new Translation3d(6.0, 0.0, 0.0)), observationAt(Pose3d.ZERO, Set.of(1)))
            .isPresent(),
        "a tag six metres away is past the gate");

    assertTrue(
        process(fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)), observationAt(Pose3d.ZERO, Set.of(1)))
            .isPresent(),
        "a tag four metres away is inside it");
  }

  /**
   * One close tag is enough however far the others are, which is why the distance gate reads the
   * nearest rather than the average: under an average this pair would have been thrown away.
   */
  @Test
  void aCloseTagPairedWithAVeryDistantOneIsStillUsed() {
    Optional<VisionObservation> observation = process(
        fieldWithTagsAt(new Translation3d(2.0, 0.0, 0.0), new Translation3d(30.0, 0.0, 0.0)),
        observationAt(Pose3d.ZERO, Set.of(1, 2)));

    assertTrue(observation.isPresent(), "a tag two metres away is a good look at the field");
  }

  // MARK: - The single-tag toggle

  /**
   * With the toggle off the robot runs on multi-tag frames and odometry. This is the switch to reach
   * for when the log shows the estimate being pulled about by one-tag frames -- a camera whose
   * calibration has drifted or whose mounting transform was measured wrong shows up there first,
   * since a lone tag has no second tag to contradict it.
   */
  @Test
  void aSingleTagObservationIsRefusedWhenTheFeatureIsOff() {
    m_singleTagEstimationEnabled = false;

    assertFalse(
        process(
            fieldWithTagsAt(new Translation3d(3.0, 0.0, 0.0)),
            observationAt(Pose3d.ZERO, Set.of(1)))
            .isPresent(),
        "a one-tag frame is not a measurement when single-tag estimation is off");
  }

  /** The toggle turns off one-tag solves and nothing else. */
  @Test
  void aMultiTagObservationIsStillUsedWhenTheSingleTagFeatureIsOff() {
    m_singleTagEstimationEnabled = false;

    assertTrue(
        process(
            fieldWithTagsAt(new Translation3d(3.0, -0.25, 0.0), new Translation3d(3.0, 0.25, 0.0)),
            observationAt(Pose3d.ZERO, Set.of(1, 2)))
            .isPresent(),
        "two tags are two tags however the single-tag toggle is set");
  }

  /**
   * The toggle counts the tags the layout recognised, not the tags the solve claimed. A two-tag
   * solve where one tag is off this field is a one-tag solve, and is the case where believing the
   * claim would be worst: it means the cameras and the filter disagree about what field they are on.
   */
  @Test
  void aSolveWithOnlyOneRecognisedTagCountsAsSingleTagForTheToggle() {
    m_singleTagEstimationEnabled = false;

    assertFalse(
        process(
            fieldWithTagsAt(new Translation3d(3.0, 0.0, 0.0)),
            observationAt(Pose3d.ZERO, Set.of(1, 999)))
            .isPresent(),
        "a solve the layout can only place one tag of is a one-tag solve");
  }

  // MARK: - Where the range comes from

  /**
   * The range is the model's main input, and it is a measurement rather than a consequence of the
   * solve. Taking it from the solved pose instead would have the solve price its own error against
   * its own answer: a multi-tag fit reconciles its tags into one pose, so the ranges that pose
   * implies agree with it by construction however wrong it is, and a flipped solve landing nearer a
   * tag than it really is would be rewarded with a smaller standard deviation for the privilege.
   */
  @Test
  void theRangeTheCameraMeasuredIsWhatPricesTheObservation() {
    // The layout puts this tag 4 m out, so the solved pose implies 3.65 m from the lens. The camera
    // says it measured 1 m. The second number is the one that should matter.
    Field field = fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0));

    double asMeasuredNear = submit(field, measuring(1.0)).stdDevs().get(0, 0);
    double asMeasuredFar = submit(field, measuring(4.4)).stdDevs().get(0, 0);
    double fromTheSolvedPose = submit(field, observationAt(Pose3d.ZERO, Set.of(1)))
        .stdDevs().get(0, 0);

    assertTrue(asMeasuredNear < fromTheSolvedPose,
        "a tag measured at 1 m should be trusted more than the pose's 3.65 m implies: %f vs %f"
            .formatted(asMeasuredNear, fromTheSolvedPose));
    assertTrue(asMeasuredFar > fromTheSolvedPose,
        "a tag measured at 4.4 m should be trusted less: %f vs %f"
            .formatted(asMeasuredFar, fromTheSolvedPose));
  }

  /**
   * A camera that could not supply a transform publishes no range rather than a zero one, since zero
   * would tell the filter the robot is standing on the tag and be believed. The model falls back to
   * the distances the solved pose implies, which is worse but not dangerous.
   */
  @Test
  void anObservationWithNoMeasuredRangesFallsBackToTheSolvedPose() {
    Field field = fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0));

    VisionObservation observation = submit(field, observationAt(Pose3d.ZERO, Set.of(1)));

    // 3.65 m from the lens, well clear of the floor, so the fallback plainly ran.
    assertTrue(observation.stdDevs().get(0, 0) > FLOOR_METERS * 2.0,
        "the fallback should price a 3.65 m tag as a 3.65 m tag, got "
            + observation.stdDevs().get(0, 0));
  }

  /** The distance gate reads the measured range too, so a tag measured beyond the field is refused. */
  @Test
  void anObservationMeasuredBeyondTheFieldIsRejected() {
    assertFalse(
        process(fieldWithTagsAt(new Translation3d(4.0, 0.0, 0.0)), measuring(40.0)).isPresent(),
        "a tag measured forty metres away is not a tag this robot can see");
  }

  // MARK: - Several observations in one loop

  /**
   * A camera's pipeline runs on its own clock, so a loop can take a backlog of frames from it at
   * once. Every one of them is a measurement of a different instant and gets weighed on its own.
   */
  @Test
  void everyObservationFromABacklogIsWeighedSeparately() {
    Field field = fieldWithTagsAt(new Translation3d(2.0, 0.0, 0.0));
    AprilTagCameraConfig cameraIO = FAKE_CAMERA;

    m_accepted.clear();
    processor(field).process(List.of(
        observationFrom(cameraIO, Pose3d.ZERO, Set.of(1), NOW_SECONDS - 0.06),
        observationFrom(cameraIO, Pose3d.ZERO, Set.of(1), NOW_SECONDS - 0.03),
        observationFrom(cameraIO, Pose3d.ZERO, Set.of(1), NOW_SECONDS)));

    assertEquals(3, m_accepted.size(), "each frame in the backlog is its own measurement");
    assertEquals(
        Set.of(NOW_SECONDS - 0.06, NOW_SECONDS - 0.03, NOW_SECONDS),
        m_accepted.stream().map(o -> o.timestamp().in(Seconds)).collect(Collectors.toSet()),
        "each frame keeps the instant it was captured at");
  }

  /** One bad frame in a backlog is dropped without taking the good ones with it. */
  @Test
  void aBadFrameInABacklogDoesNotDiscardTheGoodOnes() {
    Field field = fieldWithTagsAt(new Translation3d(2.0, 0.0, 0.0));
    AprilTagCameraConfig cameraIO = FAKE_CAMERA;

    m_accepted.clear();
    processor(field).process(List.of(
        observationFrom(cameraIO, Pose3d.ZERO, Set.of(1), NOW_SECONDS - 0.03),
        // Solved a metre into the air.
        observationFrom(
            cameraIO,
            new Pose3d(new Translation3d(0.0, 0.0, 1.0), new Rotation3d()),
            Set.of(1),
            NOW_SECONDS - 0.01),
        observationFrom(cameraIO, Pose3d.ZERO, Set.of(1), NOW_SECONDS)));

    assertEquals(2, m_accepted.size(), "only the airborne frame should have been rejected");
  }

  // MARK: - Fixtures

  /**
   * A processor reading this test's velocities and toggle, so a test can vary either by assigning the
   * field rather than by building its own.
   */
  private AprilTagVisionProcessor processor(Field field) {
    return new AprilTagVisionProcessor(
        field, () -> m_velocities, () -> m_singleTagEstimationEnabled, m_accepted::add);
  }

  /** Runs one observation through a processor and returns the accepted result, if there was one. */
  private Optional<VisionObservation> process(Field field, AprilTagPoseObservation observation) {
    m_accepted.clear();

    processor(field).process(List.of(observation));

    return m_accepted.stream().findFirst();
  }

  /** As {@link #process}, for the tests that are about the weight rather than the verdict. */
  private VisionObservation submit(Field field, AprilTagPoseObservation observation) {
    return process(field, observation)
        .orElseThrow(() -> new AssertionError("observation was rejected, expected it to be weighed"));
  }

  private AprilTagPoseObservation observationAt(Pose3d robotPose, Set<Integer> tags) {
    return observation(robotPose, tags, 0.0, -1.0, NOW_SECONDS);
  }

  private AprilTagPoseObservation observation(
      Pose3d robotPose,
      Set<Integer> tags,
      double ambiguity,
      double reprojectionErrorPixels,
      double timestampSeconds) {
    return new AprilTagPoseObservation(
        FAKE_CAMERA,
        robotPose,
        Seconds.of(timestampSeconds),
        tags,
        ambiguity,
        reprojectionErrorPixels,
        NO_MEASURED_RANGES);
  }

  /**
   * One standard deviation with the processor's floor taken back off, leaving the part the solve's
   * geometry earned.
   *
   * <p>The floor is a fixed term added in quadrature, and the distance gate caps how large the
   * geometric term beside it can get, so within the allowed range the two are comparable in size: a
   * pair of tags four metres out prices itself at a few centimetres, which is roughly the floor.
   * Subtracting it is what lets a test assert the exponent rather than measure the floor. Exact
   * rather than approximate, since these tests hold the robot still and so contribute no motion
   * term.
   */
  private static double geometricPart(double stdDev) {
    return Math.sqrt(stdDev * stdDev - FLOOR_METERS * FLOOR_METERS);
  }

  /** An observation of tag 1 whose camera measured it at {@code rangeMeters}. */
  private AprilTagPoseObservation measuring(double rangeMeters) {
    return new AprilTagPoseObservation(
        FAKE_CAMERA,
        Pose3d.ZERO,
        Seconds.of(NOW_SECONDS),
        Set.of(1),
        0.0,
        -1.0,
        new double[] { rangeMeters });
  }

  /** An observation attributed to a particular camera, for the tests about backlogs. */
  private static AprilTagPoseObservation observationFrom(
      AprilTagCameraConfig camera, Pose3d robotPose, Set<Integer> tags, double timestampSeconds) {
    return new AprilTagPoseObservation(
        camera, robotPose, Seconds.of(timestampSeconds), tags, 0.0, -1.0, NO_MEASURED_RANGES);
  }

  /** Tags at floor level on a wall ahead of the robot, numbered from one in the order given. */
  private static Field fieldWithTagsAt(Translation3d... positions) {
    FieldTag[] tags = new FieldTag[positions.length];

    for (int i = 0; i < positions.length; i++) {
      tags[i] = new FieldTag(i + 1, new Pose3d(positions[i], FACING_BACK));
    }

    return fieldWithTags(tags);
  }

  private static Field fieldWithTags(FieldTag... tags) {
    return new Field(
        "Test", "2027", "Test", null, 16.0, 8.0, "FRC", List.of(tags));
  }

  /**
   * A camera a third of a metre in front of the robot origin, far enough out that the lever arm on a
   * spinning robot is worth measuring and that measuring range from the origin instead would be
   * visibly wrong.
   */
  private static final AprilTagCameraConfig FAKE_CAMERA = new AprilTagCameraConfig(
      "FakeCamera", new Transform3d(new Translation3d(0.35, 0.0, 0.0), new Rotation3d()));
}
