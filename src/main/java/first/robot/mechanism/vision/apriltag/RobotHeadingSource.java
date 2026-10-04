package first.robot.mechanism.vision.apriltag;

import org.wpilib.math.geometry.Rotation2d;

/**
 * Where an AprilTag camera gets the robot's heading.
 *
 * <p>A single-tag solve cannot recover a full pose on its own: one tag fixes the camera's distance
 * and bearing, and the robot's own heading is what turns those into a field position. So the IO
 * layer needs heading from outside itself, and this is the narrow interface it asks for rather than
 * taking a dependency on the drivetrain or the pose estimator. Multi-tag solves do not need it and
 * ignore it.
 *
 * <p>The heading must be <b>field-relative</b> — the same frame the tag layout is in, not the raw
 * gyro reading, which is only field-relative until the pose is reset — and it should be the
 * <b>best heading estimate available</b>, which here means the pose estimator's fused estimate
 * rather than its odometry pose.
 *
 * <p>That is worth spelling out, because feeding a vision solve part of the estimate it will go on
 * to correct sounds like exactly the loop one should avoid. What makes it the right choice is that
 * the trig solve hands the heading straight back: the pose it returns carries the heading it was
 * given, verbatim. So a single-tag observation built on the fused estimate arrives with no
 * rotational disagreement, and contributes only translation — which is all a single tag actually
 * measures. Fed odometry instead, every single-tag observation would arrive disagreeing with the
 * estimate by however much vision had corrected the heading so far, and would drag it back toward
 * the uncorrected gyro; at a 2 m standoff that erases more than half of a multi-tag heading fix
 * within a second. Heading observability comes from multi-tag solves, and this keeps them in sole
 * charge of it.
 *
 * <p>The cost is that a badly wrong heading produces a position rotated about the tag by the same
 * error, which none of {@link AprilTagVisionProcessor}'s checks will catch and which single-tag
 * frames cannot themselves correct. Recovering from one takes a multi-tag sighting or a pose reset.
 *
 * <p>This is a separate question from what a simulated camera is posed from, which must be a pose
 * vision has never touched; see {@link AprilTagVisionSim#update}.
 */
@FunctionalInterface
public interface RobotHeadingSource {
  /** The robot's field-relative heading, now. */
  public Rotation2d getRobotHeading();
}
