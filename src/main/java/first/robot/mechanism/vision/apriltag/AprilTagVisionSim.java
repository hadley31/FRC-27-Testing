package first.robot.mechanism.vision.apriltag;

import org.wpilib.math.geometry.Pose2d;

/**
 * A simulated field for the AprilTag cameras to look at.
 *
 * <p>Two things happen to a simulated world: it advances, and occasionally the robot in it is picked
 * up and put somewhere. Those are the two methods here, and nothing above this interface needs to
 * know what renders either of them — which is the point, since the tooling that simulates cameras
 * need not come from the same vendor as the cameras on the real robot.
 *
 * <p>Deliberately not an {@link AprilTagCameraIO}. A simulated field is scoped to the robot rather
 * than to a camera: one of these holds the tag layout and the robot's true pose, and every camera
 * registers into it. How a camera registers is the implementation's own business and does not appear
 * here, because it is the one part that cannot be vendor-neutral.
 */
public interface AprilTagVisionSim {
  /**
   * Renders what every camera on the robot can see from where the robot now really is.
   *
   * <p>Called once per loop by {@link AprilTagVision}, immediately before the cameras are read:
   * rendering after they had looked would show them the previous loop's field, a latency no real
   * camera has and no log would explain.
   *
   * <p>Takes no pose because an implementation is built knowing where to read the robot's true
   * position from. Per-loop ground truth is pulled rather than pushed so that nothing between here
   * and the pose estimator has to carry a pose it has no other use for.
   */
  public void update();

  /**
   * Puts the robot somewhere it did not drive to, discarding the history behind it.
   *
   * <p>Pushed rather than pulled, unlike {@link #update}, because a reset is an event rather than a
   * reading: the caller knows a pose has just been declared and no amount of looking at the world
   * afterwards distinguishes that from having driven there.
   *
   * <p>Discarding the history is the part that matters, and it is why this cannot be left to
   * {@link #update} noticing. A simulation that models camera latency has to render each frame from
   * where the robot was when that frame was exposed, so it keeps a trail of recent poses; a reset
   * leaves a discontinuity in that trail, and interpolating across it renders the robot part way
   * back to where it came from, on frames stamped after the reset. Nothing downstream can tell those
   * from honest frames, because they are not stale: they are wrong.
   *
   * @param robotPose where the robot now is, with no history behind it
   */
  public void resetRobotPose(Pose2d robotPose);
}
