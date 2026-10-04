package first.robot.mechanism.vision.apriltag;

import java.util.List;
import java.util.Optional;

import org.wpilib.math.geometry.Pose2d;

import first.lib.mechanism.LoggedComponent;
import first.lib.mechanism.LoggedMultiComponentMechanism;

public class AprilTagVision implements LoggedMultiComponentMechanism {
  private final List<AprilTagCamera> m_cameras;
  private final Optional<AprilTagVisionSim> m_sim;

  /** Cameras with nothing simulated behind them, which is every camera on a real robot. */
  public AprilTagVision(List<AprilTagCamera> cameras) {
    this(cameras, Optional.empty());
  }

  /**
   * @param cameras the cameras on the robot, each pairing a declaration with the IO that reads it.
   *                Built outside rather than from configs here, so that what reads a camera stays a
   *                decision for {@link AprilTagVisionFactory} and this class never has to know a
   *                mode exists.
   * @param sim     the field these cameras are looking at, when it is one this robot code is
   *                rendering; empty on a real robot, where the field is simply there. Held rather
   *                than merely stepped so that {@link #resetRobotPose} has something to forward to.
   */
  public AprilTagVision(List<AprilTagCamera> cameras, Optional<AprilTagVisionSim> sim) {
    m_cameras = List.copyOf(cameras);
    m_sim = sim;

    getRegisteredScheduler().addPeriodic(this::periodic);
  }

  /**
   * Tells the simulated field, if there is one, that the robot has been put somewhere rather than
   * having driven there. A no-op on a real robot, where moving a pose estimate moves no robots.
   *
   * <p>This is why the mechanism holds the simulation rather than just a step to run: a pose reset
   * happens far from here, in whatever owns the estimate, and that is the only place that knows one
   * happened. See {@link AprilTagVisionSim#resetRobotPose} for what goes wrong when it is not told.
   *
   * @param robotPose where the robot has just been declared to be
   */
  public void resetRobotPose(Pose2d robotPose) {
    m_sim.ifPresent(sim -> sim.resetRobotPose(robotPose));
  }

  /**
   * Steps whatever the cameras are looking at, then reads them.
   *
   * <p>One periodic rather than two, and that is the point of the simulation's step living here.
   * Rendering a simulated field after its cameras had already looked at it would show them the
   * previous loop's field -- a latency no real camera has and no log would explain. While the
   * renderer and the cameras were registered separately, the only thing preventing that was the
   * order the two happened to be registered in, across two files, with a comment asking the next
   * person not to disturb it. Here it is the order of two statements.
   */
  private void periodic() {
    m_sim.ifPresent(AprilTagVisionSim::update);
    updateComponents();
  }

  @Override
  public List<? extends LoggedComponent<?, ?>> getComponents() {
    return m_cameras;
  }

  /**
   * Returns every observation from every camera since the last loop. A camera can contribute more
   * than one, so this is not one entry per camera.
   */
  public List<AprilTagPoseObservation> getLatestObservations() {
    return m_cameras.stream().map(AprilTagCamera::getObservations).flatMap(List::stream).toList();
  }
}
