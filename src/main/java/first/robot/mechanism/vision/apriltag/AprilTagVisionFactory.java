package first.robot.mechanism.vision.apriltag;

import java.util.List;
import java.util.Optional;

/**
 * One way of reading AprilTag cameras, and the seam a vendor is swapped at.
 *
 * <p>What varies between a real robot, a simulation and a log replay is only how a camera is read —
 * never which cameras exist or where they are mounted. Those are declared once as
 * {@link AprilTagCameraConfig}s, and an implementation of this turns every declaration into the IO
 * that reads it. Nothing above the IO layer branches on the mode, and a test can supply cameras of
 * its own by passing a factory rather than by pretending to be a mode.
 *
 * <p>An implementation holds whatever its vendor needs and nothing else: a tag layout, a heading
 * source, a simulated field. Everything PhotonVision-specific lives in
 * {@link AprilTagVisionFactoryPhotonVision} and {@link AprilTagVisionFactoryPhotonVisionSim}, so
 * reading cameras some other way means writing one more implementation of this and nothing else.
 *
 * <h2>Mixing vendors across modes</h2>
 *
 * <p>Each mode picks its factory independently — see {@code Robot.createVisionFactory} — so the
 * robot and the simulation need not read cameras the same way. A robot on Limelights can still be
 * simulated with PhotonVision's tooling by choosing a Limelight factory for {@code REAL} and
 * {@link AprilTagVisionFactoryPhotonVisionSim} for {@code SIM}. That pairing is not a compromise but
 * the only thing that can work: a simulated PhotonVision camera publishes to PhotonVision's
 * NetworkTables topics, so the IO reading a simulated field has to belong to whatever rendered it.
 */
@FunctionalInterface
public interface AprilTagVisionFactory {
  /**
   * Returns the IO to read {@code config}'s camera through.
   *
   * @param config the camera to build for
   */
  public AprilTagCameraIO createCameraIO(AprilTagCameraConfig config);

  /**
   * The simulated field this factory's cameras are looking at, if this robot code is rendering one.
   *
   * <p>Empty for a real robot, where what a camera sees is the world and nothing has to put it
   * there. Simulation needs it for two reasons that travel together: the field has to be rendered
   * before the cameras look at it each loop, and it has to be told when the robot's pose is reset.
   * {@link AprilTagVision} owns both, which is why this returns the field rather than just the work
   * of rendering it.
   */
  default Optional<AprilTagVisionSim> createSim() {
    return Optional.empty();
  }

  /**
   * Returns the vision mechanism for a robot with these cameras, one IO per declaration.
   *
   * <p>This is the whole of what a mode decides: which cameras exist is declaration data passed in,
   * and the {@link AprilTagCamera}s themselves are built by the mechanism that owns them, so a
   * factory only ever says how a camera is read.
   *
   * @param cameras the cameras on the robot
   */
  default AprilTagVision createVision(List<AprilTagCameraConfig> cameras) {
    return new AprilTagVision(cameras.stream().map(this::createCameraIO).toList(), createSim());
  }
}
