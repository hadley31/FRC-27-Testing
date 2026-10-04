package first.robot.mechanism.vision.apriltag;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.wpilib.command3.Scheduler;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;

import first.robot.util.Constants.RobotModeConstants.Mode;

/**
 * Builds the {@link AprilTagVision} mechanism for a robot mode.
 *
 * <p>This is the one place that knows a mode exists. Every difference between running on a robot,
 * running in simulation and replaying a log lives in the switch below: which IO each declared camera
 * gets, and whether there is a simulated field to render. Above this class the cameras are the same
 * cameras, and below it each IO knows only how to read the thing it was handed.
 */
public final class AprilTagVisionFactory {
  private AprilTagVisionFactory() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Returns a vision mechanism with one camera per declaration, read the way {@code mode} requires.
   *
   * @param mode                     the mode the robot code is running in
   * @param field                    the tag layout the cameras solve against, and in simulation the
   *                                 layout that gets rendered
   * @param cameras                  the cameras on the robot
   * @param headingSource            the robot heading single-tag solves need; see
   *                                 {@link RobotHeadingSource} for what it must be
   * @param groundTruthPoseSupplier  where the robot really is, used only in simulation and only to
   *                                 render what the cameras see. This must be a pose no vision
   *                                 observation has corrected; see {@link PhotonVisionSim#update}.
   */
  public static AprilTagVision create(
      Mode mode,
      Field field,
      List<AprilTagCameraConfig> cameras,
      RobotHeadingSource headingSource,
      Supplier<Pose2d> groundTruthPoseSupplier) {
    Wiring wiring = wiringFor(mode, field, headingSource, groundTruthPoseSupplier);

    return new AprilTagVision(
        wiring.onPoseReset(),
        cameras.stream().map(wiring.ioFactory()::create).toArray(AprilTagCameraIO[]::new));
  }

  /**
   * What a mode needs wired up: how to build its cameras, and what a pose reset has to tell.
   *
   * @param ioFactory   turns one camera declaration into the IO that reads it
   * @param onPoseReset what to notify when the pose estimate is teleported, which is nothing outside
   *                    simulation; see {@link AprilTagVision#onPoseReset}
   */
  private record Wiring(AprilTagCameraIOFactory ioFactory, Consumer<Pose2d> onPoseReset) {
  }

  /**
   * The strategy for building camera IOs in {@code mode}.
   *
   * <p>Simulation shares the real robot's IO rather than getting one of its own, because there is
   * nothing for a second IO to implement: PhotonVision simulates the camera, not the reading of it,
   * so a simulated camera publishes to the same NetworkTables topics a coprocessor would and the IO
   * reads them with the same code. Only the construction differs. Replay reads nothing, since the
   * log supplies the inputs, and must not construct a real camera: that would open NetworkTables
   * subscriptions and run a coprocessor version check for values that are about to be overwritten
   * from the log.
   */
  private static Wiring wiringFor(
      Mode mode,
      Field field,
      RobotHeadingSource headingSource,
      Supplier<Pose2d> simGroundTruthPoseSupplier) {
    return switch (mode) {
      case REAL -> new Wiring(
          config -> new AprilTagCameraIOPhotonVision(config, field, headingSource),
          // Moving the estimate does not move a real robot, and its cameras go on reporting where it
          // really is. There is nothing to tell.
          pose -> {
          });

      case SIM -> {
        PhotonVisionSim visionSim = new PhotonVisionSim(field);

        // The simulated field has to be rendered before the cameras look at it, and the scheduler
        // runs periodics in registration order. Registering here rather than in Robot is what makes
        // that ordering structural: this runs while the factory is being chosen, so it cannot help
        // but come before the AprilTagVision that is built from the factory's cameras.
        Scheduler.getDefault().addPeriodic(() -> visionSim.update(simGroundTruthPoseSupplier.get()));

        yield new Wiring(
            config -> AprilTagCameraIOPhotonVision.simulated(config, field, headingSource, visionSim),
            visionSim::resetRobotPose);
      }

      // Replay renders nothing and reads nothing: the log supplies the inputs, including whatever
      // the cameras saw around a reset the log already recorded.
      case REPLAY -> new Wiring(AprilTagCameraIOReplay::new, pose -> {
      });
    };
  }
}
