package first.robot.mechanism.vision.apriltag;

import org.wpilib.math.geometry.Transform3d;

/**
 * One AprilTag camera as declared, with nothing in it about how the camera is read.
 *
 * <p>These two values are everything that distinguishes one camera from another and that is true in
 * every robot mode: the real camera, its simulated stand-in and its replay placeholder all describe
 * the same camera in the same place. Separating them from the IO is what lets the cameras be
 * declared once, as data, and lets an {@link AprilTagVisionFactory} decide per mode what to build
 * from each declaration.
 *
 * @param name          the camera's name, which must match the name the coprocessor publishes under
 *                      and the name its inputs were logged under, since that is what selects the
 *                      recorded inputs during replay
 * @param robotToCamera where the camera sits relative to the robot origin
 */
public record AprilTagCameraConfig(String name, Transform3d robotToCamera) {
}
