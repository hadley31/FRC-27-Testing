package first.robot.mechanism.vision.apriltag;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

import first.lib.mechanism.LoggedComponentIO;

public interface AprilTagCameraIO extends LoggedComponentIO<AprilTagCameraIOInputsAutoLogged> {
  /**
   * One entry per estimate the camera reported since the last loop, oldest first. Usually one, but
   * zero or several are both normal: the camera's pipeline runs on its own clock, so a loop can
   * observe no new frames or a backlog of them.
   *
   * <p>An estimate is spread across several arrays rather than gathered into one record because the
   * tag IDs cannot be gathered with the rest: a struct is fixed-size and the number of tags in a
   * solve is not. The arrays are index-aligned and always the same length, and every implementation
   * of {@link AprilTagCameraIO} owes that invariant to {@link AprilTagCamera}, which reads them
   * back as whole observations.
   */
  @AutoLog
  public static class AprilTagCameraIOInputs {
    public boolean connected = false;
    public boolean enabled = false;
    /**
     * When each frame was captured, on the same timebase as {@code Timer.getTimestamp()}.
     *
     * <p>Seconds rather than a {@code Time} on purpose. A {@code Time} is itself a record, and its
     * unit is a type AdvantageKit cannot pack into a struct, so a logged one comes back from replay
     * carrying a null unit and blows up on the first arithmetic that touches it. {@link
     * AprilTagCamera} attaches the unit on the way out.
     */
    public double[] timestampsSeconds = new double[0];

    /** The robot pose each frame was solved to. */
    public Pose3d[] observedRobotPoses = new Pose3d[0];

    /**
     * How ambiguous each solve was, 0 (unambiguous) to 1, or -1 where the camera reported none.
     *
     * <p>-1 means unknown, not bad. A multi-tag solve has no alternate solution to be ambiguous
     * against and always reports 0, and a single-tag trig solve never consults the orientation solve
     * this number describes, so treating an absent value as the worst case would throw away good
     * frames. {@link AprilTagVisionProcessor} skips the term instead.
     */
    public double[] ambiguities = new double[0];

    /**
     * The reprojection error of each solve in pixels, or -1 where the camera reported none.
     *
     * <p>Only a multi-tag solve has one: it is the residual left over after fitting one camera pose
     * to every tag corner at once, which makes it the most direct measure of solve quality available
     * -- a mis-surveyed tag, a stale calibration or a misassociated corner all show up here and
     * nowhere else. A single-tag solve fits as many parameters as it has observations, so its
     * residual is zero by construction and says nothing.
     */
    public double[] reprojectionErrorsPixels = new double[0];

    /** The IDs of the tags behind each solve, one row per estimate. */
    public int[][] tagIds = new int[0][];

    /**
     * How far the camera measured each of those tags to be, in metres, one row per estimate.
     *
     * <p>Measured, not inferred. This is the range solvePnP read off the tag's apparent size, which
     * is what the single-tag strategy builds its pose out of in the first place; the alternative is
     * to take the pose the solve arrived at and work backwards to how far away the tags must have
     * been, which asks the solve to price its own error against its own answer. A multi-tag fit
     * reconciles several tags into one pose, so the ranges implied by that pose are the fitted ones
     * and agree with it by construction however wrong it is.
     *
     * <p>A row is not aligned with {@link #tagIds} and can be shorter than it: a tag whose transform
     * the camera could not supply is left out rather than published as zero, and nothing above needs
     * the pairing, because combining ranges is a sum and finding the nearest is a minimum. An empty
     * row means no tag in that solve had a usable transform.
     */
    public double[][] tagRangesMeters = new double[0][];

    public int pipelineId = -1;
  }

  public String getName();

  /**
   * Where this camera sits relative to the robot origin.
   *
   * <p>A constant of the camera rather than logged data, so it is the same on the robot and in
   * replay. {@link AprilTagVisionProcessor} needs it because tag range has to be measured from the
   * lens: the error model squares that range, and at close quarters the camera's offset from the
   * robot origin is a large fraction of it.
   */
  public Transform3d getRobotToCamera();

  public int getPipelineIndex();

  public void setPipelineIndex(int pipelineId);
}
