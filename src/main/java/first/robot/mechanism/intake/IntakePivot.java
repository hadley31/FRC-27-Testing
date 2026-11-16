package first.robot.mechanism.intake;

import first.lib.mechanism.angle.AngleMechanism;
import first.lib.mechanism.angle.AngleMechanismIO;
import first.robot.RobotGeometryViz;

/**
 * The intake's deploy pivot: the arm that swings the roller assembly between stowed and floor.
 *
 * <p>Zero is the stowed position the robot model is drawn in, so {@link RobotGeometryViz} can take
 * this angle as the rotation to apply to the intake component and nothing has to subtract a
 * mechanical offset first.
 */
public class IntakePivot extends AngleMechanism<AngleMechanismIO> {
  public IntakePivot(AngleMechanismIO io) {
    super(io);
  }
}
