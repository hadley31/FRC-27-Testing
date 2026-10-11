package first.robot.mechanism.intake;

import first.lib.mechanism.angle.AngleMechanism;
import first.lib.mechanism.angle.AngleMechanismIO;
import first.lib.mechanism.angle.AngleMechanismInputsAutoLogged;
import first.robot.RobotGeometryViz;

/**
 * The intake's deploy pivot: the arm that swings the roller assembly between stowed and floor.
 *
 * <p>Zero is the stowed position the robot model is drawn in, so {@link RobotGeometryViz} can take
 * this angle as the rotation to apply to the intake component and nothing has to subtract a
 * mechanical offset first.
 */
public class IntakePivot implements AngleMechanism<AngleMechanismIO> {
  private final AngleMechanismIO m_io;
  private final AngleMechanismInputsAutoLogged m_inputs = new AngleMechanismInputsAutoLogged();

  public IntakePivot(AngleMechanismIO io) {
    m_io = io;
    getRegisteredScheduler().addPeriodic(this::update);
    registerTuningCommand();
  }

  @Override
  public AngleMechanismIO getIO() {
    return m_io;
  }

  @Override
  public AngleMechanismInputsAutoLogged getInputs() {
    return m_inputs;
  }
}
