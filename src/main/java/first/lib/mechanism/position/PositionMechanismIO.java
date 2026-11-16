package first.lib.mechanism.position;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import first.lib.mechanism.ZeroableMechanismIO;

/**
 * Reads and commands a mechanism's position along its travel, in the zeroed frame.
 *
 * <p>{@link ZeroableMechanismIO} is where the raw sensor reading lives: {@code currentPosition} and
 * {@code targetPosition} below are both already offset, so nothing above an implementation of this
 * has a raw encoder value to get wrong.
 */
public interface PositionMechanismIO
    extends ZeroableMechanismIO<PositionMechanismInputsAutoLogged, Distance> {
  /**
   * Everything a {@link PositionMechanismIO} reports each loop. {@code @AutoLog} generates
   * {@link PositionMechanismInputsAutoLogged}, which adds the serialization the logger needs.
   */
  @AutoLog
  public static class PositionMechanismInputs {
    public Distance currentPosition = Inches.of(0);
    public Distance targetPosition = Inches.of(0);
    public LinearVelocity currentVelocity = InchesPerSecond.of(0);
  }

  public void setTargetPosition(Distance position);
}
