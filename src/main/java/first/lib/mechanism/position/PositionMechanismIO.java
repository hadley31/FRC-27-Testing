package first.lib.mechanism.position;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import first.lib.mechanism.G3MechanismIO;

public interface PositionMechanismIO extends G3MechanismIO<PositionMechanismInputsAutoLogged> {
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
