package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3MechanismIO;

public interface AngleMechanismIO extends G3MechanismIO<AngleMechanismInputsAutoLogged> {
  /**
   * Everything an {@link AngleMechanismIO} reports each loop. {@code @AutoLog} generates
   * {@link AngleMechanismInputsAutoLogged}, which adds the serialization the logger needs.
   */
  @AutoLog
  public static class AngleMechanismInputs {
    public Angle currentAngle = Degrees.of(0);
    public Angle targetAngle = Degrees.of(0);
    public AngularVelocity currentAngularVelocity = RPM.of(0);
  }

  public void setTargetAngle(Angle angle);
}
