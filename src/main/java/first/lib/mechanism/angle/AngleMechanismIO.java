package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.ZeroableMechanismIO;

/**
 * Reads and commands a mechanism's angle, in the zeroed frame.
 *
 * <p>{@link ZeroableMechanismIO} is where the raw sensor reading lives: {@code currentAngle} and
 * {@code targetAngle} below are both already offset, so nothing above an implementation of this has
 * a raw encoder value to get wrong.
 */
public interface AngleMechanismIO
    extends ZeroableMechanismIO<AngleMechanismInputsAutoLogged, Angle> {
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
