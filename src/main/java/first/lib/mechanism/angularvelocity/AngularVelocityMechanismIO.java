package first.lib.mechanism.angularvelocity;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import first.lib.mechanism.G3MechanismIO;

public interface AngularVelocityMechanismIO
    extends G3MechanismIO<AngularVelocityMechanismInputsAutoLogged> {
  /**
   * Everything an {@link AngularVelocityMechanismIO} reports each loop. {@code @AutoLog} generates
   * {@link AngularVelocityMechanismInputsAutoLogged}, which adds the serialization the logger
   * needs.
   */
  @AutoLog
  public static class AngularVelocityMechanismInputs {
    public Angle currentAngle = Degrees.of(0);
    public AngularVelocity currentAngularVelocity = RPM.of(0);
    public AngularVelocity targetAngularVelocity = RPM.of(0);
  }

  public void setTargetAngularVelocity(AngularVelocity angularVelocity);
}
