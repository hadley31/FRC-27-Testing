package first.lib.mechanism.angle;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.units.measure.Angle;

import first.lib.mechanism.G3Mechanism;

public interface AngleMechanism<T extends AngleMechanismIO> extends G3Mechanism<T, Angle> {
  @Override
  public default Angle getCurrentMeasurement() {
    return getIO().getAngle();
  }

  @Override
  public default Angle getTarget() {
    return getIO().getTargetAngle();
  }

  @Override
  public default void setTarget(Angle angle) {
    getIO().setTargetAngle(angle);
  }

  @Override
  public default boolean isNear(Angle measure, Angle tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  default Angle getDefaultTolerance() {
    return Degrees.of(0.5);
  }
}
