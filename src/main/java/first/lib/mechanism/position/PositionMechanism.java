package first.lib.mechanism.position;

import static org.wpilib.units.Units.Inches;

import org.wpilib.units.measure.Distance;

import first.lib.mechanism.G3Mechanism;

public interface PositionMechanism<T extends PositionMechanismIO> extends G3Mechanism<T, Distance> {
  @Override
  public default Distance getCurrentMeasurement() {
    return getIO().getPosition();
  }

  @Override
  public default Distance getTarget() {
    return getIO().getTargetPosition();
  }

  @Override
  public default void setTarget(Distance position) {
    getIO().setTargetPosition(position);
  }

  @Override
  default boolean isNear(Distance measure, Distance tolerance) {
    return getCurrentMeasurement().isNear(measure, tolerance);
  }

  @Override
  default Distance getDefaultTolerance() {
    return Inches.of(0.25);
  }
}
