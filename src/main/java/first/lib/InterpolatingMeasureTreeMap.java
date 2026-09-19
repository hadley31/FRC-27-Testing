package first.lib;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleFunction;
import java.util.function.Function;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public class InterpolatingMeasureTreeMap<K extends Measure<?>, V extends Measure<?>> {
  private static final TimeUnit TIME_UNIT = Seconds;
  private static final DistanceUnit DISTANCE_UNIT = Meters;
  private static final AngleUnit ANGLE_UNIT = Radians;
  private static final AngularVelocityUnit ANGULAR_VELOCITY_UNIT = RotationsPerSecond;

  private final InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();
  private final Function<K, Double> m_keyToInternalMapper;
  private final DoubleFunction<V> m_internalToValueMapper;
  private final Function<V, Double> m_valueToInternalMapper;

  public InterpolatingMeasureTreeMap(Function<K, Double> keyMapper, DoubleFunction<V> valueMapper,
      Function<V, Double> inverseValueMapper) {
    m_keyToInternalMapper = keyMapper;
    m_internalToValueMapper = valueMapper;
    m_valueToInternalMapper = inverseValueMapper;
  }

  public V get(K key) {
    return m_internalToValueMapper.apply(m_map.get(m_keyToInternalMapper.apply(key)));
  }

  public InterpolatingMeasureTreeMap<K, V> put(K key, V value) {
    m_map.put(m_keyToInternalMapper.apply(key), m_valueToInternalMapper.apply(value));
    return this;
  }

  public static InterpolatingMeasureTreeMap<Distance, Angle> distanceToAngle() {
    return new InterpolatingMeasureTreeMap<Distance, Angle>(
        (Distance distance) -> distance.in(DISTANCE_UNIT),
        ANGLE_UNIT::of,
        (Angle angle) -> angle.in(ANGLE_UNIT));
  }

  public static InterpolatingMeasureTreeMap<Distance, AngularVelocity> distanceToAngularVelocity() {
    return new InterpolatingMeasureTreeMap<Distance, AngularVelocity>(
        (Distance distance) -> distance.in(DISTANCE_UNIT),
        ANGULAR_VELOCITY_UNIT::of,
        (AngularVelocity angularVelocity) -> angularVelocity.in(ANGULAR_VELOCITY_UNIT));
  }

  public static InterpolatingMeasureTreeMap<Distance, Time> distanceToTime() {
    return new InterpolatingMeasureTreeMap<Distance, Time>(
        (Distance distance) -> distance.in(DISTANCE_UNIT),
        TIME_UNIT::of,
        (Time time) -> time.in(TIME_UNIT));
  }

  public static InterpolatingMeasureTreeMap<Time, Distance> timeToDistance() {
    return new InterpolatingMeasureTreeMap<Time, Distance>(
        (Time time) -> time.in(TIME_UNIT),
        DISTANCE_UNIT::of,
        (Distance distance) -> distance.in(DISTANCE_UNIT));
  }
}
