package first.lib;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleFunction;
import java.util.function.Function;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public class InterpolatingMeasureTreeMap<K extends Measure<?>, V extends Measure<?>> {
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

  public static InterpolatingMeasureTreeMap<Distance, Angle> createDistanceToAngleMap() {
    return new InterpolatingMeasureTreeMap<Distance, Angle>(
        (Distance distance) -> distance.in(Feet),
        Degrees::of,
        (Angle angle) -> angle.in(Degrees));
  }

  public static InterpolatingMeasureTreeMap<Distance, AngularVelocity> createDistanceToAngularVelocityMap() {
    return new InterpolatingMeasureTreeMap<Distance, AngularVelocity>(
        (Distance distance) -> distance.in(Feet),
        RPM::of,
        (AngularVelocity angularVelocity) -> angularVelocity.in(RPM));
  }

  public static InterpolatingMeasureTreeMap<Distance, Time> createDistanceToTimeMap() {
    return new InterpolatingMeasureTreeMap<Distance, Time>(
        (Distance distance) -> distance.in(Feet),
        Seconds::of,
        (Time time) -> time.in(Seconds));
  }
}
