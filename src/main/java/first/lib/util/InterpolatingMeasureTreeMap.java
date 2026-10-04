package first.lib.util;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.function.Consumer;
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

/**
 * An {@link InterpolatingDoubleTreeMap} with {@link Measure} keys and values, so a lookup table can
 * be written in whatever units it was measured in and read in whatever units the caller wants.
 *
 * <p>Points are stored as plain doubles in one fixed unit per dimension, so a conversion happens at
 * {@link #put} and {@link #get} rather than on every interpolation step.
 *
 * <h2>Mutability</h2>
 *
 * <p>A map filled with {@link #put} and then left alone is effectively immutable, which is how the
 * hand-tuned tables in {@code ShotProfile} are declared: the builder-style {@code put} exists for
 * that, not as an invitation to edit a table later.
 *
 * <p>{@link #replaceAll} is the exception, for tables a dashboard edits while the robot is running.
 * It swaps the entire set of points in a single reference write, so a reader can never observe a
 * half-built table, and a failure part-way through building the replacement leaves the previous
 * points untouched. That is what lets a live-tuned table be handed out as an ordinary field — see
 * {@code TunableShotProfile} — without every reader having to re-fetch it.
 */
public class InterpolatingMeasureTreeMap<K extends Measure<?>, V extends Measure<?>> {
  private static final TimeUnit TIME_UNIT = Seconds;
  private static final DistanceUnit DISTANCE_UNIT = Meters;
  private static final AngleUnit ANGLE_UNIT = Radians;
  private static final AngularVelocityUnit ANGULAR_VELOCITY_UNIT = RotationsPerSecond;

  private InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();
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
    Double interpolated = m_map.get(m_keyToInternalMapper.apply(key));

    if (interpolated == null) {
      // InterpolatingTreeMap returns null only when it holds no points at all. Left alone that
      // unboxes into an NPE thrown from inside a unit conversion, several frames away from the
      // actual mistake, which is a table that was never filled in.
      throw new IllegalStateException("Cannot interpolate a table with no points");
    }

    return m_internalToValueMapper.apply(interpolated);
  }

  public InterpolatingMeasureTreeMap<K, V> put(K key, V value) {
    m_map.put(m_keyToInternalMapper.apply(key), m_valueToInternalMapper.apply(value));
    return this;
  }

  /**
   * Replaces every point in this map with the points {@code replacement} fills in.
   *
   * <p>The swap is a single reference write, so {@link #get} either sees the whole old table or the
   * whole new one, and throwing out of {@code replacement} leaves the old table in place. Callers
   * must still ensure the replacement ends up non-empty; {@link #get} on an empty table throws.
   *
   * @param replacement receives an empty map carrying this map's units, to fill via {@link #put}
   */
  public void replaceAll(Consumer<InterpolatingMeasureTreeMap<K, V>> replacement) {
    var next = new InterpolatingMeasureTreeMap<K, V>(
        m_keyToInternalMapper, m_internalToValueMapper, m_valueToInternalMapper);

    replacement.accept(next);

    m_map = next.m_map;
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
