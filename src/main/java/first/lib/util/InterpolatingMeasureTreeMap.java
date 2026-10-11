package first.lib.util;

import java.util.function.Consumer;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * An {@link InterpolatingDoubleTreeMap} with {@link Measure} keys and values, so a lookup table can
 * be written in whatever units it was measured in and read in whatever units the caller wants.
 *
 * <p>Points are stored as plain doubles in the units the map was built with, so a conversion happens
 * at {@link #put} and {@link #get} rather than on every interpolation step. Those units are carried as
 * {@link UnitDoubleConverter}s, which is what keeps the conversion in and the conversion back out from ever
 * disagreeing about a unit.
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
  private InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();
  private final UnitDoubleConverter<K> m_keyType;
  private final UnitDoubleConverter<V> m_valueType;

  /**
   * A map that stores its points in the given units.
   *
   * @param keyType the unit keys are stored in
   * @param valueType the unit values are stored in
   */
  public InterpolatingMeasureTreeMap(UnitDoubleConverter<K> keyType, UnitDoubleConverter<V> valueType) {
    m_keyType = keyType;
    m_valueType = valueType;
  }

  public V get(K key) {
    Double interpolated = m_map.get(m_keyType.toDouble(key));

    if (interpolated == null) {
      // InterpolatingTreeMap returns null only when it holds no points at all. Left alone that
      // unboxes into an NPE thrown from inside a unit conversion, several frames away from the
      // actual mistake, which is a table that was never filled in.
      throw new IllegalStateException("Cannot interpolate a table with no points");
    }

    return m_valueType.of(interpolated);
  }

  public InterpolatingMeasureTreeMap<K, V> put(K key, V value) {
    m_map.put(m_keyType.toDouble(key), m_valueType.toDouble(value));
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
    var next = new InterpolatingMeasureTreeMap<K, V>(m_keyType, m_valueType);

    replacement.accept(next);

    m_map = next.m_map;
  }

  public static InterpolatingMeasureTreeMap<Distance, Angle> distanceToAngle() {
    return new InterpolatingMeasureTreeMap<>(UnitDoubleConverter.METERS, UnitDoubleConverter.RADIANS);
  }

  public static InterpolatingMeasureTreeMap<Distance, AngularVelocity> distanceToAngularVelocity() {
    return new InterpolatingMeasureTreeMap<>(
        UnitDoubleConverter.METERS, UnitDoubleConverter.ROTATIONS_PER_SECOND);
  }

  public static InterpolatingMeasureTreeMap<Distance, Time> distanceToTime() {
    return new InterpolatingMeasureTreeMap<>(UnitDoubleConverter.METERS, UnitDoubleConverter.SECONDS);
  }

  public static InterpolatingMeasureTreeMap<Time, Distance> timeToDistance() {
    return new InterpolatingMeasureTreeMap<>(UnitDoubleConverter.SECONDS, UnitDoubleConverter.METERS);
  }
}
