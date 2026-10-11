package first.lib.util;

import java.util.function.DoubleFunction;
import java.util.function.ToDoubleFunction;

import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * One unit choice, paired with both directions of conversion between that unit and a plain double.
 *
 * <h2>Why this exists when {@link Unit} is right there</h2>
 *
 * <p>A concrete unit already converts both ways: {@code Degrees.of(90)} hands back an {@link Angle},
 * and {@code angle.in(Degrees)} hands back a double. But the typed {@code of} is a covariant override
 * on each unit class, while {@link Unit#of} itself is declared as returning {@code Measure<?>}. So the
 * moment the unit is behind a type variable -- which is exactly the situation of anything generic over
 * a measure, like {@link InterpolatingMeasureTreeMap} or {@code TuningState} -- a {@code Unit} alone
 * cannot produce a correctly typed measure. Carrying the two method references alongside the unit does,
 * because they are bound where the unit's real type is still known: at the constants below.
 *
 * <h2>Why both directions belong together</h2>
 *
 * <p>Code that stores measures as doubles has to convert out and back in, and those two conversions
 * must agree on a unit or the value silently changes meaning on a round trip. Passing them separately
 * -- as a {@code DoubleFunction} here and an {@code in(...)} call there -- makes a mismatch a thing you
 * have to notice in review; passing one of these makes it unrepresentable.
 *
 * <p>Pick the unit that suits the job: a stored table uses whatever is convenient internally, while a
 * dashboard-facing value uses what a person wants to read, which is why the catalog offers both
 * {@link #RADIANS} and {@link #DEGREES} rather than one blessed angle unit.
 *
 * @param <M> the measure this unit measures
 */
public final class UnitDoubleConverter<M extends Measure<?>> {
  public static final UnitDoubleConverter<Distance> METERS = of(Units.Meters);
  public static final UnitDoubleConverter<Distance> FEET = of(Units.Feet);
  public static final UnitDoubleConverter<Distance> INCHES = of(Units.Inches);

  public static final UnitDoubleConverter<Time> SECONDS = of(Units.Seconds);

  public static final UnitDoubleConverter<Angle> RADIANS = of(Units.Radians);
  public static final UnitDoubleConverter<Angle> DEGREES = of(Units.Degrees);

  public static final UnitDoubleConverter<AngularVelocity> ROTATIONS_PER_SECOND = of(Units.RotationsPerSecond);
  public static final UnitDoubleConverter<AngularVelocity> RPM = of(Units.RPM);

  private final Unit m_unit;
  private final DoubleFunction<M> m_fromDouble;
  private final ToDoubleFunction<M> m_toDouble;

  private UnitDoubleConverter(Unit unit, DoubleFunction<M> fromDouble, ToDoubleFunction<M> toDouble) {
    m_unit = unit;
    m_fromDouble = fromDouble;
    m_toDouble = toDouble;
  }

  /**
   * The type for a unit, for anything the catalog above does not already name.
   *
   * @param unit the unit to measure in
   * @return a type that converts between doubles and measures in that unit
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> UnitDoubleConverter<M> of(U unit) {
    // Unit.of is declared to return Measure<?> because the base class cannot name its own measure.
    // Every unit overrides it to return exactly the measure it belongs to -- AngleUnit returns Angle
    // -- which is what the M extends Measure<U> bound says, so the cast holds for every WPILib unit.
    return new UnitDoubleConverter<>(unit, value -> (M) unit.of(value), measure -> measure.in(unit));
  }

  /**
   * The measure this many of the unit makes.
   *
   * @param value a magnitude in this type's unit
   * @return that magnitude as a measure
   */
  public M of(double value) {
    return m_fromDouble.apply(value);
  }

  /**
   * A measure's magnitude in this type's unit, whatever unit the measure itself happens to carry.
   *
   * @param measure the measure to convert
   * @return its magnitude in this type's unit
   */
  public double toDouble(M measure) {
    return m_toDouble.applyAsDouble(measure);
  }

  /**
   * The unit itself, for names and symbols -- a log key or a dashboard label, say.
   *
   * @return the unit this type measures in
   */
  public Unit unit() {
    return m_unit;
  }

  @Override
  public String toString() {
    return "MeasureType[" + m_unit.name() + "]";
  }
}
