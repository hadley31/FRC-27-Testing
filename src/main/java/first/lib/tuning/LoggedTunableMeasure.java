package first.lib.tuning;

import org.littletonrobotics.junction.LogTable;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

import first.lib.util.UnitDoubleConverter;

/**
 * A tuned physical quantity -- an angle, a distance, a velocity -- that survives a reboot. See
 * {@link LoggedTunable} for what that buys and what it costs.
 *
 * <p>The unit comes from the default value, so a declaration names it once and never again:
 *
 * <pre>
 * public static final LoggedTunableMeasure&lt;Angle&gt; kAimOffset =
 *     LoggedTunableMeasure.of("/Tuning/Turret/AimOffset", Degrees.of(2.5));
 * </pre>
 *
 * <p>That value is stored, read and displayed in degrees, and {@link #get()} hands back an
 * {@link org.wpilib.units.measure.Angle}. A caller that sets it in some other unit of the same
 * dimension -- {@code Rotations.of(0.25)} -- writes the value it means rather than the number
 * {@code 0.25}, because the conversion goes through {@link UnitDoubleConverter}, which cannot
 * disagree with itself about which unit it is using.
 *
 * <h2>The unit is part of the path</h2>
 *
 * <p>The declared unit's name is appended, so the example above lives at
 * {@code /Tuning/Turret/AimOffset (Degree)}. Someone editing a bare {@code 2.5} on a dashboard
 * otherwise has no way to tell degrees from radians, and a tuned value that is wrong by a factor of
 * 57 is the kind of mistake that costs a practice session.
 *
 * <p>The side effect is that changing a declaration's unit orphans whatever was stored and falls
 * back to the new code default. That is the safe outcome: a unit change must never silently
 * reinterpret a number a human typed.
 *
 * @param <M> the measure this value holds
 */
public final class LoggedTunableMeasure<M extends Measure<?>> extends LoggedTunable<M> {
  private final UnitDoubleConverter<M> m_converter;

  /**
   * A tuned quantity, stored and displayed in its default value's own unit.
   *
   * <p>The two type parameters are what make the unit derivable. {@code Measure<U>.unit()} returns
   * {@code U}, and {@link UnitDoubleConverter#of(Unit)} needs exactly that to hand back a converter
   * that produces a correctly typed measure -- see its javadoc for why a bare {@link Unit} cannot.
   * The class itself is parameterised on {@code M} alone so a declaration reads
   * {@code LoggedTunableMeasure<Angle>} rather than {@code LoggedTunableMeasure<AngleUnit, Angle>},
   * which matters because {@code var} is unavailable on a field.
   *
   * @param path absolute NT path, e.g. {@code "/Tuning/Turret/AimOffset"}; the unit name is appended
   * @param codeDefault the value committed in code, whose unit this value is published in
   */
  public static <U extends Unit, M extends Measure<U>> LoggedTunableMeasure<M> of(
      String path, M codeDefault) {
    // The explicit type witness is not decoration. UnitDoubleConverter.of infers M from its target
    // type, and nesting it in a diamond makes the two inferences depend on each other.
    return new LoggedTunableMeasure<>(
        path, codeDefault, UnitDoubleConverter.<U, M>of(codeDefault.unit()), true);
  }

  /**
   * A tuned quantity published in a unit other than its default value's.
   *
   * <p>For a default expressed in whatever was convenient in code -- {@code Rotations.of(0.25)} --
   * that a person should read and edit in something else. Also the factory to use when the caller
   * already holds a converter rather than a measure, which is the situation {@code TuningState} is
   * in.
   *
   * @param path absolute NT path; the unit name is appended
   * @param converter the unit to store and display in
   * @param codeDefault the value committed in code, in any unit of the same dimension
   */
  public static <M extends Measure<?>> LoggedTunableMeasure<M> of(
      String path, UnitDoubleConverter<M> converter, M codeDefault) {
    return new LoggedTunableMeasure<>(path, codeDefault, converter, true);
  }

  /**
   * A tuned quantity belonging to a larger tunable, whose owner calls {@code periodic()}.
   *
   * @param path absolute NT path; the unit name is appended
   * @param converter the unit to store and display in
   * @param codeDefault the value committed in code
   */
  public static <M extends Measure<?>> LoggedTunableMeasure<M> owned(
      String path, UnitDoubleConverter<M> converter, M codeDefault) {
    return new LoggedTunableMeasure<>(path, codeDefault, converter, false);
  }

  private LoggedTunableMeasure(
      String path, M codeDefault, UnitDoubleConverter<M> converter, boolean register) {
    // The codec is complete before super() runs, which is the whole reason LoggedTunableCodec
    // exists: a base-class constructor calling back into a subclass would find m_converter null.
    super(pathWithUnit(path, converter), codeDefault, new Codec<>(converter), register);
    m_converter = converter;
  }

  /** The unit this value is stored and displayed in. */
  public UnitDoubleConverter<M> converter() {
    return m_converter;
  }

  private static String pathWithUnit(String path, UnitDoubleConverter<?> converter) {
    // Hand a malformed path straight through so the base class rejects it quoting what the caller
    // actually wrote, rather than one with a unit suffix they never typed.
    if (!path.startsWith("/") || path.endsWith("/")) {
      return path;
    }

    return path + " (" + converter.unit().name() + ")";
  }

  /**
   * Stores the magnitude in the declared unit.
   *
   * <p>Note this does <em>not</em> use {@code LogTable.putMeasure}, despite it existing.
   * {@code putMeasure} is hardcoded to base units, so a value tuned in degrees would be logged in
   * radians -- the wrong log for a number whose entire reason to exist is that a person reads it in
   * degrees. {@code put(key, double, unit)} writes the same log value with a unit string of our
   * choosing. It also keeps the generics to one type parameter: {@code getMeasure} is declared
   * {@code <U extends Unit, M extends Measure<U>>} and cannot infer {@code U} from this class's
   * {@code M extends Measure<?>}.
   */
  private static final class Codec<M extends Measure<?>> implements LoggedTunableCodec<M> {
    private final UnitDoubleConverter<M> m_converter;
    private DoubleEntry m_entry;

    Codec(UnitDoubleConverter<M> converter) {
      m_converter = converter;
    }

    @Override
    public void bind(String path, M codeDefault) {
      var topic = NetworkTableInstance.getDefault().getDoubleTopic(path);
      m_entry = topic.getEntry(m_converter.toDouble(codeDefault));
      m_entry.setDefault(m_converter.toDouble(codeDefault));
      topic.setPersistent(true);
    }

    @Override
    public M read(M codeDefault) {
      return m_converter.of(m_entry.get(m_converter.toDouble(codeDefault)));
    }

    @Override
    public void write(M value) {
      m_entry.set(m_converter.toDouble(value));
    }

    @Override
    public void toLog(LogTable table, String key, M value) {
      table.put(key, m_converter.toDouble(value), m_converter.unit().name());
    }

    @Override
    public M fromLog(LogTable table, String key, M codeDefault) {
      return m_converter.of(table.get(key, m_converter.toDouble(codeDefault)));
    }

    @Override
    public boolean valuesEqual(M a, M b) {
      // Not equals: 90 degrees and a quarter rotation are the same angle, and a setter handed one
      // against a default of the other must not read as a change forever.
      return a.isEquivalent(b);
    }
  }
}
