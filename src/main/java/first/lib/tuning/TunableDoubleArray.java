package first.lib.tuning;

import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.wpilib.networktables.DoubleArrayEntry;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A variable-length array of numbers a dashboard can edit, that replays faithfully from a log, and
 * that can optionally survive a robot reboot.
 *
 * <p>AdvantageKit ships {@code LoggedNetworkNumber}, {@code LoggedNetworkBoolean} and {@code
 * LoggedNetworkString} but no array type, so this fills the gap the same way they do: the value is
 * logged as an <em>input</em>, which is what makes a log replay against the table the robot actually
 * used rather than against whatever the code's default happened to be. {@link Toggle} spells out that
 * argument, and it matters more here — a shot table read as its default during replay produces
 * plausible-looking but wrong solutions rather than an obvious failure.
 *
 * <h2>Length is part of the value</h2>
 *
 * <p>A dashboard may publish a longer or shorter array at any time, so nothing about length is fixed
 * at construction. Consequently, several arrays that are <em>meant</em> to line up must have their
 * lengths checked together on every cycle: NetworkTables makes no promise that separate topics update
 * in the same network packet, so a UI adding a row to four arrays at once can be observed part-way
 * through. See {@code ShotProfileTable.validate} for one way to handle that.
 */
public final class TunableDoubleArray extends LoggedNetworkInput implements Supplier<double[]> {
  private final String m_path;
  private final double[] m_competitionDefault;
  private final DoubleArrayEntry m_entry;
  private double[] m_value;

  private final LoggableInputs m_inputs = new LoggableInputs() {
    @Override
    public void toLog(LogTable table) {
      table.put(removeSlash(m_path), m_value);
    }

    @Override
    public void fromLog(LogTable table) {
      m_value = table.get(removeSlash(m_path), m_competitionDefault);
    }
  };

  private TunableDoubleArray(
      String path, double[] competitionDefault, boolean persistent, boolean register) {
    if (!path.startsWith("/")) {
      // AdvantageKit uses this key verbatim as the NT topic name, so a missing leading slash yields
      // a malformed path that dashboards will not nest and that setPersistent cannot be matched to.
      throw new IllegalArgumentException("Tunable path must be absolute, got: " + path);
    }

    m_path = path;
    m_competitionDefault = competitionDefault.clone();
    m_value = m_competitionDefault.clone();
    m_entry = NetworkTableInstance.getDefault().getDoubleArrayTopic(path)
        .getEntry(m_competitionDefault);

    // Publish before flagging persistence: the NT server ignores setPersistent on a topic that has
    // never published a value. Re-publishing whatever is already there keeps a value restored from
    // networktables.json rather than stamping the default over it.
    m_entry.set(m_entry.get(m_competitionDefault));

    if (persistent) {
      NetworkTableInstance.getDefault().getDoubleArrayTopic(path).setPersistent(true);
    }

    if (register) {
      Logger.registerDashboardInput(this);
    }
  }

  /**
   * An array that resets to its default every boot.
   *
   * @param path absolute NT path, e.g. {@code "/Tuning/ShotProfile/Scoring/DistancesFeet"}
   * @param competitionDefault the value a robot with no dashboard should use
   */
  public static TunableDoubleArray of(String path, double[] competitionDefault) {
    return new TunableDoubleArray(path, competitionDefault, false, true);
  }

  /**
   * An array whose value is saved on the robot and restored at boot.
   *
   * <p>Unlike a behaviour switch, measured data is exactly what {@link Toggle}'s notes say persistence
   * is for: values you would be annoyed to re-enter. The hazard is different rather than absent
   * though — a persisted table is invisible until someone looks at it, so whatever owns the array is
   * responsible for publishing whether it still matches the code default.
   *
   * @param path absolute NT path
   * @param competitionDefault the value used when nothing has been persisted yet
   */
  public static TunableDoubleArray persistent(String path, double[] competitionDefault) {
    return new TunableDoubleArray(path, competitionDefault, true, true);
  }

  /**
   * An array that belongs to a larger tunable, whose owner calls {@link #periodic} instead of
   * AdvantageKit.
   *
   * <p>For arrays that only make sense together — the columns of a table, say — where the owner has to
   * see a consistent set. Registering each column separately would work, but it leaves the owner
   * depending on having been registered after all of them, which is invisible in the code and
   * impossible to reproduce in a test. Owning the refresh makes that explicit instead.
   *
   * @param path absolute NT path
   * @param competitionDefault the value used when nothing has been published
   * @param persistent whether the robot saves the value across a reboot; see {@link #persistent}
   */
  public static TunableDoubleArray owned(
      String path, double[] competitionDefault, boolean persistent) {
    return new TunableDoubleArray(path, competitionDefault, persistent, false);
  }

  /** Returns the current value. The array is a copy; mutating it changes nothing. */
  @Override
  public double[] get() {
    return m_value.clone();
  }

  /**
   * Publishes a new value, of any length.
   *
   * <p>AdvantageKit refreshes inputs once per loop before user code, so {@link #get} keeps returning
   * the old value until the next cycle.
   */
  public void set(double[] value) {
    m_entry.set(value.clone());
  }

  public String path() {
    return m_path;
  }

  public double[] competitionDefault() {
    return m_competitionDefault.clone();
  }

  public boolean isDefault() {
    return Arrays.equals(m_value, m_competitionDefault);
  }

  /**
   * Reads the dashboard, or the log during replay.
   *
   * <p>Called once per cycle before user code: by AdvantageKit for an array from {@link #of} or
   * {@link #persistent}, or by the owner for one from {@link #owned}.
   */
  @Override
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      m_value = m_entry.get(m_competitionDefault);
    }

    Logger.processInputs(prefix, m_inputs);
  }
}
