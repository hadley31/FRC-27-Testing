package first.lib.tuning;

import java.util.Objects;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/**
 * A tuned value that survives a reboot, replays faithfully from a log, and says so when it no longer
 * matches the code.
 *
 * <p>Three properties, each of which something in this package already had and none of which
 * anything had together:
 *
 * <ul>
 *   <li><b>Persistent.</b> The topic is flagged persistent, so the robot's NT server saves it to
 *       {@code /home/systemcore/networktables.json} and reloads it at boot. A gain found at 4pm is
 *       still there after the next code deploy. {@code org.wpilib.tunable} values are not
 *       persistent, which is what made tuning with them a matter of scribbling numbers on paper.
 *   <li><b>Cached.</b> {@link #get()} is a field read. The value is refreshed once per loop in
 *       {@link #periodic()}, before user code, so everything that reads it within one cycle sees
 *       the same number -- and reading it in a hot loop costs nothing.
 *   <li><b>Logged as an input.</b> This extends {@link LoggedNetworkInput}, so AdvantageKit records
 *       the value as an <em>input</em> and a replay runs the number the robot really ran rather than
 *       whatever the code default happens to be today. {@link Toggle} spells out that argument; it
 *       matters more here, because a persisted value is by definition one that may differ from the
 *       source you are reading.
 * </ul>
 *
 * <h2>Defaults, and the alert</h2>
 *
 * <p>The default passed at the declaration site is the value committed in code. A value already
 * stored on the robot <em>wins</em> over it -- that is the point -- which means the running robot
 * can silently disagree with the source. {@link LoggedTunables#refresh()} raises one alert naming
 * every value in that state, so "this robot is still carrying Thursday evening's experiment" is
 * visible from the driver station rather than discovered mid-match.
 *
 * <p>Tuned values belong back in source. Treat the alert as a to-do list, not a status line.
 *
 * <h2>Paths are absolute</h2>
 *
 * <p>Same rule as {@link Toggle} and {@link TunableDoubleArray}: the path names the NT topic
 * outright, and a relative one is rejected. Keep them under {@link LoggedTunables#kRoot} so
 * everything tunable on the robot sits in one subtree.
 *
 * <p>This family talks to NetworkTables directly rather than through
 * {@code org.wpilib.preferences.Preferences}, which is itself only a wrapper over a fixed
 * {@code Preferences} table. Going direct keeps every tunable in one namespace instead of two,
 * drops a dependency on a static table binding that nothing else here needs, uses typed entries,
 * and persists exactly what is asked for -- the {@code Preferences} table force-persists every
 * topic that so much as appears in it.
 *
 * <h2>Do not sample a tuned value once</h2>
 *
 * <p>Read it every time you need it, or react to {@link #changedThisLoop()}. Anything that latches
 * a value -- building a {@code TalonFXConfiguration} in a constructor, computing a derived constant
 * -- defeats both the caching and the persistence, and will also catch the one case where
 * construction runs before the server's stored value has reached this client.
 *
 * <h2>Declaration order matters</h2>
 *
 * <p>Declare these in {@code first.robot.util.Tuning}, and make sure that class is first touched
 * from the {@code Robot} constructor or later. By then {@code RobotBase} has started the NT server
 * and the server has loaded {@code networktables.json}. Touch it from a class initializer reachable
 * earlier -- {@code Constants}, a static field on a mechanism -- and there is no server to persist
 * to.
 *
 * @param <T> the value type
 */
public abstract sealed class LoggedTunable<T> extends LoggedNetworkInput
    implements Supplier<T>, LoggableInputs
    permits LoggedTunableBoolean, LoggedTunableDouble, LoggedTunableEnum, LoggedTunableMeasure {
  private final String m_path;
  private final String m_logKey;
  private final T m_codeDefault;
  private final LoggedTunableCodec<T> m_codec;

  private T m_value;
  private boolean m_changedThisLoop;

  /**
   * @param path the absolute NT path
   * @param codeDefault the value committed in code
   * @param codec how this value type reaches NetworkTables and the log; built by the subclass's
   *     factory, so it is complete before this constructor runs
   * @param register whether AdvantageKit drives {@link #periodic()}, or an owner does
   */
  LoggedTunable(String path, T codeDefault, LoggedTunableCodec<T> codec, boolean register) {
    if (!path.startsWith("/") || path.endsWith("/")) {
      throw new IllegalArgumentException(
          "A tunable path must be absolute and name a topic, got: '" + path + "'");
    }

    m_path = path;
    // Mirrors the NT tree, which is what AdvantageKit's own dashboard inputs do: every
    // LoggedNetworkInput writes into one shared "NetworkInputs" subtable, so a leading slash there
    // would produce an empty-named level.
    m_logKey = removeSlash(path);
    m_codeDefault = Objects.requireNonNull(codeDefault, "codeDefault");
    m_codec = codec;

    codec.bind(path, codeDefault);
    // In replay the first periodic() replaces this from the log; seeding from the topic anyway
    // keeps get() meaningful for anything that reads before then.
    m_value = codec.read(codeDefault);

    // Before registering with AdvantageKit, so a duplicate path throws before AK holds a reference
    // to an object that is about to be discarded.
    LoggedTunables.register(this);

    if (register) {
      Logger.registerDashboardInput(this);
    }
  }

  /** Returns the current value. A field read -- never a NetworkTables lookup. */
  @Override
  public final T get() {
    return m_value;
  }

  /**
   * Publishes a new value, which the robot will also keep across a reboot.
   *
   * <p>Inputs refresh once per loop before user code, so {@link #get()} keeps returning the old
   * value until the next cycle.
   *
   * @param value the value to publish
   */
  public final void set(T value) {
    m_codec.write(value);
  }

  /** Publishes the value committed in code, clearing this value's contribution to the alert. */
  public final void resetToDefault() {
    set(m_codeDefault);
  }

  /** The absolute NT path this value is published at. */
  public final String path() {
    return m_path;
  }

  /** The value committed in code, which a stored value overrides. */
  public final T codeDefault() {
    return m_codeDefault;
  }

  /** Whether the current value still matches the one committed in code. */
  public final boolean isDefault() {
    return m_codec.valuesEqual(m_value, m_codeDefault);
  }

  /**
   * Whether the value changed during this loop's refresh.
   *
   * <p>True for the whole loop in which the change landed, and recomputed on the next
   * {@link #periodic()}. Deliberately <em>not</em> a latch that the reader clears, which is the
   * subtler half of why {@code org.wpilib.tunable}'s equivalent is unusable: a flag cleared by
   * whoever reads it first silently steals the change from everyone else. This one is idempotent,
   * so any number of consumers get the same answer and ownership never becomes a question.
   *
   * <p>A consumer that starts after a change misses it, so apply the value once on entry as well --
   * see {@code G3Mechanism.tuningCommand}. There is deliberately no {@code onChange} callback: it
   * would fire from inside AdvantageKit's input pass, before user code and regardless of what is
   * scheduled, which for the one real consumer means a blocking retrying CAN config write inside
   * the window AdvantageKit measures as {@code Logger/DashboardInputsMS}.
   */
  public final boolean changedThisLoop() {
    return m_changedThisLoop;
  }

  /**
   * Reads NetworkTables, or the log during replay.
   *
   * <p>Called once per cycle before user code: by AdvantageKit for a value from an {@code of}
   * factory, or by the owner for one from an {@code owned} factory.
   */
  @Override
  public final void periodic() {
    T previous = m_value;

    if (!Logger.hasReplaySource()) {
      m_value = m_codec.read(m_codeDefault);
    }

    Logger.processInputs(prefix, this);

    // After processInputs, which is what assigns m_value during replay. Comparing here rather than
    // before it is what makes change detection replay-faithful: a replayed log sees the edit land
    // on the same cycle it landed on the robot.
    m_changedThisLoop = !m_codec.valuesEqual(previous, m_value);
  }

  /**
   * Records the value for AdvantageKit. Called by {@link #periodic()}; not by user code.
   */
  @Override
  public final void toLog(LogTable table) {
    m_codec.toLog(table, m_logKey, m_value);
  }

  /**
   * Reads the value back during replay, replacing whatever is cached. Called by
   * {@link #periodic()}; not by user code.
   */
  @Override
  public final void fromLog(LogTable table) {
    m_value = m_codec.fromLog(table, m_logKey, m_codeDefault);
  }

  @Override
  public final String toString() {
    return getClass().getSimpleName() + "[" + m_path + " = " + m_value
        + (isDefault() ? "]" : ", code: " + m_codeDefault + "]");
  }
}
