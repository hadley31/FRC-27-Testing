package first.lib.tuning;

import java.util.Objects;

import org.littletonrobotics.junction.LogTable;

/**
 * How one value type reaches its NetworkTables topic and the log.
 *
 * <h2>Why a codec rather than abstract methods on the base</h2>
 *
 * <p>The obvious shape for this family is a handful of {@code protected abstract} hooks on
 * {@link LoggedTunable}, called from its constructor. That shape is broken, quietly and only for
 * some subclasses: {@link LoggedTunableMeasure} needs its unit to read a topic at all, and a
 * subclass field is assigned <em>after</em> {@code super(...)} returns, so the base constructor
 * would call into a subclass whose fields are still null.
 *
 * <p>A codec is built by the subclass's static factory, so it is fully constructed before
 * {@code super(...)} is even called. The base can bind and use it immediately, and the hazard is
 * unrepresentable rather than merely avoided.
 *
 * <p>Package-private on purpose. Getting the publish ordering wrong fails silently in a way that
 * only shows up after a reboot, which is the worst possible time to find out. That is not a
 * decision to expose.
 *
 * @param <T> the value type
 */
interface LoggedTunableCodec<T> {
  /**
   * Opens the topic, seeds it without disturbing a stored value, and flags it persistent.
   *
   * <p>Implementations must do those three things <em>in that order</em>, and must seed with
   * {@code setDefault} rather than {@code set}:
   *
   * <ul>
   *   <li>{@code setDefault} publishes only when the topic has no value, and the value it publishes
   *       carries timestamp zero. The server stamps values loaded from {@code networktables.json}
   *       with a real time and rejects a zero-stamped write against them, so a stored value always
   *       wins. A plain {@code set} -- even one that reads first and writes back what it found --
   *       races the server's value on its way to this client and can stamp the code default over a
   *       whole tuning session.
   *   <li>{@code setPersistent} must come after a value exists. The server ignores it on a topic
   *       that has never published one, and only logs a warning.
   * </ul>
   *
   * @param path the absolute NT path
   * @param codeDefault the default committed in code
   */
  void bind(String path, T codeDefault);

  /**
   * Reads the current value off the topic.
   *
   * @param codeDefault the value to fall back to when the topic holds nothing
   * @return the published value, or the code default
   */
  T read(T codeDefault);

  /**
   * Publishes a value, unconditionally.
   *
   * @param value the value to publish
   */
  void write(T value);

  /**
   * Records the value as an AdvantageKit input.
   *
   * @param table the log table
   * @param key the log key
   * @param value the value to record
   */
  void toLog(LogTable table, String key, T value);

  /**
   * Reads the value back during replay.
   *
   * @param table the log table
   * @param key the log key
   * @param codeDefault the value to fall back to when the log has no such key
   * @return the logged value, or the code default
   */
  T fromLog(LogTable table, String key, T codeDefault);

  /**
   * Whether two values count as the same, for change detection and the off-default alert.
   *
   * <p>Defaults to {@link Objects#equals}. {@link LoggedTunableMeasure} overrides it, because two
   * measures can be equal without being {@code equals} -- 90 degrees and a quarter rotation are the
   * same angle and must not read as a change.
   *
   * @param a one value, may be null
   * @param b the other value, may be null
   * @return true if the two are the same value
   */
  default boolean valuesEqual(T a, T b) {
    return Objects.equals(a, b);
  }
}
