package first.lib.tuning;

import org.littletonrobotics.junction.LogTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringEntry;
import org.wpilib.util.Alert;

/**
 * A tuned choice from a fixed set, that survives a reboot. See {@link LoggedTunable} for what that
 * buys and what it costs.
 *
 * <p>Stored as the constant's {@code name()}, so {@code networktables.json} stays readable and a
 * reordered enum cannot silently come back as the wrong constant -- it comes back as no constant at
 * all, which is handled explicitly below. The enum's class comes from the default value, so a
 * declaration names the type once:
 *
 * <pre>
 * public static final LoggedTunableEnum&lt;ShotMode&gt; kShotMode =
 *     LoggedTunableEnum.of("/Tuning/Shot/Mode", ShotMode.AUTO);
 * </pre>
 *
 * <h2>An unrecognised stored name gets its own alert</h2>
 *
 * <p>A stored name that no longer parses -- a constant deleted or renamed since it was stored, or a
 * typo from a dashboard -- falls back to the code default and raises a <em>separate</em>
 * medium-priority alert, rather than joining the aggregated off-default list in
 * {@link LoggedTunables}.
 *
 * <p>That split is deliberate. An off-default value is a normal, expected state during tuning. This
 * is a misconfiguration: the dashboard says one thing and the robot is doing another, and someone
 * editing that field is getting no feedback that their edit did nothing.
 *
 * @param <E> the enum this value chooses from
 */
public final class LoggedTunableEnum<E extends Enum<E>> extends LoggedTunable<E> {
  /**
   * A tuned choice that AdvantageKit refreshes once per loop.
   *
   * @param path absolute NT path, e.g. {@code "/Tuning/Shot/Mode"}
   * @param codeDefault the constant committed in code, which also supplies the enum type
   */
  public static <E extends Enum<E>> LoggedTunableEnum<E> of(String path, E codeDefault) {
    return new LoggedTunableEnum<>(path, codeDefault, true);
  }

  /**
   * A tuned choice belonging to a larger tunable, whose owner calls {@code periodic()}.
   *
   * @param path absolute NT path
   * @param codeDefault the constant committed in code, which also supplies the enum type
   */
  public static <E extends Enum<E>> LoggedTunableEnum<E> owned(String path, E codeDefault) {
    return new LoggedTunableEnum<>(path, codeDefault, false);
  }

  private LoggedTunableEnum(String path, E codeDefault, boolean register) {
    super(path, codeDefault, new Codec<>(codeDefault.getDeclaringClass()), register);
  }

  /**
   * Translates between a constant and its stored name, and complains when that fails.
   *
   * <p>Holds per-instance state -- the enum class, the entry and a lazily created alert -- which is
   * the other reason {@link LoggedTunableCodec} is an object rather than a set of abstract methods.
   */
  private static final class Codec<E extends Enum<E>> implements LoggedTunableCodec<E> {
    private final Class<E> m_type;

    private StringEntry m_entry;
    private String m_path;

    /**
     * Created on the first bad parse rather than at bind time, so declaring an enum value costs no
     * alert handle in the overwhelmingly common case where nothing is ever wrong with it.
     */
    private Alert m_parseAlert;

    Codec(Class<E> type) {
      m_type = type;
    }

    @Override
    public void bind(String path, E codeDefault) {
      m_path = path;
      var topic = NetworkTableInstance.getDefault().getStringTopic(path);
      m_entry = topic.getEntry(codeDefault.name());
      m_entry.setDefault(codeDefault.name());
      topic.setPersistent(true);
    }

    @Override
    public E read(E codeDefault) {
      return parse(m_entry.get(codeDefault.name()), codeDefault);
    }

    @Override
    public void write(E value) {
      m_entry.set(value.name());
    }

    @Override
    public void toLog(LogTable table, String key, E value) {
      table.put(key, value.name());
    }

    @Override
    public E fromLog(LogTable table, String key, E codeDefault) {
      return parse(table.get(key, codeDefault.name()), codeDefault);
    }

    private E parse(String name, E codeDefault) {
      try {
        E parsed = Enum.valueOf(m_type, name);
        setParseAlert(false, name, codeDefault);
        return parsed;
      } catch (IllegalArgumentException e) {
        setParseAlert(true, name, codeDefault);
        return codeDefault;
      }
    }

    private void setParseAlert(boolean active, String name, E codeDefault) {
      if (m_parseAlert == null) {
        if (!active) {
          return;
        }

        m_parseAlert = new Alert("tunableEnum" + m_path, "", Alert.Level.MEDIUM);
      }

      if (active) {
        m_parseAlert.setText(
            "Tunable '" + m_path + "' holds '" + name + "', which is not a "
                + m_type.getSimpleName() + ". Using " + codeDefault.name() + " instead.");
      }

      m_parseAlert.set(active);
    }
  }
}
