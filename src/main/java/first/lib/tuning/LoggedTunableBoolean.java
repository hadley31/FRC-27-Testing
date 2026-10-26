package first.lib.tuning;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.LogTable;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A tuned flag that survives a reboot. See {@link LoggedTunable} for what that buys and what it
 * costs.
 *
 * <h2>This is not {@link Toggle}, and the difference is the point</h2>
 *
 * <p>A {@code Toggle} is a <em>behaviour</em> switch and resets to its competition default every
 * boot, deliberately: its javadoc spells out that a persisted kill switch from a Thursday debug
 * session will quietly still be set at Saturday's event. This class persists, so it is for a flag
 * that is part of a <em>tuned configuration</em> -- something you would be annoyed to re-enter and
 * that you intend to commit back to source.
 *
 * <p>If you are reaching for this to turn a feature on and off, you want {@code Toggle}.
 */
public final class LoggedTunableBoolean extends LoggedTunable<Boolean> implements BooleanSupplier {
  /**
   * A tuned flag that AdvantageKit refreshes once per loop.
   *
   * @param path absolute NT path
   * @param codeDefault the value committed in code, used until something is stored on the robot
   */
  public static LoggedTunableBoolean of(String path, boolean codeDefault) {
    return new LoggedTunableBoolean(path, codeDefault, true);
  }

  /**
   * A tuned flag belonging to a larger tunable, whose owner calls {@code periodic()}.
   *
   * @param path absolute NT path
   * @param codeDefault the value committed in code
   */
  public static LoggedTunableBoolean owned(String path, boolean codeDefault) {
    return new LoggedTunableBoolean(path, codeDefault, false);
  }

  private LoggedTunableBoolean(String path, boolean codeDefault, boolean register) {
    super(path, codeDefault, new Codec(), register);
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }

  private static final class Codec implements LoggedTunableCodec<Boolean> {
    private BooleanEntry m_entry;

    @Override
    public void bind(String path, Boolean codeDefault) {
      var topic = NetworkTableInstance.getDefault().getBooleanTopic(path);
      m_entry = topic.getEntry(codeDefault);
      m_entry.setDefault(codeDefault);
      topic.setPersistent(true);
    }

    @Override
    public Boolean read(Boolean codeDefault) {
      return m_entry.get(codeDefault);
    }

    @Override
    public void write(Boolean value) {
      m_entry.set(value);
    }

    @Override
    public void toLog(LogTable table, String key, Boolean value) {
      table.put(key, (boolean) value);
    }

    @Override
    public Boolean fromLog(LogTable table, String key, Boolean codeDefault) {
      return table.get(key, (boolean) codeDefault);
    }
  }
}
