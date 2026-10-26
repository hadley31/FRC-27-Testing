package first.lib.tuning;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.LogTable;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A tunable flag: one a dashboard can flip, that replays faithfully from a log, and that can
 * optionally survive a robot reboot. See {@link LoggedTunable} for the whole contract.
 *
 * <p>This covers both halves of what used to be two classes. {@link #of} is a feature toggle: it
 * resets to its competition default every boot, which is what a behaviour switch wants.
 * {@link #persistent} saves the value, which is what a tuned configuration wants. The hazard of
 * confusing them is spelled out on {@link LoggedTunable}.
 */
public final class LoggedTunableBoolean extends LoggedTunable<Boolean> implements BooleanSupplier {
  /**
   * A feature toggle: a flag that resets to its competition default every boot.
   *
   * <p>The default is the <em>competition</em> behaviour, because a robot with a wiped dashboard
   * must still play a valid match. Anything else makes the dashboard a hidden dependency of being
   * able to compete.
   *
   * @param path absolute NT path, e.g. {@code "/Tuning/Shot/VelocityCompensation"}
   * @param codeDefault the value a robot with no dashboard should use
   */
  public static LoggedTunableBoolean of(String path, boolean codeDefault) {
    return new LoggedTunableBoolean(path, codeDefault, false, true);
  }

  /**
   * A flag the robot saves and restores at boot.
   *
   * <p>Read {@link LoggedTunable}'s note before reaching for this: most behaviour switches should
   * <em>not</em> persist. This is for a flag that is part of a tuned configuration.
   *
   * @param path absolute NT path
   * @param codeDefault the value used until something is stored on the robot
   */
  public static LoggedTunableBoolean persistent(String path, boolean codeDefault) {
    return new LoggedTunableBoolean(path, codeDefault, true, true);
  }

  /**
   * A flag belonging to a larger tunable, whose owner calls {@code periodic()}.
   *
   * @param path absolute NT path
   * @param codeDefault the value used until something is stored on the robot
   * @param persistent whether the robot saves the value across a reboot
   */
  public static LoggedTunableBoolean owned(String path, boolean codeDefault, boolean persistent) {
    return new LoggedTunableBoolean(path, codeDefault, persistent, false);
  }

  private LoggedTunableBoolean(
      String path, boolean codeDefault, boolean persistent, boolean register) {
    super(path, codeDefault, new Codec(), persistent, register);
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }

  private static final class Codec implements LoggedTunableCodec<Boolean> {
    private BooleanEntry m_entry;

    @Override
    public void bind(String path, Boolean codeDefault, boolean persistent) {
      var topic = NetworkTableInstance.getDefault().getBooleanTopic(path);
      m_entry = topic.getEntry(codeDefault);
      m_entry.setDefault(codeDefault);

      if (persistent) {
        topic.setPersistent(true);
      }
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
