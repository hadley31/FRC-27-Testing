package first.lib.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A feature toggle: a boolean a dashboard can flip, that replays faithfully from a log, and that can
 * optionally survive a robot reboot.
 *
 * <p>This wraps {@link LoggedNetworkBoolean} rather than raw NetworkTables because AdvantageKit logs
 * it as an <em>input</em>. Replaying a log therefore takes the branch the robot actually took; a
 * plain NT value or a {@code org.wpilib.tunable} value would read as its initial value during replay
 * and silently run the other branch.
 *
 * <p>Declare toggles in one robot-wide registry, not at their point of use, so the full set — and
 * every competition default — is reviewable in a single file.
 *
 * <h2>Defaults</h2>
 *
 * <p>The default passed here is the <em>competition</em> behaviour: a robot with a wiped dashboard
 * must play a valid match. Anything else makes the dashboard a hidden dependency of being able to
 * compete.
 *
 * <h2>Persistence</h2>
 *
 * <p>{@link #persistent} marks the underlying NT topic persistent, so the robot's NT server saves it
 * to {@code /home/systemcore/networktables.json} and reloads it at boot. Use it for values you would
 * be annoyed to re-enter, <em>not</em> for behaviour switches: a persisted kill switch from a debug
 * session will quietly still be set at an event.
 */
public final class Toggle implements BooleanSupplier {
  private static final List<Toggle> s_registry = new ArrayList<>();

  private final String m_path;
  private final boolean m_competitionDefault;
  private final LoggedNetworkBoolean m_input;

  private Toggle(String path, boolean competitionDefault, boolean persistent) {
    if (!path.startsWith("/")) {
      // AdvantageKit uses this key verbatim as the NT topic name, so a missing leading slash yields
      // a malformed path that dashboards will not nest and that setPersistent cannot be matched to.
      throw new IllegalArgumentException("Toggle path must be absolute, got: " + path);
    }

    m_path = path;
    m_competitionDefault = competitionDefault;
    // Construct before flagging persistence: this publishes the default value, and the NT server
    // ignores setPersistent on a topic that has never published one.
    m_input = new LoggedNetworkBoolean(path, competitionDefault);

    if (persistent) {
      NetworkTableInstance.getDefault().getBooleanTopic(path).setPersistent(true);
    }

    s_registry.add(this);
  }

  /**
   * A toggle that resets to its default every boot.
   *
   * @param path absolute NT path, e.g. {@code "/Tuning/Shot/VelocityCompensation"}
   * @param competitionDefault the value a robot with no dashboard should use
   */
  public static Toggle of(String path, boolean competitionDefault) {
    return new Toggle(path, competitionDefault, false);
  }

  /**
   * A toggle whose value is saved on the robot and restored at boot. See the class notes before
   * reaching for this: most behaviour switches should <em>not</em> persist.
   *
   * @param path absolute NT path
   * @param competitionDefault the value used when nothing has been persisted yet
   */
  public static Toggle persistent(String path, boolean competitionDefault) {
    return new Toggle(path, competitionDefault, true);
  }

  @Override
  public boolean getAsBoolean() {
    return m_input.get();
  }

  /**
   * Publishes a new value.
   *
   * <p>AdvantageKit refreshes inputs once per loop before user code, so {@link #getAsBoolean} keeps
   * returning the old value until the next cycle.
   */
  public void set(boolean value) {
    m_input.set(value);
  }

  public String path() {
    return m_path;
  }

  public boolean competitionDefault() {
    return m_competitionDefault;
  }

  public boolean isDefault() {
    return getAsBoolean() == m_competitionDefault;
  }

  /**
   * Logs every toggle currently away from its default.
   *
   * <p>AdvantageKit already logs each toggle's value, but that only helps if you know which key to
   * look at. This answers "is anything unusual switched on?" in one glance from the driver station,
   * which is where stale debug toggles get caught before a match rather than after one.
   */
  public static void logNonDefaults() {
    List<String> changed = new ArrayList<>();

    for (Toggle toggle : s_registry) {
      if (!toggle.isDefault()) {
        changed.add(toggle.m_path + " = " + toggle.getAsBoolean());
      }
    }

    Logger.recordOutput("Tuning/NonDefaultToggles", changed.toArray(String[]::new));
    Logger.recordOutput("Tuning/NonDefaultToggleCount", changed.size());
  }
}
