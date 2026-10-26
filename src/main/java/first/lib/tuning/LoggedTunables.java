package first.lib.tuning;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.util.Alert;

/**
 * Every {@link LoggedTunable} that has been declared, and the one alert that says when the robot is
 * no longer running what the code says.
 *
 * <h2>Why one alert and not one per value</h2>
 *
 * <p>Per-value alerts flood the one list that has to stay readable. {@link Alert} orders by level
 * then recency, so a dozen tuning alerts will not outrank a brownout -- but they will push every
 * other medium-priority alert off the visible part of the driver station widget, and a list you
 * scroll is a list you stop reading. The failure this exists to prevent is a robot arriving at a
 * match still carrying the previous evening's experiment, and catching that needs one unmissable
 * line, not a dozen ignorable ones.
 *
 * <p>The alert text names the first few offenders, which recovers most of the diagnostic value. The
 * full list goes to {@code Tuning/NonDefaultTunables}, which is where you actually look, because by
 * then you are already in AdvantageScope.
 *
 * <h2>Only persistent values alert</h2>
 *
 * <p>Everything off-default is logged, but only a {@link LoggedTunable#isPersistent()} one raises
 * the alert. The hazard being guarded against is a value <em>outliving the session that set it</em>,
 * and a non-persistent one resets at boot, so it cannot. Alerting on those too would nag a driver
 * who flipped a feature toggle on purpose, which is how an alert panel gets ignored.
 */
public final class LoggedTunables {
  /**
   * Where tunable values belong in the NT tree.
   *
   * <p>Shared with {@link TunableDoubleArray} by convention rather than by enforcement -- a path is
   * just a path -- so that everything tunable on the robot sits in one subtree a dashboard can be
   * pointed at.
   */
  public static final String kRoot = "/Tuning";

  /** The id of the aggregated alert, within {@link Alert}'s default {@code "Alerts"} group. */
  public static final String kAlertId = "nonDefaultTunables";

  /** How many paths the alert text names before it summarises the rest. */
  private static final int kNamesInAlert = 5;

  private static final List<LoggedTunable<?>> s_registry = new ArrayList<>();
  private static final Set<String> s_paths = new HashSet<>();

  /**
   * Medium, not low: the whole point is to be noticed, and a green "i" reads as "informational,
   * ignore". Not high either -- a tuned value is expected during a tuning session, and crying wolf
   * in the normal case costs you the next real high-priority alert.
   */
  private static final Alert s_alert = new Alert(kAlertId, "", Alert.Level.MEDIUM);

  private static String[] s_declaredPaths = new String[0];

  private LoggedTunables() {
  }

  /**
   * Adds a newly constructed value to the registry.
   *
   * <p>Throws on a duplicate path rather than warning, which is stricter than
   * {@link TuningStateCache}. Two values sharing a path would share one topic and write the same log
   * key, and the second one's code default would be a phantom that the alert compares against
   * forever.
   *
   * @param value the value being constructed
   */
  static void register(LoggedTunable<?> value) {
    if (!s_paths.add(value.path())) {
      throw new IllegalArgumentException("Duplicate tunable path: '" + value.path() + "'");
    }

    s_registry.add(value);
    s_declaredPaths = s_paths.stream().sorted().toArray(String[]::new);
  }

  /**
   * Updates the alert and the logged outputs. Call once per loop from {@code Robot}.
   *
   * <p>This is deliberately separate from each value's {@code periodic()}. Aggregating needs every
   * value already refreshed, and {@code periodic()} ordering across registered inputs is
   * registration order, which is invisible in the code. AdvantageKit runs the whole input pass
   * before {@code robotPeriodic()} runs the scheduler, so a value read here was refreshed this same
   * cycle.
   */
  public static void refresh() {
    List<String> paths = new ArrayList<>();
    List<String> descriptions = new ArrayList<>();

    for (LoggedTunable<?> tunable : s_registry) {
      if (tunable.isDefault()) {
        continue;
      }

      descriptions.add(
          tunable.path() + " = " + tunable.get() + " (code: " + tunable.codeDefault() + ")");

      if (tunable.isPersistent()) {
        paths.add(tunable.path());
      }
    }

    Logger.recordOutput("Tuning/NonDefaultTunables", descriptions.toArray(String[]::new));
    Logger.recordOutput("Tuning/NonDefaultTunableCount", descriptions.size());
    Logger.recordOutput("Tuning/NonDefaultPersistentTunableCount", paths.size());
    // Declared paths stay in networktables.json forever after a declaration is deleted from source,
    // because nothing ever un-persists a topic. This output is what lets a human diff declared
    // against stored and spot the orphans.
    Logger.recordOutput("Tuning/DeclaredTunablePaths", s_declaredPaths);
    Logger.recordOutput("Tuning/TunablesPersisting", isPersisting());

    if (!paths.isEmpty()) {
      s_alert.setText(paths.size() + " tuned value(s) differ from code: " + summarise(paths));
    }

    s_alert.set(!paths.isEmpty());
  }

  /**
   * Every declared value currently sitting on something other than its code default, sorted.
   *
   * <p>The same set {@code Tuning/NonDefaultTunables} records -- persistent or not. Useful for a
   * pre-match check that wants to assert the robot is running what the source says.
   */
  public static String[] nonDefaultPaths() {
    return s_registry.stream()
        .filter(tunable -> !tunable.isDefault())
        .map(LoggedTunable::path)
        .sorted()
        .toArray(String[]::new);
  }

  /**
   * Every off-default value the robot will still be carrying after a reboot, sorted.
   *
   * <p>The set the alert counts. This is the pre-match question: a non-persistent value resets on
   * its own, so only these can surprise you tomorrow.
   */
  public static String[] nonDefaultPersistentPaths() {
    return s_registry.stream()
        .filter(tunable -> !tunable.isDefault() && tunable.isPersistent())
        .map(LoggedTunable::path)
        .sorted()
        .toArray(String[]::new);
  }

  /**
   * Publishes the code default for every declared value.
   *
   * <p>This <em>sets</em> each value rather than unpublishing its topic, so a dashboard keeps
   * showing the entries and the reset is logged like any other change. Unpublishing would leave a
   * dashboard staring at nothing and would not clear the saved value until the next write.
   */
  public static void resetAllToDefaults() {
    for (LoggedTunable<?> tunable : s_registry) {
      tunable.resetToDefault();
    }
  }

  /** Every declared path, sorted. */
  public static String[] declaredPaths() {
    return s_declaredPaths.clone();
  }

  /**
   * Whether NetworkTables is running, and so whether anything declared will actually persist.
   *
   * <p>False means something was declared before {@code RobotBase} started NetworkTables -- see
   * {@link LoggedTunable}'s notes on declaration order. Reported as an output rather than thrown,
   * because a robot that cannot save its tuning values should still drive.
   */
  public static boolean isPersisting() {
    var mode = NetworkTableInstance.getDefault().getNetworkMode();
    return mode.contains(NetworkTableInstance.NetworkMode.SERVER)
        || mode.contains(NetworkTableInstance.NetworkMode.CLIENT)
        || mode.contains(NetworkTableInstance.NetworkMode.STARTING);
  }

  /**
   * Forgets every declared value.
   *
   * <p>For tests, which build robot objects repeatedly -- a mechanism constructed in each of ten
   * tests claims the same paths ten times, and {@link #register} throws on the second. On a real
   * robot everything is constructed exactly once, which is why a duplicate there is a bug rather
   * than a situation to recover from.
   *
   * <p>Pair it with {@link TuningStateCache#clear()}, which caches objects that hold these paths.
   */
  public static void reset() {
    s_registry.clear();
    s_paths.clear();
    s_declaredPaths = new String[0];
    s_alert.set(false);
    s_alert.setText("");
  }

  /** The first few off-default paths, with a count for the rest. */
  private static String summarise(List<String> paths) {
    String names = paths.stream().limit(kNamesInAlert).collect(Collectors.joining(", "));

    return paths.size() > kNamesInAlert
        ? names + " and " + (paths.size() - kNamesInAlert) + " more"
        : names;
  }
}
