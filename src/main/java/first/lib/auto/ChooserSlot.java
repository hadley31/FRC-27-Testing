package first.lib.auto;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.SequencedMap;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;

/**
 * One drop-down on the dashboard, as whoever is driving sees it: a list of things to pick from and
 * whichever one is picked.
 *
 * <p>Options are given as a name for each value, and a pick comes back as the value itself. The
 * names exist because a dashboard can only show and send strings; nothing above this interface has
 * to hold one to refer to an option.
 *
 * <p>The options are replaced whenever what may legally go in the slot changes, so an implementation
 * has to cope with its list being rewritten and with the picked value disappearing out from under
 * it. Falling back to the default when that happens is what makes the slots downstream of a changed
 * one clear themselves.
 *
 * @param <V> what the options stand for
 */
public interface ChooserSlot<V> {
  /**
   * Replaces what this slot offers.
   *
   * <p>Only called when the options have actually changed, so an implementation is free to
   * republish everything.
   *
   * @param options      the values this slot may hold, by the name to show for each, in the order to
   *                     show them
   * @param defaultValue which of them to fall back to when nothing valid is picked, or null to fall
   *                     back to nothing
   */
  void setOptions(SequencedMap<String, V> options, V defaultValue);

  /**
   * What is picked right now.
   *
   * @return the picked value, or null when the slot holds nothing.
   */
  V getSelected();

  /**
   * A slot backed by an AdvantageKit chooser, which is what puts it on the dashboard and puts the
   * pick in the log, so a replay reproduces what really ran.
   *
   * <p>One per key per robot program. AdvantageKit has no way to unregister a dashboard input and
   * its chooser has no close, so building a second one for a key would leave two publishers on the
   * same topics and two inputs in the log. The slots therefore outlive whatever asked for them,
   * which is what an opmode-owned chooser needs anyway: the framework rebuilds it every time the
   * opmode is selected.
   *
   * @param <V> what the options stand for
   * @param key the NetworkTables path to publish the chooser at
   * @return the slot for that key, created on the first call
   */
  @SuppressWarnings("unchecked")
  static <V> ChooserSlot<V> networkBacked(String key) {
    return (ChooserSlot<V>) NetworkBacked.s_byKey.computeIfAbsent(key, NetworkBacked::new);
  }

  /** The {@link #networkBacked(String)} implementation, and the registry of them. */
  final class NetworkBacked<V> implements ChooserSlot<V> {
    private static final Map<String, ChooserSlot<?>> s_byKey = new HashMap<>();

    private final LoggedNetworkChooser<V> m_chooser;

    private NetworkBacked(String key) {
      m_chooser = new LoggedNetworkChooser<>(key);
    }

    @Override
    public void setOptions(SequencedMap<String, V> options, V defaultValue) {
      m_chooser.clear();
      options.forEach(m_chooser::add);

      // The chooser names its default rather than holding it, so the name has to be found again.
      options.entrySet().stream()
          .filter(option -> option.getValue() == defaultValue)
          .findFirst()
          .ifPresent(option -> m_chooser.setDefault(option.getKey()));
    }

    @Override
    public V getSelected() {
      return m_chooser.getSelected();
    }
  }

  /**
   * Builds the option map {@link #setOptions} takes, which is a plain insertion-ordered map.
   *
   * @param <V> what the options stand for
   * @return an empty map to fill in dashboard order
   */
  static <V> SequencedMap<String, V> options() {
    return new LinkedHashMap<>();
  }
}
