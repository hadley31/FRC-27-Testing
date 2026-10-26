package first.lib.tuning;

import java.util.IdentityHashMap;
import java.util.Map;

import org.wpilib.command3.Mechanism;
import org.wpilib.units.Measure;

import first.lib.util.UnitDoubleConverter;

/**
 * The one {@link TuningState} each mechanism gets, created on first ask.
 *
 * <p>A {@code TuningState} claims preference keys named after its mechanism, and
 * {@link LoggedTunables} rejects a duplicate key outright. Building one per mechanism is therefore
 * not merely wasteful, it throws. Mechanisms are interfaces all the way down and cannot hold a field
 * to cache one in, so the cache lives here instead and {@code getTuningState()} stays safe to call
 * as often as anyone likes.
 *
 * <p>Keyed by mechanism identity, so two instances of the same mechanism class get their own state
 * and collide on their key names, which throws. Keying by name would hide that by quietly handing
 * both instances the same setpoint and the same gains, which is worse: two turrets silently sharing
 * one kP is a bug you find on the field.
 */
public final class TuningStateCache {
  private static final Map<Mechanism, TuningState<?>> s_cache = new IdentityHashMap<>();

  private TuningStateCache() {
  }

  /**
   * The tuning state for a mechanism, creating it if this is the first ask.
   *
   * @param mechanism the mechanism being tuned, which names its preference keys
   * @param setpointType the unit its setpoint is stored and displayed in
   * @return the mechanism's one tuning state
   */
  @SuppressWarnings("unchecked")
  public static <U extends Measure<?>> TuningState<U> getOrCreate(
      Mechanism mechanism, UnitDoubleConverter<U> setpointType) {
    // Safe by construction: a mechanism's measure type is fixed by its interface, so the state filed
    // under it was created from this same type.
    return (TuningState<U>) s_cache.computeIfAbsent(
        mechanism, keyed -> new TuningState<>(keyed.getName(), setpointType));
  }

  /**
   * Forgets every cached state.
   *
   * <p>For tests, which build mechanisms repeatedly. Pair it with {@link LoggedTunables#reset()}:
   * a state cached across a registry reset holds keys the registry no longer knows about, and a
   * registry reset without this leaves the next test's state colliding with a cached one.
   */
  public static void clear() {
    s_cache.clear();
  }
}
