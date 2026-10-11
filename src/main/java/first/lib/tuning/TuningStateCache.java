package first.lib.tuning;

import java.util.IdentityHashMap;
import java.util.Map;

import org.wpilib.command3.Mechanism;
import org.wpilib.units.Measure;

import first.lib.util.UnitDoubleConverter;

/**
 * The one {@link TuningState} each mechanism gets, created on first ask.
 *
 * <p>A {@code TuningState} publishes NetworkTables topics named after its mechanism, and the tunable
 * backend rejects a second publish to a path it already serves. Building one per mechanism is
 * therefore not just wasteful but visibly broken -- the second attempt warns and hands back tunables
 * nothing is listening to. Mechanisms are interfaces all the way down and cannot hold a field to cache
 * one in, so the cache lives here instead and {@code getTuningState()} stays safe to call as often as
 * anyone likes.
 *
 * <p>Keyed by mechanism identity, so two instances of the same mechanism class get their own state and
 * collide on their topic names, warning the way any other duplicate publish does. Keying by name would
 * silence that warning by quietly handing both instances the same setpoint, which is worse.
 */
public final class TuningStateCache {
  private static final Map<Mechanism, TuningState<?>> s_cache = new IdentityHashMap<>();

  private TuningStateCache() {
  }

  /**
   * The tuning state for a mechanism, creating and publishing it if this is the first ask.
   *
   * @param mechanism the mechanism being tuned, which names the published topics
   * @param setpointType the unit its setpoint is published in
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
   * <p>For tests, which build mechanisms and reset the tunable registry repeatedly: a state cached
   * across a reset holds tunables whose backend entries are gone.
   */
  public static void clear() {
    s_cache.clear();
  }
}
