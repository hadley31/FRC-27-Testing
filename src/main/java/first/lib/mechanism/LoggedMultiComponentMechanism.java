package first.lib.mechanism;

import java.util.List;

import org.wpilib.command3.Mechanism;

import first.lib.util.LoggedTracer;

/**
 * A mechanism built out of several {@link LoggedComponent}s rather than being one itself — a
 * drivetrain over its modules and gyro, a vision mechanism over its cameras.
 *
 * <p>The components are owned here because each of them has to name this mechanism to know what to
 * log under, and refreshing them is one call so a mechanism's periodic cannot forget a component or
 * read one twice. Every component ends up logged beneath this mechanism's {@link #getName()}, which
 * is the whole reason the ownership runs this way; see {@link LoggedComponent#getLogPrefix()}.
 */
public interface LoggedMultiComponentMechanism extends Mechanism {
  /**
   * The components this mechanism is made of, in the order they should be refreshed. Each must be
   * owned by this mechanism, so that its inputs are logged beneath this one's name.
   */
  public List<? extends LoggedComponent<?, ?>> getComponents();

  /**
   * Refreshes every component's inputs, which is the first thing a mechanism's periodic should do:
   * nothing may read a component's inputs until they have been read out of the IO layer (or out of
   * the log, during replay) this loop.
   *
   * <p>Traced as {@code <name>.updateComponents}, with each component's own {@code update} span
   * nested inside it, so the cost of reading a mechanism is one number that can still be broken
   * down per component.
   */
  public default void updateComponents() {
    LoggedTracer.startTrace("%s.updateComponents".formatted(getName()));
    getComponents().forEach(LoggedComponent::update);
    LoggedTracer.endTrace("%s.updateComponents".formatted(getName()));
  }
}
