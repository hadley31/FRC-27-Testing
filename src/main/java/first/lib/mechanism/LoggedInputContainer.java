package first.lib.mechanism;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Something that owns a {@link LoggedIO} together with the inputs object that IO fills — typically
 * a mechanism.
 *
 * <p>The inputs type appears as a second parameter because Java has no way to project it back out
 * of {@code IO} alone; naming it here is what lets {@link #update()} type check while
 * {@link #getIO()} still hands back the concrete IO interface (so callers keep its command
 * methods) and {@link #getInputs()} hands back the concrete inputs class (so callers keep its
 * fields). Subinterfaces for a specific kind of mechanism pin {@code I} to a concrete class, so
 * implementations only ever name their IO type.
 *
 * @param <IO> the IO interface this container talks to
 * @param <I>  the inputs class {@code IO} populates
 */
public interface LoggedInputContainer<IO extends LoggedIO<I>, I extends LoggableInputs> {
  /** Returns the IO layer. Use it to command the hardware; use {@link #getInputs()} to read it. */
  public IO getIO();

  /**
   * Returns the inputs most recently filled by {@link #update()}. This is the single source of
   * truth for this container's state, and the only one that replays.
   */
  public I getInputs();

  /** Returns the key these inputs are logged under. */
  public String getLogName();

  /**
   * Refreshes the inputs from the IO layer and hands them to the logger, which records them on a
   * real robot and overwrites them from the log during replay.
   *
   * <p>Must be called exactly once per loop, before any code reads the inputs.
   */
  public default void update() {
    getIO().updateInputs(getInputs());
    Logger.processInputs(getLogName(), getInputs());
  }
}
