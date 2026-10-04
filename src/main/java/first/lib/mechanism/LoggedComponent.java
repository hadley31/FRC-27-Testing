package first.lib.mechanism;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.command3.Mechanism;

import first.lib.util.LoggedTracer;

/**
 * Something that owns a {@link LoggedComponentIO} together with the inputs object that IO fills —
 * either a mechanism itself or one of the pieces a mechanism is built out of, such as a swerve
 * module or a camera.
 *
 * <p>A component names the {@link Mechanism} it belongs to rather than the key it logs under: that
 * is what makes {@link #getLogPrefix()} able to produce {@code <mechanism>/<component>} with no
 * caller spelling a path out, and it keeps a component's inputs filed beneath its owner even after
 * either is renamed. The owner is passed in at construction, so a component is built by the
 * mechanism that owns it.
 *
 * <p>The inputs type appears as a second parameter because Java has no way to project it back out
 * of {@code T} alone; naming it here is what lets {@link #update()} type check while
 * {@link #getIO()} still hands back the concrete IO interface (so callers keep its command
 * methods) and {@link #getInputs()} hands back the concrete inputs class (so callers keep its
 * fields). Subinterfaces for a specific kind of mechanism pin {@code I} to a concrete class, so
 * implementations only ever name their IO type.
 *
 * @param <T> the IO interface this component talks to
 * @param <I> the inputs class {@code T} populates
 */
public interface LoggedComponent<T extends LoggedComponentIO<I>, I extends LoggableInputs> {
  /** Returns the IO layer. Use it to command the hardware; use {@link #getInputs()} to read it. */
  public T getIO();

  /**
   * Returns the inputs most recently filled by {@link #update()}. This is the single source of
   * truth for this component's state, and the only one that replays.
   */
  public I getInputs();

  /**
   * What this component is called beneath its mechanism, in both the log key and its trace span.
   *
   * <p>Defaults to the implementing class's simple name, which is all a mechanism needs when it
   * owns one of a thing. Anything a mechanism owns several of -- a swerve module, a camera -- must
   * override this with something that tells the siblings apart, or they would share a key and
   * overwrite each other's inputs.
   */
  public default String getName() {
    return this.getClass().getSimpleName();
  }

  /** Returns the key these inputs are logged under: this component's name beneath its owner's. */
  public default String getLogPrefix() {
    return "%s/%s".formatted(getMechanism().getName(), getName());
  }

  /**
   * The mechanism this component belongs to, which is what its log key is nested beneath. A
   * mechanism that is its own component answers with itself; see {@link G3Mechanism}.
   */
  public Mechanism getMechanism();

  /**
   * Refreshes the inputs from the IO layer and hands them to the logger, which records them on a
   * real robot and overwrites them from the log during replay.
   *
   * <p>Traced as {@code <name>.update}, so what it costs to read this component shows up on its own
   * rather than only inside whatever its mechanism does with it.
   *
   * <p>Must be called exactly once per loop, before any code reads the inputs.
   */
  public default void update() {
    LoggedTracer.startTrace("%s.update".formatted(getName()));
    getIO().updateInputs(getInputs());
    Logger.processInputs(getLogPrefix(), getInputs());
    LoggedTracer.endTrace("%s.update".formatted(getName()));
  }
}
