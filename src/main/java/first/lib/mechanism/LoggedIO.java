package first.lib.mechanism;

/**
 * An IO layer that reports its state by filling in an inputs object rather than by returning
 * values from getters.
 *
 * <p>This is the AdvantageKit contract: every read of the outside world happens in exactly one
 * place, {@link #updateInputs}, so a log replay can substitute recorded values for real hardware
 * without the code above the IO layer noticing the difference. Anything an implementation returns
 * from its own getters is invisible to the logger and will not replay, so state belongs in
 * {@code T}.
 *
 * <p>{@code T} is the plain inputs class, not the generated {@code ...AutoLogged} subclass, so an
 * IO implementation never has to name generated code. The container that owns the IO holds the
 * {@code AutoLogged} subclass and supplies it here; see {@link LoggedInputContainer}.
 *
 * @param <T> the inputs class this IO populates
 */
public interface LoggedIO<T> {
  /**
   * Reads the current state of the hardware into {@code inputs}. Called once per loop by
   * {@link LoggedInputContainer#update()}, before anything reads the inputs.
   *
   * @param inputs the inputs object to populate
   */
  public void updateInputs(T inputs);
}
