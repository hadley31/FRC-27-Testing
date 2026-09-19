package first.lib.command3;

import java.util.Collections;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;

/**
 * A {@link Command} that owns an inclusive {@code [start, end]} sub-range of a linear resource
 * (such as an addressable LED strip) rather than the whole {@link Mechanism}. Two {@code
 * RangedLEDCommand}s that require the same mechanism conflict with each other only if their ranges
 * overlap; commands with disjoint ranges may run concurrently. Comparisons against anything else
 * (a different mechanism, or a plain command requiring the whole mechanism) fall back to the
 * normal requirement-set check, so a whole-mechanism command still correctly conflicts with any
 * range command that touches the same mechanism.
 *
 * <p>A command may instead be marked as an <b>overlay</b>, in which case it never conflicts on
 * range overlap at all, with any other command (overlay or not). Overlays are expected to resolve
 * pixel ownership some other way (e.g. a priority-sorted compositing pass) rather than through
 * scheduler exclusivity.
 *
 * <p>Build one with {@link #wrapping(NeedsNameBuilderStage, int, int, boolean)}, wrapping the
 * builder stage returned from {@link Mechanism#run(java.util.function.Consumer)} or similar:
 *
 * <pre>{@code
 * RangedLEDCommand.wrapping(led.run(coroutine -> { ... }), 0, 49, false).named("Left Half");
 * }</pre>
 */
public final class RangedLEDCommand implements Command {
  private final Command m_delegate;
  private final int m_start;
  private final int m_end;
  private final boolean m_overlay;

  private RangedLEDCommand(Command delegate, int start, int end, boolean overlay) {
    m_delegate = delegate;
    m_start = start;
    m_end = end;
    m_overlay = overlay;
  }

  /**
   * Wraps a {@link NeedsNameBuilderStage} so that the {@link Command} it eventually builds is a
   * range-exclusive {@code RangedLEDCommand} instead of a normal, whole-mechanism command.
   *
   * @param delegate the builder stage to wrap, e.g. from {@link
   *     Mechanism#run(java.util.function.Consumer)}
   * @param startIndex the first index in the range, inclusive
   * @param endIndex the last index in the range, inclusive
   * @param overlay if true, this command never conflicts with another command over range overlap;
   *     it's expected to resolve pixel ownership through some other means, such as compositing
   * @return a builder stage that produces a {@code RangedLEDCommand} once named
   */
  public static NeedsNameBuilderStage wrapping(
      NeedsNameBuilderStage delegate, int startIndex, int endIndex, boolean overlay) {
    return new BuilderStage(delegate, startIndex, endIndex, overlay);
  }

  @Override
  public void run(Coroutine coroutine) {
    m_delegate.run(coroutine);
  }

  @Override
  public String name() {
    return m_delegate.name();
  }

  @Override
  public Set<Mechanism> requirements() {
    return m_delegate.requirements();
  }

  @Override
  public int priority() {
    return m_delegate.priority();
  }

  @Override
  public void onCancel() {
    m_delegate.onCancel();
  }

  @Override
  public boolean conflictsWith(Command other) {
    if (other instanceof RangedLEDCommand ranged
        && !Collections.disjoint(requirements(), ranged.requirements())) {
      if (m_overlay || ranged.m_overlay) {
        // Overlays resolve pixel ownership through compositing, not scheduler exclusivity.
        return false;
      }
      return m_start <= ranged.m_end && ranged.m_start <= m_end;
    }
    return Command.super.conflictsWith(other);
  }

  /**
   * A {@link NeedsNameBuilderStage} that wraps the eventual {@link Command} in a {@link
   * RangedLEDCommand} once it's named, so range-overlap conflict checking survives the rest of the
   * builder chain (priority, cancellation hook, end condition).
   */
  private static final class BuilderStage implements NeedsNameBuilderStage {
    private final NeedsNameBuilderStage m_delegate;
    private final int m_start;
    private final int m_end;
    private final boolean m_overlay;

    BuilderStage(NeedsNameBuilderStage delegate, int start, int end, boolean overlay) {
      m_delegate = delegate;
      m_start = start;
      m_end = end;
      m_overlay = overlay;
    }

    @Override
    public NeedsNameBuilderStage whenCanceled(Runnable onCancel) {
      return new BuilderStage(m_delegate.whenCanceled(onCancel), m_start, m_end, m_overlay);
    }

    @Override
    public NeedsNameBuilderStage withPriority(int priority) {
      return new BuilderStage(m_delegate.withPriority(priority), m_start, m_end, m_overlay);
    }

    @Override
    public NeedsNameBuilderStage until(BooleanSupplier endCondition) {
      return new BuilderStage(m_delegate.until(endCondition), m_start, m_end, m_overlay);
    }

    @Override
    public Command named(String name) {
      return new RangedLEDCommand(m_delegate.named(name), m_start, m_end, m_overlay);
    }
  }
}
