// Copyright (c) 2025 Edan Liahovetsky
// Copyright (c) 2025 Rebel Robotics (FRC 2638)
//
// Use of this source code is governed by the BSD 3-Clause license that can be
// found in licenses/BLine-LICENSE.txt at the root of this repository.
//
// Modified from upstream BLine to target WPILib 2027 and Commands v3.

package BLine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.SequentialGroup;

/**
 * Command composition helpers that do not hold their children's requirements.
 *
 * <p>This class mirrors the Commands v3 composition factories on {@link Command},
 * but builds each composition as a {@link Command#noRequirements} command whose body
 * {@link Coroutine#await awaits} its children. Because the composition itself requires
 * nothing, each child claims its own {@link Mechanism} requirements only while that child
 * is actually running, and releases them as soon as it finishes.
 *
 * <p>This matters because the built-in groups behave differently. A
 * {@link SequentialGroup} (and the parallel groups) <i>own</i> every mechanism required by
 * any child for the entire duration of the group. A mechanism required by one step but not
 * by a later one stays owned, and therefore uncommanded, while that later step runs -- its
 * default command cannot take over, and neither can an event-triggered command.
 *
 * <p>These helpers are the Commands v3 replacement for the Commands v2 {@code asProxy()}
 * wrapping that this class used to perform.
 *
 * <p>This is useful when a path contains event triggers:
 *
 * <pre>{@code
 * import static BLine.BLineCommands.sequence;
 * import org.wpilib.command3.Command;
 *
 * Command auto = sequence(
 *     shooter.shoot().withTimeout(Seconds.of(2.0)),
 *     pathing.followPath("intakethroughdepot"),
 *     shooter.shoot()
 * );
 * }</pre>
 *
 * <p>Real requirement conflicts still exist. If two running commands require the same
 * mechanism at the same time, the {@link org.wpilib.command3.Scheduler} resolves that
 * conflict normally using command priority.
 *
 * @see Command
 * @see Coroutine#await(Command)
 */
public final class BLineCommands {
  private BLineCommands() {
    throw new UnsupportedOperationException("This is a utility class");
  }

  /**
   * Requirement-free counterpart to {@link Command#sequence(Command...)}.
   *
   * <p>Each command is awaited in order. Unlike {@link SequentialGroup}, the returned
   * command requires nothing, so each child owns its requirements only while it runs.
   *
   * @param commands commands to run in sequence
   * @return a sequence that does not hold its children's requirements
   * @see Command#sequence(Command...)
   */
  public static Command sequence(Command... commands) {
    List<Command> steps = copyOf(commands, "commands");
    return Command.noRequirements(coroutine -> {
      for (Command command : steps) {
        coroutine.await(command);
      }
    }).named(joinNames(steps, " -> "));
  }

  /**
   * Requirement-free counterpart to {@link Command#sequence(Command...)} that repeats forever.
   *
   * <p>Each command is awaited in order, and the whole sequence restarts once the last
   * command completes. The returned command requires nothing, so each child owns its
   * requirements only while it runs. The command never finishes on its own; cancel it or
   * bound it with {@link Command#withTimeout} or an end condition.
   *
   * @param commands commands to run in sequence repeatedly
   * @return a repeating sequence that does not hold its children's requirements
   */
  public static Command repeatingSequence(Command... commands) {
    List<Command> steps = copyOf(commands, "commands");
    return Command.noRequirements(coroutine -> {
      while (true) {
        for (Command command : steps) {
          coroutine.await(command);
        }
        // An empty step list would otherwise spin without ever reaching an await.
        coroutine.yield();
      }
    }).named("Repeating: " + joinNames(steps, " -> "));
  }

  /**
   * Requirement-free counterpart to {@link Command#parallel(Command...)}.
   *
   * <p>All commands are started together and the composition finishes once every one of
   * them has finished. The returned command requires nothing, so each child owns its
   * requirements only while it runs.
   *
   * @param commands commands to run in parallel
   * @return a parallel group that does not hold its children's requirements
   * @throws IllegalArgumentException if any two commands require the same mechanism
   * @see Command#parallel(Command...)
   */
  public static Command parallel(Command... commands) {
    List<Command> group = copyOf(commands, "commands");
    return Command.noRequirements(coroutine -> coroutine.awaitAll(group))
        .named(joinNames(group, " & "));
  }

  /**
   * Requirement-free counterpart to {@link Command#race(Command...)}.
   *
   * <p>All commands are started together and the composition finishes as soon as the first
   * one finishes; the rest are canceled. The returned command requires nothing, so each
   * child owns its requirements only while it runs.
   *
   * @param commands commands to run until the first one finishes
   * @return a race group that does not hold its children's requirements
   * @throws IllegalArgumentException if any two commands require the same mechanism
   * @see Command#race(Command...)
   */
  public static Command race(Command... commands) {
    List<Command> group = copyOf(commands, "commands");
    return Command.noRequirements(coroutine -> coroutine.awaitAny(group))
        .named(joinNames(group, " | "));
  }

  /**
   * Requirement-free deadline group.
   *
   * <p>The {@code deadline} command and all other {@code commands} start together. The
   * composition finishes when {@code deadline} finishes, canceling any of the others that
   * are still running. The returned command requires nothing, so each child owns its
   * requirements only while it runs.
   *
   * @param deadline the command that ends the group when it finishes
   * @param commands other commands to run alongside the deadline
   * @return a deadline group that does not hold its children's requirements
   * @throws IllegalArgumentException if any two commands require the same mechanism
   */
  public static Command deadline(Command deadline, Command... commands) {
    Objects.requireNonNull(deadline, "deadline");
    List<Command> others = copyOf(commands, "commands");
    return Command.noRequirements(coroutine -> {
      // Forked commands are canceled when this parent exits, which happens as soon as
      // the deadline command below completes.
      if (!others.isEmpty()) {
        coroutine.fork(others);
      }
      coroutine.await(deadline);
    }).named("Deadline: " + deadline.name());
  }

  /**
   * Requirement-free counterpart to a conditional command.
   *
   * <p>{@code selector} is evaluated once when the returned command starts running, and the
   * chosen branch is awaited. The returned command requires nothing, so the selected branch
   * owns its requirements only while it runs; the branch that is not selected never claims
   * any.
   *
   * @param onTrue the command selected when {@code selector} returns true
   * @param onFalse the command selected when {@code selector} returns false
   * @param selector selects which command branch to run
   * @return a conditional command that does not hold its children's requirements
   */
  public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
    Objects.requireNonNull(onTrue, "onTrue");
    Objects.requireNonNull(onFalse, "onFalse");
    Objects.requireNonNull(selector, "selector");
    return Command.noRequirements(
        coroutine -> coroutine.await(selector.getAsBoolean() ? onTrue : onFalse))
        .named("Either: " + onTrue.name() + " or " + onFalse.name());
  }

  /**
   * Requirement-free counterpart to a selection command.
   *
   * <p>{@code selector} is evaluated once when the returned command starts running and the
   * matching command is awaited. If no entry matches, the command finishes immediately. The
   * returned command requires nothing, so the selected command owns its requirements only
   * while it runs.
   *
   * @param commands map of selector values to commands
   * @param selector supplies the selector value at runtime
   * @param <K> selector key type
   * @return a select command that does not hold its children's requirements
   */
  public static <K> Command select(Map<K, Command> commands, Supplier<? extends K> selector) {
    Objects.requireNonNull(commands, "commands");
    Objects.requireNonNull(selector, "selector");
    Map<K, Command> options = new LinkedHashMap<>();
    for (Map.Entry<K, Command> entry : commands.entrySet()) {
      options.put(entry.getKey(), Objects.requireNonNull(entry.getValue(), "commands value"));
    }
    return Command.noRequirements(coroutine -> {
      Command selected = options.get(selector.get());
      if (selected != null) {
        coroutine.await(selected);
      }
    }).named("Select: " + joinNames(new ArrayList<>(options.values()), " / "));
  }

  /**
   * Requirement-free counterpart to a deferred command with explicit requirements.
   *
   * <p>The command is obtained from {@code supplier} when the returned command starts
   * running, and is then awaited. The explicit {@code requirements} belong to the returned
   * command itself, matching the deferred-command contract; the supplied command's own
   * requirements are claimed only while it runs.
   *
   * @param supplier supplies the command to run when the deferred command starts
   * @param requirements requirements for the returned deferred command
   * @return a deferred command holding only {@code requirements}
   */
  public static Command defer(Supplier<Command> supplier, Set<Mechanism> requirements) {
    Objects.requireNonNull(supplier, "supplier");
    Objects.requireNonNull(requirements, "requirements");
    return Command.requiring(requirements).executing(coroutine -> {
      Command command = supplier.get();
      if (command != null) {
        coroutine.await(command);
      }
    }).named("Deferred");
  }

  /**
   * Requirement-free counterpart to a deferred command with no requirements.
   *
   * <p>The command is obtained from {@code supplier} when the returned command starts
   * running, and is then awaited. The returned command requires nothing; the supplied
   * command's requirements are claimed only while it runs.
   *
   * @param supplier supplies the command to run when the deferred command starts
   * @return a deferred command that does not hold its child's requirements
   */
  public static Command deferredProxy(Supplier<Command> supplier) {
    Objects.requireNonNull(supplier, "supplier");
    return Command.noRequirements(coroutine -> {
      Command command = supplier.get();
      if (command != null) {
        coroutine.await(command);
      }
    }).named("Deferred Proxy");
  }

  private static List<Command> copyOf(Command[] commands, String argumentName) {
    Objects.requireNonNull(commands, argumentName);
    for (int i = 0; i < commands.length; i++) {
      Objects.requireNonNull(commands[i], argumentName + "[" + i + "]");
    }
    return List.copyOf(Arrays.asList(commands));
  }

  private static String joinNames(List<Command> commands, String joiner) {
    if (commands.isEmpty()) {
      return "<empty>";
    }
    return commands.stream().map(Command::name).collect(Collectors.joining(joiner));
  }
}
