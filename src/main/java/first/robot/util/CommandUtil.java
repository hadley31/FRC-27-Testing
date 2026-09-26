package first.robot.util;

import java.util.List;
import java.util.function.Predicate;
import java.util.function.UnaryOperator;
import java.util.stream.Stream;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

public class CommandUtil {
  public static Command print(String message) {
    return Command.noRequirements(coroutine -> System.out.println(message)).named("PrintCommand");
  }

  private static <T> Predicate<T> isNotNullPredicate(UnaryOperator<T> consumer) {
    return t -> {
      if (t == null) {
        return false;
      }
      return consumer.apply(t) != null;
    };
  }

  public static List<Command> lineage(Scheduler scheduler, Command command) {
    UnaryOperator<Command> getParent = scheduler::getParentOf;
    return Stream.iterate(command, isNotNullPredicate(getParent), getParent).toList()
        .reversed();
  }
}
