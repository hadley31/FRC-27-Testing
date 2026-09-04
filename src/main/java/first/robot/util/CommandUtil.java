package first.robot.util;

import org.wpilib.command3.Command;

public class CommandUtil {
  public static Command print(String message) {
    return Command.noRequirements(coroutine -> System.out.println(message)).named("PrintCommand");
  }
}
