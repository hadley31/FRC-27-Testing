package first.robot.util;

import org.wpilib.command3.Command;

public class CommandUtil {
  public static Command print(String message) {
    return Command.noRequirements(coroutine -> System.out.println(message)).named("PrintCommand");
  }

  // // v2Command's requirements (Subsystems) must be re-declared as v3 Mechanisms yourself.
  // public static Command fromV2( v2Command, Mechanism... requirements) {
  //   return Command.requiring(List.of(requirements))
  //       .executing(coroutine -> {
  //         v2Command.initialize();
  //         while (!v2Command.isFinished()) {
  //           v2Command.execute();
  //           coroutine.yield();
  //         }
  //         v2Command.end(false);
  //       })
  //       .whenCanceled(() -> v2Command.end(true))
  //       .named(v2Command.getName());
  // }
}
