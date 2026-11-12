package first.lib.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;

/**
 * Holds every tunable declared in the robot code to the path rule {@link LoggedTunable} enforces.
 *
 * <p>{@code LoggedTunableTest} already covers the rule itself. What it cannot cover is a call site
 * that breaks it, because a tunable declared in a static field throws where it is declared: the
 * class fails to initialize, and nothing finds out until something touches that class. Since the
 * robot's modes are opmodes now, the first touch can be the driver station switching to autonomous
 * between matches, which is a poor time to discover a typo.
 *
 * <p>This reads the source rather than loading the classes because a bad path in a class no test
 * instantiates is exactly the one that gets through. Only {@code src/main/java} is scanned; the
 * tuning library's own tests name relative paths on purpose, to prove they are refused.
 */
class TunablePathsTest {
  /**
   * Matches a tunable declared against a literal path, across the line break that a long
   * declaration usually falls over.
   */
  private static final Pattern kDeclaration =
      Pattern.compile("LoggedTunable\\w*\\s*\\.of\\w*\\(\\s*\"([^\"]*)\"", Pattern.DOTALL);

  @Test
  void everyTunablePathInTheRobotCodeIsAbsolute() throws IOException {
    List<String> offenders = new ArrayList<>();
    List<String> found = new ArrayList<>();

    try (Stream<Path> sources = Files.walk(Path.of("src", "main", "java"))) {
      for (Path source : sources.filter(path -> path.toString().endsWith(".java")).toList()) {
        Matcher matcher = kDeclaration.matcher(Files.readString(source));
        while (matcher.find()) {
          String path = matcher.group(1);
          found.add(path);
          if (!path.startsWith("/") || path.endsWith("/")) {
            offenders.add(source.getFileName() + ": '" + path + "'");
          }
        }
      }
    }

    assertEquals(List.of(), offenders, "tunable paths must be absolute and name a topic");

    // A regex that silently stops matching would pass the assertion above by finding nothing.
    assertTrue(found.size() >= 7, "expected to find the robot's tunables, found " + found);
  }
}
