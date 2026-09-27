package first.lib.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Pins the two NetworkTables details that make {@link Toggle} work, both of which fail silently if
 * they regress.
 *
 * <p>AdvantageKit uses a toggle's key verbatim as the NT topic name, so a path without a leading
 * slash produces a malformed topic that {@code setPersistent} can never be matched against. And the
 * NT server ignores {@code setPersistent} on a topic that has not published a value yet, logging a
 * warning rather than failing — so the publish-then-flag order in the constructor is load-bearing.
 */
class ToggleTest {
  private static MultiSubscriber s_subAll;

  @BeforeAll
  static void startServer() throws Exception {
    var inst = NetworkTableInstance.getDefault();
    inst.startServer(Files.createTempFile("nt-toggle-test", ".json").toString(), "", "", 0);
    // RobotBase does the same: persistent values land in server storage and need a local subscriber
    // to propagate back out to getters.
    s_subAll = new MultiSubscriber(inst, new String[] {""});
  }

  @AfterAll
  static void stopServer() {
    s_subAll.close();
    NetworkTableInstance.getDefault().stopServer();
  }

  @Test
  void relativePathsAreRejected() {
    // "Toggles/Foo" would create a topic literally named "Toggles/Foo", with no leading slash.
    assertThrows(IllegalArgumentException.class, () -> Toggle.of("Tuning/Relative", true));
  }

  @Test
  void aToggleStartsAtItsCompetitionDefault() {
    var toggle = Toggle.of("/Tuning/Test/Default", true);

    assertTrue(toggle.getAsBoolean());
    assertTrue(toggle.competitionDefault());
    assertTrue(toggle.isDefault());
    assertEquals("/Tuning/Test/Default", toggle.path());
  }

  @Test
  void persistentTogglesFlagTheirTopicSoTheServerSavesThem() throws Exception {
    Toggle.persistent("/Tuning/Test/Persisted", true);
    NetworkTableInstance.getDefault().flush();

    assertTrue(
        NetworkTableInstance.getDefault().getBooleanTopic("/Tuning/Test/Persisted").isPersistent(),
        "the topic must be flagged, or nothing survives a reboot");
  }

  @Test
  void ordinaryTogglesAreNotPersisted() throws Exception {
    Toggle.of("/Tuning/Test/Transient", true);
    NetworkTableInstance.getDefault().flush();

    assertFalse(
        NetworkTableInstance.getDefault().getBooleanTopic("/Tuning/Test/Transient").isPersistent(),
        "a behaviour switch must not survive a debug session into an event");
  }

  @Test
  void publishesAtTheAbsolutePathGiven() throws Exception {
    Toggle.of("/Tuning/Test/Path", true);
    NetworkTableInstance.getDefault().flush();

    assertTrue(NetworkTableInstance.getDefault().getBooleanTopic("/Tuning/Test/Path").exists());
  }
}
