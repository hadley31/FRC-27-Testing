package first.lib.tuning;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import java.nio.file.Files;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableInstance;

import first.lib.util.UnitDoubleConverter;

/**
 * Covers what has to hold for live tuning to be trustworthy: a setpoint means the same value on the
 * way out as on the way in, a mechanism's paths are claimed exactly once, gains tuned in a session
 * are still there after a reboot, and a gain edit is reported for exactly one loop.
 */
class TuningStateTest {
  private static MultiSubscriber s_subAll;

  @BeforeAll
  static void startServer() throws Exception {
    var inst = NetworkTableInstance.getDefault();
    inst.startServer(Files.createTempFile("nt-tuning-state-test", ".json").toString(), "", "", 0);
    s_subAll = new MultiSubscriber(inst, new String[] {""});
  }

  @AfterAll
  static void stopServer() {
    s_subAll.close();
    NetworkTableInstance.getDefault().stopServer();
  }

  @AfterEach
  void tearDown() {
    LoggedTunables.reset();
  }

  @Test
  void aSetpointGivenInAnotherUnitIsConvertedIntoThePublishedOne() {
    var state = new TuningState<>("Thing", UnitDoubleConverter.DEGREES);

    state.setSetpoint(Rotations.of(0.5));
    state.periodic();

    assertEquals(180.0, state.setpoint().in(Degrees), 1e-9);
  }

  @Test
  void aStateClaimsItsPathsUnderTheNameItWasGiven() {
    new TuningState<>("Thing", UnitDoubleConverter.DEGREES);

    assertTrue(
        NetworkTableInstance.getDefault().getDoubleTopic("/Tuning/Thing/kP").exists(),
        "gains should be claimed under the mechanism's name");
  }

  @Test
  void aSecondStateForTheSameNameIsRejected() {
    new TuningState<>("Thing", UnitDoubleConverter.DEGREES);

    // Two states under one name would share every path, so two instances of a mechanism would
    // silently share one kP -- a bug you find on the field. Each mechanism holding its own state as
    // a field is what makes one per mechanism the only thing you can write; this is the backstop for
    // two mechanisms built under the same name.
    assertThrows(
        IllegalArgumentException.class,
        () -> new TuningState<>("Thing", UnitDoubleConverter.DEGREES));
  }

  @Test
  void gainsPersistAndAreReportedAsOffDefault() {
    // The whole point of moving gains off org.wpilib.tunable: a value stored on the robot beats the
    // code default, so a session survives the reboots between matches.
    var inst = NetworkTableInstance.getDefault();
    try (var publisher = inst.getDoubleTopic("/Tuning/Persisted/kP").publish()) {
      publisher.set(4.25);
      inst.flush();

      var state = new TuningState<>("Persisted", UnitDoubleConverter.DEGREES);

      assertEquals(4.25, state.gains().kP());

      LoggedTunables.refresh();
      assertArrayEquals(new String[] {"/Tuning/Persisted/kP"}, LoggedTunables.nonDefaultPaths());
    }
  }

  @Test
  void aGainEditIsReportedForExactlyOneLoop() {
    var state = new TuningState<>("Edited", UnitDoubleConverter.DEGREES);

    state.periodic();
    assertFalse(state.gainsChangedThisLoop());

    var inst = NetworkTableInstance.getDefault();
    try (var publisher = inst.getDoubleTopic("/Tuning/Edited/kD").publish()) {
      publisher.set(0.5);
      inst.flush();

      state.periodic();
      assertTrue(state.gainsChangedThisLoop());
      assertEquals(0.5, state.gains().kD());

      // The bug this replaces: org.wpilib.tunable's flag was never cleared, so one edit left
      // tuningCommand issuing a retrying blocking CAN config write every loop for the rest of the
      // enable.
      state.periodic();
      assertFalse(state.gainsChangedThisLoop());
    }
  }

  @Test
  void aSetpointEditDoesNotCountAsAGainEdit() {
    var state = new TuningState<>("Setpointed", UnitDoubleConverter.DEGREES);

    state.setSetpoint(Degrees.of(45));
    state.periodic();

    // Applying gains is a blocking CAN write; a setpoint is pushed every loop anyway.
    assertEquals(45.0, state.setpoint().in(Degrees), 1e-9);
    assertFalse(state.gainsChangedThisLoop());
  }
}
