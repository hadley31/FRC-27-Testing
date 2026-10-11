package first.lib.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.command3.Mechanism;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.TunableRegistry;

import first.lib.util.UnitDoubleConverter;

/**
 * Covers the two things that have to hold for live tuning to be trustworthy: a setpoint means the same
 * value on the way out as on the way in, and a mechanism's topics are published exactly once.
 */
class TuningStateTest {
  /** Named for its topics: {@code getName()} on a mechanism is its class's simple name. */
  private static final class Thing implements Mechanism {
  }

  private static final class OtherThing implements Mechanism {
  }

  private NetworkTableInstance m_inst;
  private List<String> m_warnings;

  @BeforeEach
  void setUp() {
    m_inst = NetworkTableInstance.create();
    TunableRegistry.reset();
    TuningStateCache.clear();
    TunableRegistry.registerBackend("", new NetworkTablesTunableBackend(m_inst, "/Tunables"));

    m_warnings = new ArrayList<>();
    TunableRegistry.setReportWarning(m_warnings::add);
  }

  @AfterEach
  void tearDown() {
    TunableRegistry.setReportWarning(null);
    TunableRegistry.reset();
    TuningStateCache.clear();
    m_inst.close();
  }

  @Test
  void aSetpointGivenInAnotherUnitIsConvertedIntoThePublishedOne() {
    var state = TuningStateCache.getOrCreate(new Thing(), UnitDoubleConverter.DEGREES);

    state.setSetpoint(Rotations.of(0.5));

    assertEquals(180.0, state.setpoint().in(Degrees), 1e-9);
  }

  @Test
  void aMechanismGetsOneStateHoweverOftenItIsAskedFor() {
    Thing thing = new Thing();

    assertSame(
        TuningStateCache.getOrCreate(thing, UnitDoubleConverter.DEGREES),
        TuningStateCache.getOrCreate(thing, UnitDoubleConverter.DEGREES));

    // The reason the cache exists: a second TuningState would try to publish topics the backend is
    // already serving, warn, and hand back tunables no dashboard is writing to.
    assertTrue(m_warnings.isEmpty(), m_warnings.toString());
    assertEquals(
        0.0, m_inst.getDoubleTopic("/Tunables/Thing Setpoint").subscribe(Double.NaN).get(), 0.0);
  }

  @Test
  void twoMechanismsEachGetTheirOwn() {
    assertNotSame(
        TuningStateCache.getOrCreate(new Thing(), UnitDoubleConverter.DEGREES),
        TuningStateCache.getOrCreate(new OtherThing(), UnitDoubleConverter.DEGREES));

    assertTrue(m_warnings.isEmpty(), m_warnings.toString());
  }
}
