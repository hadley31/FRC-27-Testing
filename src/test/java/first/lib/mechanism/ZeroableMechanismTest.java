package first.lib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import java.nio.file.Files;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.units.measure.Angle;

import first.lib.mechanism.angle.AngleMechanism;
import first.lib.mechanism.angle.AngleMechanismIO;
import first.lib.mechanism.angle.AngleMechanismInputsAutoLogged;
import first.lib.tuning.LoggedTunables;

/**
 * Covers what has to hold for a zero pressed on the dashboard to be worth anything: the mechanism
 * reads zero afterwards, the number is still there after a reboot, and the raw sensor reading stays
 * inside the IO where nothing above can act on it by mistake.
 */
class ZeroableMechanismTest {
  /**
   * An IO over an imaginary absolute sensor, applying the offset the way
   * {@code TalonFXAngleMechanismIO} does.
   */
  private static final class FakeAngleIO implements AngleMechanismIO {
    private Angle m_raw = Degrees.of(0);
    private Angle m_commandedRaw = Degrees.of(0);
    private Angle m_offset = Degrees.of(0);

    @Override
    public void updateInputs(AngleMechanismInputsAutoLogged inputs) {
      inputs.currentAngle = m_raw.minus(m_offset);
      inputs.targetAngle = m_commandedRaw.minus(m_offset);
    }

    @Override
    public void setTargetAngle(Angle angle) {
      m_commandedRaw = angle.plus(m_offset);
    }

    @Override
    public Angle zero() {
      m_offset = m_raw;
      return m_offset;
    }

    @Override
    public void setZeroOffset(Angle offset) {
      m_offset = offset;
    }

    @Override
    public void setGains(Gains gains) {
    }

    @Override
    public void halt() {
    }
  }

  /** Named for its paths: {@code getName()} on a mechanism is its class's simple name. */
  private static final class Arm extends AngleMechanism<FakeAngleIO> {
    Arm() {
      super(new FakeAngleIO());
    }

    /** What {@code LoggedComponent.update()} does, without asking the logger for a real log. */
    void readInputs() {
      getIO().updateInputs(getInputs());
    }
  }

  private static final class OtherArm extends AngleMechanism<FakeAngleIO> {
    OtherArm() {
      super(new FakeAngleIO());
    }
  }

  private static MultiSubscriber s_subAll;

  @BeforeAll
  static void startServer() throws Exception {
    var inst = NetworkTableInstance.getDefault();
    inst.startServer(Files.createTempFile("nt-zeroable-test", ".json").toString(), "", "", 0);
    s_subAll = new MultiSubscriber(inst, new String[] {""});
  }

  @AfterAll
  static void stopServer() {
    s_subAll.close();
    NetworkTableInstance.getDefault().stopServer();
  }

  @BeforeEach
  void setUp() {
    // Before any mechanism is built: a mechanism's constructor publishes its own buttons.
    TunableRegistry.reset();
    TunableRegistry.registerBackend(
        "", new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/Tunables"));
  }

  @AfterEach
  void tearDown() {
    TunableRegistry.reset();
    // Every mechanism here claims paths under its own name, and the registry rejects a duplicate,
    // so the next test could not build one without this.
    LoggedTunables.reset();

    // The stored value outlives the registry that published it -- that is what persistence means --
    // so every test here would otherwise start from whatever the last one zeroed to.
    var inst = NetworkTableInstance.getDefault();
    for (String name : new String[] {"Arm", "OtherArm"}) {
      try (var publisher =
          inst.getDoubleTopic("/Tuning/" + name + "/ZeroOffset (Degree)").publish()) {
        publisher.set(0.0);
        inst.flush();
      }
    }
  }

  @Test
  void zeroingTakesWhereverTheMechanismIsAsItsZero() {
    var arm = new Arm();
    arm.getIO().m_raw = Degrees.of(30);
    arm.readInputs();

    assertEquals(30.0, arm.getCurrentMeasurement().in(Degrees), 1e-9);

    arm.zero();
    arm.readInputs();

    assertEquals(0.0, arm.getCurrentMeasurement().in(Degrees), 1e-9);
  }

  @Test
  void aSecondZeroDoesNotWaitForTheStoredOffsetToRefresh() {
    var arm = new Arm();
    arm.getIO().m_raw = Degrees.of(30);
    arm.readInputs();
    arm.zero();

    // Moved 10 degrees further along the raw sensor, which now reads 10 in the zeroed frame.
    arm.getIO().m_raw = Degrees.of(40);
    arm.readInputs();
    assertEquals(10.0, arm.getCurrentMeasurement().in(Degrees), 1e-9);

    // Deliberately without refreshing the stored value in between. Working the new offset out from
    // the stored one plus the reported position would read a number that is a loop stale and land on
    // 10 rather than 40, which is why the IO takes the raw reading instead.
    arm.zero();
    arm.readInputs();

    assertEquals(0.0, arm.getCurrentMeasurement().in(Degrees), 1e-9);
    assertEquals(40.0, arm.getIO().m_offset.in(Degrees), 1e-9);
  }

  @Test
  void theOffsetIsStoredWhereADashboardCanReadAndTypeIt() {
    var arm = new Arm();
    arm.getIO().m_raw = Degrees.of(12.5);
    arm.readInputs();
    arm.zero();

    // The unit is part of the path, so nobody has to guess degrees from radians -- see
    // LoggedTunableMeasure.
    var topic = NetworkTableInstance.getDefault().getDoubleTopic("/Tuning/Arm/ZeroOffset (Degree)");
    assertTrue(topic.exists(), "the offset should be published under the mechanism's name");
    assertEquals(12.5, topic.subscribe(0.0).get(), 1e-9);

    // And persistent, which is the whole reason it is a stored value rather than a field.
    assertTrue(arm.getZeroOffset().isPersistent());
  }

  @Test
  void anOffsetStoredBeforeTheRobotStartedReachesTheIo() {
    var inst = NetworkTableInstance.getDefault();

    try (var publisher = inst.getDoubleTopic("/Tuning/Arm/ZeroOffset (Degree)").publish()) {
      publisher.set(7.5);
      inst.flush();

      // What a reboot looks like: the mechanism is new, the stored offset is not. The constructor is
      // what has to carry it across, since nothing else would.
      var arm = new Arm();
      arm.getIO().m_raw = Degrees.of(7.5);
      arm.readInputs();

      assertEquals(7.5, arm.getIO().m_offset.in(Degrees), 1e-9);
      assertEquals(0.0, arm.getCurrentMeasurement().in(Degrees), 1e-9);
    }
  }

  @Test
  void aTargetIsGivenInTheZeroedFrameAndCommandedInRawTerms() {
    var arm = new Arm();
    arm.getIO().m_raw = Degrees.of(30);
    arm.readInputs();
    arm.zero();

    arm.setTarget(Degrees.of(10));
    arm.readInputs();

    // The caller said 10 and gets 10 back; the 40 the hardware was actually given never surfaces.
    assertEquals(10.0, arm.getTarget().in(Degrees), 1e-9);
    assertEquals(40.0, arm.getIO().m_commandedRaw.in(Degrees), 1e-9);
  }

  @Test
  void anOffsetTypedInSomeOtherUnitIsConvertedIntoThePublishedOne() {
    var arm = new Arm();

    arm.getZeroOffset().set(Rotations.of(0.25));
    arm.getZeroOffset().periodic();

    assertEquals(90.0, arm.getZeroOffset().get().in(Degrees), 1e-9);
  }

  @Test
  void theZeroButtonIsPublishedUnderTheMechanismsName() {
    new Arm();
    TunableRegistry.update();

    var inst = NetworkTableInstance.getDefault();
    String path = "/Tunables/" + ZeroableMechanism.kZeroCommandRoot + "/Arm";

    // The "Command" type string is what makes a dashboard draw a button rather than a checkbox;
    // TunableCommandTest covers the rest of that shape.
    assertEquals("Command", inst.getStringTopic(path + "/.type").subscribe("").get());
    assertEquals("Zero Arm", inst.getStringTopic(path + "/name").subscribe("").get());
  }

  @Test
  void twoMechanismsEachGetTheirOwnOffset() {
    var arm = new Arm();
    var otherArm = new OtherArm();

    assertNotSame(arm.getZeroOffset(), otherArm.getZeroOffset());

    // Keyed by name rather than shared: two mechanisms quietly sharing one zero is a bug you find on
    // the field.
    assertEquals("/Tuning/Arm/ZeroOffset (Degree)", arm.getZeroOffset().path());
    assertEquals("/Tuning/OtherArm/ZeroOffset (Degree)", otherArm.getZeroOffset().path());
  }
}
