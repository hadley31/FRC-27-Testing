package first.lib.mechanism.angle;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Covers the two sign conversions in this IO, against a simulated CANcoder.
 *
 * <p>They are the whole risk in the class: {@code MagnetOffset} is <em>added</em> to what the sensor
 * reports while {@link first.lib.mechanism.ZeroableMechanismIO}'s offset is taken away from it, and
 * the two are also in different rotations. Get either backwards and a zeroed mechanism runs to twice
 * its error in the wrong direction, which is not something to find out on a field.
 */
class TalonFXFusedCANcoderAngleMechanismIOTest {
  private static final CANBus kBus = new CANBus("sim");

  /** Ids nothing else in the test suite claims. */
  private static final int kMotorId = 40;
  private static final int kEncoderId = 41;

  /**
   * The CANcoder quantizes {@code MagnetOffset} to 1/4096 of a rotation, so a value written here
   * comes back rounded. Asserting to one count is asserting everything the device can actually hold.
   */
  private static final double kMagnetOffsetResolution = 1.0 / 4096.0;

  @BeforeAll
  static void initHal() {
    // Phoenix needs the HAL up before it will bring its simulated CAN bus up.
    HAL.initialize();
  }

  @Test
  void aStoredOffsetIsWrittenToTheMagnetWithTheSignTheSensorExpects() {
    var io = new TalonFXFusedCANcoderAngleMechanismIO(kMotorId, kEncoderId, kBus, 25.0, 1.0);

    io.setZeroOffset(Degrees.of(10));

    // Negative: the mechanism reads 10 degrees high, so the sensor has to report 10 degrees less.
    assertEquals(
        Degrees.of(-10).in(Rotations),
        magnetOffsetOnTheDevice(),
        kMagnetOffsetResolution,
        "a positive offset should trim the magnet in the negative direction");
  }

  @Test
  void theRatioScalesTheOffsetIntoTheSensorsOwnRotations() {
    var io = new TalonFXFusedCANcoderAngleMechanismIO(kMotorId, kEncoderId, kBus, 25.0, 2.0);

    // Two sensor rotations per mechanism rotation, so a mechanism angle is twice as many sensor
    // rotations -- the conversion MagnetOffset is written in.
    io.setZeroOffset(Degrees.of(10));

    assertEquals(
        Degrees.of(-20).in(Rotations),
        magnetOffsetOnTheDevice(),
        kMagnetOffsetResolution,
        "the offset should be scaled by sensorToMechanismRatio");
  }

  @Test
  void zeroingAMechanismAlreadyReadingZeroLeavesTheOffsetWhereItIs() {
    var io = new TalonFXFusedCANcoderAngleMechanismIO(kMotorId, kEncoderId, kBus, 25.0, 1.0);

    io.setZeroOffset(Degrees.of(10));

    // The simulated mechanism sits at zero, so there is nothing to correct and zero() must be a
    // fixed point. That makes this a test of both conversions at once: it only comes back as 10 if
    // the offset written out and the offset read back use opposite signs and the same ratio.
    var offset = io.zero();

    assertEquals(
        10.0,
        offset.in(Degrees),
        kMagnetOffsetResolution * 360.0,
        "zeroing a mechanism that already reads zero should not move the frame");
  }

  @Test
  void aNegativeOffsetTrimsTheOtherWay() {
    var io = new TalonFXFusedCANcoderAngleMechanismIO(kMotorId, kEncoderId, kBus, 25.0, 1.0);

    io.setZeroOffset(Degrees.of(-7.5));

    assertTrue(
        magnetOffsetOnTheDevice() > 0.0,
        "a negative offset should trim the magnet in the positive direction");
    assertEquals(
        Degrees.of(7.5).in(Rotations), magnetOffsetOnTheDevice(), kMagnetOffsetResolution);
  }

  /**
   * The magnet offset as the device holds it, read through a separate handle to the same CANcoder so
   * the assertion goes to the hardware rather than to anything the IO kept.
   */
  private static double magnetOffsetOnTheDevice() {
    var magnet = new MagnetSensorConfigs();
    new CANcoder(kEncoderId, kBus).getConfigurator().refresh(magnet);
    return magnet.MagnetOffset;
  }
}
