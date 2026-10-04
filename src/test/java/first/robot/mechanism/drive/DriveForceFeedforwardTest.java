package first.robot.mechanism.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Newtons;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;

/**
 * Pins the arithmetic that turns a wanted traction force into a motor command.
 *
 * <p>Worth its own test because the failure mode is invisible. Each of the three conversions can be
 * inverted or omitted, and every such mistake scales the result by a constant -- which is
 * indistinguishable on a robot from a feedforward that was simply tuned a bit low or a bit high. The
 * velocity controller absorbs the difference, the path still gets followed, and nothing ever points
 * at the arithmetic.
 *
 * <p>Two of these are worse than invisible, because they can come out nearly right by coincidence.
 * Multiplying by the reduction instead of dividing, and then passing newton-metres into a field that
 * wants amps, overshoots and undershoots by factors whose product is {@code G^2 * Kt} -- which for a
 * 6.75:1 module on a Kraken X60 lands at 0.89, a ten percent error that reads as good tuning. So
 * these tests assert the <em>direction</em> of each conversion and not only the final number.
 */
class DriveForceFeedforwardTest {
  private static final Distance kWheelRadius = Inches.of(2.167);
  private static final double kGearRatio = 7.363636363636365;
  private static final DCMotor kMotor = DCMotor.getKrakenX60Foc(1);

  private static final DriveForceFeedforward kFeedforward =
      new DriveForceFeedforward(kWheelRadius, kGearRatio, kMotor);

  /**
   * The whole chain against numbers worked out by hand: 100 N on a 2.167 in wheel is 5.5042 Nm at
   * the wheel, 0.74748 Nm at the motor through a 7.3636:1 reduction, and 38.531 A at the Kraken's Kt
   * of 0.0193996 Nm/A.
   */
  @Test
  void aKnownForceBecomesTheCurrentThatProducesIt() {
    assertEquals(38.5308, kFeedforward.forMotorCurrent(Newtons.of(100.0)).in(Amps), 1.0e-3);
  }

  /** The same chain plus {@code I * R}, at the Kraken's winding resistance of 0.0248447 ohms. */
  @Test
  void aKnownForceBecomesTheVoltageThatProducesIt() {
    assertEquals(0.957287, kFeedforward.forMotorVoltage(Newtons.of(100.0)).in(Volts), 1.0e-5);
  }

  /**
   * The conversion that is easiest to get backwards, and the one that costs the most when it is. A
   * gearbox multiplies torque, so a taller reduction means the motor has <em>less</em> to do for the
   * same force at the wheel. Multiplying where this divides inflates the command by the square of
   * the ratio once the units are also wrong.
   */
  @Test
  void aTallerReductionNeedsLessCurrentForTheSameForce() {
    double standard = kFeedforward.forMotorCurrent(Newtons.of(100.0)).in(Amps);
    double doubled = new DriveForceFeedforward(kWheelRadius, kGearRatio * 2.0, kMotor)
        .forMotorCurrent(Newtons.of(100.0)).in(Amps);

    assertTrue(doubled < standard,
        "doubling the reduction should halve the current, not double it: %f became %f"
            .formatted(standard, doubled));
    assertEquals(standard / 2.0, doubled, 1.0e-9);
  }

  /** The wheel is the lever the force acts on, so a bigger one costs more torque. */
  @Test
  void aBiggerWheelNeedsMoreCurrentForTheSameForce() {
    double standard = kFeedforward.forMotorCurrent(Newtons.of(100.0)).in(Amps);
    double bigger = new DriveForceFeedforward(kWheelRadius.times(2.0), kGearRatio, kMotor)
        .forMotorCurrent(Newtons.of(100.0)).in(Amps);

    assertEquals(standard * 2.0, bigger, 1.0e-9);
  }

  /**
   * The result has to be a current rather than a torque. A newton-metre handed to a torque current
   * request is off by 1/Kt, which for this motor is a factor of about fifty.
   */
  @Test
  void theResultIsACurrentAndNotATorque() {
    double motorTorqueNm = 100.0 * kWheelRadius.in(Meters) / kGearRatio;

    assertEquals(motorTorqueNm / kMotor.Kt, kFeedforward.forMotorCurrent(Newtons.of(100.0)).in(Amps),
        1.0e-9);
    assertTrue(kFeedforward.forMotorCurrent(Newtons.of(100.0)).in(Amps) > motorTorqueNm * 10.0,
        "a current is far larger than the torque it produces; this looks like the torque");
  }

  /**
   * A braking force is a negative force, and has to come out as a negative command. The module
   * projects onto its actual azimuth, so a reversed setpoint arrives here already signed, and
   * anything that lost the sign would turn braking into acceleration.
   */
  @Test
  void aNegativeForceBecomesANegativeCommand() {
    assertEquals(-38.5308, kFeedforward.forMotorCurrent(Newtons.of(-100.0)).in(Amps), 1.0e-3);
    assertEquals(-0.957287, kFeedforward.forMotorVoltage(Newtons.of(-100.0)).in(Volts), 1.0e-5);
  }

  /** No force wanted, nothing fed forward -- the case every teleop loop takes. */
  @Test
  void noForceAsksForNothing() {
    assertEquals(0.0, kFeedforward.forMotorCurrent(Newtons.zero()).in(Amps));
    assertEquals(0.0, kFeedforward.forMotorVoltage(Newtons.zero()).in(Volts));
  }

  /** Twice the force is twice the current, with nothing hiding a squared term. */
  @Test
  void currentIsLinearInForce() {
    double single = kFeedforward.forMotorCurrent(Newtons.of(100.0)).in(Amps);
    double doubled = kFeedforward.forMotorCurrent(Newtons.of(200.0)).in(Amps);

    assertEquals(single * 2.0, doubled, 1.0e-9);
  }
}
