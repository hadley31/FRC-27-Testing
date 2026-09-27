package first.robot.util;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.lib.InterpolatingMeasureTreeMap;

/**
 * The empirical tuning for one way of shooting: how long fuel is in the air at a given range, and
 * what the hood and flywheel should do to put it there.
 *
 * <p>Separating this from {@link ShotCalculationUtil} keeps the tables as data and the trajectory
 * solving as behaviour, so a second way of shooting — a lower, flatter pass, say — is a new profile
 * rather than a change to the solver.
 *
 * @param actuationLatency how far ahead of the current state a shot should be aimed, covering the
 *     lag between commanding the mechanisms and fuel actually leaving
 * @param timeOfFlight range to the time fuel spends in the air
 * @param hoodAngle range to hood angle
 * @param flywheelVelocity range to flywheel speed
 */
public record ShotProfile(
    Time actuationLatency,
    InterpolatingMeasureTreeMap<Distance, Time> timeOfFlight,
    InterpolatingMeasureTreeMap<Distance, Angle> hoodAngle,
    InterpolatingMeasureTreeMap<Distance, AngularVelocity> flywheelVelocity) {

  /** Scoring fuel in the hub. */
  public static final ShotProfile kScoring = new ShotProfile(
      Seconds.of(0.03),
      InterpolatingMeasureTreeMap.distanceToTime()
          .put(Feet.of(6.7), Seconds.of(0.9))
          .put(Feet.of(10.4), Seconds.of(1.4))
          .put(Feet.of(15.2), Seconds.of(1.4))
          .put(Feet.of(20), Seconds.of(1.6)),
      InterpolatingMeasureTreeMap.distanceToAngle()
          .put(Feet.of(4), Degrees.of(0))
          .put(Feet.of(5), Degrees.of(5 - 2))
          .put(Feet.of(6), Degrees.of(6.5 - 2))
          .put(Feet.of(7), Degrees.of(9 - 2))
          .put(Feet.of(8), Degrees.of(10 - 2))
          .put(Feet.of(9), Degrees.of(10 - 2))
          .put(Feet.of(10), Degrees.of(11 - 2))
          .put(Feet.of(11), Degrees.of(11 - 2))
          .put(Feet.of(12), Degrees.of(12 - 2))
          .put(Feet.of(13), Degrees.of(12 - 2))
          .put(Feet.of(14), Degrees.of(12 - 2))
          .put(Feet.of(15), Degrees.of(12 - 2))
          .put(Feet.of(16), Degrees.of(12 - 2)),
      InterpolatingMeasureTreeMap.distanceToAngularVelocity()
          .put(Feet.of(4), RPM.of(1450 + 50 + 25 + 50))
          .put(Feet.of(5), RPM.of(1450 + 50 + 25 + 50))
          .put(Feet.of(6), RPM.of(1500 + 50 + 25 + 50))
          .put(Feet.of(7), RPM.of(1600 + 50 + 25 + 50))
          .put(Feet.of(8), RPM.of(1600 + 50 + 25 + 50))
          .put(Feet.of(9), RPM.of(1700 + 50 + 25 + 50))
          .put(Feet.of(10), RPM.of(1700 + 50 + 25 + 50))
          .put(Feet.of(11), RPM.of(1800 + 50 + 50))
          .put(Feet.of(12), RPM.of(1850 + 25 + 50))
          .put(Feet.of(13), RPM.of(2000 + 25 + 50))
          .put(Feet.of(14), RPM.of(2050 + 25 + 50))
          .put(Feet.of(15), RPM.of(2100 + 25 + 50))
          .put(Feet.of(16), RPM.of(2200 + 25 + 50))
          .put(Feet.of(25), RPM.of(2600 + 25 + 50)));
}
