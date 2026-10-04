package first.robot.util;

import static first.robot.util.ShotProfileTable.row;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.lib.util.InterpolatingMeasureTreeMap;

/**
 * The empirical tuning for one way of shooting: how long fuel is in the air at a given range, and
 * what the hood and flywheel should do to put it there.
 *
 * <p>Separating this from {@link ShotCalculationUtil} keeps the tables as data and the trajectory
 * solving as behaviour, so a second way of shooting — a lower, flatter pass, say — is a new profile
 * rather than a change to the solver.
 *
 * <p>The tables are declared as a {@link ShotProfileTable}, one row per range, and converted into the
 * three curves the solver looks up. That is the form a dashboard edits (see {@link TunableShotProfile})
 * and the form {@link ShotProfileTable#toJava} exports back to, so a tuning session ends as a diff to
 * this file.
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

  /**
   * Scoring fuel in the hub.
   *
   * <p>The ranges are the union of the breakpoints the three curves were tuned with separately, so
   * putting them on one axis left every curve unchanged — the interpolated values in between differ by
   * at most 0.5 ms of flight time and 0.03 RPM, which come from rounding the printed numbers rather
   * than from the shared axis. Ranges need not sit on the dashboard's half-foot grid; 6.70, 10.40 and
   * 15.20 ft are where the time-of-flight curve actually bends.
   */
  public static final ShotProfileTable kScoringTable = ShotProfileTable.of(
      //   range    flight     hood    flywheel
      //      ft         s      deg         rpm
      row(4.00, 0.900, 0.00, 1575.0),
      row(5.00, 0.900, 3.00, 1575.0),
      row(6.00, 0.900, 4.50, 1625.0),
      row(6.70, 0.900, 6.25, 1695.0),
      row(7.00, 0.941, 7.00, 1725.0),
      row(8.00, 1.076, 8.00, 1725.0),
      row(9.00, 1.211, 8.00, 1825.0),
      row(10.00, 1.346, 9.00, 1825.0),
      row(10.40, 1.400, 9.00, 1855.0),
      row(11.00, 1.400, 9.00, 1900.0),
      row(12.00, 1.400, 10.00, 1925.0),
      row(13.00, 1.400, 10.00, 2075.0),
      row(14.00, 1.400, 10.00, 2125.0),
      row(15.00, 1.400, 10.00, 2175.0),
      row(15.20, 1.400, 10.00, 2195.0),
      row(16.00, 1.433, 10.00, 2275.0),
      row(20.00, 1.600, 10.00, 2452.8),
      row(25.00, 1.600, 10.00, 2675.0));

  /** The committed actuation latency for {@link #kScoringTable}. */
  public static final Time kScoringActuationLatency = Seconds.of(0.03);

  /**
   * Scoring fuel in the hub, exactly as committed.
   *
   * <p>This is the reference table, not the one the robot shoots with — see
   * {@code Tuning.kScoringShotProfile} for that. Tests use it to pin solver behaviour against a table
   * no dashboard can move.
   */
  public static final ShotProfile kScoring = kScoringTable.toProfile(kScoringActuationLatency);
}
