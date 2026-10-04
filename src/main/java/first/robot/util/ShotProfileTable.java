package first.robot.util;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.function.ToDoubleFunction;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.lib.util.InterpolatingMeasureTreeMap;

/**
 * A shot table as rows rather than as three independent curves: one range, and everything the shooter
 * should do at that range.
 *
 * <p>The three curves in a {@link ShotProfile} could each carry their own breakpoints, and originally
 * did. Sharing one set of ranges instead costs nothing — a piecewise-linear curve resampled onto a
 * superset of its own breakpoints is the same curve — and buys two things. A dashboard can offer one
 * "add a range" action that grows all three plots together, and a row is a thing you can reason about
 * physically: at 13 ft the fuel is in the air 1.4 s, the hood sits at 10 degrees and the flywheel runs
 * 2075 RPM.
 *
 * <h2>Units are the wire format</h2>
 *
 * <p>Rows hold plain doubles in the units a person tunes in — feet, seconds, degrees, RPM — because
 * those are also the units the dashboard publishes and the units the table is declared in. Converting
 * only at {@link #applyTo} keeps the number someone drags, the number in NetworkTables and the number
 * in a code review identical, which is worth more here than wrapping each column in a
 * {@code Measure}. Typed accessors on {@link Row} are there for anything that needs one.
 *
 * <h2>Validation</h2>
 *
 * <p>{@link #validate} is the whole trust boundary between a dashboard and the shooter, so it is a
 * static function over raw arrays with no NetworkTables anywhere near it. See its notes for what a
 * live table has to satisfy and why.
 */
public final class ShotProfileTable {
  /** The unit the distance column is declared, published and exported in. */
  public static final DistanceUnit kDistanceUnit = Feet;

  /** The unit the time-of-flight column is declared, published and exported in. */
  public static final TimeUnit kTimeUnit = Seconds;

  /** The unit the hood angle column is declared, published and exported in. */
  public static final AngleUnit kAngleUnit = Degrees;

  /** The unit the flywheel column is declared, published and exported in. */
  public static final AngularVelocityUnit kAngularVelocityUnit = RPM;

  /**
   * A table of one row interpolates to the same values at every range, which is never what someone
   * meant to build and would silently shoot one fixed shot from anywhere on the field.
   */
  private static final int kMinimumRows = 2;

  /**
   * One range and the shot to take at it, in wire units.
   *
   * @param distanceFeet slant range from the turret to the target
   * @param timeOfFlightSeconds how long fuel is in the air over that range
   * @param hoodAngleDegrees hood angle for that range
   * @param flywheelRpm flywheel speed for that range
   */
  public record Row(
      double distanceFeet,
      double timeOfFlightSeconds,
      double hoodAngleDegrees,
      double flywheelRpm) {

    public Distance distance() {
      return kDistanceUnit.of(distanceFeet);
    }

    public Time timeOfFlight() {
      return kTimeUnit.of(timeOfFlightSeconds);
    }

    public Angle hoodAngle() {
      return kAngleUnit.of(hoodAngleDegrees);
    }

    public AngularVelocity flywheelVelocity() {
      return kAngularVelocityUnit.of(flywheelRpm);
    }
  }

  /**
   * The outcome of checking a dashboard snapshot: either a usable table, or the reason there isn't
   * one, in words that can be shown to whoever is tuning.
   *
   * @param table the table, if the snapshot was usable
   * @param rejection why it was not, or empty when it was
   */
  public record Result(Optional<ShotProfileTable> table, String rejection) {
    public boolean isAccepted() {
      return table.isPresent();
    }
  }

  private final List<Row> m_rows;

  private ShotProfileTable(List<Row> rows) {
    m_rows = List.copyOf(rows);
  }

  /** A row in wire units: feet, seconds, degrees, RPM. */
  public static Row row(
      double distanceFeet, double timeOfFlightSeconds, double hoodAngleDegrees, double flywheelRpm) {
    return new Row(distanceFeet, timeOfFlightSeconds, hoodAngleDegrees, flywheelRpm);
  }

  /**
   * A table declared in source, held to exactly the rules a dashboard table is held to.
   *
   * @throws IllegalArgumentException if the rows would be rejected coming from a dashboard, e.g. a
   *     range typed out of order
   */
  public static ShotProfileTable of(Row... rows) {
    var table = List.of(rows);
    var result = validate(
        column(table, Row::distanceFeet),
        column(table, Row::timeOfFlightSeconds),
        column(table, Row::hoodAngleDegrees),
        column(table, Row::flywheelRpm));

    return result.table().orElseThrow(
        () -> new IllegalArgumentException("Invalid shot table: " + result.rejection()));
  }

  /**
   * Checks a set of dashboard columns and turns them into a table.
   *
   * <p>Every rule here exists because breaking it fails quietly rather than loudly:
   *
   * <ul>
   *   <li><b>Equal lengths.</b> The four columns are separate NetworkTables topics with no promise of
   *       arriving in the same packet, so a dashboard adding a row can be observed part-way through.
   *       That is a normal, frequent event while someone is editing, not an error — rejecting it and
   *       keeping the previous table costs one cycle, which nobody can see.
   *   <li><b>At least {@value #kMinimumRows} rows.</b> See {@link #kMinimumRows}.
   *   <li><b>Finite values.</b> A {@code NaN} range breaks the ordering the interpolating tree map
   *       relies on, and a {@code NaN} value would be commanded straight to a mechanism.
   *   <li><b>Strictly increasing ranges.</b> Duplicates collapse two rows into one inside the tree
   *       map, so a row would vanish from under the dashboard with no indication. Re-sorting instead
   *       is worse: the dashboard's row order and the robot's would silently disagree.
   *   <li><b>No negative ranges or flight times.</b> A negative flight time makes the solver in
   *       {@link ShotCalculationUtil} lead the shot the wrong way, which looks like a tracking bug
   *       rather than a bad table.
   * </ul>
   *
   * <p>Deliberately <em>not</em> checked: whether the values are physically sensible for this robot.
   * Clamping a hood angle or a flywheel speed here would block the experiments the tuning is for; that
   * belongs to the mechanisms, which have to enforce their own limits anyway.
   */
  public static Result validate(
      double[] distancesFeet,
      double[] timeOfFlightSeconds,
      double[] hoodAngleDegrees,
      double[] flywheelRpm) {
    int rows = distancesFeet.length;

    if (timeOfFlightSeconds.length != rows
        || hoodAngleDegrees.length != rows
        || flywheelRpm.length != rows) {
      return rejected(String.format(
          Locale.ROOT,
          "column lengths disagree (distance %d, flight %d, hood %d, flywheel %d)"
              + " — usually a dashboard write still in progress",
          rows, timeOfFlightSeconds.length, hoodAngleDegrees.length, flywheelRpm.length));
    }

    if (rows < kMinimumRows) {
      return rejected(String.format(
          Locale.ROOT, "a table needs at least %d rows, got %d", kMinimumRows, rows));
    }

    for (int i = 0; i < rows; i++) {
      if (!Double.isFinite(distancesFeet[i])
          || !Double.isFinite(timeOfFlightSeconds[i])
          || !Double.isFinite(hoodAngleDegrees[i])
          || !Double.isFinite(flywheelRpm[i])) {
        return rejected("row " + i + " holds a value that is not a finite number");
      }

      if (distancesFeet[i] < 0.0) {
        return rejected(String.format(
            Locale.ROOT, "row %d has a negative range (%.3f ft)", i, distancesFeet[i]));
      }

      if (timeOfFlightSeconds[i] < 0.0) {
        return rejected(String.format(
            Locale.ROOT, "row %d has a negative flight time (%.3f s)", i, timeOfFlightSeconds[i]));
      }

      if (i > 0 && distancesFeet[i] <= distancesFeet[i - 1]) {
        return rejected(String.format(
            Locale.ROOT,
            "ranges must strictly increase, but row %d is %.3f ft and row %d is %.3f ft",
            i - 1, distancesFeet[i - 1], i, distancesFeet[i]));
      }
    }

    var built = new ArrayList<Row>(rows);

    for (int i = 0; i < rows; i++) {
      built.add(new Row(
          distancesFeet[i], timeOfFlightSeconds[i], hoodAngleDegrees[i], flywheelRpm[i]));
    }

    return new Result(Optional.of(new ShotProfileTable(built)), "");
  }

  public List<Row> rows() {
    return m_rows;
  }

  public double[] distanceColumn() {
    return column(m_rows, Row::distanceFeet);
  }

  public double[] timeOfFlightColumn() {
    return column(m_rows, Row::timeOfFlightSeconds);
  }

  public double[] hoodAngleColumn() {
    return column(m_rows, Row::hoodAngleDegrees);
  }

  public double[] flywheelColumn() {
    return column(m_rows, Row::flywheelRpm);
  }

  /**
   * Rewrites a profile's three tables in place to match this one.
   *
   * <p>The profile record and the maps it holds keep their identities, so anything already holding the
   * profile — a command mid-execution, say — picks up the new tables on its next lookup without being
   * handed a new object. Each curve is swapped whole (see
   * {@link InterpolatingMeasureTreeMap#replaceAll}), so no reader can catch one in between.
   */
  public void applyTo(ShotProfile profile) {
    profile.timeOfFlight().replaceAll(
        map -> m_rows.forEach(row -> map.put(row.distance(), row.timeOfFlight())));
    profile.hoodAngle().replaceAll(
        map -> m_rows.forEach(row -> map.put(row.distance(), row.hoodAngle())));
    profile.flywheelVelocity().replaceAll(
        map -> m_rows.forEach(row -> map.put(row.distance(), row.flywheelVelocity())));
  }

  /** Builds a fresh profile from this table. */
  public ShotProfile toProfile(Time actuationLatency) {
    var profile = new ShotProfile(
        actuationLatency,
        InterpolatingMeasureTreeMap.distanceToTime(),
        InterpolatingMeasureTreeMap.distanceToAngle(),
        InterpolatingMeasureTreeMap.distanceToAngularVelocity());

    applyTo(profile);

    return profile;
  }

  /**
   * This table as the Java literal that declares it, ready to paste over the one in
   * {@link ShotProfile}.
   *
   * <p>This is how a tuning session ends. Without it the only record of an afternoon's work is the
   * robot's {@code networktables.json}, which does not survive re-imaging the robot, is not in version
   * control, and cannot be reviewed or reverted.
   *
   * <p>Values are rounded to the precision printed, so re-importing an export is faithful to within
   * 0.0005 s of flight time, 0.005 degrees of hood angle and 0.05 RPM — far below anything the
   * hardware resolves.
   */
  public String toJava() {
    var java = new StringBuilder("ShotProfileTable.of(\n");
    java.append("    //   range    flight     hood    flywheel\n");
    java.append("    //      ft         s      deg         rpm\n");

    for (int i = 0; i < m_rows.size(); i++) {
      Row row = m_rows.get(i);
      java.append(String.format(
          Locale.ROOT,
          "    row(%6.2f, %8.3f, %7.2f, %10.1f)%s%n",
          row.distanceFeet(),
          row.timeOfFlightSeconds(),
          row.hoodAngleDegrees(),
          row.flywheelRpm(),
          i == m_rows.size() - 1 ? ");" : ","));
    }

    return java.toString();
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof ShotProfileTable table && m_rows.equals(table.m_rows);
  }

  @Override
  public int hashCode() {
    return Objects.hashCode(m_rows);
  }

  @Override
  public String toString() {
    return "ShotProfileTable" + m_rows;
  }

  private static Result rejected(String reason) {
    return new Result(Optional.empty(), reason);
  }

  private static double[] column(List<Row> rows, ToDoubleFunction<Row> field) {
    return rows.stream().mapToDouble(field).toArray();
  }
}
