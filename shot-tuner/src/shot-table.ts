/**
 * The shot table, as the robot's `ShotProfileTable` defines it: one row per range, holding everything
 * the shooter should do at that range.
 *
 * Kept free of any DOM or NetworkTables so the rules can be reasoned about on their own. The
 * validation here deliberately mirrors `ShotProfileTable.validate` on the robot — the robot remains the
 * authority and publishes its own verdict, but checking locally means the dashboard can refuse to send
 * a table it knows will be rejected, and can say why immediately instead of a cycle later.
 */

export type SeriesKey = 'tof' | 'hood' | 'rpm';

export interface Row {
  /** Slant range from the turret, in feet. */
  d: number;
  /** Time of flight, in seconds. */
  tof: number;
  /** Hood angle, in degrees. */
  hood: number;
  /** Flywheel speed, in RPM. */
  rpm: number;
}

export interface SeriesDef {
  key: SeriesKey;
  /** Topic name under the profile path. */
  topic: string;
  label: string;
  unit: string;
  /** Decimals shown, matching the precision the robot's Java export prints. */
  decimals: number;
  /** One drag notch / arrow-key press. */
  step: number;
  min?: number;
}

export const SERIES: readonly SeriesDef[] = [
  {
    key: 'tof',
    topic: 'TimeOfFlightSeconds',
    label: 'Time of flight',
    unit: 's',
    decimals: 3,
    step: 0.005,
    min: 0,
  },
  { key: 'hood', topic: 'HoodAngleDegrees', label: 'Hood angle', unit: '°', decimals: 2, step: 0.25 },
  { key: 'rpm', topic: 'FlywheelRpm', label: 'Flywheel', unit: 'RPM', decimals: 1, step: 5 },
] as const;

export const DISTANCE_TOPIC = 'DistancesFeet';

/** Ranges are added on a half-foot grid. Existing ranges off the grid are left alone — see README. */
export const RANGE_GRID = 0.5;

/** Matches `ShotProfileTable.kMinimumRows`: one row shoots the same shot from anywhere on the field. */
export const MINIMUM_ROWS = 2;

export interface Columns {
  d: number[];
  tof: number[];
  hood: number[];
  rpm: number[];
}

export type Validation = { ok: true } | { ok: false; reason: string };

export function columnsFromRows(rows: readonly Row[]): Columns {
  return {
    d: rows.map((r) => r.d),
    tof: rows.map((r) => r.tof),
    hood: rows.map((r) => r.hood),
    rpm: rows.map((r) => r.rpm),
  };
}

export function rowsFromColumns(columns: Columns): Row[] {
  return columns.d.map((d, i) => ({
    d,
    tof: columns.tof[i] ?? 0,
    hood: columns.hood[i] ?? 0,
    rpm: columns.rpm[i] ?? 0,
  }));
}

/**
 * The same rules `ShotProfileTable.validate` applies on the robot, in the same order.
 *
 * Column lengths are checked by the caller instead, because the dashboard holds rows rather than four
 * loose arrays — the torn-write case the robot guards against cannot arise on this side.
 */
export function validate(rows: readonly Row[]): Validation {
  if (rows.length < MINIMUM_ROWS) {
    return { ok: false, reason: `a table needs at least ${MINIMUM_ROWS} ranges, got ${rows.length}` };
  }

  for (let i = 0; i < rows.length; i++) {
    const row = rows[i]!;

    if (![row.d, row.tof, row.hood, row.rpm].every(Number.isFinite)) {
      return { ok: false, reason: `row ${i} holds a value that is not a finite number` };
    }

    if (row.d < 0) {
      return { ok: false, reason: `row ${i} has a negative range (${row.d.toFixed(2)} ft)` };
    }

    if (row.tof < 0) {
      return { ok: false, reason: `row ${i} has a negative flight time (${row.tof.toFixed(3)} s)` };
    }

    const previous = rows[i - 1];

    if (previous && row.d <= previous.d) {
      return {
        ok: false,
        reason:
          `ranges must strictly increase, but row ${i - 1} is ${previous.d.toFixed(2)} ft` +
          ` and row ${i} is ${row.d.toFixed(2)} ft`,
      };
    }
  }

  return { ok: true };
}

/**
 * The value a curve holds at a range, by linear interpolation, clamping outside the table.
 *
 * This is the robot's `InterpolatingDoubleTreeMap` behaviour, and it is what a new range's values
 * default to: dropping a range onto the curve should leave the curve exactly where it was.
 */
export function interpolate(rows: readonly Row[], key: SeriesKey, distance: number): number {
  if (rows.length === 0) return 0;

  const first = rows[0]!;
  const last = rows[rows.length - 1]!;

  if (distance <= first.d) return first[key];
  if (distance >= last.d) return last[key];

  for (let i = 1; i < rows.length; i++) {
    const hi = rows[i]!;
    const lo = rows[i - 1]!;

    if (distance <= hi.d) {
      const span = hi.d - lo.d;
      const t = span === 0 ? 0 : (distance - lo.d) / span;
      return lo[key] + t * (hi[key] - lo[key]);
    }
  }

  return last[key];
}

/** Where a range would be inserted, or the index of an existing range at that distance. */
export function indexOfRange(rows: readonly Row[], distance: number): number {
  return rows.findIndex((row) => nearlyEqual(row.d, distance));
}

/** Adds a range, taking each curve's value from where that curve already is. */
export function withRange(rows: readonly Row[], distance: number): Row[] {
  if (indexOfRange(rows, distance) >= 0) return [...rows];

  const added: Row = {
    d: distance,
    tof: round(interpolate(rows, 'tof', distance), SERIES[0]!.decimals),
    hood: round(interpolate(rows, 'hood', distance), SERIES[1]!.decimals),
    rpm: round(interpolate(rows, 'rpm', distance), SERIES[2]!.decimals),
  };

  return [...rows, added].sort((a, b) => a.d - b.d);
}

export function withoutRange(rows: readonly Row[], index: number): Row[] {
  return rows.filter((_, i) => i !== index);
}

/**
 * The value that puts one point exactly on the straight line between its two neighbours, flattening a
 * kink, or null where there is nothing to level against.
 *
 * Note this is the interpolation at that range, not the average of the two neighbouring values. They
 * agree only when the ranges either side are evenly spaced, and the committed table is not evenly spaced
 * — averaging would land off the line and leave a smaller kink behind rather than removing it.
 */
export function levelledValue(
  rows: readonly Row[],
  index: number,
  key: SeriesKey,
): number | null {
  const row = rows[index];

  if (!row || index <= 0 || index >= rows.length - 1) return null;

  return snapToStep(interpolate(withoutRange(rows, index), key, row.d), seriesDef(key));
}

export function withValue(rows: readonly Row[], index: number, key: SeriesKey, value: number): Row[] {
  return rows.map((row, i) => (i === index ? { ...row, [key]: value } : row));
}

export function seriesDef(key: SeriesKey): SeriesDef {
  return SERIES.find((s) => s.key === key)!;
}

export function snapToGrid(distance: number): number {
  return round(Math.round(distance / RANGE_GRID) * RANGE_GRID, 2);
}

/** Snaps an edited value to the series' notch, so tables stay as readable as the ones in source. */
export function snapToStep(value: number, def: SeriesDef, fine = false): number {
  const step = fine ? def.step / 5 : def.step;
  const snapped = Math.round(value / step) * step;
  const floored = def.min !== undefined ? Math.max(def.min, snapped) : snapped;

  return round(floored, def.decimals);
}

export function formatValue(value: number, def: SeriesDef): string {
  return value.toFixed(def.decimals);
}

export function formatRange(distance: number): string {
  return distance.toFixed(2);
}

/**
 * The table as the Java literal that declares it, byte-identical to what
 * `ShotProfileTable.toJava` produces on the robot.
 *
 * Duplicated rather than read from the robot's `AsJava` output so that copying out a table works
 * while disconnected, and so it reflects unsent edits. `ShotProfileTableTest` pins the Java side's
 * format; if the two ever drift, only the pasted formatting differs, never the values.
 */
export function toJava(rows: readonly Row[]): string {
  const lines = [
    'ShotProfileTable.of(',
    '    //   range    flight     hood    flywheel',
    '    //      ft         s      deg         rpm',
  ];

  rows.forEach((row, i) => {
    const cells = [
      row.d.toFixed(2).padStart(6),
      row.tof.toFixed(3).padStart(8),
      row.hood.toFixed(2).padStart(7),
      row.rpm.toFixed(1).padStart(10),
    ];

    lines.push(`    row(${cells.join(', ')})${i === rows.length - 1 ? ');' : ','}`);
  });

  return `${lines.join('\n')}\n`;
}

export function rowsEqual(a: readonly Row[], b: readonly Row[]): boolean {
  return (
    a.length === b.length &&
    a.every((row, i) => {
      const other = b[i]!;
      return row.d === other.d && row.tof === other.tof && row.hood === other.hood && row.rpm === other.rpm;
    })
  );
}

function nearlyEqual(a: number, b: number): boolean {
  return Math.abs(a - b) < 1e-9;
}

function round(value: number, decimals: number): number {
  const scale = 10 ** decimals;
  return Math.round(value * scale) / scale;
}
