/**
 * Tests for the table model: the rules a dashboard edit has to satisfy, and the arithmetic behind the
 * editing actions.
 */

import { describe, expect, test } from 'bun:test';

import {
  interpolate,
  levelledValue,
  toJava,
  validate,
  withRange,
  withValue,
  withoutRange,
  type Row,
} from './shot-table';

/** Evenly spaced, with a deliberate kink at 15 ft on the flywheel curve. */
const EVEN: Row[] = [
  { d: 10, tof: 1.0, hood: 5, rpm: 1800 },
  { d: 15, tof: 1.2, hood: 8, rpm: 2500 },
  { d: 20, tof: 1.4, hood: 10, rpm: 2200 },
];

/** Unevenly spaced: 12 ft is a quarter of the way from 10 to 18. */
const UNEVEN: Row[] = [
  { d: 10, tof: 1.0, hood: 5, rpm: 1000 },
  { d: 12, tof: 1.2, hood: 8, rpm: 1700 },
  { d: 18, tof: 1.4, hood: 10, rpm: 2000 },
];

describe('levelling a point', () => {
  test('puts it halfway between its neighbours when they are evenly spaced', () => {
    // 1800 -> 2200 across equal spacing, so the midpoint is 2000.
    expect(levelledValue(EVEN, 1, 'rpm')).toBe(2000);
  });

  test('puts it on the line, not at the average, when spacing is uneven', () => {
    // 12 ft is a quarter of the way from 10 to 18, so on the line is 1000 + 0.25 * 1000 = 1250.
    // The average of the neighbours would be 1500, which is not on the line at all.
    expect(levelledValue(UNEVEN, 1, 'rpm')).toBe(1250);
  });

  test('leaves a point that already sits on the line where it is', () => {
    const straight: Row[] = [
      { d: 10, tof: 1.0, hood: 5, rpm: 1000 },
      { d: 15, tof: 1.0, hood: 5, rpm: 1500 },
      { d: 20, tof: 1.0, hood: 5, rpm: 2000 },
    ];

    expect(levelledValue(straight, 1, 'rpm')).toBe(1500);
  });

  test('has nothing to level against at either end', () => {
    expect(levelledValue(EVEN, 0, 'rpm')).toBeNull();
    expect(levelledValue(EVEN, EVEN.length - 1, 'rpm')).toBeNull();
  });

  test('is out of range for an index that is not there', () => {
    expect(levelledValue(EVEN, 9, 'rpm')).toBeNull();
  });

  test('levels each curve on its own', () => {
    // The charts share their ranges, not their values, so levelling one must not touch the others.
    expect(levelledValue(EVEN, 1, 'hood')).toBe(7.5);
    expect(levelledValue(EVEN, 1, 'tof')).toBe(1.2);
  });

  test('snaps to the curve’s own step, like a drag does', () => {
    const awkward: Row[] = [
      { d: 10, tof: 1.0, hood: 5, rpm: 1001 },
      { d: 15, tof: 1.0, hood: 5, rpm: 9999 },
      { d: 20, tof: 1.0, hood: 5, rpm: 1008 },
    ];

    // Flywheel steps in 5 RPM: the line gives 1004.5, which snaps to 1005.
    expect(levelledValue(awkward, 1, 'rpm')).toBe(1005);
  });

  test('a levelled point leaves the curve straight through it', () => {
    const levelled = levelledValue(EVEN, 1, 'rpm')!;
    const after = withValue(EVEN, 1, 'rpm', levelled);

    // Reading the curve at that range with the point present or absent now gives the same answer.
    expect(interpolate(after, 'rpm', 15)).toBeCloseTo(interpolate(withoutRange(after, 1), 'rpm', 15), 9);
  });
});

describe('adding a range', () => {
  test('takes each value from where its own curve already was', () => {
    const grown = withRange(EVEN, 12.5);
    const added = grown.find((row) => row.d === 12.5)!;

    expect(grown.length).toBe(4);
    expect(added.rpm).toBeCloseTo(2150, 6);
    expect(added.hood).toBeCloseTo(6.5, 6);
  });

  test('leaves the curves where they were', () => {
    const grown = withRange(EVEN, 12.5);

    for (const at of [10, 11, 13, 17, 20]) {
      expect(interpolate(grown, 'rpm', at)).toBeCloseTo(interpolate(EVEN, 'rpm', at), 6);
    }
  });

  test('keeps ranges in order wherever it is inserted', () => {
    expect(withRange(EVEN, 4).map((row) => row.d)).toEqual([4, 10, 15, 20]);
    expect(withRange(EVEN, 30).map((row) => row.d)).toEqual([10, 15, 20, 30]);
    expect(withRange(EVEN, 17.5).map((row) => row.d)).toEqual([10, 15, 17.5, 20]);
  });

  test('is a no-op at a range that already exists', () => {
    expect(withRange(EVEN, 15)).toEqual(EVEN);
  });
});

describe('validation mirrors the robot', () => {
  test('accepts a well-formed table', () => {
    expect(validate(EVEN).ok).toBe(true);
  });

  test('rejects fewer than two ranges', () => {
    expect(validate(EVEN.slice(0, 1)).ok).toBe(false);
  });

  test('rejects duplicate and out-of-order ranges', () => {
    const duplicate = EVEN.map((row, i) => (i === 1 ? { ...row, d: 10 } : row));
    const outOfOrder = EVEN.map((row, i) => (i === 1 ? { ...row, d: 25 } : row));

    expect(validate(duplicate).ok).toBe(false);
    expect(validate(outOfOrder).ok).toBe(false);
  });

  test('rejects values that are not finite', () => {
    expect(validate(withValue(EVEN, 1, 'rpm', Number.NaN)).ok).toBe(false);
    expect(validate(withValue(EVEN, 1, 'tof', Number.POSITIVE_INFINITY)).ok).toBe(false);
  });

  test('rejects a negative flight time', () => {
    expect(validate(withValue(EVEN, 1, 'tof', -0.5)).ok).toBe(false);
  });
});

describe('the Java export', () => {
  test('rounds to the precision it prints and closes the call', () => {
    const java = toJava(EVEN);

    expect(java.startsWith('ShotProfileTable.of(')).toBe(true);
    expect(java.trim().endsWith(');')).toBe(true);
    expect(java).toContain('row( 10.00,    1.000,    5.00,     1800.0),');
  });
});
