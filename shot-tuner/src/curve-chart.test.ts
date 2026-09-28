/**
 * Regression tests for the chart's pointer handling, which is where the subtle bugs live.
 *
 * The failure these exist for: a point could be edited by *hovering*, with no button held, and the value
 * that moved belonged to a different range than the one under the cursor. Both halves are covered below.
 */

import './dom-test-setup';

import type { Row } from './shot-table';

// Imported dynamically so the DOM globals above are in place first.
const { afterEach, beforeEach, describe, expect, test } = await import('bun:test');
const { CurveChart } = await import('./curve-chart');
const { SERIES } = await import('./shot-table');

const FLYWHEEL = SERIES[2]!;

/** Two ranges close enough together that per-point hit areas would overlap, plus one far away. */
const ROWS: Row[] = [
  { d: 10, tof: 1.0, hood: 5, rpm: 1800 },
  { d: 19.6, tof: 1.5, hood: 10, rpm: 2400 },
  { d: 20, tof: 1.5, hood: 10, rpm: 2600 },
];

interface Recorded {
  starts: number;
  ends: number;
  edits: { index: number; value: number }[];
  added: number[];
  removed: number[];
  levelled: number[];
  selected: (number | null)[];
}

let host: HTMLElement;
let chart: InstanceType<typeof CurveChart>;
let log: Recorded;

function dotAt(index: number): { x: number; y: number } {
  const dots = host.querySelectorAll('.point');
  const group = dots[index]!;
  const dot = group.querySelector('.point-dot')!;
  return { x: Number(dot.getAttribute('cx')), y: Number(dot.getAttribute('cy')) };
}

/** Dispatches a pointer event the way a browser would, with only the fields the chart reads. */
function pointer(
  type: string,
  at: { x: number; y: number },
  buttons: number,
  options: { shift?: boolean; target?: Element } = {},
): void {
  const element = options.target ?? host.querySelector('.chart-hit')!;
  const event = new Event(type, { bubbles: true, cancelable: true });
  Object.assign(event, {
    clientX: at.x,
    clientY: at.y,
    buttons,
    isPrimary: true,
    shiftKey: options.shift ?? false,
    pointerId: 1,
  });
  element.dispatchEvent(event);
}

/** Presses and releases a point, which is how it becomes the keyboard's target. */
function click(index: number): void {
  const at = dotAt(index);
  pointer('pointerdown', at, 1);
  pointer('pointerup', at, 0);
}

function key(name: string, options: { shift?: boolean; meta?: boolean } = {}): void {
  const event = new Event('keydown', { bubbles: true, cancelable: true });
  Object.assign(event, {
    key: name,
    shiftKey: options.shift ?? false,
    metaKey: options.meta ?? false,
    ctrlKey: false,
    altKey: false,
    preventDefault() {},
  });
  window.dispatchEvent(event);
}

beforeEach(() => {
  document.body.innerHTML = '<div id="host"></div>';
  host = document.getElementById('host')!;
  log = { starts: 0, ends: 0, edits: [], added: [], removed: [], levelled: [], selected: [] };

  chart = new CurveChart(
    host,
    FLYWHEEL,
    {
      onHover() {},
      onEditStart() { log.starts++; },
      onEdit(index, value) { log.edits.push({ index, value }); },
      onEditEnd() { log.ends++; },
      onAddRange(distance) { log.added.push(distance); },
      onRemoveRange(index) { log.removed.push(index); },
      onLevelRange(index) { log.levelled.push(index); },
      onSelect(distance) { log.selected.push(distance); },
    },
    true,
  );

  chart.setEnabled(true);
  chart.setData(ROWS, [9, 21], [10, 15, 20]);
});

// Charts keep window listeners for releases that land outside them, so each one has to go away with
// its test or a later release would reach every chart built so far.
afterEach(() => chart.destroy());

describe('hovering never edits', () => {
  test('moving over a point with no button held changes nothing', () => {
    pointer('pointermove', dotAt(2), 0);

    expect(log.edits).toEqual([]);
    expect(log.starts).toBe(0);
  });

  test('moving across every point with no button held changes nothing', () => {
    for (let i = 0; i < ROWS.length; i++) pointer('pointermove', dotAt(i), 0);

    expect(log.edits).toEqual([]);
  });

  test('a drag whose release was missed does not turn later hovering into edits', () => {
    // The original bug. Pressing captured the pointer on the point's own element, but every edit
    // re-renders and replaces that element, which silently released the capture — so the release landed
    // somewhere that never ended the drag, and the drag stayed live forever.
    const point = dotAt(2);
    pointer('pointerdown', point, 1);
    pointer('pointermove', { x: point.x, y: point.y - 20 }, 1);

    expect(log.edits.length).toBe(1);

    // Re-render, as an accepted edit does, then hover elsewhere with no button held.
    chart.setData(ROWS, [9, 21], [10, 15, 20]);
    const editsAfterDrag = log.edits.length;

    pointer('pointermove', { x: dotAt(0).x, y: dotAt(0).y - 40 }, 0);
    pointer('pointermove', { x: dotAt(1).x, y: dotAt(1).y + 30 }, 0);

    expect(log.edits.length).toBe(editsAfterDrag);
    expect(log.ends).toBeGreaterThan(0);
  });

  test('a release anywhere ends the drag', () => {
    const point = dotAt(2);
    pointer('pointerdown', point, 1);
    pointer('pointermove', { x: point.x, y: point.y - 10 }, 1);

    // Released outside the chart entirely.
    pointer('pointerup', { x: 0, y: 0 }, 0, { target: document.body });

    const before = log.edits.length;
    pointer('pointermove', { x: point.x, y: point.y - 60 }, 1);

    expect(log.edits.length).toBe(before);
  });
});

describe('the point a press grabs', () => {
  test('is the nearest one, not whichever is on top', () => {
    // 19.6 ft and 20 ft sit a few pixels apart; hit areas big enough to click would overlap.
    const near20 = dotAt(2);
    pointer('pointermove', near20, 0);
    pointer('pointerdown', near20, 1);
    pointer('pointermove', { x: near20.x, y: near20.y - 15 }, 1);

    expect(log.edits.map((edit) => edit.index)).toEqual([2]);
  });

  test('is the other one when the pointer is closer to it', () => {
    const near196 = dotAt(1);
    pointer('pointerdown', near196, 1);
    pointer('pointermove', { x: near196.x, y: near196.y - 15 }, 1);

    expect(log.edits.map((edit) => edit.index)).toEqual([1]);
  });

  test('is nothing when the press is far from any point', () => {
    const away = { x: dotAt(0).x, y: dotAt(0).y - 60 };
    pointer('pointerdown', away, 1);
    pointer('pointermove', { x: away.x, y: away.y - 20 }, 1);

    expect(log.starts).toBe(0);
    expect(log.edits).toEqual([]);
  });
});

describe('dragging', () => {
  test('moves the value in the direction the pointer went', () => {
    const point = dotAt(0);
    pointer('pointerdown', point, 1);
    pointer('pointermove', { x: point.x, y: point.y - 25 }, 1);

    expect(log.starts).toBe(1);
    expect(log.edits[0]!.index).toBe(0);
    expect(log.edits[0]!.value).toBeGreaterThan(ROWS[0]!.rpm);
  });

  test('only moves the value, never the range', () => {
    const point = dotAt(0);
    pointer('pointerdown', point, 1);
    pointer('pointermove', { x: point.x + 120, y: point.y - 25 }, 1);

    // A horizontal drag must not re-order ranges into a table the robot would reject.
    expect(log.edits.every((edit) => edit.index === 0)).toBe(true);
    expect(log.added).toEqual([]);
  });

  test('a press that never moved leaves no edit behind', () => {
    const point = dotAt(0);
    pointer('pointerdown', point, 1);
    pointer('pointerup', point, 0);

    expect(log.edits).toEqual([]);
    expect(log.ends).toBe(1);
  });
});

describe('shift-click levels a point', () => {
  test('a shift-click on a point asks for it to be levelled', () => {
    const point = dotAt(1);
    pointer('pointerdown', point, 1, { shift: true });
    pointer('pointerup', point, 0, { shift: true });

    expect(log.levelled).toEqual([1]);
    expect(log.edits).toEqual([]);
  });

  test('a plain click does not level anything', () => {
    const point = dotAt(1);
    pointer('pointerdown', point, 1);
    pointer('pointerup', point, 0);

    expect(log.levelled).toEqual([]);
  });

  test('a shift-drag adjusts the value instead of levelling', () => {
    // Both gestures start with the same press, so the two must not both fire.
    const point = dotAt(1);
    pointer('pointerdown', point, 1, { shift: true });
    pointer('pointermove', { x: point.x, y: point.y - 30 }, 1, { shift: true });
    pointer('pointerup', { x: point.x, y: point.y - 30 }, 0, { shift: true });

    expect(log.edits.length).toBeGreaterThan(0);
    expect(log.levelled).toEqual([]);
  });

  test('pointer jitter within a few pixels stays a click', () => {
    const point = dotAt(1);
    pointer('pointerdown', point, 1, { shift: true });
    pointer('pointermove', { x: point.x + 1, y: point.y + 1 }, 1, { shift: true });
    pointer('pointerup', { x: point.x + 1, y: point.y + 1 }, 0, { shift: true });

    expect(log.edits).toEqual([]);
    expect(log.levelled).toEqual([1]);
  });

  test('the keyboard levels the selected point too', () => {
    click(1);
    key('l');

    expect(log.levelled).toEqual([1]);
  });
});

describe('the keyboard acts on the selected point', () => {
  test('nothing happens until a point is selected', () => {
    key('ArrowUp');
    key('Delete');

    expect(log.edits).toEqual([]);
    expect(log.removed).toEqual([]);
  });

  test('clicking a point selects it', () => {
    click(1);

    expect(log.selected).toEqual([ROWS[1]!.d]);
  });

  test('arrow up and down nudge by one step', () => {
    click(0);
    key('ArrowUp');
    key('ArrowDown');

    expect(log.edits[0]).toEqual({ index: 0, value: ROWS[0]!.rpm + FLYWHEEL.step });
    expect(log.edits[1]).toEqual({ index: 0, value: ROWS[0]!.rpm - FLYWHEEL.step });
  });

  test('shift makes a nudge ten steps', () => {
    click(0);
    key('ArrowUp', { shift: true });

    expect(log.edits[0]!.value).toBe(ROWS[0]!.rpm + 10 * FLYWHEEL.step);
  });

  test('left and right move the selection between ranges', () => {
    click(0);
    key('ArrowRight');
    key('ArrowUp');

    expect(log.edits[0]!.index).toBe(1);

    key('ArrowLeft');
    key('ArrowUp');

    expect(log.edits[1]!.index).toBe(0);
  });

  test('the selection does not run off either end', () => {
    click(0);
    key('ArrowLeft');
    key('ArrowUp');

    expect(log.edits[0]!.index).toBe(0);
  });

  test('delete removes the selected range', () => {
    click(2);
    key('Delete');

    expect(log.removed).toEqual([2]);
  });

  test('backspace removes it too, which is what the macOS delete key sends', () => {
    click(2);
    key('Backspace');

    expect(log.removed).toEqual([2]);
  });

  test('escape drops the selection', () => {
    click(1);
    key('Escape');
    key('ArrowUp');

    expect(log.edits).toEqual([]);
  });

  test('clicking empty space drops the selection', () => {
    click(1);
    pointer('click', { x: dotAt(0).x + 60, y: dotAt(0).y - 60 }, 0);
    key('ArrowUp');

    expect(log.edits).toEqual([]);
  });

  test('a selection survives the rebuild an edit causes', () => {
    // The point elements are replaced on every render, so anything keyed on DOM focus would stop here.
    click(0);
    key('ArrowUp');
    chart.setData(ROWS, [9, 21], [10, 15, 20]);
    key('ArrowUp');

    expect(log.edits.length).toBe(2);
    expect(log.edits[1]!.index).toBe(0);
  });

  test('keys are ignored while a number input has focus', () => {
    // The values table and the add-range box are full of inputs whose arrow keys must keep working.
    click(0);

    const input = document.createElement('input');
    input.type = 'number';
    document.body.append(input);

    const event = new Event('keydown', { bubbles: true, cancelable: true });
    Object.assign(event, { key: 'ArrowUp', shiftKey: false, preventDefault() {} });
    input.dispatchEvent(event);

    expect(log.edits).toEqual([]);
  });

  test('the app\u2019s own shortcuts are left alone', () => {
    click(0);
    key('ArrowUp', { meta: true });
    key('z', { meta: true });

    expect(log.edits).toEqual([]);
  });
});

describe('clicking empty space', () => {
  test('adds a range snapped to the half-foot grid', () => {
    const away = { x: dotAt(0).x + 60, y: dotAt(0).y - 60 };
    pointer('click', away, 0);

    expect(log.added.length).toBe(1);
    expect(log.added[0]! % 0.5).toBe(0);
  });

  test('does not add one when the release lands on a point', () => {
    pointer('click', dotAt(2), 0);

    expect(log.added).toEqual([]);
  });
});
