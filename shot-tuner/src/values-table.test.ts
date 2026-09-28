/**
 * Tests for the values table, which updates in place rather than rebuilding.
 *
 * In-place updating is what keeps a drag smooth, but it is also how a table gets subtly wrong: a cell
 * that stops tracking the data, or one that overwrites what someone is typing.
 */

import './dom-test-setup';

import type { Row } from './shot-table';

const { afterEach, beforeEach, describe, expect, test } = await import('bun:test');
const { ValuesTable } = await import('./values-table');

const ROWS: Row[] = [
  { d: 10, tof: 1.0, hood: 5, rpm: 1800 },
  { d: 15, tof: 1.2, hood: 8, rpm: 2500 },
  { d: 20, tof: 1.4, hood: 10, rpm: 2200 },
];

let body: HTMLTableSectionElement;
let table: InstanceType<typeof ValuesTable>;
let values: { index: number; key: string; value: number }[];
let removed: number[];
let created: number;
let restore: () => void;

function inputsOf(rowIndex: number): HTMLInputElement[] {
  return [...body.querySelectorAll('tr')[rowIndex]!.querySelectorAll('input')] as HTMLInputElement[];
}

beforeEach(() => {
  document.body.innerHTML = '<table><tbody id="body"></tbody></table>';
  body = document.getElementById('body') as unknown as HTMLTableSectionElement;
  values = [];
  removed = [];

  table = new ValuesTable(body, {
    onValue(index, key, value) { values.push({ index, key, value }); },
    onRemove(index) { removed.push(index); },
  });

  table.setRows(ROWS, true);

  created = 0;
  const real = document.createElement.bind(document);
  document.createElement = ((...args: Parameters<typeof real>) => {
    created++;
    return real(...args);
  }) as typeof document.createElement;
  restore = () => { document.createElement = real; };
});

afterEach(() => restore());

describe('rendering', () => {
  test('shows every range with its three values', () => {
    expect(body.querySelectorAll('tr').length).toBe(3);
    expect(inputsOf(0).map((input) => input.value)).toEqual(['1.000', '5.00', '1800.0']);
  });

  test('a value change updates the cell without rebuilding the table', () => {
    const before = inputsOf(1);

    table.setRows(ROWS.map((row, i) => (i === 1 ? { ...row, rpm: 2600 } : row)), true);

    expect(inputsOf(1)[2]!.value).toBe('2600.0');
    // The same input elements, so a drag does not churn fifty inputs a frame.
    expect(inputsOf(1)[2]).toBe(before[2]);
    expect(created).toBe(0);
  });

  test('adding a range rebuilds, since the shape changed', () => {
    table.setRows([...ROWS, { d: 25, tof: 1.5, hood: 10, rpm: 2700 }], true);

    expect(body.querySelectorAll('tr').length).toBe(4);
    expect(created).toBeGreaterThan(0);
  });

  test('removing a range rebuilds too', () => {
    table.setRows(ROWS.slice(0, 2), true);

    expect(body.querySelectorAll('tr').length).toBe(2);
  });

  test('a range moving rebuilds, so rows never show another range’s numbers', () => {
    table.setRows(ROWS.map((row, i) => (i === 1 ? { ...row, d: 16 } : row)), true);

    expect(body.querySelectorAll('tr')[1]!.querySelector('td')!.textContent).toBe('16.00');
  });
});

describe('typing', () => {
  test('a committed value is reported, snapped to the step', () => {
    const input = inputsOf(0)[2]!;
    input.value = '1823';
    input.dispatchEvent(new Event('change', { bubbles: true }));

    // Flywheel steps in 5 RPM.
    expect(values).toEqual([{ index: 0, key: 'rpm', value: 1825 }]);
  });

  test('an emptied cell is put back rather than sent as zero', () => {
    // A number input reports '' for anything unparseable, and Number('') is 0. Sending that would
    // command a real value the person never typed.
    const input = inputsOf(0)[2]!;
    input.value = '';
    input.dispatchEvent(new Event('change', { bubbles: true }));

    expect(values).toEqual([]);
    expect(input.value).toBe('1800.0');
  });

  test('text a number input cannot parse is put back too', () => {
    const input = inputsOf(0)[2]!;
    input.value = 'banana';
    input.dispatchEvent(new Event('change', { bubbles: true }));

    expect(values).toEqual([]);
    expect(input.value).toBe('1800.0');
  });

  test('an update never overwrites the cell being typed into', () => {
    const input = inputsOf(0)[2]!;
    input.focus();
    input.value = '18';

    // An echo of the robot's own value arriving mid-keystroke must not reformat it under the cursor.
    table.setRows(ROWS, true);

    expect(input.value).toBe('18');
  });

  test('other cells still update while one is being typed into', () => {
    inputsOf(0)[2]!.focus();

    table.setRows(ROWS.map((row, i) => (i === 1 ? { ...row, rpm: 2600 } : row)), true);

    expect(inputsOf(1)[2]!.value).toBe('2600.0');
  });
});

describe('the rest of the row', () => {
  test('the remove button reports its range', () => {
    const button = body.querySelectorAll('tr')[2]!.querySelector('button')!;
    button.dispatchEvent(new Event('click', { bubbles: true }));

    expect(removed).toEqual([2]);
  });

  test('everything is disabled when the table is not editable', () => {
    table.setRows(ROWS, false);

    expect(inputsOf(0).every((input) => input.disabled)).toBe(true);
    expect(body.querySelector('button')!.disabled).toBe(true);
  });

  test('hovering a range marks its row, and only its row', () => {
    table.setHover(15);

    const marked = [...body.querySelectorAll('tr')].map((tr) => tr.classList.contains('is-hovered'));
    expect(marked).toEqual([false, true, false]);
  });

  test('the mark clears when the pointer leaves', () => {
    table.setHover(15);
    table.setHover(null);

    expect(body.querySelector('.is-hovered')).toBeNull();
  });
});
