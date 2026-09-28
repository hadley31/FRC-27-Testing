/**
 * The values table: every range with its three numbers, typeable.
 *
 * Kept as its own component because it is rebuilt from the same state as the charts, on every frame of a
 * drag, and rebuilding it is by far the most expensive thing on that path — eighteen ranges means around
 * fifty number inputs and as many event listeners. So it updates its cells in place and only rebuilds
 * when the set of ranges actually changes.
 */

import { SERIES, formatRange, formatValue, snapToStep, type Row, type SeriesKey } from './shot-table';

export interface ValuesTableCallbacks {
  onValue(index: number, key: SeriesKey, value: number): void;
  onRemove(index: number): void;
}

interface RowElements {
  tr: HTMLTableRowElement;
  distance: HTMLTableCellElement;
  inputs: Map<SeriesKey, HTMLInputElement>;
  remove: HTMLButtonElement;
}

export class ValuesTable {
  private readonly body: HTMLTableSectionElement;
  private readonly callbacks: ValuesTableCallbacks;

  private elements: RowElements[] = [];
  private rows: readonly Row[] = [];
  private editable = false;
  private hover: number | null = null;

  constructor(body: HTMLTableSectionElement, callbacks: ValuesTableCallbacks) {
    this.body = body;
    this.callbacks = callbacks;
  }

  setRows(rows: readonly Row[], editable: boolean): void {
    const sameShape =
      rows.length === this.elements.length && rows.every((row, i) => row.d === this.rows[i]?.d);

    this.rows = rows;
    this.editable = editable;

    if (sameShape) this.update();
    else this.rebuild();

    this.applyHover();
  }

  setHover(distance: number | null): void {
    if (this.hover === distance) return;

    this.hover = distance;
    this.applyHover();
  }

  private update(): void {
    this.rows.forEach((row, index) => {
      const elements = this.elements[index];
      if (!elements) return;

      for (const def of SERIES) {
        const input = elements.inputs.get(def.key);
        if (!input) continue;

        // Never overwrite a cell someone is typing into; their half-finished number is the truth until
        // they commit it.
        if (document.activeElement !== input) {
          const next = formatValue(row[def.key], def);
          if (input.value !== next) input.value = next;
        }

        input.disabled = !this.editable;
      }

      elements.remove.disabled = !this.editable;
    });
  }

  private rebuild(): void {
    while (this.body.firstChild) this.body.firstChild.remove();
    this.elements = [];

    this.rows.forEach((row, index) => {
      const tr = document.createElement('tr');

      const distance = document.createElement('td');
      distance.textContent = formatRange(row.d);
      tr.append(distance);

      const inputs = new Map<SeriesKey, HTMLInputElement>();

      for (const def of SERIES) {
        const cell = document.createElement('td');
        const input = document.createElement('input');
        input.type = 'number';
        input.step = String(def.step);
        input.value = formatValue(row[def.key], def);
        input.disabled = !this.editable;
        input.setAttribute('aria-label', `${def.label} at ${formatRange(row.d)} feet`);

        input.addEventListener('change', () => {
          const current = this.rows[index]?.[def.key];
          const typed = input.value.trim();
          const parsed = Number(typed);

          // A number input reports '' for anything it cannot parse, and Number('') is 0 — so an emptied
          // or nonsense cell would otherwise send a real, very wrong value to the robot.
          if (typed === '' || !Number.isFinite(parsed)) {
            if (current !== undefined) input.value = formatValue(current, def);
            return;
          }

          this.callbacks.onValue(index, def.key, snapToStep(parsed, def));
        });

        cell.append(input);
        tr.append(cell);
        inputs.set(def.key, input);
      }

      const removeCell = document.createElement('td');
      const remove = document.createElement('button');
      remove.type = 'button';
      remove.className = 'row-remove';
      remove.textContent = '×';
      remove.disabled = !this.editable;
      remove.setAttribute('aria-label', `Remove the ${formatRange(row.d)} foot range`);
      remove.addEventListener('click', () => this.callbacks.onRemove(index));
      removeCell.append(remove);
      tr.append(removeCell);

      this.body.append(tr);
      this.elements.push({ tr, distance, inputs, remove });
    });
  }

  private applyHover(): void {
    this.elements.forEach((elements, index) => {
      const distance = this.rows[index]?.d;
      const hovered = this.hover !== null && distance !== undefined && Math.abs(distance - this.hover) < 1e-9;
      elements.tr.classList.toggle('is-hovered', hovered);
    });
  }
}
