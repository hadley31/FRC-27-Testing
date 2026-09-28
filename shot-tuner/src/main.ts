/**
 * Wiring: the three linked charts, the value table, the toolbar, and the connection to the robot.
 *
 * The rule that shapes everything here is that the robot owns the table. This dashboard adopts whatever
 * the robot publishes, never writes until the user edits something, and treats the robot's own verdict
 * as the truth about whether an edit was taken.
 */

import { CurveChart, niceTicks } from './curve-chart';
import { ValuesTable } from './values-table';
import { NtSession, type Diagnostics } from './nt-session';
import {
  SERIES,
  columnsFromRows,
  formatRange,
  formatValue,
  indexOfRange,
  interpolate,
  levelledValue,
  rowsEqual,
  rowsFromColumns,
  snapToGrid,
  toJava,
  validate,
  withRange,
  withValue,
  withoutRange,
  type Row,
  type SeriesKey,
} from './shot-table';

const STORAGE = {
  address: 'shot-tuner.address',
  path: 'shot-tuner.path',
  theme: 'shot-tuner.theme',
  table: 'shot-tuner.table-open',
} as const;

const DEFAULT_ADDRESS = 'localhost';
const DEFAULT_PATH = '/AdvantageKit/NetworkInputs/Tuning/ShotProfile/Scoring';

const el = {
  address: byId<HTMLInputElement>('address'),
  path: byId<HTMLInputElement>('path'),
  connect: byId<HTMLButtonElement>('connect'),
  status: byId<HTMLSpanElement>('status'),
  theme: byId<HTMLButtonElement>('theme'),
  addRange: byId<HTMLInputElement>('add-range'),
  add: byId<HTMLButtonElement>('add'),
  undo: byId<HTMLButtonElement>('undo'),
  redo: byId<HTMLButtonElement>('redo'),
  revert: byId<HTMLButtonElement>('revert'),
  reset: byId<HTMLButtonElement>('reset'),
  copy: byId<HTMLButtonElement>('copy'),
  toggleTable: byId<HTMLButtonElement>('toggle-table'),
  charts: byId<HTMLElement>('charts'),
  tablePanel: byId<HTMLElement>('table-panel'),
  tableBody: byId<HTMLTableSectionElement>('table-body'),
  verdict: byId<HTMLSpanElement>('verdict'),
  detail: byId<HTMLSpanElement>('detail'),
  tooltip: byId<HTMLDivElement>('tooltip'),
};

let rows: Row[] = [];
/** The table as it stood when this page last adopted one from the robot. */
let adopted: Row[] = [];
let committed: Row[] | null = null;
let loaded = false;
let editing = false;
/** Set when a drag begins; the undo entry is pushed by the first edit that actually changes something. */
let undoPending = false;
let hover: number | null = null;
let connected = false;
let diagnostics: Diagnostics = { accepted: null, rejectionReason: null, isDefault: null };

const undoStack: Row[][] = [];
const redoStack: Row[][] = [];

let writeQueued = false;

/** The tooltip's measured size, and the shape it was measured at. See showTooltip. */
let tooltipSize = { width: 170, height: 96 };
let tooltipShape: string | null = null;

const session = new NtSession({
  onConnectionChange(next) {
    connected = next;

    if (!next) {
      loaded = false;
      render();
    }

    renderStatus();
  },

  onColumns(columns) {
    // Never overwrite a drag in progress: the robot echoes our own writes back, and a frame of that
    // landing mid-gesture would fight the pointer.
    if (editing) return;

    const next = rowsFromColumns(columns);
    if (loaded && rowsEqual(next, rows)) return;

    rows = next;
    adopted = next.map((row) => ({ ...row }));
    loaded = true;
    undoStack.length = 0;
    redoStack.length = 0;
    render();
    renderStatus();
  },

  onCommittedColumns(columns) {
    committed = rowsFromColumns(columns);
    renderToolbar();
  },

  onDiagnostics(next) {
    diagnostics = next;
    renderStatus();
  },
});

const valuesTable = new ValuesTable(el.tableBody, {
  onValue(index, key, value) {
    applyRows(withValue(rows, index, key, value), { pushUndo: true });
  },
  onRemove(index) {
    removeRange(index);
  },
});

// MARK: - Charts

const charts = SERIES.map((def, index) => {
  const card = document.createElement('section');
  card.className = 'chart-card';

  const head = document.createElement('div');
  head.className = 'chart-head';

  const title = document.createElement('span');
  title.className = 'chart-title';
  title.textContent = def.label;

  const unit = document.createElement('span');
  unit.className = 'chart-unit';
  unit.textContent = `${def.unit} vs ft`;

  head.append(title, unit);
  card.append(head);
  el.charts.append(card);

  return new CurveChart(
    card,
    def,
    {
      onHover(info) {
        setHover(info?.distance ?? null);
        if (info) showTooltip(info.clientX, info.clientY, info.distance);
        else hideTooltip();
      },
      onEditStart() {
        beginEdit();
      },
      onEdit(rowIndex, value) {
        applyRows(withValue(rows, rowIndex, def.key, value), { pushUndo: false });
      },
      onEditEnd() {
        editing = false;
        undoPending = false;
        renderToolbar();
      },
      onAddRange(distance) {
        addRange(distance);
      },
      onRemoveRange(rowIndex) {
        removeRange(rowIndex);
      },
      onLevelRange(rowIndex) {
        levelRange(rowIndex, def.key);
      },
      onSelect(distance) {
        // One selection at a time, or a keypress would edit all three curves at once.
        if (distance === null) return;
        for (const other of charts) {
          if (other !== charts[index]) other.select(null);
        }
      },
    },
    index === SERIES.length - 1,
  );
});

// MARK: - Edits

function beginEdit(): void {
  // Deliberately does not push an undo entry yet. A press that turns out to be a click rather than a
  // drag would otherwise leave an entry that undoes to an identical table.
  if (!editing) undoPending = true;
  editing = true;
}

/** The single path every change takes: validate, keep, draw, send. */
function applyRows(next: Row[], options: { pushUndo: boolean }): void {
  const check = validate(next);

  if (!check.ok) {
    el.verdict.className = 'verdict is-bad';
    el.verdict.textContent = 'Not sent';
    el.detail.textContent = check.reason;
    return;
  }

  if (options.pushUndo || undoPending) {
    undoStack.push(rows.map((row) => ({ ...row })));
    redoStack.length = 0;
    undoPending = false;
  }

  rows = next;
  render();
  queueWrite();
}

/**
 * Coalesces writes to one per frame.
 *
 * A drag fires on every pointer move; the robot only reads its inputs once a cycle, so sending more
 * often than the screen refreshes is pure noise on the wire.
 */
function queueWrite(): void {
  if (writeQueued) return;

  writeQueued = true;
  requestAnimationFrame(() => {
    writeQueued = false;
    if (session.writable) session.write(columnsFromRows(rows));
    renderStatus();
  });
}

function addRange(distance: number): void {
  if (!loaded) return;

  const snapped = snapToGrid(distance);
  if (indexOfRange(rows, snapped) >= 0) return;

  applyRows(withRange(rows, snapped), { pushUndo: true });
}

function removeRange(index: number): void {
  if (!loaded) return;
  applyRows(withoutRange(rows, index), { pushUndo: true });
}

/**
 * Drops one point back onto the straight line between its two neighbours, flattening a kink.
 *
 * The value is the curve's own interpolation at that range with the point taken out, which is what puts
 * it exactly on the line. On an unevenly spaced axis — and the committed table is uneven — averaging the
 * two neighbours instead would land slightly off the line and leave a smaller kink behind.
 *
 * Only the curve that was clicked moves. The three charts share their ranges, not their values.
 */
function levelRange(index: number, key: SeriesKey): void {
  if (!loaded) return;

  const levelled = levelledValue(rows, index, key);
  if (levelled === null || levelled === rows[index]?.[key]) return;

  applyRows(withValue(rows, index, key, levelled), { pushUndo: true });
}

function undo(): void {
  const previous = undoStack.pop();
  if (!previous) return;

  redoStack.push(rows.map((row) => ({ ...row })));
  rows = previous;
  render();
  queueWrite();
}

function redo(): void {
  const next = redoStack.pop();
  if (!next) return;

  undoStack.push(rows.map((row) => ({ ...row })));
  rows = next;
  render();
  queueWrite();
}

// MARK: - Render

function render(): void {
  const editable = loaded && session.writable;

  if (rows.length === 0) {
    for (const chart of charts) {
      chart.setEnabled(false);
      chart.setData([], [0, 1], []);
    }
    renderTable();
    renderToolbar();
    return;
  }

  const lo = Math.max(0, Math.floor(rows[0]!.d - 1));
  const hi = Math.ceil(rows[rows.length - 1]!.d + 1);
  const domain: [number, number] = [lo, hi];
  const ticks = niceTicks(lo, hi, 6).filter((tick) => tick >= lo && tick <= hi);

  for (const chart of charts) {
    chart.setEnabled(editable);
    chart.setData(rows, domain, ticks);
    chart.setHover(hover);
  }

  renderTable();
  renderToolbar();
}

function renderToolbar(): void {
  const editable = loaded && session.writable;

  el.add.disabled = !editable;
  el.addRange.disabled = !editable;
  el.undo.disabled = !editable || undoStack.length === 0;
  el.redo.disabled = !editable || redoStack.length === 0;
  el.revert.disabled = !editable || rowsEqual(rows, adopted);
  el.reset.disabled = !editable || committed === null || rowsEqual(rows, committed);
  el.copy.disabled = rows.length === 0;
}

function renderStatus(): void {
  el.connect.textContent = connected ? 'Reconnect' : 'Connect';

  if (!connected) {
    el.status.className = 'chip';
    el.status.textContent = 'Offline';
  } else if (!loaded) {
    el.status.className = 'chip is-connecting';
    el.status.textContent = 'Waiting for table';
  } else {
    el.status.className = 'chip is-live';
    el.status.textContent = 'Connected';
  }

  const local = validate(rows);

  if (!loaded) {
    el.verdict.className = 'verdict is-idle';
    el.verdict.textContent = connected ? 'No table yet' : 'Not connected';
    el.detail.textContent = connected
      ? 'Subscribed; waiting for the robot to publish its columns.'
      : 'Enter the robot address and connect. Use localhost for a simulated robot.';
    return;
  }

  if (!local.ok) {
    el.verdict.className = 'verdict is-bad';
    el.verdict.textContent = 'Not sent';
    el.detail.textContent = local.reason;
    return;
  }

  // The robot's own verdict outranks the local check: it is the one that decides what gets shot.
  if (diagnostics.accepted === false) {
    el.verdict.className = 'verdict is-bad';
    el.verdict.textContent = 'Robot rejected';
    el.detail.textContent = diagnostics.rejectionReason || 'The robot is holding its previous table.';
    return;
  }

  el.verdict.className = 'verdict is-good';
  el.verdict.textContent = 'Robot accepted';
  el.detail.textContent =
    diagnostics.isDefault === false
      ? 'This table differs from the one committed in ShotProfile.java — copy it out before you lose it.'
      : 'Matches the table committed in ShotProfile.java.';
}

function renderTable(): void {
  if (el.tablePanel.hidden) return;

  valuesTable.setRows(rows, loaded && session.writable);
}

// MARK: - Hover

function setHover(distance: number | null): void {
  if (hover === distance) return;

  hover = distance;
  for (const chart of charts) chart.setHover(distance);
  highlightTableRow();
}

function highlightTableRow(): void {
  valuesTable.setHover(hover);
}

/** One readout for all three curves, so correlating them never means chasing three tooltips. */
function showTooltip(clientX: number, clientY: number, distance: number): void {
  if (rows.length === 0) return;

  while (el.tooltip.firstChild) el.tooltip.firstChild.remove();

  const heading = document.createElement('div');
  heading.className = 'tooltip-range';
  heading.textContent = `${formatRange(distance)} ft`;
  el.tooltip.append(heading);

  for (const def of SERIES) {
    const row = document.createElement('div');
    row.className = 'tooltip-row';

    const key = document.createElement('span');
    key.className = 'tooltip-key';

    const value = document.createElement('span');
    value.className = 'tooltip-value';
    value.textContent = `${formatValue(interpolate(rows, def.key, distance), def)} ${def.unit}`;

    const label = document.createElement('span');
    label.className = 'tooltip-label';
    label.textContent = def.label;

    row.append(key, value, label);
    el.tooltip.append(row);
  }

  const hasNote = indexOfRange(rows, distance) < 0 && loaded && session.writable;

  if (hasNote) {
    const note = document.createElement('div');
    note.className = 'tooltip-note';
    note.textContent = 'Click to add this range';
    el.tooltip.append(note);
  }

  el.tooltip.classList.add('is-visible');
  el.tooltip.setAttribute('aria-hidden', 'false');

  // Measured only when the shape of the tooltip changes, not on every move. Reading a rect straight after
  // writing to the DOM forces the browser to lay the page out again, and on a drag that happens inside
  // the same frame as the chart redraw, so it is the difference between one layout and two.
  const shape = `${hasNote ? 1 : 0}`;
  if (shape !== tooltipShape) {
    const box = el.tooltip.getBoundingClientRect();
    tooltipSize = { width: box.width, height: box.height };
    tooltipShape = shape;
  }

  const left = Math.min(clientX + 14, window.innerWidth - tooltipSize.width - 8);
  const top = Math.min(
    Math.max(8, clientY - tooltipSize.height - 12),
    window.innerHeight - tooltipSize.height - 8,
  );

  el.tooltip.style.left = `${left}px`;
  el.tooltip.style.top = `${top}px`;
}

function hideTooltip(): void {
  el.tooltip.classList.remove('is-visible');
  el.tooltip.setAttribute('aria-hidden', 'true');
  setHover(null);
}

// MARK: - Controls

el.connect.addEventListener('click', () => void openSession());

for (const input of [el.address, el.path]) {
  input.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') void openSession();
  });
}

async function openSession(): Promise<void> {
  const address = el.address.value.trim() || DEFAULT_ADDRESS;
  const path = el.path.value.trim() || DEFAULT_PATH;

  el.address.value = address;
  el.path.value = path;
  localStorage.setItem(STORAGE.address, address);
  localStorage.setItem(STORAGE.path, path);

  el.status.className = 'chip is-connecting';
  el.status.textContent = 'Connecting';

  try {
    await session.open(address, path);
  } catch (error) {
    el.status.className = 'chip is-error';
    el.status.textContent = 'Failed';
    el.detail.textContent = error instanceof Error ? error.message : String(error);
  }
}

el.add.addEventListener('click', () => {
  const parsed = Number(el.addRange.value);
  if (Number.isFinite(parsed)) addRange(parsed);
});

el.addRange.addEventListener('keydown', (event) => {
  if (event.key !== 'Enter') return;
  const parsed = Number(el.addRange.value);
  if (Number.isFinite(parsed)) addRange(parsed);
});

el.undo.addEventListener('click', undo);
el.redo.addEventListener('click', redo);

el.revert.addEventListener('click', () => {
  applyRows(adopted.map((row) => ({ ...row })), { pushUndo: true });
});

el.reset.addEventListener('click', () => {
  if (committed) applyRows(committed.map((row) => ({ ...row })), { pushUndo: true });
});

el.copy.addEventListener('click', async () => {
  try {
    await navigator.clipboard.writeText(toJava(rows));
    el.copy.textContent = 'Copied';
  } catch {
    el.copy.textContent = 'Copy failed';
  }
  setTimeout(() => (el.copy.textContent = 'Copy as Java'), 1200);
});

el.toggleTable.addEventListener('click', () => {
  const open = el.tablePanel.hidden;
  el.tablePanel.hidden = !open;
  el.toggleTable.textContent = open ? 'Hide values' : 'Show values';
  el.toggleTable.setAttribute('aria-expanded', String(open));
  localStorage.setItem(STORAGE.table, open ? '1' : '0');
  renderTable();
});

el.theme.addEventListener('click', () => {
  const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
  document.documentElement.dataset.theme = next;
  localStorage.setItem(STORAGE.theme, next);
});

window.addEventListener('keydown', (event) => {
  if (!(event.metaKey || event.ctrlKey)) return;
  if (event.key.toLowerCase() !== 'z') return;

  event.preventDefault();
  if (event.shiftKey) redo();
  else undo();
});

// MARK: - Start

function byId<T extends HTMLElement>(id: string): T {
  const found = document.getElementById(id);
  if (!found) throw new Error(`Missing element #${id}`);
  return found as T;
}

function start(): void {
  const storedTheme = localStorage.getItem(STORAGE.theme);
  if (storedTheme === 'dark' || storedTheme === 'light') document.documentElement.dataset.theme = storedTheme;

  el.address.value = localStorage.getItem(STORAGE.address) ?? DEFAULT_ADDRESS;
  el.path.value = localStorage.getItem(STORAGE.path) ?? DEFAULT_PATH;

  if (localStorage.getItem(STORAGE.table) === '1') {
    el.tablePanel.hidden = false;
    el.toggleTable.textContent = 'Hide values';
    el.toggleTable.setAttribute('aria-expanded', 'true');
  }

  render();
  renderStatus();
  void openSession();
}

start();
