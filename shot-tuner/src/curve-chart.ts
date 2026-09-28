/**
 * One editable curve: range along the bottom, one measure up the side, a draggable point per range.
 *
 * All three charts share an identical horizontal geometry — same gutter, same width, same range domain —
 * which is what lets one crosshair line up across them and makes the three readable as one shot.
 * Vertical geometry is per-chart, so only the bottom chart pays for tick labels.
 */

import {
  formatValue,
  indexOfRange,
  interpolate,
  snapToGrid,
  snapToStep,
  type Row,
  type SeriesDef,
} from './shot-table';

const SVG_NS = 'http://www.w3.org/2000/svg';

/** Shared so every chart's plot area starts and ends at the same x. */
export const GUTTER_LEFT = 58;
export const PAD_RIGHT = 18;
const PAD_TOP = 14;
const PLOT_HEIGHT = 132;
const X_AXIS_HEIGHT = 26;
const NO_AXIS_HEIGHT = 12;

/**
 * How close the pointer has to be to grab a point, in pixels.
 *
 * Selection is nearest-wins rather than a hit area per point, so the point grabbed is always the one
 * closest to the cursor. With a target per point, two close ranges — 15.00 and 15.20 ft are a few pixels
 * apart on a wide chart — would have overlapping targets, and whichever sat last in DOM order would win.
 */
const GRAB_RADIUS = 16;

/**
 * How far the pointer must travel before a press counts as a drag, in pixels.
 *
 * Below this a press is a click, which is what separates shift-click (level the point) from shift-drag
 * (adjust it finely). It also means a plain click on a point cannot nudge its value by a pixel of
 * pointer jitter.
 */
const DRAG_THRESHOLD = 3;

export interface HoverInfo {
  distance: number;
  clientX: number;
  clientY: number;
}

export interface ChartCallbacks {
  onHover(info: HoverInfo | null): void;
  /** Put this range back on the straight line between its neighbours, on this chart's curve only. */
  onLevelRange(index: number): void;
  /** This chart took or dropped the keyboard selection; only one chart may hold it at a time. */
  onSelect(distance: number | null): void;
  onEditStart(): void;
  onEdit(index: number, value: number): void;
  onEditEnd(): void;
  onAddRange(distance: number): void;
  onRemoveRange(index: number): void;
}

export class CurveChart {
  private readonly def: SeriesDef;
  private readonly callbacks: ChartCallbacks;
  private readonly showXAxis: boolean;

  private readonly svg: SVGSVGElement;
  private readonly gridLayer: SVGGElement;
  private readonly axisLayer: SVGGElement;
  private readonly curveLayer: SVGGElement;
  private readonly hoverLayer: SVGGElement;
  private readonly pointLayer: SVGGElement;
  private readonly hit: SVGRectElement;

  private readonly crosshair: SVGLineElement;
  private readonly marker: SVGCircleElement;
  private readonly ghost: SVGCircleElement;

  private rows: readonly Row[] = [];
  private xDomain: [number, number] = [0, 1];
  private xTicks: number[] = [];
  private yDomain: [number, number] | null = null;
  private width = 600;
  private enabled = false;
  /** What the last draw was made from, so an unchanged chart can skip redrawing entirely. */
  private drawn: string | null = null;

  private dragIndex: number | null = null;
  private dragOrigin: { x: number; y: number } | null = null;
  private dragMoved = false;
  private dragShift = false;
  private nearIndex: number | null = null;
  /**
   * The selected range, held as a distance rather than an index.
   *
   * Indices shift when a range is added or removed, and the point elements are rebuilt on every edit, so
   * anything keyed on an index or on DOM focus goes stale exactly when it is being used.
   */
  private selectedRange: number | null = null;
  private readonly listeners = new AbortController();
  private readonly resize: ResizeObserver;

  constructor(host: HTMLElement, def: SeriesDef, callbacks: ChartCallbacks, showXAxis: boolean) {
    this.def = def;
    this.callbacks = callbacks;
    this.showXAxis = showXAxis;

    this.svg = document.createElementNS(SVG_NS, 'svg');
    this.svg.classList.add('chart-svg');
    this.svg.setAttribute('aria-label', `${def.label} versus range, editable`);

    this.gridLayer = this.layer('grid');
    this.axisLayer = this.layer('axis');
    this.curveLayer = this.layer('curve');
    this.hoverLayer = this.layer('hover');
    this.pointLayer = this.layer('points');

    // Above every other layer, so one surface sees all pointer events. With the points on top
    // instead, moving onto one stopped the crosshair updating, because the hit rect never saw the move.
    this.hit = document.createElementNS(SVG_NS, 'rect');
    this.hit.classList.add('chart-hit');
    this.hit.setAttribute('fill', 'transparent');

    this.crosshair = document.createElementNS(SVG_NS, 'line');
    this.crosshair.classList.add('crosshair');
    this.marker = document.createElementNS(SVG_NS, 'circle');
    this.marker.classList.add('crosshair-marker');
    this.marker.setAttribute('r', '4');
    this.ghost = document.createElementNS(SVG_NS, 'circle');
    this.ghost.classList.add('crosshair-ghost');
    this.ghost.setAttribute('r', '5.5');
    this.hoverLayer.append(this.crosshair, this.marker, this.ghost);
    this.setHover(null);

    this.svg.append(this.hit);

    host.append(this.svg);
    this.bindPointer();

    this.resize = new ResizeObserver(() => this.measure());
    this.resize.observe(host);
    this.measure();
  }

  private layer(name: string): SVGGElement {
    const group = document.createElementNS(SVG_NS, 'g');
    group.classList.add(`layer-${name}`);
    this.svg.append(group);
    return group;
  }

  private get height(): number {
    return PAD_TOP + PLOT_HEIGHT + (this.showXAxis ? X_AXIS_HEIGHT : NO_AXIS_HEIGHT);
  }

  private get plotWidth(): number {
    return Math.max(10, this.width - GUTTER_LEFT - PAD_RIGHT);
  }

  private measure(): void {
    const next = this.svg.clientWidth || this.svg.parentElement?.clientWidth || 600;

    if (Math.abs(next - this.width) < 0.5) return;

    this.width = next;
    this.drawn = null;
    this.render();
  }

  setEnabled(enabled: boolean): void {
    if (this.enabled === enabled) return;

    this.enabled = enabled;
    this.svg.classList.toggle('is-disabled', !enabled);
    this.render();
  }

  setData(rows: readonly Row[], xDomain: [number, number], xTicks: number[]): void {
    this.rows = rows;
    this.xDomain = xDomain;
    this.xTicks = xTicks;

    // Dragging one curve re-renders the app, but the other two charts are drawing the same picture they
    // already drew. Skipping those is the difference between one chart's worth of work per frame and
    // three. State the pointer reads — rows, domain — is assigned above either way.
    if (this.signature() === this.drawn) return;

    this.render();
  }

  /**
   * Everything the drawing depends on, as one comparable value.
   *
   * Only this chart's own measure is included, so an edit to another curve does not invalidate this one.
   */
  private signature(): string {
    const points = this.rows.map((row) => `${row.d}:${row[this.def.key]}`).join(',');
    return `${this.width}|${this.enabled ? 1 : 0}|${this.xDomain.join('-')}|${this.xTicks.join('-')}|${points}`;
  }

  // MARK: - Scales

  private x(distance: number): number {
    const [lo, hi] = this.xDomain;
    const span = hi - lo || 1;
    return GUTTER_LEFT + ((distance - lo) / span) * this.plotWidth;
  }

  private distanceAt(clientX: number): number {
    const box = this.svg.getBoundingClientRect();
    const [lo, hi] = this.xDomain;
    const ratio = (clientX - box.left - GUTTER_LEFT) / this.plotWidth;
    return lo + clamp(ratio, 0, 1) * (hi - lo);
  }

  private y(value: number): number {
    const [lo, hi] = this.resolveYDomain();
    const span = hi - lo || 1;
    return PAD_TOP + (1 - (value - lo) / span) * PLOT_HEIGHT;
  }

  private valueAt(clientY: number): number {
    const box = this.svg.getBoundingClientRect();
    const [lo, hi] = this.resolveYDomain();
    const ratio = (clientY - box.top - PAD_TOP) / PLOT_HEIGHT;
    return hi - clamp(ratio, 0, 1) * (hi - lo);
  }

  /**
   * The vertical domain, recomputed only when the data no longer sits comfortably inside it.
   *
   * A domain derived fresh from the data every frame would rescale under the pointer mid-drag, so the
   * point would chase the cursor. Holding the domain until the data leaves it, or shrinks to fill less
   * than half of it, keeps dragging predictable without stranding the curve in a corner.
   */
  private resolveYDomain(): [number, number] {
    if (this.rows.length === 0) return this.yDomain ?? [0, 1];

    const values = this.rows.map((row) => row[this.def.key]);
    const min = Math.min(...values);
    const max = Math.max(...values);

    if (this.dragIndex !== null && this.yDomain) {
      const [lo, hi] = this.yDomain;
      if (min >= lo && max <= hi) return this.yDomain;
    }

    if (this.yDomain) {
      const [lo, hi] = this.yDomain;
      const fillsEnough = (max - min) >= 0.35 * (hi - lo);
      if (min >= lo && max <= hi && fillsEnough) return this.yDomain;
    }

    this.yDomain = niceDomain(min, max, this.def.step);
    return this.yDomain;
  }

  // MARK: - Render

  private render(): void {
    if (this.width <= 0) return;

    this.svg.setAttribute('width', String(this.width));
    this.svg.setAttribute('height', String(this.height));
    this.svg.setAttribute('viewBox', `0 0 ${this.width} ${this.height}`);

    this.hit.setAttribute('x', String(GUTTER_LEFT));
    this.hit.setAttribute('y', String(PAD_TOP));
    this.hit.setAttribute('width', String(this.plotWidth));
    this.hit.setAttribute('height', String(PLOT_HEIGHT));

    const [yLo, yHi] = this.resolveYDomain();
    const yTicks = niceTicks(yLo, yHi, 4);

    replaceChildren(this.gridLayer);
    replaceChildren(this.axisLayer);
    replaceChildren(this.curveLayer);
    replaceChildren(this.pointLayer);

    for (const tick of yTicks) {
      const y = this.y(tick);
      this.gridLayer.append(this.line(GUTTER_LEFT, y, GUTTER_LEFT + this.plotWidth, y, 'gridline'));
      this.axisLayer.append(this.text(GUTTER_LEFT - 8, y + 4, formatTick(tick, this.def.decimals), 'tick tick-y'));
    }

    for (const tick of this.xTicks) {
      const x = this.x(tick);
      this.gridLayer.append(this.line(x, PAD_TOP, x, PAD_TOP + PLOT_HEIGHT, 'gridline gridline-v'));

      if (this.showXAxis) {
        this.axisLayer.append(
          this.text(x, PAD_TOP + PLOT_HEIGHT + 17, formatTick(tick, tick % 1 === 0 ? 0 : 1), 'tick tick-x'),
        );
      }
    }

    this.axisLayer.append(
      this.line(GUTTER_LEFT, PAD_TOP + PLOT_HEIGHT, GUTTER_LEFT + this.plotWidth, PAD_TOP + PLOT_HEIGHT, 'baseline'),
    );

    if (this.rows.length === 0) {
      this.drawn = this.signature();
      return;
    }

    const points = this.rows.map((row) => `${this.x(row.d)},${this.y(row[this.def.key])}`);

    const area = document.createElementNS(SVG_NS, 'polygon');
    area.classList.add('curve-area');
    area.setAttribute(
      'points',
      `${GUTTER_LEFT},${PAD_TOP + PLOT_HEIGHT} ${points.join(' ')} ${GUTTER_LEFT + this.plotWidth},${PAD_TOP + PLOT_HEIGHT}`,
    );

    const curve = document.createElementNS(SVG_NS, 'polyline');
    curve.classList.add('curve-line');
    curve.setAttribute('points', points.join(' '));

    this.curveLayer.append(area, curve);

    this.rows.forEach((row, index) => this.pointLayer.append(this.point(row, index)));

    this.drawn = this.signature();
  }

  private point(row: Row, index: number): SVGGElement {
    const value = row[this.def.key];
    const group = document.createElementNS(SVG_NS, 'g');
    group.classList.add('point');
    group.dataset.index = String(index);

    const dot = document.createElementNS(SVG_NS, 'circle');
    dot.classList.add('point-dot');
    dot.setAttribute('cx', String(this.x(row.d)));
    dot.setAttribute('cy', String(this.y(value)));
    dot.setAttribute('r', '5');

    // Focusable and arrow-key adjustable, so the curve is editable without a pointer and every value
    // a tooltip would show is reachable from the keyboard.
    group.setAttribute('tabindex', this.enabled ? '0' : '-1');
    group.setAttribute('role', 'slider');
    group.setAttribute('aria-valuenow', formatValue(value, this.def));
    group.setAttribute(
      'aria-label',
      `${this.def.label} at ${row.d.toFixed(2)} feet: ${formatValue(value, this.def)} ${this.def.unit}`,
    );

    // Marks the point the pointer would grab, so it is obvious which one a drag will move.
    group.classList.toggle('is-near', index === this.nearIndex || index === this.dragIndex);
    group.classList.toggle('is-selected', index === this.selectedIndex);

    group.append(dot);
    return group;
  }

  setHover(distance: number | null): void {
    if (distance === null || this.rows.length === 0) {
      this.hoverLayer.classList.add('is-hidden');
      return;
    }

    this.hoverLayer.classList.remove('is-hidden');

    const x = this.x(distance);
    this.crosshair.setAttribute('x1', String(x));
    this.crosshair.setAttribute('x2', String(x));
    this.crosshair.setAttribute('y1', String(PAD_TOP));
    this.crosshair.setAttribute('y2', String(PAD_TOP + PLOT_HEIGHT));

    const y = this.y(interpolate(this.rows, this.def.key, distance));
    const onExisting = indexOfRange(this.rows, distance) >= 0;

    // A hollow ring where a new point would land, filled where one already is: the ring shows the value
    // the range would take if added, which is the interpolated value on this curve.
    this.marker.classList.toggle('is-hidden', !onExisting);
    this.marker.setAttribute('cx', String(x));
    this.marker.setAttribute('cy', String(y));

    this.ghost.classList.toggle('is-hidden', onExisting || !this.enabled);
    this.ghost.setAttribute('cx', String(x));
    this.ghost.setAttribute('cy', String(y));
  }

  // MARK: - Interaction

  private bindPointer(): void {
    this.hit.addEventListener('pointermove', (event) => {
      if (this.dragIndex !== null) {
        this.dragTo(event);
        return;
      }

      this.setNear(this.nearestPoint(event.clientX, event.clientY));

      this.callbacks.onHover({
        distance: snapToGrid(this.distanceAt(event.clientX)),
        clientX: event.clientX,
        clientY: event.clientY,
      });
    });

    for (const target of [this.hit, this.svg]) {
      target.addEventListener('pointerleave', () => {
        if (this.dragIndex !== null) return;

        this.setNear(null);
        this.callbacks.onHover(null);
      });
    }

    this.hit.addEventListener('pointerdown', (event) => {
      if (!this.enabled || !event.isPrimary) return;

      const index = this.nearestPoint(event.clientX, event.clientY);
      if (index === null) return;

      event.preventDefault();
      this.select(this.rows[index]?.d ?? null);
      this.dragIndex = index;
      this.dragOrigin = { x: event.clientX, y: event.clientY };
      this.dragMoved = false;
      this.dragShift = event.shiftKey;
      this.setNear(index);
      this.svg.classList.add('is-dragging');

      // Captured on the svg, never on the point. Every edit re-renders, which replaces the point
      // elements; capturing one of those would release the capture the moment it was removed from the
      // DOM, and the release would then land somewhere that never ends the drag.
      this.hit.setPointerCapture?.(event.pointerId);
      this.callbacks.onEditStart();
    });

    this.hit.addEventListener('click', (event) => {
      if (!this.enabled) return;

      // A release that ends a drag must not also add a range.
      if (this.nearestPoint(event.clientX, event.clientY) !== null) return;

      this.select(null);

      const distance = snapToGrid(this.distanceAt(event.clientX));
      if (indexOfRange(this.rows, distance) < 0) this.callbacks.onAddRange(distance);
    });

    for (const event of ['pointerup', 'pointercancel', 'lostpointercapture'] as const) {
      this.hit.addEventListener(event, () => this.endDrag());
      // Also on the window: if the capture is ever lost, the release lands somewhere else entirely, and
      // a drag left running would keep editing on every later mouse move. Tied to this chart's lifetime,
      // since a window listener is the one kind that outlives the element it was made for.
      window.addEventListener(event, () => this.endDrag(), { signal: this.listeners.signal });
    }

    // On the window rather than the points: a point can only be keyboard-focused by tabbing, which
    // Safari does not do by default on macOS, and every edit rebuilds the points and drops focus anyway.
    // Selection is explicit instead, so the keys work the moment a point has been pressed.
    window.addEventListener('keydown', (event) => this.onKeyDown(event), { signal: this.listeners.signal });

    // Tabbing to a point still selects it, so the keyboard-only path lands in the same place.
    this.pointLayer.addEventListener('focusin', (event) => {
      const index = indexFromEvent(event);
      const row = index === null ? null : this.rows[index];
      if (!row) return;

      this.select(row.d);

      const box = this.svg.getBoundingClientRect();
      this.callbacks.onHover({
        distance: row.d,
        clientX: box.left + this.x(row.d),
        clientY: box.top + this.y(row[this.def.key]),
      });
    });
  }

  /** The selected range's current index, or null if nothing is selected or it has been removed. */
  private get selectedIndex(): number | null {
    if (this.selectedRange === null) return null;

    const index = indexOfRange(this.rows, this.selectedRange);
    return index < 0 ? null : index;
  }

  /** Takes or drops the keyboard selection. Passing a distance no longer in the table clears it. */
  select(distance: number | null): void {
    const next = distance !== null && indexOfRange(this.rows, distance) >= 0 ? distance : null;

    if (this.selectedRange === next) return;

    this.selectedRange = next;
    this.applySelectionClass();
    this.callbacks.onSelect(next);
  }

  private applySelectionClass(): void {
    const selected = this.selectedIndex;

    for (const group of this.pointLayer.querySelectorAll('.point')) {
      const at = Number((group as HTMLElement).dataset.index);
      group.classList.toggle('is-selected', at === selected);
    }
  }

  private onKeyDown(event: KeyboardEvent): void {
    if (!this.enabled) return;

    // Never while someone is typing: the values table and the add-range box are full of number inputs
    // whose own arrow-key behaviour must win. Checked with instanceof because a key event on the window
    // can carry the window or the document as its target, neither of which has closest().
    const target = event.target;
    if (target instanceof Element && target.closest('input, textarea, select, [contenteditable="true"]')) {
      return;
    }

    // Leave the app's own shortcuts, and the browser's, alone.
    if (event.ctrlKey || event.metaKey || event.altKey) return;

    const index = this.selectedIndex;
    if (index === null) return;

    const row = this.rows[index];
    if (!row) return;

    switch (event.key) {
      case 'ArrowUp':
      case 'ArrowDown': {
        event.preventDefault();
        const direction = event.key === 'ArrowUp' ? 1 : -1;
        const notches = event.shiftKey ? 10 : 1;
        this.callbacks.onEditStart();
        this.callbacks.onEdit(index, snapToStep(row[this.def.key] + direction * notches * this.def.step, this.def));
        this.callbacks.onEditEnd();
        return;
      }

      case 'ArrowLeft':
      case 'ArrowRight': {
        // Moves the selection between ranges, so the keyboard alone can reach every point.
        event.preventDefault();
        const next = this.rows[index + (event.key === 'ArrowRight' ? 1 : -1)];
        if (next) this.select(next.d);
        return;
      }

      case 'l':
      case 'L':
        event.preventDefault();
        this.callbacks.onLevelRange(index);
        return;

      case 'Delete':
      // macOS labels its Backspace key "delete", and that is the one people reach for.
      case 'Backspace':
        event.preventDefault();
        this.callbacks.onRemoveRange(index);
        return;

      case 'Escape':
        event.preventDefault();
        this.select(null);
        return;

      default:
        return;
    }
  }

  /** Releases the listeners that outlive the chart's own DOM. */
  destroy(): void {
    this.endDrag();
    this.listeners.abort();
    this.resize.disconnect();
    this.svg.remove();
  }

  private dragTo(event: PointerEvent): void {
    if (this.dragIndex === null) return;

    // Self-heal: if the button is no longer down, the release was missed and this is a plain hover.
    // Without this, one lost pointerup turns every later mouse move into an edit of a stale range.
    if (event.buttons === 0) {
      this.endDrag();
      return;
    }

    // Hold the value until the pointer has actually travelled, so a click stays a click.
    if (!this.dragMoved) {
      const origin = this.dragOrigin;
      const travelled = origin
        ? Math.hypot(event.clientX - origin.x, event.clientY - origin.y)
        : DRAG_THRESHOLD;

      if (travelled < DRAG_THRESHOLD) return;
      this.dragMoved = true;
    }

    const value = snapToStep(this.valueAt(event.clientY), this.def, event.shiftKey);
    this.callbacks.onEdit(this.dragIndex, value);

    const row = this.rows[this.dragIndex];
    if (row) {
      this.callbacks.onHover({ distance: row.d, clientX: event.clientX, clientY: event.clientY });
    }
  }

  private endDrag(): void {
    if (this.dragIndex === null) return;

    const index = this.dragIndex;
    const wasClick = !this.dragMoved;
    const withShift = this.dragShift;

    this.dragIndex = null;
    this.dragOrigin = null;
    this.dragMoved = false;
    this.dragShift = false;
    this.svg.classList.remove('is-dragging');
    this.callbacks.onEditEnd();

    // Shift-click, decided here rather than on press: until the pointer is released there is no telling
    // a click from the start of a shift-drag, which means something else entirely.
    if (wasClick && withShift) this.callbacks.onLevelRange(index);
  }

  /** The point the pointer would grab, by straight-line distance, or nothing if none is close. */
  private nearestPoint(clientX: number, clientY: number): number | null {
    const box = this.svg.getBoundingClientRect();
    const x = clientX - box.left;
    const y = clientY - box.top;

    let best: number | null = null;
    let bestDistance = GRAB_RADIUS;

    this.rows.forEach((row, index) => {
      const distance = Math.hypot(this.x(row.d) - x, this.y(row[this.def.key]) - y);
      if (distance <= bestDistance) {
        bestDistance = distance;
        best = index;
      }
    });

    return best;
  }

  private setNear(index: number | null): void {
    if (this.nearIndex === index) return;

    this.nearIndex = index;
    this.hit.classList.toggle('is-over-point', index !== null);

    for (const group of this.pointLayer.querySelectorAll('.point')) {
      const at = Number((group as HTMLElement).dataset.index);
      group.classList.toggle('is-near', at === index);
    }
  }

  // MARK: - SVG helpers

  private line(x1: number, y1: number, x2: number, y2: number, className: string): SVGLineElement {
    const line = document.createElementNS(SVG_NS, 'line');
    line.setAttribute('x1', String(x1));
    line.setAttribute('y1', String(y1));
    line.setAttribute('x2', String(x2));
    line.setAttribute('y2', String(y2));
    line.setAttribute('class', className);
    return line;
  }

  private text(x: number, y: number, content: string, className: string): SVGTextElement {
    const text = document.createElementNS(SVG_NS, 'text');
    text.setAttribute('x', String(x));
    text.setAttribute('y', String(y));
    text.setAttribute('class', className);
    text.textContent = content;
    return text;
  }
}

function indexFromEvent(event: Event): number | null {
  const group = (event.target as Element | null)?.closest('.point');
  const raw = (group as HTMLElement | null)?.dataset.index;
  return raw === undefined ? null : Number(raw);
}

function replaceChildren(element: Element): void {
  while (element.firstChild) element.firstChild.remove();
}

function clamp(value: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, value));
}

/** A padded, rounded domain that never collapses on a flat curve. */
function niceDomain(min: number, max: number, step: number): [number, number] {
  const span = Math.max(max - min, step * 8);
  const pad = span * 0.22;
  const lo = min - pad;
  const hi = max + pad;
  const unit = tickUnit(hi - lo, 4);

  return [Math.floor(lo / unit) * unit, Math.ceil(hi / unit) * unit];
}

function niceTicks(lo: number, hi: number, count: number): number[] {
  const unit = tickUnit(hi - lo, count);
  const ticks: number[] = [];

  for (let tick = Math.ceil(lo / unit) * unit; tick <= hi + unit * 1e-6; tick += unit) {
    ticks.push(Math.abs(tick) < unit * 1e-6 ? 0 : tick);
  }

  return ticks;
}

function tickUnit(span: number, count: number): number {
  const rough = Math.abs(span || 1) / Math.max(1, count);
  const magnitude = 10 ** Math.floor(Math.log10(rough));
  const normalized = rough / magnitude;
  const snapped = normalized >= 5 ? 10 : normalized >= 2 ? 5 : normalized >= 1 ? 2 : 1;

  return snapped * magnitude;
}

function formatTick(value: number, decimals: number): string {
  const shown = Math.min(decimals, value % 1 === 0 ? 0 : decimals);
  return value.toFixed(shown);
}

export { niceTicks, tickUnit };
