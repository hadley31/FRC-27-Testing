import './src/dom-test-setup';

const { CurveChart, niceTicks } = await import('./src/curve-chart');
const { ValuesTable } = await import('./src/values-table');
const { SERIES, withValue } = await import('./src/shot-table');
type Row = import('./src/shot-table').Row;

const d = [4, 5, 6, 6.7, 7, 8, 9, 10, 10.4, 11, 12, 13, 14, 15, 15.2, 16, 20, 25];
const tof = [0.9, 0.9, 0.9, 0.9, 0.941, 1.076, 1.211, 1.346, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.433, 1.6, 1.6];
const hood = [0, 3, 4.5, 6.25, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10];
const rpm = [1575, 1575, 1625, 1695, 1725, 1725, 1825, 1825, 1855, 1900, 1925, 2075, 2125, 2175, 2195, 2275, 2452.8, 2675];
let rows: Row[] = d.map((dd, i) => ({ d: dd, tof: tof[i]!, hood: hood[i]!, rpm: rpm[i]! }));

document.body.innerHTML = '<div id="charts"></div><table><tbody id="body"></tbody></table>';
const host = document.getElementById('charts')!;
const body = document.getElementById('body') as unknown as HTMLTableSectionElement;

const noop = {
  onHover() {}, onEditStart() {}, onEdit() {}, onEditEnd() {},
  onAddRange() {}, onRemoveRange() {}, onLevelRange() {}, onSelect() {},
};

const charts = SERIES.map((def, i) => {
  const card = document.createElement('section');
  host.append(card);
  return new CurveChart(card, def, noop, i === SERIES.length - 1);
});
const table = new ValuesTable(body, { onValue() {}, onRemove() {} });

const domain: [number, number] = [3, 26];
const ticks = niceTicks(3, 26, 6);

for (const chart of charts) {
  chart.setEnabled(true);
  chart.setData(rows, domain, ticks);
}
table.setRows(rows, true);

// Count DOM construction, which is what the frame budget actually goes on.
let created = 0;
const realNS = document.createElementNS.bind(document);
const real = document.createElement.bind(document);
document.createElementNS = ((...a: Parameters<typeof realNS>) => { created++; return realNS(...a); }) as typeof realNS;
document.createElement = ((...a: Parameters<typeof real>) => { created++; return real(...a); }) as typeof real;

// 60 frames of dragging the flywheel value at 12 ft, as a real drag does.
const FRAMES = 60;
const started = performance.now();

for (let frame = 0; frame < FRAMES; frame++) {
  rows = withValue(rows, 10, 'rpm', 1925 + frame * 5);

  for (const chart of charts) {
    chart.setEnabled(true);
    chart.setData(rows, domain, ticks);
    chart.setHover(12);
  }
  table.setRows(rows, true);
}

const elapsed = performance.now() - started;

console.log(`frames:                 ${FRAMES}`);
console.log(`elements created:       ${created}  (${(created / FRAMES).toFixed(1)} per frame)`);
console.log(`time in happy-dom:      ${elapsed.toFixed(0)} ms  (${(elapsed / FRAMES).toFixed(2)} ms per frame)`);
