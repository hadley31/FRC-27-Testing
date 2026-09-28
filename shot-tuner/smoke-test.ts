/**
 * Integration smoke test: drives a real robot over NetworkTables and checks what it actually did.
 *
 * Needs a robot running (simulator or real) reachable at localhost:5810. Run with `bun run smoke`.
 *
 * This exists because the failure that matters here is invisible from the dashboard side: a write can
 * be accepted by the NT server, echo back to every other client, and still never reach the robot. Every
 * assertion below is therefore read back from what the ROBOT published about the table it accepted,
 * never from the value this process just wrote.
 */
import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';

const PATH = '/Tuning/ShotProfile/Scoring';
const OUT = '/AdvantageKit/RealOutputs' + PATH;
const COLUMNS = ['DistancesFeet', 'TimeOfFlightSeconds', 'HoodAngleDegrees', 'FlywheelRpm'];

const nt = NetworkTables.getInstanceByURI('localhost');
const seen: Record<string, unknown> = {};

const cols = COLUMNS.map((c) => nt.createTopic<number[]>(`${PATH}/${c}`, NetworkTablesTypeInfos.kDoubleArray));
for (const c of COLUMNS) {
  const t = nt.createTopic<number[]>(`${OUT}/Default/${c}`, NetworkTablesTypeInfos.kDoubleArray);
  t.subscribe((v) => { if (v) seen['default:' + c] = v; }, { all: false });
}
nt.createTopic<boolean>(`${OUT}/Accepted`, NetworkTablesTypeInfos.kBoolean)
  .subscribe((v) => { if (v !== null) seen.accepted = v; }, { all: false });
nt.createTopic<boolean>(`${OUT}/IsDefault`, NetworkTablesTypeInfos.kBoolean)
  .subscribe((v) => { if (v !== null) seen.isDefault = v; }, { all: false });
nt.createTopic<string>(`${OUT}/RejectionReason`, NetworkTablesTypeInfos.kString)
  .subscribe((v) => { if (v !== null) seen.reason = v; }, { all: false });
nt.createTopic<string>(`${OUT}/AsJava`, NetworkTablesTypeInfos.kString)
  .subscribe((v) => { if (v) seen.asJava = v; }, { all: false });

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
let failures = 0;
function check(label: string, ok: boolean, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${label}${detail ? '  — ' + detail : ''}`);
  if (!ok) failures++;
}

await sleep(2000);
// Publish before subscribing: see the note in nt-session.ts.
for (const t of cols) await t.publish();
COLUMNS.forEach((c, i) => cols[i]!.subscribe((v) => { if (v) seen[c] = v; }, { all: false }));
await sleep(1200);
check('connected', nt.isRobotConnected());
check('all columns publishable', cols.every((t) => t.publisher && t.pubuid !== undefined),
  cols.map((t) => t.pubuid).join(','));
const d = seen['DistancesFeet'] as number[] | undefined;
check('columns arrived', !!d, d ? `${d.length} ranges` : 'none');
check('18 ranges, 4..25 ft', d?.length === 18 && d?.[0] === 4 && d?.[17] === 25, JSON.stringify(d?.slice(0, 4)));
check('committed default mirrored', Array.isArray(seen['default:FlywheelRpm']),
  JSON.stringify((seen['default:FlywheelRpm'] as number[] | undefined)?.slice(0, 3)));
check('starts accepted + default', seen.accepted === true && seen.isDefault === true,
  `accepted=${seen.accepted} isDefault=${seen.isDefault}`);

// A good edit: bump the first flywheel value.
const rpm = [...(seen['FlywheelRpm'] as number[])];
rpm[0] = 1610;
cols[3]!.setValue(rpm);
await sleep(700);
check('good edit accepted', seen.accepted === true, `reason="${seen.reason}"`);
check('edit took effect', (seen['FlywheelRpm'] as number[])[0] === 1610,
  String((seen['FlywheelRpm'] as number[])[0]));
check('flagged as non-default', seen.isDefault === false, `isDefault=${seen.isDefault}`);
check('AsJava updated', typeof seen.asJava === 'string' && (seen.asJava as string).includes('1610.0'),
  (seen.asJava as string | undefined)?.split('\n')[3]);

// A torn write: one column longer than the rest.
cols[0]!.setValue([...(seen['DistancesFeet'] as number[]), 30]);
await sleep(700);
check('torn write rejected', seen.accepted === false, `reason="${seen.reason}"`);
check('previous table held', (seen['FlywheelRpm'] as number[])[0] === 1610);

// Duplicate range: same length, invalid ordering.
const dupes = [...(seen['DistancesFeet'] as number[])];
dupes.pop();
dupes[1] = dupes[0]!;
cols[0]!.setValue(dupes);
await sleep(700);
check('duplicate range rejected', seen.accepted === false && String(seen.reason).includes('strictly increase'),
  `reason="${seen.reason}"`);

// Restore the committed table, the way the dashboard's reset does.
for (let i = 0; i < COLUMNS.length; i++) {
  cols[i]!.setValue([...(seen['default:' + COLUMNS[i]!] as number[])]);
}
await sleep(700);
check('reset to committed restores default', seen.accepted === true && seen.isDefault === true,
  `accepted=${seen.accepted} isDefault=${seen.isDefault}`);

console.log(failures === 0 ? '\nALL PASS' : `\n${failures} FAILED`);
process.exit(failures === 0 ? 0 : 1);
