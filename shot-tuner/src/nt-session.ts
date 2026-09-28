/**
 * The NetworkTables side of the tuner: the eight topics one tunable shot profile exposes, and the rules
 * about when this dashboard is allowed to write to them.
 *
 * Built on `ntcore-ts-client`, which implements NT 4.1 including the RTT clock sync. That last part
 * matters more than it looks: a value published with a timestamp of 0 means "set default" in NT4 and
 * loses to the robot's retained value, so a client that skips the sync appears to write and silently
 * does nothing.
 */

import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';
import type { NetworkTablesTopic } from 'ntcore-ts-client';

import { DISTANCE_TOPIC, SERIES, type Columns } from './shot-table';

/** What the robot publishes about the table it accepted. See `TunableShotProfile`. */
export interface Diagnostics {
  accepted: boolean | null;
  rejectionReason: string | null;
  isDefault: boolean | null;
}

export interface SessionCallbacks {
  onConnectionChange(connected: boolean): void;
  /** A complete set of four columns arrived. Lengths are guaranteed to agree. */
  onColumns(columns: Columns): void;
  /** The committed table the robot was built with, for the "reset" action. */
  onCommittedColumns(columns: Columns): void;
  onDiagnostics(diagnostics: Diagnostics): void;
}

/**
 * AdvantageKit publishes `Logger.recordOutput` under this prefix, so the robot's read-only diagnostics
 * live one level deeper than the editable columns even though the robot code uses one path for both.
 */
const OUTPUT_PREFIX = '/AdvantageKit/RealOutputs';

const COLUMN_TOPICS = [DISTANCE_TOPIC, ...SERIES.map((s) => s.topic)] as const;

type ColumnName = (typeof COLUMN_TOPICS)[number];

export class NtSession {
  private readonly callbacks: SessionCallbacks;
  private nt: NetworkTables | null = null;
  private path = '';

  private columns = new Map<ColumnName, number[]>();
  private committed = new Map<ColumnName, number[]>();
  private diagnostics: Diagnostics = { accepted: null, rejectionReason: null, isDefault: null };

  private editable = new Map<ColumnName, NetworkTablesTopic<number[]>>();
  private publishing = false;

  constructor(callbacks: SessionCallbacks) {
    this.callbacks = callbacks;
  }

  get connected(): boolean {
    return this.nt?.isRobotConnected() ?? false;
  }

  /** Whether every editable column is published, i.e. whether writes will actually go out. */
  get writable(): boolean {
    return this.connected && this.editable.size === COLUMN_TOPICS.length;
  }

  get serverUri(): string {
    return this.nt?.getURI() ?? '';
  }

  /**
   * Points the session at a robot and a profile path.
   *
   * `ntcore-ts-client` keeps one instance per URI, so switching address reuses the same client and
   * changes its target rather than leaving a second socket behind.
   */
  async open(address: string, path: string): Promise<void> {
    const normalizedPath = path.replace(/\/+$/, '');
    const sameTarget = this.nt !== null && this.path === normalizedPath;

    this.path = normalizedPath;

    if (this.nt === null) {
      this.nt = NetworkTables.getInstanceByURI(address);
      this.nt.addRobotConnectionListener((connected) => this.callbacks.onConnectionChange(connected), true);
    } else if (this.nt.getURI() !== address) {
      this.nt.changeURI(address);
    }

    if (!sameTarget) {
      this.columns.clear();
      this.committed.clear();
      this.editable.clear();
      await this.subscribeAll();
    }
  }

  private async subscribeAll(): Promise<void> {
    const nt = this.nt!;

    for (const column of COLUMN_TOPICS) {
      // Subscribed and published on the same topic: the robot owns the value, this dashboard edits it.
      // Publishing up front rather than on the first edit means a write never waits on a round trip.
      const topic = nt.createTopic<number[]>(`${this.path}/${column}`, NetworkTablesTypeInfos.kDoubleArray);

      // Publish BEFORE subscribing, and keep it that way. `ntcore-ts-client` decides a topic is its own
      // publisher by comparing the announced pubuid with its own, and an announce for a topic we merely
      // subscribed to carries no pubuid — so `undefined === undefined` marks it a publisher, after which
      // publish() early-returns without ever claiming a pubuid and setValue throws "is not a publisher".
      // Publishing first assigns the pubuid, so the first announce matches it and writes go out. This
      // survives reconnects, which republish with the pubuid already assigned.
      await topic.publish();

      topic.subscribe((value) => {
        if (value === null) return;
        this.columns.set(column, [...value]);
        this.emitColumnsIfComplete();
      });

      this.editable.set(column, topic);

      // The robot mirrors the table committed in source as a read-only output, which is the only way a
      // dashboard can offer "put it back how it shipped" without the robot exposing a write-only command.
      const defaultTopic = nt.createTopic<number[]>(
        `${OUTPUT_PREFIX}${this.path}/Default/${column}`,
        NetworkTablesTypeInfos.kDoubleArray,
      );

      defaultTopic.subscribe((value) => {
        if (value === null) return;
        this.committed.set(column, [...value]);
        this.emitCommittedIfComplete();
      });
    }

    const accepted = nt.createTopic<boolean>(`${OUTPUT_PREFIX}${this.path}/Accepted`, NetworkTablesTypeInfos.kBoolean);
    accepted.subscribe((value) => this.updateDiagnostics({ accepted: value }));

    const isDefault = nt.createTopic<boolean>(`${OUTPUT_PREFIX}${this.path}/IsDefault`, NetworkTablesTypeInfos.kBoolean);
    isDefault.subscribe((value) => this.updateDiagnostics({ isDefault: value }));

    const reason = nt.createTopic<string>(
      `${OUTPUT_PREFIX}${this.path}/RejectionReason`,
      NetworkTablesTypeInfos.kString,
    );
    reason.subscribe((value) => this.updateDiagnostics({ rejectionReason: value }));
  }

  /**
   * Writes all four columns.
   *
   * Sent together so the robot sees a consistent set: it rejects a snapshot whose column lengths
   * disagree, which is exactly what it would see if a range were added to one column a frame before the
   * others.
   */
  write(columns: Columns): void {
    if (!this.writable) return;

    this.publishing = true;

    try {
      this.editable.get(DISTANCE_TOPIC)?.setValue(columns.d);

      for (const series of SERIES) {
        this.editable.get(series.topic as ColumnName)?.setValue(columns[series.key]);
      }
    } finally {
      this.publishing = false;
    }

    // Keep the local mirror in step so the echo of our own write is recognised as a no-op rather than
    // arriving as a fresh value from the robot.
    this.columns.set(DISTANCE_TOPIC, [...columns.d]);
    for (const series of SERIES) {
      this.columns.set(series.topic as ColumnName, [...columns[series.key]]);
    }
  }

  get committedColumns(): Columns | null {
    return NtSession.assemble(this.committed);
  }

  private emitColumnsIfComplete(): void {
    if (this.publishing) return;

    const assembled = NtSession.assemble(this.columns);
    if (assembled) this.callbacks.onColumns(assembled);
  }

  private emitCommittedIfComplete(): void {
    const assembled = NtSession.assemble(this.committed);
    if (assembled) this.callbacks.onCommittedColumns(assembled);
  }

  private updateDiagnostics(patch: Partial<Diagnostics>): void {
    this.diagnostics = { ...this.diagnostics, ...patch };
    this.callbacks.onDiagnostics(this.diagnostics);
  }

  /**
   * Turns four separately-arriving columns into one snapshot, or nothing.
   *
   * Each topic updates on its own, so a partial or mismatched set is normal in the moments after a
   * connection or an edit rather than an error — it resolves on the next update.
   */
  private static assemble(source: Map<ColumnName, number[]>): Columns | null {
    const d = source.get(DISTANCE_TOPIC);
    const tof = source.get(SERIES[0]!.topic as ColumnName);
    const hood = source.get(SERIES[1]!.topic as ColumnName);
    const rpm = source.get(SERIES[2]!.topic as ColumnName);

    if (!d || !tof || !hood || !rpm) return null;
    if (d.length !== tof.length || d.length !== hood.length || d.length !== rpm.length) return null;
    if (d.length === 0) return null;

    return { d: [...d], tof: [...tof], hood: [...hood], rpm: [...rpm] };
  }
}
