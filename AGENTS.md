# AGENTS.md — WPILib 2027 Java Libraries Reference

This document summarizes the new/updated WPILib Java libraries introduced for the 2027 season:
**CommandsV3** (`commandsv3`), **Tunables** (`tunables`), **OpModes** (design in
`design-docs/opmodes.md`), and **Units** (`wpiunits`). It's intended both as an onboarding
reference for programmers and as context for AI coding agents working in this repository.

Package roots:
- `org.wpilib.command3` — CommandsV3 (`commandsv3/src/main/java`)
- `org.wpilib.tunable` — Tunables (`tunables/src/main/java`)
- `org.wpilib.units`, `org.wpilib.units.measure` — Units (`wpiunits/src/main/java`)

---

## 1. CommandsV3 (`org.wpilib.command3`)

### 1.1 Core problem it solves

V1/V2 commands split logic across `initialize()`/`execute()`/`isFinished()`/`end()`. V3 uses
Java 21 **continuations** (`Coroutine`/`Continuation`/`ContinuationScope`) so a command body is a
single method that "pauses" with `coroutine.yield()` instead of being called repeatedly by the
scheduler:

```java
void commandBody(Coroutine coroutine) {
    initialize();
    while (!isFinished()) {
        execute();
        coroutine.yield(); // MUST be called every loop iteration, or the whole robot program hangs
    }
    end();
}
```

### 1.2 Key types

| Type | Role |
|---|---|
| `Mechanism` | Replaces `SubsystemBase`. Represents a physical mechanism/hardware group. Provides `run(Consumer<Coroutine>)`, `runRepeatedly(Runnable)`, `idle()`, `idleFor(Time)`, `setDefaultCommand(Command)`. |
| `Command` | Interface: `run(Coroutine)`, `name()` (required), `requirements()` (set of `Mechanism`s), `priority()` (default `0`; `LOWEST_PRIORITY`/`HIGHEST_PRIORITY` exist). Annotated `@NoDiscard` — building one and never using it is a **compile error** via the WPILib javac plugin. |
| `Coroutine` | Injected into `run()`. Key methods: `yield()`, `park()` (yields forever), `fork(Command...)` (schedule without waiting), `await(Command)` (schedule + block until done), `awaitAll(...)`, `awaitAny(...)`, `wait(Time)`, `scheduler()`. |
| `Scheduler` | Runs everything. Call `Scheduler.getDefault().run()` every robot loop (e.g. in `robotPeriodic()`). Single-threaded only — **never use from multiple threads or virtual threads**; can crash the JVM. |
| `Trigger` | Wraps a `BooleanSupplier`. Bind with `onTrue`/`onFalse`/`whileTrue`/`whileFalse`/`retryWhileTrue`/`retryWhileFalse`/`toggleOnTrue`/`toggleOnFalse`. |
| `StateMachine` | Declarative state machine: `addState(Command)`, `setInitialState(State)`, `state.switchTo(other).when(condition)`, `.whenComplete()`, `switchFromAny(...)`. |
| `ParallelGroup` / `ParallelGroupBuilder` | `.requiring(...)` (must all finish), `.optional(...)` (canceled when required ones finish), `.until(condition)`. Claims **all** child mechanisms for the whole group duration. |
| `SequentialGroup` / `SequentialGroupBuilder` | `.andThen(...)` chaining. |
| `CommandGamepad` / `CommandJoystick` / etc. (`org.wpilib.command3.button`) | Controller wrappers returning `Trigger` objects per button. |

### 1.3 The `@NoDiscard` gotcha (very common student mistake)

`Command`-returning factory methods **must** be consumed via `coroutine.await(...)`, `coroutine.fork(...)`, or a trigger binding. Just calling `someCommand();` inside another command's body builds a `Command` object and throws it away — it never runs. The WPILib javac plugin (`javacPlugin/`) flags this as a compile error:
`"Result of method returning @NoDiscard type Command is ignored"`.

```java
// WRONG — builds and discards a Command, does nothing:
otherCommand();

// RIGHT:
coroutine.await(otherCommand());
```

### 1.4 `await()` runs a command to full completion — it does not interleave per-tick

`coroutine.await(cmd)` schedules `cmd` and loops `yield()` until that *specific instance* finishes.
Each call builds a **brand-new** `Command` object with its own fresh `Coroutine`/`Continuation` —
there is no concept of "resuming" a previous run; a completed continuation can never be remounted.

```java
// This is a SEQUENCE, not per-tick interleaving:
while (condition) {
    coroutine.await(anotherCommand());     // runs fully to completion first
    coroutine.await(yetAnotherCommand());   // then this, fully, before looping
    coroutine.yield();
}
```

If you want logic that runs every tick alongside a loop, call plain methods directly instead of
wrapping them as `Command`s.

### 1.5 Command ownership / "uncommanded state" avoidance

Nested/awaited commands only own their own mechanisms **while actually running** — unlike
`ParallelGroup`/`SequentialGroup`, which claim every child's mechanisms for the entire composition
duration (even mechanisms they aren't currently using). Prefer:

```java
Command.noRequirements(coroutine -> {
    coroutine.await(elevator.moveToScoringHeight());
    coroutine.await(gripper.release());
}).named("Score");
```

over the old V2 "proxy command" workaround — in V3, all child commands behave like proxies by
default.

### 1.6 Priority vs. execution order (do not conflate these)

- **Priority** (`withPriority(int)`) only determines who wins when two commands **conflict** over
  a shared mechanism. It has no bearing on execution order between non-conflicting commands.
- To run something before everything else on a *given* tick, priority does not help.

### 1.7 Scheduler tick order — the actual `Scheduler.run()` sequence

```
1. cancelStaleBindings()      — cancel commands whose binding scope died
2. unbindStaleTriggers()      — unbind triggers whose creation scope died
3. runPeriodicSideloads()     — run addPeriodic()/sideload() callbacks (ALWAYS before any command)
4. m_eventLoop.poll()         — poll Triggers, queue/cancel bound commands
5. scheduleDefaultCommands()  — queue default commands for idle mechanisms
6. promoteScheduledCommands() — move queued commands into the running set
7. runCommands()              — mount + run every running command until yield()/exit
```

**Use `Scheduler.addPeriodic(Runnable)` / `coroutine.scheduler().sideload(Consumer<Coroutine>)`**
for any shared computation (e.g. odometry updates) that multiple parallel commands depend on and
that must be fresh **before** those commands run, every tick. This is the direct replacement for
V2's automatic `Subsystem.periodic()`. Sideloaded callbacks should not directly drive
mechanisms/motors — only compute/cache shared state.

```java
coroutine.scheduler().addPeriodic(() ->
    odometry.update(gyro.getAngle(), leftEncoder.getDistance(), rightEncoder.getDistance()));
```

Calling `sideload`/`addPeriodic` from inside a running command scopes it to that command's
lifetime automatically (auto-unregistered when the command ends).

### 1.8 Fork/scheduling order nuances (implementation detail — do not rely on it for correctness)

- Forking `coroutine.fork(a, b, c)` from inside a running command executes `a`, then `b`, then `c`
  **immediately and synchronously**, in that exact order, on the tick `fork()` is called (schedule()
  recursively runs each child right away when called from a mounted command).
- On **every subsequent tick**, all running commands (siblings + parent) are stored in a
  `LinkedHashMap` (`m_runningCommands`) and iterated in **reverse insertion order** in
  `runCommands()` — so children run before their parent (so a parent's `await()` can resolve in the
  same tick a child finishes). This means `a, b, c` will run in **reverse** (`c, b, a`) from tick 2
  onward, not the order passed to `fork()`. The class-level Javadoc claims "commands run in the
  order they were scheduled," which does not match this reversed iteration — treat as an
  undocumented implementation detail, not an API contract.
- **Do not depend on fork-argument order for per-tick behavior.** Use `addPeriodic`/`sideload` (see
  1.7) for anything that truly needs "runs before everything else, every tick" semantics.

### 1.9 Triggers created inside a running command

```java
Command outer = Command.noRequirements(coroutine -> {
    Trigger t = new Trigger(() -> condition());   // creation scope = ForCommand(scheduler, outer)
    t.onTrue(nestedCommand());                     // binding scope also = ForCommand(scheduler, outer)
    coroutine.await(somethingElse());
}).named("Outer");
```

- The trigger's creation scope and each binding's scope are computed independently via
  `BindingScope.createNarrowestScope()`, which returns `ForCommand(scheduler, currentCommand)` when
  created inside a running command, `ForOpmode(id)` inside an opmode, or `Global` otherwise.
- When the bound trigger fires, `Scheduler.schedule(Binding)` sets `nestedCommand`'s parent to
  `outer` (from the binding's scope) — making it a genuine child in the command hierarchy, even
  though `outer` never called `fork()`/`await()` on it directly.
- Trigger-scheduled commands are **queued** (`m_queuedToRun`), not run synchronously like
  `fork()` — they get their first execution turn later in the *same* tick, after
  `promoteScheduledCommands()`.
- Two independent cleanup mechanisms exist: (1) the whole trigger is unbound
  (`unbindStaleTriggers()`) once its creation scope (`outer`) stops being active; (2) individual
  bindings are checked every `poll()` via `clearStaleBindings()`; (3) separately, if `outer`
  completes/cancels, `removeOrphanedChildren(outer)` cancels `nestedCommand` directly regardless of
  trigger state. Scoped trigger bindings never outlive their enclosing command.

### 1.10 Nested parallel commands with controlled start order

`ParallelGroup` starts every command in the group simultaneously — no built-in staggering. For
controlled/staggered starts while still running concurrently, write a custom `noRequirements`
command and call `fork()` at the moments you choose, keeping the returned `ForkResult` to sync up
later with `.awaitCompletion()`:

```java
Command choreographed = Command.noRequirements(coroutine -> {
    var driveTask = coroutine.fork(drivetrain.driveToScoringPosition());
    while (!drivetrain.isPastHalfway()) {
        coroutine.yield();
    }
    var elevatorTask = coroutine.fork(elevator.moveToScoringHeight());

    driveTask.awaitCompletion();
    elevatorTask.awaitCompletion();
}).named("Choreographed");
```

---

## 2. Tunables (`org.wpilib.tunable`)

Replaces ad hoc `SmartDashboard.putNumber`/`getNumber` calls and `SendableChooser` with a typed,
pluggable-backend API (NetworkTables by default; `MockTunableBackend` for unit tests).

### 2.1 Key types

| Type | Role |
|---|---|
| `Tunables` (static utility) | `Tunables.addDouble/addInt/addLong/addFloat/addBoolean(name, initial)`, `Tunables.getTable(name)`, `Tunables.publish(name, tunable)`. |
| `Tunable<T>` | Generic wrapper; `Supplier<T>`/`Consumer<T>`. Primitive-specialized subclasses avoid boxing: `TunableBoolean`, `TunableInt`, `TunableLong`, `TunableFloat`, `TunableDouble`. |
| `TunableTable` | Hierarchical namespace ("folder"); `getTable(name)` nests tables. |
| `ComplexTunable` | Interface for objects with multiple related tunables (e.g. a PID controller); implement `publishTunable(TunableTable)`. |
| `Selectable<V>` | Dropdown chooser (`SendableChooser` replacement): `add(name, value)`, `addDefault(...)`, `getSelected()`, `onChange(listener)`. |
| `TunableConfig` / `TunableOption` | Metadata: `ROBUST`/`NOT_ROBUST`, `MUTABLE`/`IMMUTABLE`, `ALWAYS_GET`/`GET_ON_CHANGE` polling, `onTune(Runnable)` callback, custom `property(key, value)`. |

### 2.2 Creation patterns

- **Internal value**: `Tunables.addDouble("kP", 0.5)` — value stored inside the tunable.
- **Getter/setter delegation**: `Tunables.publishDouble(name, getter, setter)` — delegates to
  existing fields; defaults to `ALWAYS_GET` polling unless configured otherwise.
- **Read-only telemetry**: pass a no-op setter and/or `TunableOption.IMMUTABLE`.
- **React to dashboard changes**: `TunableConfig.of(TunableOption.onTune(() -> controller.setP(kP.getAsDouble())))`.

```java
TunableDouble kP = Tunables.addDouble("Elevator/kP", 0.5);
controller.setP(kP.getAsDouble()); // always reads the live value
```

---

## 3. OpModes (design: `design-docs/opmodes.md`)

Replaces the single `Robot` class's `teleopPeriodic()`/`autonomousPeriodic()` methods with
separate, annotated classes selectable from a Driver Station dropdown (borrows FTC's concept,
integrates FRC's enable/disable safety model).

### 3.1 Key types

| Type | Role |
|---|---|
| `OpModeRobot` | Base class for the team's `Robot`. Auto-scans the package for `@Autonomous`/`@Teleop`/`@Utility` classes and publishes them to the DS. Holds shared hardware (mechanisms, controllers) constructed once. |
| `OpMode` / `PeriodicOpMode` | Per-mode class. Lifecycle: `disabledPeriodic()` → `start()` (once, on enable) → `periodic()` (while enabled) → `end()` → `close()`. A fresh instance is constructed each time the opmode is (re)selected. |
| `@Autonomous` / `@Teleop` / `@Utility` | Class annotations: optional `name`, `group` (DS dropdown grouping), `description`, colors. |

### 3.2 Scoping — the key mechanic

Default commands and trigger bindings set/created inside an opmode's constructor are automatically
scoped to that opmode (via the same `BindingScope` mechanism as commands) and are cleaned up when
the opmode exits — no manual teardown required.

### 3.3 Instructor/student control-scheme patterns

- **Priority override**: student drive is the mechanism's default command at low/negative
  priority; instructor's drive command runs at a higher priority bound to `whileTrue` on a
  button — it interrupts the student's command while held and the student's default command
  resumes automatically on release.
- **Explicit hand-off**: instructor explicitly grants/revokes control via buttons that schedule a
  "student control" command (normal priority) or an "instructor reclaim" command (very high
  priority, interrupts anything).
- **`StateMachine`-based hand-off**: cleanest for named, explicit states — `instructorControl` and
  `studentControl` states with `switchTo(...).when(...)` transitions, plus
  `switchFromAny().to(instructorControl).when(...)` for an always-available reclaim path.

---

## 4. Units (`org.wpilib.units`, `org.wpilib.units.measure`)

Type-safe physical units library used throughout WPILib APIs (e.g. `Time` parameters on
`Coroutine.wait(Time)`, `Command.idleFor(Time)`) to prevent unit-mismatch bugs (the classic
"is this meters or feet?" mistake) at compile time.

### 4.1 Core types

| Type | Role |
|---|---|
| `Unit` (abstract) | Defines a unit: conversion functions to/from its base unit, `name()`, `symbol()`, `of(double)` factory. Subclasses: `DistanceUnit`, `TimeUnit`, `AngleUnit`, `LinearVelocityUnit`, `AngularVelocityUnit`, `VoltageUnit`, `CurrentUnit`, `ForceUnit`, `TorqueUnit`, `MassUnit`, `EnergyUnit`, `PowerUnit`, `TemperatureUnit`, `ResistanceUnit`, `FrequencyUnit`, `DimensionlessUnit`, etc. |
| `Measure<U>` | Immutable magnitude + unit pair. `.magnitude()`, `.baseUnitMagnitude()`, `.in(otherUnit)` for conversion, arithmetic: `.plus()`, `.minus()`, `.times()`, `.divide()`, `.unaryMinus()`, `Comparable`. Concrete measure interfaces live in `org.wpilib.units.measure` (e.g. `Time`, `Distance`, `Angle`, `LinearVelocity`, `Voltage`, `Force`...). |
| `Units` (static constants) | All predefined units, e.g. `Meters`, `Feet`, `Inches`, `Seconds`, `Minutes`, `Degrees`, `Radians`, `Rotations`, `MetersPerSecond`, `RPM`, `Volts`, `Amps`, `Newtons`, `Watts`, `Ohms`, `Hertz`. Intended to be statically imported: `import static org.wpilib.units.Units.*;`. |
| `PerUnit<N, D>` / `Per` | Derived "ratio" unit from `numerator.per(denominator)`, e.g. `Meters.per(Second)` → `MetersPerSecond`; `Volts.per(MetersPerSecond)` for feedforward gains (kV). |
| `MultUnit` / `Mult` | Derived "product" unit from `.times(...)`. |

### 4.2 Common usage pattern

```java
import static org.wpilib.units.Units.*;

Distance target = Meters.of(2.5);
double feet = target.in(Feet);              // 8.202...

Time timeout = Seconds.of(2.0);
coroutine.wait(timeout);

LinearVelocity maxSpeed = MetersPerSecond.of(4.5);
Voltage kV = Volts.per(MetersPerSecond).of(2.3); // feedforward gain, still type-checked
```

### 4.3 Why it matters for teaching

- Compile-time dimensional safety: you cannot accidentally pass a `Distance` where a `Time` is
  expected, or add `Meters` to `Feet` without an explicit, correct conversion — the library
  converts through the base unit internally.
- Derived units (`per()`, `times()`) are cached (`CombinatoryUnitCache`) so expressions like
  `Volts.per(MetersPerSecond)` don't allocate on every call after the first.
- Prefer `Units.*` static imports and typed `Measure` parameters over raw `double`s in method
  signatures for anything physical (distances, times, voltages, velocities) — this is the
  convention used throughout `commandsv3` (e.g. `Time` params on `Coroutine.wait`, `Command.idleFor`,
  `Command.withTimeout`).

---

## 5. Quick Reference: Common Gotchas

| Gotcha | Fix |
|---|---|
| Forgetting `coroutine.yield()` in a loop | Freezes the entire robot program — always yield once per loop iteration. |
| Calling a `Command`-returning method without `await`/`fork`/trigger binding | Compile error via `@NoDiscard` javac plugin — must consume the returned `Command`. |
| Expecting `await()` in a loop to interleave per-tick with the outer loop | It runs the awaited command to full completion first; use plain method calls for true per-tick logic. |
| Relying on `fork(a, b, c)` order for ongoing per-tick behavior | Only guaranteed on the triggering tick; reverses on subsequent ticks. Use `addPeriodic`/`sideload` instead. |
| Using `ParallelGroup` expecting staggered starts | It starts everything at once; write a custom coroutine body with manually-timed `fork()` calls for staggering. |
| Confusing priority with execution order | Priority only resolves mechanism conflicts, not tick-by-tick ordering. |
| Manually tracking trigger cleanup inside commands/opmodes | Unnecessary — triggers/bindings created inside a command or opmode are auto-scoped and auto-cleaned-up. |
| Using raw `double` for physical quantities | Use `Units` `Measure` types (`Time`, `Distance`, etc.) for compile-time safety. |
