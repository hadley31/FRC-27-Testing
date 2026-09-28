# Shot Tuner

A dashboard for editing the robot's shot profile live, the way you'd edit a fan curve: three linked
charts over one shared range axis, with draggable points.

- **Time of flight** vs range
- **Hood angle** vs range
- **Flywheel speed** vs range

Ranges are shared, so adding one adds a point to all three charts, and each new point takes the value
its own curve already had at that range — dropping a range in leaves the curves exactly where they
were. A crosshair follows the pointer across all three charts at once, with one readout for all three
measures, so correlating them never means reading three tooltips.

The robot side is [`TunableShotProfile`](../src/main/java/first/robot/util/TunableShotProfile.java);
its Javadoc is the authority on the contract.

## Running it

Needs [Bun](https://bun.sh).

```sh
bun install     # once
bun dev         # serves on http://localhost:3000
```

Then set **Robot** to `localhost` for a simulated robot, or `10.TE.AM.2` /
`roborio-TEAM-frc.local` for a real one, and press Connect. The address and profile path are
remembered per browser.

`bun run build` writes a static bundle to `dist/` if you want to host it somewhere.

## How it behaves

**The robot owns the table.** This dashboard subscribes first and stays read-only until the robot
publishes its four columns — it never writes on connect, so opening it cannot disturb a tuned robot.
It only writes once you edit something.

**Edits go out as a complete set.** All four columns are written together. The robot rejects any
snapshot whose column lengths disagree, which is exactly what it would see if a range reached one
column a frame before the others.

**The robot's verdict is the truth.** The status bar shows what the robot published about the table it
accepted, not what this page thinks it sent. A table the robot refused says so, in the robot's own
words. `Robot accepted` with a note that the table differs from source means the robot is shooting
your edits.

**Nothing here persists your work.** The table survives robot reboots (the columns are persistent NT
topics), but that is not version control — re-imaging the robot loses it. Press **Copy as Java** and
paste the result over `kScoringTable` in
[`ShotProfile.java`](../src/main/java/first/robot/util/ShotProfile.java). That is how a tuning session
is meant to end.

### Editing

Clicking a point **selects** it, and the keyboard then acts on the selection. That is the only
discoverable way in: a point can otherwise only be reached by tabbing, which Safari on macOS does not do
by default. Only one point across the three charts is ever selected, so a keypress can only move one
value.

| Action | How |
|---|---|
| Add a range | Click any chart where no point is, or type one and press Add |
| Change a value | Drag a point; hold <kbd>Shift</kbd> for fine steps |
| Flatten a kink | <kbd>Shift</kbd>-click a point, or <kbd>L</kbd> on the selected one |
| Change a value precisely | Show values, and type into the table |
| Select a point | Click it, or <kbd>Tab</kbd> to it |
| Nudge a value | <kbd>↑</kbd>/<kbd>↓</kbd> on the selection (<kbd>Shift</kbd> for ×10) |
| Move between ranges | <kbd>←</kbd>/<kbd>→</kbd> |
| Remove a range | <kbd>Delete</kbd> (or <kbd>Backspace</kbd>) on the selection, or the × in the values table |
| Deselect | <kbd>Esc</kbd>, or click empty space |
| Undo / redo | <kbd>Ctrl</kbd>+<kbd>Z</kbd> / <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>Z</kbd> |
| Start over | **Revert edits** (back to what this page loaded) or **Reset to committed** (back to `ShotProfile.java`) |

The keyboard is deliberately ignored while a text or number input has focus, so the values table and the
add-range box keep their own arrow-key behaviour.

**Shift-click levels a point.** It moves onto the straight line between its two neighbours, which takes
the kink out without disturbing anything else — useful when one range is obviously off the trend. It is
the interpolated value at that range, not the average of the neighbours: those agree only on an evenly
spaced axis, and the committed table is not evenly spaced. Only the curve you clicked moves, since the
three charts share their ranges and not their values, and the first and last range have nothing to level
against.

Ranges are added on a half-foot grid, and dragging only moves values vertically — a range's position
changes by adding or removing it, never by dragging, so points can't be dragged out of order into a
table the robot would reject. The grid is a dashboard convention, not a rule: the committed table has
points at 6.70, 10.40 and 15.20 ft, which is where the time-of-flight curve actually bends, and those
are left alone.

## Topics

Editable, directly under the profile path (default `/Tuning/ShotProfile/Scoring`):

```
DistancesFeet          double[]   shared range axis, strictly increasing
TimeOfFlightSeconds    double[]
HoodAngleDegrees       double[]
FlywheelRpm            double[]
```

Read-only. These are AdvantageKit outputs, so they sit under `/AdvantageKit/RealOutputs` plus the same
path:

```
Accepted               boolean    whether the robot took the last snapshot
RejectionReason        string     why not, in words
IsDefault              boolean    whether the live table still matches source
AsJava                 string     the live table as a Java literal
Default/<column>       double[]   the committed table, for "Reset to committed"
```

## The patched dependency

`patches/ntcore-ts-client@3.1.3.patch` fixes a bug that makes **every write silently do nothing**
against a NetworkTables 4.1 server — which is any current robot.

On a 4.1 connection the library starts no timestamp heartbeat, on the assumption that the separate
`rtt.networktables.first.wpi.edu` subprotocol carries RTT messages. No such socket is ever opened, so
the `id = -1` exchange never happens, the clock offset stays 0, and `getServerTime()` returns raw
`performance.now()` microseconds — a few seconds of page uptime instead of the robot's time base.
Values are then published with timestamps far in the past. The NT server keeps its retained value and
the robot's own subscribers discard the update as stale.

It fails in the most misleading way available: the server still fans the value out live, so a second
dashboard shows the new number, and this one shows it too. Only the robot never sees it. The patch
starts the heartbeat on 4.1 as well, which is what the spec asks of clients regardless of version —
4.1 only prefers WebSocket PING for *aliveness*, a separate job from clock sync.

The other thing to know, handled in code rather than by patch (see the note in
[`nt-session.ts`](src/nt-session.ts)): a topic must be **published before it is subscribed**. The
library decides a topic is its own publisher by comparing the announced pubuid against its own, and an
announce for a topic you merely subscribed to carries no pubuid — so `undefined === undefined` marks
it a publisher, after which `publish()` early-returns without claiming a pubuid and `setValue` throws.

Both are worth reporting upstream at https://github.com/cjlawson02/ntcore-ts.

## Tests

```sh
bun test          # no robot needed
bun run bench     # cost of one drag frame
bun run smoke     # end-to-end against a robot at localhost:5810
```

`bun test` covers the table model, the values table, and the chart's pointer and keyboard handling —
where the awkward bugs live. Among them: that hovering never edits, that a drag whose release went
missing cannot turn later mouse movement into edits, that the selection survives the rebuild an edit
causes, and that an emptied cell is not sent as zero.

`bun run bench` counts the DOM built for one frame of a drag, which is the frame budget that matters.
Dragging re-renders the app, so two things are deliberately cheap: each chart skips the redraw when
nothing it draws has changed (so dragging one curve does not redraw the other two), and the values table
updates its cells in place rather than rebuilding fifty inputs. Together those took a drag frame from
around 515 elements to 54. If a drag ever feels heavy again, run the bench first — the NetworkTables
write is already coalesced to one per frame and carries about 600 bytes, so it is almost never the cause.

Drives a real robot and checks what it actually did — every assertion reads back what the *robot*
published about the table it accepted, never the value the test just wrote. That distinction is the
whole point: it is what caught the timestamp bug above, which every dashboard-side check passed.
