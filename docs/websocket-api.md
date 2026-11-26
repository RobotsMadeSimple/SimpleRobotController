# SimpleRobotController — WebSocket Command Reference

The controller exposes a single WebSocket endpoint that accepts JSON command
messages. This document lists every command, its parameters, and what it does.

- **Endpoint:** `ws://<robot-ip>:9000/control`
- **Discovery:** the controller advertises itself over mDNS as `_robot._tcp`.

---

## Message envelope

Every command is a JSON text frame of this shape:

```json
{ "type": "Command", "id": "any-correlation-id", "command": "<CommandName>", "params": { ... } }
```

- `type` — always `"Command"` for commands.
- `id` — an arbitrary string echoed back in the ACK so you can match replies.
- `command` — one of the names below.
- `params` — command-specific object (omit or `{}` when a command takes none).

Parameter names are **case-insensitive** (`speed` and `Speed` both work); this
doc uses the camelCase names the app sends.

### Response

Every command receives an ACK. Commands that return data merge it into the ACK:

```json
{ "type": "ack", "command": "<CommandName>", "id": "any-correlation-id", "ok": true, "<data>": ... }
```

A command that fails (unknown name, missing or malformed `params`, or an
exception while handling it) still gets an ACK, with `ok: false` and a message:

```json
{ "type": "ack", "command": "<CommandName>", "id": "any-correlation-id", "ok": false, "error": "unknownCommand" }
```

The socket stays open; only that one command is rejected.

The controller also **broadcasts** `GetStatus`-style state to all clients on a
timer, so most UIs poll `GetStatus` rather than reacting to individual ACKs.

### Units & frames

- Cartesian position: `x y z` in **mm**, `rx ry rz` in **degrees**.
- Linear speed/accel/decel: **mm/s**, **mm/s²**.
- Joint speed/accel/decel: **deg/s**, **deg/s²**.
- Aux velocity: **steps/second**; aux accel/decel: **steps/s²**.

---

## Motion

These are enqueued and executed in order on the motion thread. A move runs only
when the robot is idle (queued moves wait for the current one to finish). Targets
may be given as explicit coordinates **or** by `name` (a saved point, or a grid /
stack cell reference).

| Command | Params | Description |
|---|---|---|
| `MoveL` | `x,y,z,rx,ry,rz` or `name` or a grid cell `grid,row,col` / stack slot `stack,index` ([Grids & Stacks](#grids--stacks)); optional `speed,accel,decel`, tool offset `tx,ty,tz,trx,try,trz`, `blend`, `blendRadius` | Linear (straight-line) move to a Cartesian target. Consecutive blend-enabled `MoveL`s round their corners into one continuous path. |
| `MoveJ` | same as `MoveL` | Joint-interpolated move to a target (joints move proportionally; path is not a straight line). |
| `OffsetL` | `x,y,z,rx,ry,rz` (relative), optional `speed,accel,decel` | Linear move **relative** to the current position. |
| `SetTool` | `x,y,z,rx,ry,rz` | Set the active TCP tool offset used for subsequent moves. |
| `SpeedS` | `speed` (mm/s) | Set default linear speed. |
| `AccelS` | `accel,decel` (mm/s²) | Set default linear accel/decel. |
| `SpeedJ` | `speed` (deg/s) | Set default joint speed. |
| `AccelJ` | `accel,decel` (deg/s²) | Set default joint accel/decel. |
| `JogL` | direction `x,y,z,rx,ry,rz`; optional `speed` | Continuously jog the TCP in a Cartesian direction until `StopJog`. |
| `JogJ` | direction `x,y,z,rx,ry,rz`; optional `speed` | Continuously jog joints until `StopJog`. |
| `JogTool` | direction `x,y,z,rx,ry,rz`; optional `speed` | Jog along the tool frame until `StopJog`. |
| `StopJog` | — | Stop any active jog. Also invalidates any jog command still queued (so a trailing jog can't re-start motion after release). |

> `StartContinuous` also exists but is **internal** — the program executor uses
> it to run blended paths on the motion thread. It is not meant to be sent by
> clients directly.

---

## Robot control

| Command | Params | Description |
|---|---|---|
| `Home` | — | Clears any latched fault and starts the homing sequence. |
| `SetHomed` | — | Marks the robot as homed at the current position **without** running homing (use with care). |
| `Reset` | — | Resets the STB motor driver. |
| `HardStop` | — | Immediately stops all motion (deferred to the motion thread; safe from any client). |
| `ClearFault` | — | Clears a latched joint soft-limit fault. |
| `SetLimitBypass` | `enable` (bool) | Temporarily bypass joint soft-limits (e.g. to jog out of a fault). |
| `SetSpeedOverride` | `percent` (double, clamped 5–200%) | Global speed scale applied to **program** moves only (manual/jog moves are unaffected). |

---

## Status & configuration

| Command | Params | Description |
|---|---|---|
| `GetStatus` | — | The main state snapshot (see fields below). |
| `GetRobotInfo` | — | Returns `robotName, robotType, serialNumber`. |
| `SetRobotIdentity` | `robotName?, robotType?` | Rename the robot / change its type, persisted. |
| `GetRobotConfig` | — | Returns the full config (homing speeds/directions, motor directions, enabled peripherals, jog speeds, CNC motor setup, joint limits, motion profile). |
| `SetRobotConfig` | any subset of the config fields | Patch config (only present fields change). An explicit `null` on a `jointNMin/Max` clears that bound. Persisted. |

**`GetStatus` returns** (selected fields): `moving`, `wasHomed`, `homingState`,
`isHoming`, `driverConnected`, `driverOk`; current pose `x y z rx ry rz` and
`localX/Y/Z/RZ` (in the active local frame); `targetX…targetRZ`; joint readouts
`joint1Angle, joint2X, joint2Z, joint4Angle`; visual `poseX…poseRZ`; motion
defaults `speedS/accelS/decelS`, `speedJ/accelJ/decelJ`; STB `input1–4` /
`output1–4`; `programs` summary (per program: `name, status, currentStepNumber,
maxStepCount, currentStepDescription, errorDescription, warningDescription`,
the current point/offsets, the action flags, plus `runCount` and
`lastStartedUnixMs` — these two change on every start, so a client polling
status can detect a run that started and finished between two polls even
though `status` reads `Complete` both times); `activeTool`, `activeLocal`;
`backgroundPrograms`; repository update timestamps; `speedOverridePercent`; fault
state `faulted, faultJoint, faultDirection, faultMessage, limitBypass`;
`jointLimitsEnabled`, `robotType`, `version`, `isLinux`; `plugins` — one
`{ id, name, state, statusState }` per installed plugin (see **Plugins**);
`activeToolId` — the active tool's stable `id` (null when none); `lastCellUpdate` —
bumped on any cell package upload/delete, active-cell change, or change to
`astroVerticalTravelMm` / `astroRadialTravelMm` / `robotType` (refetch `GET /cells/active`);
the e-stop and alarm fields `estopEnabled, estopActive, estopLatched, estopInput,
estopInputHealthy, estopSourceOnline, estopSource, estopReason, activeAlarmCount,
lastAlarmUpdate` (see **Safety & alarms**); `conveyors` — per conveyor travel, speed, queued
parts and the tracked part (see **Conveyors**).

Robot config also carries the 3D cell viewer settings `astroVerticalTravelMm` and
`astroRadialTravelMm` (built column / reach travel, mm, default 400 — drawing only, never
kinematics) and `activeCellId` (null = none; `SetRobotConfig` rejects an id that isn't stored,
explicit `null` clears). `ResetRobotConfig` keeps all three.

Robot config also carries the **motion profile** program and point moves run with (MoveL /
MoveJ / MoveJoints, blended paths, Jump legs, MoveC): `motionProfile` — `"trapezoid"` (default,
the classic constant-acceleration ramps) or `"scurve"` (jerk-limited; an unknown value rejects
the whole patch with `ok: false`); `scurveSmoothness` — 1–100 % (default 50, clamped), the share
of each acceleration phase spent ramping the acceleration; `scurveJerk` — an absolute jerk limit
(mm/s³ linear, joint units/s³ joint) or `null` (present-is-authoritative like the joint limits:
`null` or ≤ 0 clears it, and the jerk is then derived from the smoothness). A move step's own
`motionProfile` overrides the config per move. Homing and jogging always use the trapezoid.
`ResetRobotConfig` (full) resets these to the defaults.

---

## Points (taught positions)

| Command | Params | Description |
|---|---|---|
| `GetPoints` | — | Returns all saved points as JSON. |
| `TeachPoint` | `name` | Save the current position under `name` (stored in the base frame). |
| `DeletePoint` | `name` | Delete a point. |
| `EditPoint` | `name`; optional `newName, x, y, z, rx, ry, rz` | Rename and/or edit a point's coordinates (only present fields change). |

---

## Tools

| Command | Params | Description |
|---|---|---|
| `GetTools` | — | Returns all tools. |
| `CreateTool` | `name`, `x,y,z,rx,ry,rz`, optional `description` | Create a TCP tool frame. |
| `EditTool` | `name`; optional `newName, description, x…rz` | Edit a tool. |
| `DeleteTool` | `name` | Delete a tool. |
| `SetActiveTool` | `name` (`""`/`"None"` clears) | Set the active tool used for moves and status. |

---

## Cells (3D cell viewer packages)

Packages are uploaded and read over HTTP (`POST /cells`, `GET /cells/active`, meshes under
`GET /cells/{id}/…`) — see [cell-package.md](cell-package.md) §4. Over the socket:

| Command | Params | Description |
|---|---|---|
| `GetCells` | — | `{ cells: [{ id, name, updatedUnixMs, active, tools, fixtures }], activeCellId }` (`tools`/`fixtures` are counts; `activeCellId` null when none). |
| `SetActiveCell` | `id` (string, or `null` to clear) | Make a stored package the active one; returns `{ activeCellId }`. Unknown id → `ok: false, error: "unknownCell"`. Persisted as the config's `activeCellId`. |
| `DeleteCell` | `id` | Remove a stored package (clears the active id if it was active). Unknown id → `error: "unknownCell"`. |

---

## Locals (coordinate frames)

| Command | Params | Description |
|---|---|---|
| `GetLocals` | — | Returns all local frames. |
| `CreateLocal` | `name`, `x,y,z,rx,ry,rz`, optional `description` | Create a local coordinate frame. |
| `EditLocal` | `name`; optional `newName, description, x…rz` | Edit a local frame (only present fields change). |
| `DeleteLocal` | `name` | Delete a local frame (clears it if it was active). |
| `SetActiveLocal` | `name` (`""` clears) | Apply a local frame; positions in `GetStatus` (`localX…`) are then reported in it. |

---

## Grids & Stacks

Grids are 2-D position arrays; stacks are 1-D. Program moves reference them by id + index
(`gridPoint` / `stackPoint` on a MoveL / MoveJ step).

**Manual moves to a cell (additive).** `MoveL` / `MoveJ` also take a grid cell or stack slot as
the target, by the array's stable **id** (never its name), zero-based:

| Params | Target |
|---|---|
| `grid, row, col` | grid cell `row`, `col` |
| `stack, index` | stack slot `index` |

plus the usual optional `speed, accel, decel` and tool offset `tx…trz`. The cell resolves exactly
as a program's grid / stack move with no per-step local: `MoveTargetResolver.GridCell` /
`StackSlot` (base point + offsets, grid rotation about the base Z, the base point's rx/ry/rz),
then the **active local** frame, with the **active tool**. It is checked before anything is
queued and refused with `{ ok:false, error, message }`:

| `error` | When |
|---|---|
| `estopActive` / `estopLatched` | the e-stop gate (as every `Move…`) |
| `pointNotFound` | no grid / stack with that id, or its base point is missing |
| `indexOutOfRange` | a negative index, or one past `rowCount` / `colCount` / `maxCount` (unlimited directions take any index ≥ 0) |
| `badParams` | `grid` without both `row` and `col`, or `stack` without `index` |
| `outOfReach` | the reach pre-check (as program moves): no IK solution or a joint outside its soft limits at the cell — for `MoveL` also along the line from the live position |

On success the ACK adds `resolvedBy: "controller"`, `cell` (`{ grid, row, col }` /
`{ stack, index }`) and `target: { x, y, z, rx, ry, rz }` (world frame), and the move is queued
as that raw vector. The grid / stack target takes precedence over `name` and `x…rz`, so a
client can also send its own resolved `x…rz`: an older controller ignores the unknown fields and
moves there, and the missing `resolvedBy` in its ACK tells the client which path ran.

| Command | Params | Description |
|---|---|---|
| `GetGrids` | — | Returns all grids. |
| `SaveGrid` | `id, name, basePointName, rowOffsetX/Y/Z, colOffsetX/Y/Z, rowCount?, colCount?, rotation` | Create/update a grid (position = base + row×rowOffset + col×colOffset, rotated about base Z). |
| `DeleteGrid` | `id` | Delete a grid. |
| `GetStacks` | — | Returns all stacks. |
| `SaveStack` | `id, name, basePointName, offsetX/Y/Z, maxCount?` | Create/update a stack (position = base + index×offset; `maxCount` wraps round-robin). |
| `DeleteStack` | `id` | Delete a stack. |

---

## Built programs (the visual program builder)

| Command | Params | Description |
|---|---|---|
| `GetBuiltPrograms` | — | Returns all built programs (id, name, steps) as JSON in `programs`. |
| `SaveBuiltProgram` | a built-program object | Create/update a program. |
| `SaveBuiltProgramImage` | program name + image | Attach a thumbnail image to a program. |
| `DeleteBuiltProgram` | name | Delete a program. |
| `ExecuteBuiltProgram` | `name` | Start running a built program from the beginning. |
| `StopBuiltProgram` | — | Stop/pause the running built program (frame stack kept for resume). |
| `GetProgramVariables` | program name | Returns the program's current variable values, plus its display images as name + revision. |
| `GetProgramVariableImage` | `name` (program) + `variable` | Returns the base64 bytes of one display image variable in `image`. |
| `GetBuiltProgramRevisions` | `name` | Returns saved snapshots for a program, newest first, in `revisions`. |
| `GetBuiltProgramRevision` | `name, id` | Returns one saved snapshot as JSON in `program`. |
| `RestoreBuiltProgramRevision` | `name, id` | Makes that snapshot the current content and returns it as JSON in `program`. |
| `ValidateBuiltProgram` | `program` | `{ problems: [{ stepId, stepPath, field?, severity, code, message }] }` — codes in [`expressions-and-variables.md`](expressions-and-variables.md) §4, including the plugin codes (`unknownPlugin`, `unknownPluginStep`, `pluginNotRunning`, `pluginParamMissing`, `pluginParamEnum`, `pluginOutputType`, `unknownPluginFunction`). |
| `FormatProgramText` | `program, includeIds?, includeHeader?` | The program as Simple Robot Script: `{ text, languageVersion, map: [{ stepId, stepPath, line, endLine }] }`. See [`program-text.md`](program-text.md). |
| `ParseProgramText` | `text, base?, validate?` (default true) | `{ ok, program?, problems: [{ line, col, endLine, endCol, severity, code, message, stepId?, field?, source }], map }`. `ok` (the text parsed) replaces the ack's own `ok`: read problems even when it is false. `base` keeps its metadata and step ids. |
| `EvaluateExpression` | `expression, programName?` | `{ ok, value, error?, isBoolean, code?, position? }`. Plugin functions (`scale.tare()`) and properties (`$scale.weight`) evaluate live. |
| `GetExpressionSymbols` | `programName?` | `{ variables, properties, functions, io, plugins }`. Plugin functions are listed in `functions` as `{ name: "scale.tare", signature, description, pluginId }` and plugin properties in `properties` as `{ name: "scale.weight", description, type, pluginId, value }` (`value` null until known). `plugins: [{ id, name, running, functions: [{ name, fullName, signature, description, minArgs, maxArgs, timeoutMs }], properties: [{ name, fullName, description, type, value }] }]` — bare manifest names plus `fullName`. Additive: older clients ignore it. |

**Motion additions (additive).** `GetStepCatalog` lists the step type `MoveJoints` (alias
`MoveAbsJ`; fields `jointsExpr`, `j1`…`j4`, `speed/accel/decel`, `motionProfile`, stop
condition), the field `motionProfile` on MoveL / MoveJ / JumpL / JumpJ, the SavePosition source
`joints`, and the catchable error code `outOfReach` in `errorCodes` (a move target, a MoveJoints
target or a `jointsOf()` point the robot cannot reach — checked before the move starts).
`GetExpressionSymbols` / `EvaluateExpression` gain the functions `reachable(point[, tool][, local])`,
`jointsOf(point[, tool][, local])` and `poseOf(joints[, tool][, local])`
([`expressions-and-variables.md`](expressions-and-variables.md) §2). No new commands: MoveJoints
is a program step only.

### Revision history

`SaveBuiltProgram` archives a revision of the program's **previous** on-disk
content whenever the new save's serialized JSON differs from what is already
there — an unchanged save creates nothing. Up to 30 revisions are kept per
program, oldest dropped first; deleting a program deletes its history, and
renaming a program (same id, new name) carries its history over to the new
name. `RestoreBuiltProgramRevision` restores by saving the chosen snapshot as
the current content — which itself archives the pre-restore state as a new
revision — and keeps the program's current id/name even if the snapshot
predates a rename.

```json
// GetBuiltProgramRevisions { "name": "PickAndPlace" }
{ "revisions": [
  { "id": "1758556800000", "savedUnixMs": 1758556800000, "stepCount": 42, "variableCount": 6, "note": null },
  { "id": "1758553200000", "savedUnixMs": 1758553200000, "stepCount": 40, "variableCount": 6, "note": null }
] }

// GetBuiltProgramRevision { "name": "PickAndPlace", "id": "1758556800000" }
{ "program": "{...}" }   // same JSON-string shape as GetBuiltPrograms' programs entries

// RestoreBuiltProgramRevision { "name": "PickAndPlace", "id": "1758556800000" }
{ "program": "{...}" }
```

An unknown `name` or `id` answers `{ "ok": false, "error": "unknownProgram" }`
or `{ "ok": false, "error": "unknownRevision" }` instead of the normal payload.

### Display images

Variables flagged both `isImage` and `displayOnMonitor` are reported by
`GetProgramVariables` under `images`, as name and revision only:

```json
{ "variables": [ … ], "images": [ { "name": "boardImage", "revision": 7 } ] }
```

The bytes are fetched separately with `GetProgramVariableImage` — note that
`GetProgramImages` is the unrelated program-thumbnail list. That split is deliberate:
the monitor polls variables several times a second, and a base64 camera frame runs to
a few hundred kilobytes — inlining one would make every poll carry a picture that has
almost always not changed. The revision counts writes to the variable, so a client
re-fetches only when it moves.

- **Revision `0`** means declared but never written. There is nothing to fetch yet.
- Compare revisions for **inequality**, not for increase. A program restarted in a
  fresh executor starts counting again, so a revision can legitimately go down.
- `GetProgramVariableImage` answers `""` for an unknown program, an unknown variable, or one
  that is not flagged for display — a monitor asking about a program that has just
  stopped is ordinary, not an error.
- The format is whatever was written. `CaptureImage` writes JPEG; an `HttpRequest`
  mapped onto an image variable writes whatever the server sent. Detect it from the
  data rather than assuming.

---

## Program lifecycle & status (built + external/Python programs)

For a **built** program these route to the executor; otherwise they set a flag
that an external program runner can poll.

| Command | Params | Description |
|---|---|---|
| `StartProgram` | `programName` | Start (or resume if paused) a program by name. |
| `StopProgram` | `programName` | Stop/pause a program. |
| `ResetProgram` | `programName` | Reset a program to its ready state. |
| `AbortProgram` | `programName` | Abort and reset a program. |
| `ClearProgramActions` | `programName` | Clear queued actions for a program. |
| `SetAvailablePrograms` | program list | Register the set of programs the UI should show. |
| `SetProgramStatus` | a status update | Push a program cycle/status update. |
| `GetProgramImages` | — | Returns program thumbnail images (live + persisted). |
| `GetProgramLogs` | `programName, start?, end?` | Returns a page of a program's run logs (`total, start, logs`). |
| `StartBackgroundProgram` | program ref | Start a program as a background thread (motion steps are skipped in background). |
| `StopBackgroundProgram` | program ref | Stop a background program. |

---

## Aux axis (Arduino stepper board)

Default `deviceId` is `AUX_STEPPER_001`; `axis` is the channel index (0–3).

| Command | Params | Description |
|---|---|---|
| `MoveAux` | `deviceId?, axis, steps, velocity=1000, accel=10000, decel=10000` | Indexed move by `steps` (signed). Runs a trapezoidal profile on the board. |
| `JogAux` | `deviceId?, axis, velocity, accel=10000, decel=10000` | Continuous jog at `velocity` steps/s (sign = direction); `velocity: 0` stops. |
| `StopAux` | `deviceId?, axis?, decel=10000, immediate=false` | Stop an axis (or all); `immediate` halts without a decel ramp. |
| `EnableAux` | `deviceId?, enable=true` | Enable/disable the stepper drivers. |
| `SetAuxAxisConfig` | `deviceId?, axisIndex, name, stepsPerRev=1600, invertDirection, axisType, gearRatio=1, mmPerRev, role, …` | Configure an aux channel's physical units and role. Silo fields `siloPitchMm, siloCapacity, siloClearanceMm, siloHomeOverTravelMm, siloHomeSpeed` keep their stored value when left out; `siloCount` is never set here (`SiloAction SetCount`). |
| `GetAuxState` | — | Returns per-device live state (`state` JSON): `connected, motorEnabled, deviceId, deviceName, portName, firmwareVersion, capabilities, axes[]` (each axis also carries the silo fields, `siloCount` and the runtime `siloHomed`). See [Arduino firmware handshake](#arduino-firmware-handshake--capabilities). |
| `GetAuxConfig` | — | Returns aux configuration (`config` JSON). |
| `SiloAction` | `deviceId, axis, action, count?` | Silo-role axis ([aux-silo.md](aux-silo.md)). `action`: `Home` \| `Present` \| `Refill` \| `SetCount` (`count`, clamped to 0…capacity). **Acks at once** (like `MoveAux` / `Home` — a socket runs its commands one at a time, so a waiting ack would hold back a `StopAux`): `{ siloCount, siloHomed, moving }`. When `moving`, the finish (Home/Refill: zero + homed; Present: `siloCount − 1`) runs when the lift stops — watch `GetAuxState`. A move stopped part-way is not finished. Refused with `error`: `siloNotHomed`, `siloEmpty` (Present) or `siloConfig` (not a usable silo axis / unknown action). |

`MoveAux`/`JogAux` are refused (ack error) on a board whose firmware reports no `stepper`
capability — see the handshake section below.

---

## Conveyors (conveyor tracking)

Configured belts the robot can track parts on — the full contract (model, travel sources,
motion integration, calibration) is [conveyor-tracking.md](conveyor-tracking.md). Conveyors
are stored in `conveyors.json` and linked by their stable `id` (letters, digits and `_` only,
generated as `c` + 12 hex digits, so it can be written in `$conveyor.<id>.…`). A command naming
an unknown `id` acks `ok: false` with `error: "Conveyor not found: …"`.

| Command | Params | Result |
|---|---|---|
| `GetConveyors` | — | `{ conveyors: Conveyor[] }` (sorted by name) |
| `SaveConveyor` | `conveyor` — the whole object; `id` empty → new | `{ conveyor }` as stored (with its id). A bad config acks `ok: false` with the reason. Changing `source` or `scale` clears that conveyor's queue. |
| `DeleteConveyor` | `id` | — |
| `ConveyorAddPart` | `id, offsetMm` | `{ partId }` — a part at conveyor X = `offsetMm` now |
| `ConveyorClearQueue` | `id` | — (a tracked part of it stops being tracked) |
| `ConveyorSetTravel` | `id, travelMm` | — (`manual` source only; else `ok: false`) |
| `ConveyorSample` | `id` | `{ x, y, z, rz, travelMm }` — the live TCP and the belt travel now (a calibration sample) |
| `ConveyorCalibrate` | `id, samples: [{x, y, z, travelMm}, …]` (≥ 2, the same belt mark, in the order taken), optional `sensorSample: {x, y, z, travelMm, latchTravelMm}` | `{ frame: {x, y, z, rz}, scale, sensorOffsetMm, residualMm }` — a proposal, **not saved** (`sensorOffsetMm` null without a sensor sample; `residualMm` = the largest distance of a sample from the fitted belt line) |

`Conveyor`:

```jsonc
{
  "id": "c3fa9b1e2c4d", "name": "Conveyor 1",
  "source": { "kind": "aux|timed|manual", "deviceId": "uno-1", "axis": 0, "mmPerStep": null,
              "runOutput": { "outputCard": "stb|relay|io", "outputDeviceId": null, "outputNumber": 1 },
              "speedMmS": 0 },
  "scale": 1.0,
  "frame": { "x": 0, "y": 0, "z": 0, "rz": 0 },
  "sensor": { "kind": "none|auxPin|io|stb", "deviceId": null, "pin": null, "input": null,
              "activeHigh": false, "offsetMm": 0, "minSpacingMm": 10 },
  "window": { "startMm": 0, "endMm": 300 },
  "maxQueue": 32,
  "lastUpdatedUnixMs": 0
}
```

**`GetStatus` → `conveyors`**: one entry per conveyor, rebuilt at most every 100 ms:
`{ id, travelMm, speedMmS, running, tracking, trackedPartId, parts: [{ id, x, travelAtLatch }],
latched, picked, missed, lastLatchTravelMm, sourceOk }` — `parts` are the queued parts oldest
first (released parts left out), capped at 16; `x` is the part's X in the conveyor frame now;
`lastLatchTravelMm` is the travel at the last sensor latch (what the calibration wizard's sensor
step sends as `latchTravelMm`); `sourceOk` is false while an aux source is not reporting.

---

## IO — STB driver, Nano cards, USB relay

| Command | Params | Description |
|---|---|---|
| `SetSTBOutput` | `pin, value` | Set an STB digital output (pins 1–4). |
| `GetIO` | — | Returns IO device states (`ioDevices` JSON: one row per Nano **and** per aux-stepper board, each `id, name, connected, firmwareVersion, capabilities, pins[]` — an aux board is listed even with no pins yet) and relay state (`relay`). The same `ioDevices` string rides in every `GetStatus` broadcast. |
| `SetIoOutput` | `deviceId, pin, value` | Set an IO device output pin (Nano or Uno). |
| `SetNeoPixel` | `nanoId, pin, colors: [{r,g,b}, …]` | Set a NeoPixel strip's colors (one entry per pixel). |
| `RenameIoPin` | `deviceId, pin, name` | Give an IO device pin a friendly name. |
| `ConfigureIoPin` | `deviceId, pin, type` (`Input`/`Output`/`Latch`/`Neopixel`/`Unconfigured`), `pixelCount=8` | Set an IO device pin's mode. `Latch` (Uno firmware with the `latch` cap) is an input that reports edges with the motor positions at the edge; `Neopixel` is Nano only. |
| `SetRelay` | `relay` (1–4), `value` | Set a USB relay channel. |
| `RenameRelay` | `relay` (1–4), `name` | Name a relay channel. |
| `GetRelayState` | — | Returns relay board state (`connected, serial, relays[], names[]`). |

### Arduino firmware handshake + capabilities

Every Arduino serial device (Nano edge device, aux stepper board) is discovered by
probing COM ports with `ID?`. The reply identifies the board and, from firmware 1.1.0,
what it can do:

```
ID:<DEVICE_ID>                                  legacy (pre-1.1.0)
ID:<DEVICE_ID>;fw=<major.minor.patch>;caps=<cap1,cap2,...>
```

e.g. `ID:ROBOT_NANO_001;fw=1.1.0;caps=io,neopixel,loadcell` or
`ID:AUX_STEPPER_001;fw=1.1.0;caps=stepper,io`. The id must match exactly; unknown
`key=value` fields and surrounding whitespace are ignored. Capability names:

| Capability | Meaning | Sketch |
|---|---|---|
| `io` | digital input/output pins (`CFG`/`SET`/`GET`) | NanoEdgeDevice, AuxStepperDriver |
| `neopixel` | NeoPixel strips (`NEO`) | NanoEdgeDevice |
| `loadcell` | HX711 load cell | NanoEdgeDevice |
| `stepper` | stepper motion (`M`/`C`/`S`) | AuxStepperDriver |

The controller exposes the result on every device row — `firmwareVersion` (`""` when
unknown) and `capabilities` (string array, or `null` when unknown) — in `GetIO`
`ioDevices`, `GetStatus` `ioDevices` and `GetAuxState`. Both are reset when the board
disconnects, and the connect log line reads
`[Nano:ROBOT_NANO_001] Connected on COM4 — firmware 1.1.0 (io, loadcell, neopixel)`
or `… — firmware unknown (legacy, pre-1.1.0)`.

**Gating rule.** When capabilities are known, a command that needs one the board lacks
is refused with an ack error naming the device, its version and what to flash, e.g.
`Load cell not supported by ROBOT_NANO_001 firmware 1.0.0 — flash NanoEdgeDevice 1.1.0 or newer`.
When capabilities are unknown (legacy firmware) nothing is blocked — the boards in the
field today are legacy — and the app should use the empty `firmwareVersion` to hint
"update firmware". Gated commands: `ConfigureLoadCell` (`loadcell`), `ConfigureIoPin`
with `type: "Neopixel"` and `SetNeoPixel` (`neopixel`), `ConfigureIoPin` with
`Input`/`Output` and `SetIoOutput` (`io`, Nano or aux board), `MoveAux`/`JogAux`
(`stepper`). Removing a pin (`type: "Unconfigured"`) is never gated.

---

## Cameras

| Command | Params | Description |
|---|---|---|
| `GetCameras` | — | Returns camera states (each with `calibrated` and `calibratedUnixMs`, see below). |
| `AddCamera` | `name, deviceIndex, enabled, width=640, height=480, targetFps=15`, optional `sourceType="usb"`, `url`, `username`, `password`, `transport="tcp"`, and for Sofia `host`, `port=34567`, `stream="Main"`, `codec="h264"`, `decoder="opencv"`, `ffmpegPath="ffmpeg"`, `hwaccel=""` | Add a USB, network or Sofia camera (a Sofia camera without `username` logs in as `admin`). |
| `RemoveCamera` | `id` | Remove a camera. |
| `SetCameraConfig` | `id, name, deviceIndex, enabled, width, height, targetFps`, optional `sourceType, url, username, password, transport, host, port, stream, codec, decoder, ffmpegPath, hwaccel` | Update a camera's config. An absent network or Sofia field keeps the camera's current value. The capture restarts only when a capture-affecting field changed (device index, size, fps, enabled, or any source field). |
| `GetCameraResolutions` | `deviceIndex`, optional `id` | Probe supported resolutions for a device index. Returns `[]` for a network or Sofia camera (`id` of one); those never match a `deviceIndex`. |
| `TestCameraSource` | `url`, optional `username`, `password`, `transport` (`tcp`/`udp`), `timeoutMs` (8000); or `sourceType: "sofia"` with `host`, optional `port`, `username`, `password`, `stream`, `codec`, `decoder`, `ffmpegPath`, `hwaccel`, `timeoutMs` | Opens a network stream once on a worker thread, reads one frame and closes it. Returns `ok, width, height, openMs, firstFrameMs, error` (`error` null on success). For Sofia see [Sofia cameras](#sofia--dvrip-xmeye-cameras). |

Camera frames are streamed separately over `GET /camera/{id}/ws` (base64 MJPEG
text frames) with a `GET /camera/{id}/snapshot` still image.

### Network cameras

A camera with `sourceType: "network"` is an RTSP (`rtsp://`, `rtsps://`) or HTTP
MJPEG/snapshot (`http://`, `https://`) stream opened through OpenCV's FFmpeg
backend — see [network-cameras.md](network-cameras.md). `url` is stored without
credentials; `username`/`password` are injected (URL-encoded) at open time and are
returned by `GetCameras`, but logs only ever show the password as `***`.
`deviceIndex` is ignored, and `width`/`height`/`supportedResolutions` are
informational: the stream's own size is used.

Extra `GetCameras` state fields (all cameras):

| Field | Meaning |
|---|---|
| `sourceType` | `"usb"` (also when absent in an old `camera_config.json`) or `"network"`. |
| `url`, `username`, `password`, `transport` | Network source settings (`""` / `"tcp"` for USB cameras). |
| `streamWidth`, `streamHeight` | Size of the frames actually delivered; 0 until the first frame. |
| `latencyMs` | Best-effort network decode latency estimate (wall clock vs the stream's presentation clock); 0 when unknown. |

`TestCameraSource` errors:

| `error` | Meaning |
|---|---|
| `invalidUrl` | Not an absolute `rtsp`/`rtsps`/`http`/`https` URL with a host. |
| `openFailed` | FFmpeg could not open the stream (wrong address, port or credentials, host unreachable), or the controller's OpenCV build has no FFmpeg. |
| `noFrame` | The stream opened but no frame could be read. |
| `timeout` | The open or first read took longer than `timeoutMs` (+1 s margin); the worker closes the stream when the call returns. |

RTSP transport is passed to FFmpeg through the process-wide
`OPENCV_FFMPEG_CAPTURE_OPTIONS` environment variable, set just before each network
open (last writer wins), so two network opens racing with different transports may
pick up each other's setting.

### Sofia / DVRIP (XMeye) cameras

A camera with `sourceType: "sofia"` is pulled over the vendor Sofia protocol (DVRIP,
TCP 34567), which buffers far less than those cameras' RTSP server — see
[network-cameras.md](network-cameras.md#sofia--dvrip-xmeye-cameras--sourcetype-sofia).
`url`/`transport`/`deviceIndex` are unused; `username`/`password` are the Sofia login
(may differ from RTSP; never logged). The first video frame's codec is detected and
wins over `codec` when they differ.

Extra `GetCameras` state fields (all cameras; defaults shown for non-Sofia cameras):

| Field | Meaning |
|---|---|
| `host` | Camera IP / hostname (`""`). |
| `port` | DVRIP port (`34567`). |
| `stream` | `"Main"` or `"Extra1"` (sub-stream). |
| `codec` | `"h264"` or `"hevc"`. |
| `decoder` | `"opencv"` (in-process: loopback socket into OpenCV's FFmpeg backend) or `"ffmpeg"` (external process). |
| `ffmpegPath` | Executable for the `ffmpeg` decoder (`"ffmpeg"`). |
| `hwaccel` | `ffmpeg` decoder only: `""`, `"auto"`, `"d3d11va"`, … |

`streamWidth`/`streamHeight` come from the first decoded picture; `latencyMs` is the
best-effort time from a frame's arrival to its decoded picture.

`TestCameraSource` with `sourceType: "sofia"` connects, logs in, claims the stream and
waits for the first video frame within `timeoutMs`; if time remains it decodes one
picture with the chosen decoder. Response:

| Field | Meaning |
|---|---|
| `ok` | Handshake and first frame succeeded (the decode is optional). |
| `loginMs` | Time to finish the handshake (connect, login, claim/start). |
| `firstFrameMs` | Time until the first video frame arrived. |
| `detectedCodec` | `"h264"`, `"hevc"` or `"unknown"` (from the first frame's NAL headers). |
| `firstFrameBytes` | Size of the first video frame. |
| `width`, `height` | Decoded size; 0 when the decoder could not be exercised within the budget. |
| `error` | null, or `connectFailed` (host/port unreachable), `loginFailed` (credentials; the camera's `Ret` code is in `message`), `claimFailed`, `noFrame` (logged in but no video — try the other stream), `timeout`, `decoderUnavailable` (`ffmpeg` not found at `ffmpegPath`, or no FFmpeg backend for the in-process decoder). |
| `message` | Error detail, e.g. `DVRIP login failed (Ret=203)`; never contains the password. |

---

## Camera calibration

Camera-to-robot calibration with a dot sheet — the method, the stored model and
the grid rules are in [camera-calibration.md](camera-calibration.md). Image
positions are normalized `u, v` (0–1). A wizard run is a **session** held in
memory for 30 minutes after its last use.

| Command | Params | Response (merged into the ack) |
|---|---|---|
| `GetCameraCalibration` | `cameraId` | `calibration` (the stored object) or `calibration: null`. |
| `DeleteCameraCalibration` | `cameraId` | — |
| `CalibrationStart` | `cameraId`, `dotPitchMm` (> 0), optional `minDotAreaPx` (30), `maxDotAreaPx` (20 000), `darkDots` (true) | `sessionId, cameraId, dotPitchMm, imageWidth, imageHeight, dots: [{ index, i, j, u, v, areaPx }], gridRows, gridCols, gridRmsPx, warnings: [string], imageUrl, taught: []` |
| `CalibrationRedetect` | `sessionId`, optional detector params as above and `dotPitchMm` | Same as `CalibrationStart`, from a new frame. Taught dots with a dot still within 0.3 spacings of where they were are kept (re-indexed); others are dropped with a warning. |
| `CalibrationTeachDot` | `sessionId`, `dotIndex` | `taught: [{ dotIndex, i, j, u, v, robot: {x,y,z}, tool }]` — records the current TCP position and active tool; replaces an earlier teach of the same dot. |
| `CalibrationUnteachDot` | `sessionId`, `dotIndex` | `taught: […]` |
| `CalibrationSolve` | `sessionId`, optional `save` (true) | `calibration, taughtRmsMm, taughtMaxMm, pitchScaleEstimate, mirrored, residuals: [{ dotIndex, i, j, errorMm }], warnings, saved` |
| `CalibrationPredict` | `u`, `v`, and `sessionId` (its solved result, saved or not) or `cameraId` (the saved calibration) | `robot: {x, y, z}`, `source: "session"\|"saved"` |
| `CalibrationDiscard` | `sessionId` | — |

A failure answers `{ "ok": false, "error": "<code>", "message": "…" }`:

| Code | When |
|---|---|
| `invalidParams` | A required parameter is missing or out of range (`dotPitchMm ≤ 0`, `minDotAreaPx ≥ maxDotAreaPx`, no `u`/`v`, …). |
| `unknownCamera` | No camera with that id. |
| `cameraNotConnected` | The camera exists but has no frame. |
| `noDotsFound` | No blob passed the area/circularity filter. |
| `gridNotFound` | Fewer than 4 dots fit a lattice, or the fit RMS exceeds 2 % of the dot spacing. |
| `unknownSession` | Unknown or expired `sessionId`. |
| `unknownDot` | `dotIndex` is not in the detected grid. |
| `notEnoughTaught` | Solve with fewer than 2 taught dots (or before any grid was detected). |
| `taughtCollinear` | Solve with 3+ taught dots that all lie on one grid line. With exactly 2 dots the solve succeeds with a warning, and the handedness of a camera looking down at the sheet is assumed. |
| `notCalibrated` | Predict against a camera with no saved calibration, or a session not yet solved. |

When `CalibrationStart` fails with `noDotsFound` or `gridNotFound` the session is
still created: the error also carries `sessionId` and `imageUrl` (showing the raw
frame) so the wizard can show what the camera saw and call `CalibrationRedetect`
with other detector parameters.

`GET /calibration/{sessionId}/image` returns the session's annotated JPEG: every
dot circled and numbered, taught dots highlighted, and `i`/`j` arrows at dot
(0, 0). 404 for an unknown or expired session, 204 when it has no image.

`GetCameras` states carry `calibrated` (true/false) and `calibratedUnixMs` (null
when not calibrated). Programs read `$camera.<id>.calibrated` (1/0).

---

## Vision

A vision program is image layers (filter stacks) plus a list of building-block tools —
the model, every tool and the result shape are in [vision-tools.md](vision-tools.md).

| Command | Params | Description |
|---|---|---|
| `GetVisionPrograms` | — | Returns vision programs (JSON string in `programs`) and `runningIds`. |
| `SaveVisionProgram` | a vision-program object | Create/update a vision program (auto-assigns `id` if empty). |
| `DeleteVisionProgram` | `id` | Stop and delete a vision program. |
| `StartVision` | `id` | Start a vision program on its camera. |
| `StopVision` | `id` | Stop a vision program. |
| `GetVisionResult` | `id` | Latest `VisionResult` (JSON string in `result`): live if running, else the last one from a RunVision step or `RunVisionOnImage`. |
| `RunVisionOnImage` | `programId` or `program`, `image` (base64 PNG/JPEG, data URL ok), optional `includeAnnotated` | Runs the program once on the image. Returns `result` (JSON string) and, when asked, `annotated` (base64 JPEG). With a saved `programId`, a fixture reference captured on this run is saved and the result becomes the program's latest. |
| `SetVisionToolReference` | `programId`, `toolId`, optional `clear` | Sets a pose tool's `reference` to the pose it found in the program's latest result (start the program or run it on an image first), or clears it with `clear: true` (it is then captured again on the next successful run). Saves the program; returns `reference` (JSON string). Errors when the tool makes no pose or found nothing. |
| `TeachVisionTool` | `programId`, `toolId`, optional `image`, `region`, `preview` | Teaches a `patternMatch` (gray patch) or `contourMatch` (outline) tool from its region — or from `region` (normalized, no fixture; a rotated rect teaches the pattern's 0° angle): on `image`, else the program's newest camera frame. Earlier tools run first, so a fixtured region is cut where its fixture puts it. Stores `params.template`, saves the program and returns a `taught` summary, the `template` (JSON) and, for patternMatch, a `preview` PNG (base64). With `preview: true` nothing is changed or saved. |

HTTP (JPEG unless noted; 404 for an unknown program/layer/tool, 204 while there is no frame):

| Endpoint | Description |
|---|---|
| `GET /vision/{id}/ws` | WebSocket stream of annotated frames (base64 data URIs) of a running program. |
| `GET /vision/{id}/snapshot` | The running program's latest raw frame. |
| `GET /vision/{id}/annotated` | The running program's latest annotated frame. |
| `GET /vision/{id}/layer/{layerId}?upTo=N` | Layer preview for tuning: the layer's filters applied up to and including index `N` (omit for all, `-1` for the layer's input), on the program's newest frame (its processor's, else its camera's). `layerId` `camera` = the frame itself. |
| `GET /vision/{id}/tool/{toolId}/debug` | One tool's result (region, cells, edges, lines, pose) drawn over the tool's own input layer, with its status. Uses the running processor's latest result, else runs the program once on the camera's frame. |
| `GET /program-vision-snapshot/{visionProgramId}` | Annotated frame captured at the end of the last RunVision step. |

---

## CNC

| Command | Params | Description |
|---|---|---|
| `GetCncToolpath` | — | The resolved toolpath (anchor + variables applied) of the CNC block currently executing, or `null`. Returns `programName, paths, holes`. |

---

## Plugins

External plugin processes the controller launches and supervises. The full
contract (manifest, lifecycle, the plugin-side protocol, SDKs) is
[`plugins.md`](plugins.md); this is the app-facing command set. Every command
taking `id` answers `ok:false, error:"unknownPlugin"` for an id that is not
installed.

| Command | Params | Description |
|---|---|---|
| `GetPlugins` | — | `{ plugins: [PluginSummary] }`. |
| `GetPlugin` | `id` | `{ plugin: PluginDetail }`. |
| `GetPluginContributions` | — | `{ steps: [{ pluginId, pluginName, running, step }], functions: [{ pluginId, pluginName, running, function }], properties: [{ pluginId, pluginName, running, property, value? }] }` — the builder's step tiles and the expression assist in one call. |
| `SetPluginEnabled` | `id, enabled` | `{ plugin }`. Disabling stops the plugin (state `disabled`); enabling an `autoStart` plugin starts it. Persisted. |
| `StartPlugin` / `StopPlugin` / `RestartPlugin` | `id` | `{ plugin }`, returned immediately — poll for the state. A manual start resets the crash budget. `pluginDisabled` / `pluginError` when it cannot start. |
| `SetPluginConfig` | `id, config` | `{ plugin }`. The whole config object, validated against the manifest's `configSchema`; on failure `ok:false, error:"badConfig", message, field` (the schema key). Persisted; a running plugin gets `config.changed`. |
| `GetPluginLogs` | `id, start?, end?` | `{ totalCount, start, logs }` — same half-open absolute-index paging as `GetProgramLogs` (500-line ring). |
| `ClearPluginLogs` | `id` | Empties the ring buffer (`plugin.log` on disk is kept). |
| `UninstallPlugin` | `id` | Stops the plugin, deletes `plugins/<id>` and `pluginConfigs/<id>.json`. |
| `ReloadPlugins` | — | `{ plugins }`. Rescans the plugins folder: new folders appear (and start), removed ones are stopped and vanish, manifests of inactive plugins reload. |
| `RotatePluginToken` | `id` | `{ plugin }`. New connect token; an `external` plugin's live connection is closed and must reconnect with the new token. |

`PluginSummary`: `id, name, version, description, author, runtime, state,
message, enabled, connected, pid, startedUnixMs, restartCount, stepCount,
functionCount, propertyCount, hasConfig, statusState, statusMessage`. `state` is
one of `disabled | stopped | installing | starting | running | degraded |
crashed | error`; `statusState` is the plugin's own `ok | degraded | error`.
`PluginDetail` adds `manifest` (in effect, including a `plugin.ready`
override), `config` (schema defaults merged in; passwords returned as-is),
`token` (`external` runtime only), `properties` (live values), `logTail` (last
50 lines) and `problems` (manifest validation problems, `{ code, message, field }`).

In programs: the `Plugin` step type (`pluginId`, `pluginStepId`, `pluginParams`,
`pluginOutputs`, `pluginTimeoutMs` — see [`program-blocks.md`](program-blocks.md#plugins)),
plugin functions `id.name(…)` and properties `$id.name` in expressions, and the
plugin parts of `ValidateBuiltProgram` / `GetExpressionSymbols` (Built programs, above).

HTTP:

- `POST /plugins/install[?replace=true]` — body is the zip. `plugin.json` must
  be at the root or inside a single top-level folder. `200 { id, plugin }`;
  `400 { error: "badZip" | "badManifest" | "idExists", message }`;
  `413 { error: "tooLarge" }` above 200 MB.
- `GET /plugins/{id}/download` — the plugin folder as a zip (without `.venv`
  and `plugin.log*`).
- `GET /plugin` — the WebSocket plugins connect to (not for apps).

---

## Safety & alarms

### Emergency stop

The hardwired e-stop sense input — wiring, states and the recovery sequence are in
[estop.md](estop.md). **Not a certified safety function.**

| Command | Params | Description |
|---|---|---|
| `GetEstopConfig` | — | `{ config, state }`. `config`: `enabled` (default false), `source` (`"stb"` \| `"io"`), `stbInput` (1–4, default 4), `deviceId` + `pin` (an IO board or aux board pin, by stable device id), `polarity` (`"normallyClosed"` default \| `"normallyOpen"`). `state`: `enabled, active, latched, sourceOnline, input` (the raw level, null when offline), `inputHealthy` (null when off/offline), `source` (label), `reason`, `trippedUnixMs`. Reads the input fresh. |
| `SetEstopConfig` | any subset of the `config` fields | Patch, validated as a whole (`badConfig` + `message` on a bad field; an enabled `io` source needs `deviceId` and `pin`). Saved in `robot-config.json` under `estop`; `ResetRobotConfig` keeps it. Returns `{ ok, config, state }`. |
| `ResetEstop` | — | Clears the latch once the input reads healthy (or monitoring is off): `{ ok, state, message }`. Otherwise `{ ok:false, error:"estopNotHealthy", message, state }`. Home the robot afterwards — a trip marks every joint unreferenced. |

While `estopActive` or `estopLatched`, these are refused with `error: "estopActive"` /
`"estopLatched"` and a `message`: every command starting with `Move` or `Jog` (MoveL, MoveJ,
JogL/J/Tool, MoveAux, JogAux, …), `OffsetL`, `StartContinuous`, `Home`, `SetHomed`,
`SetJointPosition`, `ExecuteBuiltProgram`, `StartBackgroundProgram`, `StartProgram`,
`StepBuiltProgram`, `RunPluginStep`, `SiloAction`. On a trip the controller hard-stops, ends every program with
the uncatchable `emergencyStop` error, marks the robot unhomed, publishes the `robot.estop` plugin
event `{ active, latched, reason, source }` (again on reset) and raises an `emergencyStop` alarm.

### Alarms

Each alarm: `id, code, severity` (`critical` \| `error` \| `warning` \| `info`), `title`, `message`,
`source { programId?, programName?, stepId?, deviceId?, pluginId?, background? }`,
`timestampUnixMs`, `acknowledged`, `acknowledgedUnixMs`, `cleared` (the condition has gone),
`clearedUnixMs`, `active`. An alarm is **active** until it is both cleared and acknowledged; a
program error has no lasting condition, so it is raised cleared and is active until acknowledged.

Raised for: program errors (foreground and background, with the program's stable id and the
failing step id; the code is the program's error code, or `programError`), joint-limit faults
(`jointLimit`, cleared with the fault), the e-stop (`emergencyStop`, cleared by `ResetEstop`), the
motor driver disconnecting (`driverDisconnected`), an IO or aux board disconnecting
(`deviceDisconnected`, cleared when it reconnects; only devices that had connected), and a plugin
crash (`pluginCrashed`, cleared when it runs again or is stopped).

The history holds the newest 500 alarms (an alarm whose condition is still present is never
dropped) in `alarms.json` in the data folder, written on the thread pool shortly after a change.

| Command | Params | Description |
|---|---|---|
| `GetAlarmCatalog` | — | `{ alarms: [{ code, title, severity, cause, remedy, catchable }] }` — help for every program error code plus `deviceDisconnected`, `pluginCrashed`, `programError`. The one place the help text lives. |
| `GetAlarms` | `offset?` (0), `limit?` (50), `activeOnly?` | `{ total, activeCount, offset, lastAlarmUpdate, alarms }` — active alarms first (most severe, then newest), then the rest newest first. |
| `AcknowledgeAlarm` | `id`, or none / `all: true` for every alarm | `{ ok, acknowledged, activeCount }`; `unknownAlarm` for a bad id. |
| `ClearAlarms` | — | Removes every alarm whose condition has gone (acknowledged or not); alarms still present stay. `{ ok, removed, remaining, activeCount }`. |

`GetStatus` carries `activeAlarmCount` and `lastAlarmUpdate` (refetch `GetAlarms` when it changes).

---

## Backup & restore

One zip of the data folder's **user data**: programs (with revisions and images), points, tools,
locals, grids, stacks (with their history files), robot config, IO device / aux / relay / camera /
phone-device config, vision programs, camera calibrations, cell packages, persistent variables,
plugin configs and DXF files. Never included: caches, logs, IO history samples, the alarm log,
`identity.json` (a restore never changes the robot's serial or name) and plugin binaries —
`manifest.json` lists the installed plugins instead. Both routes send
`Access-Control-Allow-Origin: *` (on errors too) and answer `OPTIONS` preflights, like `/cells`.

- `GET /backup` — the zip (`application/zip`, `Content-Disposition: attachment;
  filename="<robot>-backup-<yyyyMMdd-HHmmss>.zip"`). `manifest.json`: `format`
  (`"rms-controller-backup"`), `formatVersion` (1), `controllerVersion`, `robotType`, `robotName`,
  `serialNumber`, `createdUnixMs`, `createdUtc`, `plugins: [{ id, name, version }]`, `files`.
- `POST /backup/restore` — raw zip body (≤ 500 MB). Validates the manifest and every entry path
  (absolute paths, drive letters, `..` or empty segments refuse the whole zip:
  `400 { error: "unsafePath" }`; also `badZip`, `badManifest`, `413 tooLarge`) and returns
  `{ ok, applied: false, preview }` — `preview`: `manifest`, `categories: [{ key, label, current,
  backup }]` (item counts now and in the backup), `missingPlugins`, `warnings` (another
  controller's serial, another robot type, plugin version differences, ignored files),
  `fileCount`, `totalBytes`. Nothing changes.
- `POST /backup/restore?apply=true` — the same validation, then: refused with `409 { error:
  "busy" }` while a program (foreground or background) is running or the robot or an aux axis is
  moving or homing, and with `409 { error: "restoring" }` once a restore has been applied; writes a pre-restore backup to `backups/pre-restore-<stamp>.zip` (the newest 5
  are kept); extracts to a staging folder and swaps each allowlisted file and folder in (an
  allowlisted item the backup does not have is removed, so the data matches the backup; rolled back
  on failure). Returns `{ ok, applied: true, preview, filesWritten, preRestoreBackup, restarting:
  true }`, then **restarts the controller** (the same helper as `RestartController`; under systemd it
  exits and the service restarts it). Every repository, device manager and plugin then loads the
  restored data exactly as at boot — chosen over reloading each repository live, which would leave
  device managers, plugins and cached state half old, half new. Between the reply and the restart
  only `Get*` commands are served (`error: "restoring"`). Re-home after the restart.

---

## System

| Command | Params | Description |
|---|---|---|
| `Update` | — | **Linux only.** Downloads the latest release binary from GitHub, replaces the running binary, and exits for systemd to relaunch. Returns `{ ok }`. |
| `RestartController` | — | Restarts the controller process (on a dev machine, rebuilds the source first). |

---

## Diagnostics (operational note)

The controller has an opt-in diagnostic mode (control-loop/program timing, GC
cadence, step/aux traces) written to the log with a `[diag]` prefix. It is
**off by default**. Enable it for a debugging session by launching with the
environment variable `RMS_DIAG=1` (e.g. a systemd drop-in), then read
`journalctl -u robot-controller | grep '\[diag\]'`.
