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
| `MoveL` | `x,y,z,rx,ry,rz` or `name`; optional `speed,accel,decel`, tool offset `tx,ty,tz,trx,try,trz`, `blend`, `blendRadius` | Linear (straight-line) move to a Cartesian target. Consecutive blend-enabled `MoveL`s round their corners into one continuous path. |
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
| `GetRobotConfig` | — | Returns the full config (homing speeds/directions, motor directions, enabled peripherals, jog speeds, CNC motor setup, joint limits). |
| `SetRobotConfig` | any subset of the config fields | Patch config (only present fields change). An explicit `null` on a `jointNMin/Max` clears that bound. Persisted. |

**`GetStatus` returns** (selected fields): `moving`, `wasHomed`, `homingState`,
`isHoming`, `driverConnected`, `driverOk`; current pose `x y z rx ry rz` and
`localX/Y/Z/RZ` (in the active local frame); `targetX…targetRZ`; joint readouts
`joint1Angle, joint2X, joint2Z, joint4Angle`; visual `poseX…poseRZ`; motion
defaults `speedS/accelS/decelS`, `speedJ/accelJ/decelJ`; STB `input1–4` /
`output1–4`; `programs` summary; `activeTool`, `activeLocal`;
`backgroundPrograms`; repository update timestamps; `speedOverridePercent`; fault
state `faulted, faultJoint, faultDirection, faultMessage, limitBypass`;
`jointLimitsEnabled`, `robotType`, `version`, `isLinux`.

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

Grids are 2-D position arrays; stacks are 1-D. Both are referenced by index as
move targets (a `MoveL`/`MoveJ` `name` can be a grid/stack cell reference).

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
| `GetProgramVariables` | program name | Returns the program's current variable values. |

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
| `SetAuxAxisConfig` | `deviceId?, axisIndex, name, stepsPerRev=1600, invertDirection, axisType, gearRatio=1, mmPerRev` | Configure an aux channel's physical units. |
| `GetAuxState` | — | Returns per-axis live state (`state` JSON). |
| `GetAuxConfig` | — | Returns aux configuration (`config` JSON). |

---

## IO — STB driver, Nano cards, USB relay

| Command | Params | Description |
|---|---|---|
| `SetSTBOutput` | `pin, value` | Set an STB digital output (pins 1–4). |
| `GetIO` | — | Returns Nano card states (`nanos`) and relay state (`relay`). |
| `SetNanoOutput` | `nanoId, pin, value` | Set a Nano output pin. |
| `SetNeoPixel` | `nanoId, pin, colors: [{r,g,b}, …]` | Set a NeoPixel strip's colors (one entry per pixel). |
| `RenameNanoPin` | `nanoId, pin, name` | Give a Nano pin a friendly name. |
| `ConfigureNanoPin` | `nanoId, pin, type` (`Input`/`Output`/`Neopixel`/`Unconfigured`), `pixelCount=8` | Set a Nano pin's mode. |
| `SetRelay` | `relay` (1–4), `value` | Set a USB relay channel. |
| `RenameRelay` | `relay` (1–4), `name` | Name a relay channel. |
| `GetRelayState` | — | Returns relay board state (`connected, serial, relays[], names[]`). |

---

## Cameras

| Command | Params | Description |
|---|---|---|
| `GetCameras` | — | Returns camera states. |
| `AddCamera` | `name, deviceIndex, enabled, width=640, height=480, targetFps=15` | Add a USB camera. |
| `RemoveCamera` | `id` | Remove a camera. |
| `SetCameraConfig` | `id, name, deviceIndex, enabled, width, height, targetFps` | Update a camera's config. |
| `GetCameraResolutions` | `deviceIndex` | Probe supported resolutions for a device index. |

Camera frames are streamed separately over `GET /camera/{id}/ws` (base64 MJPEG
text frames) with a `GET /camera/{id}/snapshot` still image.

---

## Vision

| Command | Params | Description |
|---|---|---|
| `GetVisionPrograms` | — | Returns vision programs and `runningIds`. |
| `SaveVisionProgram` | a vision-program object | Create/update a vision program (auto-assigns `id` if empty). |
| `DeleteVisionProgram` | `id` | Stop and delete a vision program. |
| `StartVision` | `id` | Start a vision program. |
| `StopVision` | `id` | Stop a vision program. |
| `GetVisionResult` | `id` | Latest inspection result (live if running, else the last captured). |

Annotated vision frames stream over `GET /vision/{id}/ws`.

---

## CNC

| Command | Params | Description |
|---|---|---|
| `GetCncToolpath` | — | The resolved toolpath (anchor + variables applied) of the CNC block currently executing, or `null`. Returns `programName, paths, holes`. |

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
