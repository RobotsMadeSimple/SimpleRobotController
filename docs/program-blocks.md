# Program Builder — Block Reference

A program is an ordered list of **steps** ("blocks"). The controller executes
them top to bottom; flow-control blocks (Loop, IfCondition, GoToLabel,
CallRoutine) change that order. This document describes every block type, its
parameters, and its options.

Field names below are the JSON property names stored on each step (a step is a
`ProgramStep`). All are optional unless noted; only the fields relevant to a
block's `type` are used.

- **Program kinds:** a program can be a normal program, a **routine** (hidden
  from the list, only runnable via `CallRoutine`), or a **background** program
  (runs in parallel with the main program; motion / tool / homing blocks are
  skipped in background).
- **Every step has:** `id` (string), `name` (optional label), and `type`.

---

## Shared concepts

### Variables

Programs define variables (`ProgramVariable`) that blocks can read and write:

- **Scalar** (number), **boolean**, or **string** (`isString` + `stringValue`).
- **Array** (`values`) and **point-array** (`points`, a list of Vector6) — e.g.
  vision blob results.
- **`isGlobal`** — shared across all concurrently running programs.
- **`isPersistent`** — value is saved on finish and restored next run.
- **`isStopwatch`** — value holds elapsed milliseconds (driven by
  `StopwatchControl`).
- **`displayOnMonitor`** — current value is shown on the monitor page while
  running.

### Expressions

Most numeric fields can be a **math expression** instead of a literal. A step's
`expressions` map holds `{ "<fieldName>": "<expr>" }` keyed by the JSON field
name (e.g. `"speed": "baseSpeed * 2"`, `"offsetZ": "$layer * 5"`). Expressions
are evaluated at execution time and reference variables by name (or `$name`).
The built-in `$time_ms` is also available (e.g. in `SaveImage` paths).

### Conditions

`Wait` (condition mode), `IfCondition`, and `while`-loops use a **ConditionGroup**:

- `combinator`: `"ALL"` (AND) or `"ANY"` (OR).
- `items`: a list of `{ left, operator, right }` where `left`/`right` are
  expressions/variables and `operator` is one of `==`, `!=`, `<`, `<=`, `>`,
  `>=`.

### Move target & modifiers (MoveL, MoveJ, JumpL, JumpJ)

A move's destination is resolved from the first of these that is set:

| Field | Meaning |
|---|---|
| `pointName` | A saved point by name. |
| `gridPoint` `{ gridId, rowIndex, colIndex }` | A cell of a named grid. |
| `stackPoint` `{ stackId, index }` | An entry of a named stack. |
| `varPointName` (+ `varPointIndex`) | A point stored in a point-array variable. |

Then these modifiers apply (all optional):

| Field | Meaning |
|---|---|
| `offsetX…offsetRZ` | Offset added to the target (mm / deg). |
| `overrideX…overrideRZ` | Replace the computed axis value (base + offset) absolutely. |
| `toolOffsetX…toolOffsetRZ` | Per-step tool offset applied on top of the active tool. |
| `localName` | Resolve the target in a named local frame for this step. |
| `speed`, `accel`, `decel` | Motion dynamics for this move (else the program defaults). |

Program moves are scaled by the global speed override (manual moves are not).

---

## Motion blocks

### MoveL — linear move
Straight-line Cartesian move to the target.
- **Params:** move target & modifiers (above); `blend`, `blendRadius`.
- **Blend:** when `blend` is true the move rounds its corner into the *next*
  blend-enabled move instead of stopping; consecutive blended `MoveL`s become one
  continuous path. `blendRadius` overrides the current default (see
  `SetBlendRadius`).

### MoveJ — joint move
Joint-interpolated move to the target (joints move proportionally; path is not a
straight line). Same target & modifier fields as `MoveL`.

### JumpL / JumpJ — lift, traverse, lower
A three-leg pick-and-place move: lift to a Z height, traverse, then lower to the
target. `JumpL` uses linear legs, `JumpJ` joint legs.
- **Params:** move target & modifiers; plus:

| Field | Meaning |
|---|---|
| `jumpZ` | Z height for both the lift and lower legs (mm). |
| `jumpZStart` | Override the lift-leg height independently. |
| `jumpZEnd` | Override the lower-leg height independently. |

Blend fields apply to the traverse leg.

### ThreadMove — synchronized rotary/linear thread
Drives a coordinated threading move (e.g. screwing into a hole).

| Field | Meaning |
|---|---|
| `threadDistance` | Depth to thread (mm). |
| `threadPitch` | Thread pitch (mm/rev). |
| `threadPeck` | When true, peck (retract periodically to clear). |
| `threadPeckDepth` | Depth per peck. |
| `threadReverseOut` | Reverse the rotation to back out at the end. |

---

## Speed, blend, tool & frame

| Block | Params | Description |
|---|---|---|
| `SetSpeedL` | `speed`, `accel`, `decel` | Set the default **linear** dynamics for subsequent moves. |
| `SetSpeedJ` | `speed`, `accel`, `decel` | Set the default **joint** dynamics. |
| `SetBlendRadius` | `blendRadius` | Set the program's default corner blend radius used by blend-enabled moves. |
| `SetTool` | `toolName` | Set the active TCP tool by name for subsequent moves. |
| `SetLocal` | `localName` | Apply a named local coordinate frame to subsequent moves. |
| `ClearLocal` | — | Clear the active local frame. |

---

## IO

### SetOutput — set a digital output
| Field | Meaning |
|---|---|
| `outputCard` | `"stb"` (default), `"nano"`, or `"relay"`. |
| `outputNumber` | Output/pin/relay number (1-based). |
| `outputValue` | `true`/`false` to set. |
| `outputNanoId` | Required when `outputCard` is `"nano"` — which Nano card. |
| `pulseMs` | If > 0, set to `outputValue` for this many ms, then flip back. `0`/null = hold. |
| `pulseBlocking` | When true (with `pulseMs` > 0), the program waits for the pulse to finish before advancing; otherwise the flip-back happens in the background. |

---

## Flow control

### Wait
| Field | Meaning |
|---|---|
| `waitMode` | `"duration"` (default) or `"condition"`. |
| `waitMs` | Duration to wait (duration mode). |
| `waitCondition` | A ConditionGroup to wait until true (condition mode). |
| `waitTimeoutMs` | Optional max wait in condition mode. |
| `waitTimeoutVariableName` | If set, a boolean written `true` when the wait timed out (vs. the condition becoming true). |

### Loop
Repeats its child steps (`loopSteps`).
| Field | Meaning |
|---|---|
| `loopMode` | `"count"` (default), `"forEach"`, or `"while"`. |
| `loopCount` | Iterations for count mode; **`0` = infinite**. |
| `loopWhileCondition` | ConditionGroup re-checked each iteration (while mode). |
| `forEachVariableName` | Array variable to iterate (forEach mode). |
| `forEachValueVariableName` | Variable that receives the current element. |
| `forEachIndexVariableName` | Variable that receives the current index. |
| `loopSteps` | The child steps executed each iteration. |

### IfCondition
| Field | Meaning |
|---|---|
| `condition` | ConditionGroup for the `if` branch. |
| `ifSteps` | Steps run when `condition` is true. |
| `elseIfBranches` | Ordered `[{ condition, steps }]` — first matching branch runs. |
| `elseSteps` | Steps run when nothing matched. |

### Label / GoToLabel
- **Label** — a jump target. Params: `labelId`, `labelName`.
- **GoToLabel** — jump to a label. Params: `labelId` (or `labelName`).

### CallRoutine
Runs another program marked as a routine. Params: `routineId` (or
`routineName`). Execution returns to the next step when the routine finishes.

### PauseProgram
Pauses execution, keeping the frame stack so it can be resumed. No params.

---

## Variables & timing

### SetVariable
| Field | Meaning |
|---|---|
| `variableName` | The variable to assign. |
| `variableExpr` | Expression evaluated and stored (numbers, booleans, or string ops per the variable's type). |

### StopwatchControl
| Field | Meaning |
|---|---|
| `stopwatchAction` | `"Start"`, `"Stop"`, or `"Reset"`. |
| `stopwatchVariableName` | The stopwatch variable to control (its value is elapsed ms). |

---

## Aux axis (Arduino stepper board)

Aux blocks drive an external stepper channel. Distance can be given in raw
`auxSteps` **or** as `auxDistance` + `auxUnit` (`"mm"`/`"deg"`), in which case
velocity/accel/decel are also in those physical units. Default `auxDeviceId` is
the configured aux board; `auxAxisIndex` selects the channel.

| Block | Key params | Description |
|---|---|---|
| `AuxMove` | `auxSteps` **or** `auxDistance`+`auxUnit`; `auxVelocity`, `auxAccel`, `auxDecel`; `auxWaitForDone`; `auxAbsolute` | Indexed move. Sign of steps/distance = direction. `auxWaitForDone` (default true) blocks the program until the motor finishes; false = fire-and-forget. `auxAbsolute` moves to an absolute position instead of relative. |
| `AuxContinuous` | `auxVelocity`, `auxAccel` | Start continuous jogging at a velocity until stopped. |
| `AuxStop` | `auxDecel`, `auxImmediate` | Stop the axis; `auxImmediate` halts without a decel ramp. |
| `AuxEnable` | `auxEnable` (bool) | Enable or disable the stepper drivers. |

---

## Vision & camera

### RunVision
Runs a vision program on the current camera frame and writes results into
program variables.
| Field | Meaning |
|---|---|
| `visionProgramId` / `visionProgramName` | Which vision program to run. |
| `visionZoneId` / `visionZoneVar` | Optional inspection zone (fixed or from a variable). |
| `visionOutputs` | Blob/measurement outputs → `{ inspectionId, countVar, pointsVar, detectedVar }`. |
| `colorOutputs` | Color-coverage → `{ inspectionId, coverageVar, passedVar }`. |
| `polygonOutputs` | Polygon/shape → `{ inspectionId, countVar, foundVar, angleVar, centerXVar, centerYVar }`. |
| `arucoOutputs` | ArUco markers → `{ inspectionId, countVar, foundVar, firstIdVar, firstCenterXVar, firstCenterYVar }`. |

Each `*Var` names a program variable that receives that result (counts, points,
booleans, angles, coordinates).

### SaveImage
Saves the current camera frame to disk.
| Field | Meaning |
|---|---|
| `saveImagePath` | Destination path. Supports `$variable` interpolation, including built-in `$time_ms`. |
| `saveImageCameraId` | Which camera to capture from. |

---

## Background programs

| Block | Params | Description |
|---|---|---|
| `StartBackground` | `backgroundProgramId` / `backgroundProgramName` | Start a background program running in parallel. |
| `StopBackground` | `backgroundProgramId` / `backgroundProgramName` | Stop a background program. |
| `WaitForBackground` | `backgroundProgramId` / `backgroundProgramName` | Block until a background program finishes. |

---

## Homing & CNC

### RunHoming
Runs the homing sequence from within a program. No params.

### CncProgram
Executes a CNC toolpath block. The toolpath is defined by `cncSpec` (a `CncSpec`
— DXF file, contours, holes, safe-Z, dynamics, tool-radius offset, origin mode,
per-field `expressions`) and is expanded into `MoveL`/`ThreadMove` steps at
runtime. `cncDxfFile` and `cncSafeZ` are convenience fields; `cncProgramSteps`
holds pre-baked steps from older app versions.

---

## Status

### StatusUpdate
Pushes a message to the program monitor / logs.
| Field | Meaning |
|---|---|
| `statusMessage` | Info message. |
| `statusWarning` | Warning text. |
| `statusError` | Error text. |
| `statusSeverity` | `"Info"`, `"Warning"`, or `"Error"` hint. |

---

## Notes for tooling (app helpers / website)

Every block is a `ProgramStep` with a `type` and the subset of fields above.
Programs are fetched via the `GetBuiltPrograms` WebSocket command and saved via
`SaveBuiltProgram` (see `docs/websocket-api.md`). Because the field set is a
single flat model keyed by `type`, this reference can be generated into a
structured catalog (block → fields → types/defaults) for in-app help or a
downloadable web page without duplicating the definitions.
