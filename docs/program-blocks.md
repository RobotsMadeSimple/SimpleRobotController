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
  A number or boolean can start from an expression via `valueExpression` — see
  [Expressions](#expressions).
- **List** (`items` + `elementType`) — see below.
- **`isGlobal`** — shared across all concurrently running programs.
- **`isPersistent`** — value is saved on finish and restored next run.
- **`isStopwatch`** — value holds elapsed milliseconds (driven by
  `StopwatchControl`).
- **`displayOnMonitor`** — current value is shown on the monitor page while
  running.

#### List variables

There is one list type. `items` holds the elements and `elementType` says what
they are: `Number`, `Boolean`, `Point`, or `Record`. Every element is stored as a
record of named **number** fields whatever the element type, because a point is a
record with a known field set and a number is a record with one reserved key.
Numbers only — a boolean is `0`/`1` — because expressions evaluate to numbers, so
a string field would have nothing to evaluate to.

| `elementType` | Element on the wire | Read as |
|---|---|---|
| `Number` | `{"value": 5}` | `$name[expr]` |
| `Boolean` | `{"value": 1}` | `$name[expr]` → `1`/`0`; interpolates as `True`/`False` |
| `Point` | `{"x":1,"y":2, … ,"rz":6}` | `$name[expr].x`, or `$name[expr][2]` for z |
| `Record` | `{"coverage": 42.5, …}` | `$name[expr].coverage` |

`$name.length` and `$name.count` give the element count for all four. In an
expression an out-of-range index or an unknown field reads as `0` rather than
throwing — the field set depends on whatever produced the elements.
[Interpolation](#variable-interpolation) instead clamps the index to the nearest
element, since a status message reading `0` would look like real data.

The element type decides which index *forms* apply, which is why it is carried
even though the storage is the same:

- `Number` and `Boolean` elements **are** the value, so `$name[expr]` resolves on
  its own and any accessor after it is left unconsumed rather than applied —
  `$flags[0].foo` and `$flags[0][2]` both read the element itself. On a `Point`
  or `Record` list that bare form has no field to read and evaluates to `0`.
- Positional `$name[expr][2]` only *means* something on a `Point` list, which is
  the only one with a defined axis order. On a `Record` list it reads `0`; on the
  two scalar types it is the unconsumed-accessor case above.
- Only a `Point` list can be a [move target](#pointnameexpr). A `Record` that
  happens to carry `x`/`y`/`z` is still refused, so a typo'd variable name
  cannot become a move to somewhere unintended.

`Number` and `Boolean` lists are the ones authored by hand — the variable editor
gives the first value rows and the second toggles. `Point` and `Record` lists are
normally declared empty and filled at run time by `RunVision`: blob results fill
a `Point` list, and a color inspection on a [gridded zone](#zone-grids) fills a
`Record` list with one record per cell.

A `forEach` loop works on any element type, but only `Number` and `Boolean` lists
have a scalar to hand to `forEachValueVariableName` (a boolean arrives as the
`0`/`1` it is stored as). On a `Point` or `Record` list that variable receives the
index instead, and the element is read with `$name[$i].field`.

**Legacy fields.** Programs saved before the list types were unified carry
`values`, `points`, or `objects` instead of `items`. Those still load and mean
the same thing (`Number`, `Point`, `Record` respectively — there was never a
legacy spelling for `Boolean`, so it only ever appears as `items`), and the controller
writes them back unchanged; re-saving from the app is what migrates a program to
`items`. One quirk is preserved on purpose: an *empty* legacy `values` list was
indistinguishable from a scalar and is still read as one. An empty `items` with
`elementType: "Number"` is a real empty list. Write `items` + `elementType` in
anything new.

### Expressions

Most numeric fields can be a **math expression** instead of a literal. A step's
`expressions` map holds `{ "<fieldName>": "<expr>" }` keyed by the JSON field
name (e.g. `"speed": "$baseSpeed * 2"`, `"offsetZ": "$layer * 5"`). Expressions
are evaluated at execution time. Variable references **require the `$` sigil** —
a bare `baseSpeed` is not a lookup, it evaluates to `0`. The built-in `$time_ms`
is also available (e.g. in `SaveImage` paths).

| Operators | | 
|---|---|
| Arithmetic | `+` `-` `*` `/` |
| Comparison | `==` `!=` `<` `<=` `>` `>=` |
| Logic | `and` `or` `not` — `&&`, `\|\|` and `!` are accepted spellings of the same three |
| Grouping | `(…)` |

Precedence, tightest first: `* /`, `+ -`, comparison, `not`, `and`, `or`. So
`$count > 5 and $count < 10` needs no parentheses, and `not $a > 5` means
`not ($a > 5)`.

Comparison and logic yield `1` or `0` — which is exactly how a boolean variable
is stored, so a comparison can be assigned to one directly. Any non-zero value
counts as true on the way in. Chained comparisons are left-associative as in C
rather than mathematical: `1 < 2 < 3` is `(1 < 2) < 3`, which is `1`. `==`
compares within `1e-9`, the same tolerance an `IfCondition` row uses, so
`0.1 + 0.2 == 0.3` holds.

#### Expressions as a variable's initial value

A number or boolean variable can carry `valueExpression` instead of a literal
`value`. It is evaluated **once at program start**, and again each time a routine
is entered, against the variables declared *above* it plus IO — variables
initialise in declaration order, so it cannot see one declared below. Any
non-zero result makes a boolean `True`.

`value` should still be written alongside it: it is the fallback used if the
expression cannot be evaluated. An unknown `$name` in it is not a fallback case —
it errors the program at start, the same as an unknown variable anywhere else.

### Variable interpolation

Text fields — `statusMessage`/`statusWarning`/`statusError`, `saveImagePath`,
`pointNameExpr`, and `variableExpr` on a string variable — substitute variable
references into the surrounding text:

| Form | Meaning |
|---|---|
| `$name` / `{$name}` | Scalar → value, string → value, list → a count worded by element type: `"N items"`, `"N points"`, `"N objects"`. |
| `$name[expr]` / `{$name[expr]}` | One element, rendered by element type — a `Number` as the value, a `Boolean` as `True`/`False`, a `Point` as `(x=…, y=…, …)`, a `Record` as `(row=0, col=1, …)`. `expr` is itself an expression, and an empty list renders `(empty)`. |
| `$name[expr].z` / `{$name[expr].z}` | One named field of that element — a point's axis, or a record's field. Unknown fields render `0`. |
| `{expr}` | Any math expression, e.g. `{$index + 1}` or `{$row * 3 + $col}`. Braced form only. |

The braces are purely a delimiter — the `$` is required inside them just as it is
everywhere else. They exist for two reasons. They let a reference sit directly
against other text: `{$prefix}{$index}` → `bin3`, where `$prefix_2` would instead
read the `_2` as part of the variable name. And they bound a full expression,
which the bare `$` form has no way to terminate.

Anything that resolves to neither a known variable nor a valid expression is left
in the text as written rather than substituted. A braced body containing a name
written *without* its `$` is left alone for the same reason: to the evaluator a
bare word is not a lookup but the value `0`, so substituting it would quietly
produce a wrong answer instead of an obvious one.

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
| `gridPoint` `{ gridId, rowIndex, colIndex }` | A cell of a named grid. |
| `stackPoint` `{ stackId, index }` | An entry of a named stack. |
| `varPointName` (+ `varPointIndex`) | **Deprecated** — an element of a `Point` list. Still honoured for programs saved before the merge; `pointNameExpr` expresses the same thing as `$name[index]`. |
| `pointNameExpr` | A variable target, resolved fresh on every execution (see below). |
| `pointName` | A saved point by name. |

With none of them set, the move uses the robot's current TCP position, so offsets
act as relative displacements.

#### `pointNameExpr`

One field covering both kinds of variable target. Which one applies depends on
the shape of the expression:

| Expression | Resolves to |
|---|---|
| `$pts[$i]`, `{$pts[0]}` | The **coordinates** of that element, when `pts` is a list with `elementType: "Point"`. The index is itself an expression. |
| `$target`, `{$binPrefix}{$index}`, `bin{$i + 1}` | Interpolated to text, then looked up as the **name** of a saved point. Errors if nothing matches. |

The coordinate form is deliberately anchored — the expression must be *only* the
indexed reference. `bin$pts[0]` is text being assembled, so it takes the name
path. Either way the expression is re-resolved on every execution, so assigning
the variables it references retargets the move.

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
| `forEachVariableName` | [List variable](#list-variables) to iterate (forEach mode) — any `elementType`. |
| `forEachValueVariableName` | Variable that receives the current element. Only a `Number` or `Boolean` list has a scalar to give it; on a `Point` or `Record` list it receives the index instead, and the element is read with `$name[$i]`. |
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
| `visionOutputs` | Blob/measurement outputs → `{ inspectionId, countVar, pointsVar, detectedVar }`. `pointsVar` receives a `Point` list. |
| `colorOutputs` | Color-coverage → `{ inspectionId, coverageVar, passedVar, cellsVar, cellsPassedVar }`. |
| `polygonOutputs` | Polygon/shape → `{ inspectionId, countVar, foundVar, angleVar, centerXVar, centerYVar }`. |
| `arucoOutputs` | ArUco markers → `{ inspectionId, countVar, foundVar, firstIdVar, firstCenterXVar, firstCenterYVar }`. |
| `waitTimeoutMs` | Max time to wait for a fresh vision result (default 30 000 ms; `0` or less waits forever). Expiry ends the program with an error. |

Each `*Var` names a program variable that receives that result (counts, points,
booleans, angles, coordinates).

If an inspection throws while processing a frame, the vision result carries an
`errors` entry (`"<inspectionId>: <message>"`) and that inspection reports no
detections for the frame; check the vision program's editor card or the
controller log when an output stays at zero unexpectedly.

#### Zone grids

A vision zone can carry a `grid` of `{ rows, cols }`, a lattice laid over the
zone's bounding box. Cells are clipped to the zone shape, so a corner cell of a
circular zone only covers the part inside the circle. No grid, or a 1×1 one,
leaves behaviour unchanged.

A rectangle zone can also carry a `rotation` in degrees, and its lattice turns
with the rectangle rather than sitting on the bounding box — so a tray that is
not square to the camera still gets one cell per pocket. Rotation is a
rectangle-only idea: circles ignore it, and a polygon's points are already
absolute. A zone saved before rotation existed has none, which reads as 0 and
takes the original untilted path.

Only **color coverage** inspections measure per cell today; the other inspection
types ignore the grid.

On a gridded zone the color output changes shape:

| Field | Meaning |
|---|---|
| `coverageVar` | Still the whole-zone percentage. |
| `passedVar` | **Every cell passed**, not the zone average. A zone-wide average hides half-full cells, which is the thing a grid exists to catch. |
| `cellsVar` | A [list variable](#list-variables) with `elementType: "Record"`, filled with one record per cell, row-major: `row`, `col`, `index` (= `row * cols + col`), `coverage`, `passed` (1/0). |
| `cellsPassedVar` | How many cells passed. |

`cellsVar` is emptied rather than left alone when the zone has no grid, so a
stale lattice from a previous run cannot be read as current.

```
$cells.length            → number of cells
$cells[0].coverage       → coverage % of the top-left cell
$cells[$i].passed == 1   → condition on the cell a forEach loop is on
```

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
