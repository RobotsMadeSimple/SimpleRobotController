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
- **Every step has:** `id` (string), `name` (optional label), `type`, `comment`
  (free text, not executed) and `enabled` (optional; `false` skips the step —
  and a block's whole body — at run time, logged as `[Skipped — disabled]`; it
  still counts as a step, and the validator reports it once as a
  `disabledStep` warning without checking anything inside it).
- **Step catalog:** `GetStepCatalog` describes every block type below — category,
  label, typed fields (kind, unit, default, range, expression key, visibility)
  and text syntax. See [program-language.md §5](program-language.md#5-the-step-catalog).

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
- **`isComputed`** — a named formula (user-defined property); see
  [Computed variables](#computed-variables).

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
normally declared empty and filled at run time by `RunVision`: a tool's `points`
fill a `Point` list, and the `cells` of a [gridded tool](#runvision) fill a
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
a bare `baseSpeed` is not a lookup, it evaluates to `0` (the validator warns
about it). The built-in `$time_ms` is also available (e.g. in `SaveImage` paths).
The full contract is [docs/expressions-and-variables.md](expressions-and-variables.md).

| Syntax | |
|---|---|
| Literals | `3.14` `-5` `1e3` `true` `false` |
| Variables | `$name` — program, routine, global and persistent variables |
| IO | `$stb.in1` `$stb.out2` `$relay.3` `$nano.<board>.<pin>` |
| Properties | `$robot.x` `$program.runCount` `$time.hour` … — read-only, see below |
| Lists | `$list.length` `$list.count` `$list[i]` `$pts[i].x` `$pts[i][0]` `$recs[i].field` |
| Arithmetic | `+` `-` `*` `/` `%` `^` — `%` is the remainder (sign of the left side), `^` is power; `/` and `%` by zero give `0` |
| Comparison | `==` `!=` `<` `<=` `>` `>=` |
| Logic | `and` `or` `not` — `&&`, `\|\|` and `!` are accepted spellings of the same three |
| Conditional | `cond ? a : b` |
| Functions | `abs(x)`, `min(a, b, …)`, `round(x, 2)`, `sin(deg)`, `len($list)` … — see below |
| Grouping | `(…)` — braces `{…}` are ignored, so `{$x + 1}` also works in a numeric field |

Precedence, tightest first: `^`, unary `-`, `* / %`, `+ -`, comparison, `not`,
`and`, `or`, `? :`. So `$count > 5 and $count < 10` needs no parentheses,
`not $a > 5` means `not ($a > 5)`, `-2 ^ 2` is `-4`, `2 ^ 3 ^ 2` is `2 ^ 9`
(`^` is right-associative, as is `?:`), and `$n > 0 ? $sum / $n : 0` needs no
parentheses either. A `not` written as an operand (`$a == not $b`) binds like
unary minus.

Comparison and logic yield `1` or `0` — which is exactly how a boolean variable
is stored, so a comparison can be assigned to one directly. Any non-zero value
counts as true on the way in. Chained comparisons are left-associative as in C
rather than mathematical: `1 < 2 < 3` is `(1 < 2) < 3`, which is `1`. `==`
compares within `1e-9`, the same tolerance an `IfCondition` row uses, so
`0.1 + 0.2 == 0.3` holds. Both sides of `and`/`or` and both branches of `?:` /
`if()` are always evaluated — nothing has side effects, and it means a typo'd
variable fails the run whichever way the condition goes.

#### Functions

Names are case-insensitive. Angles are in **degrees**, in and out.

| Function | Result |
|---|---|
| `abs(x)` `sign(x)` `sqrt(x)` `pow(x, y)` | absolute value, -1/0/1, square root, x to the y |
| `min(a, b, …)` `max(a, b, …)` | smallest / largest of any number of arguments |
| `clamp(x, lo, hi)` | x limited to lo…hi |
| `round(x)` `round(x, digits)` | rounds half **away from zero** (`round(2.5)` = 3, `round(-2.5)` = -3); negative digits round to tens, hundreds… |
| `floor(x)` `ceil(x)` `trunc(x)` | down, up, toward zero |
| `mod(a, b)` | same as `a % b` |
| `sin(deg)` `cos(deg)` `tan(deg)` | trigonometry on degrees |
| `asin(x)` `acos(x)` `atan(x)` `atan2(y, x)` | inverse trigonometry, result in degrees |
| `deg(rad)` `rad(deg)` | unit conversion |
| `hypot(x, y)` `dist(x1, y1, x2, y2)` `dist3(x1, y1, z1, x2, y2, z2)` | lengths and distances |
| `if(cond, a, b)` | same as `cond ? a : b` |
| `len($list)` `sum($list)` `avg($list)` `minOf($list)` `maxOf($list)` | over a list variable; `len` works on any list, the others on number/boolean lists (an empty list gives 0). The argument must be a bare list reference. |
| `rand()` `rand(lo, hi)` | uniform random number in [0, 1) or [lo, hi) |
| `map(x, inLo, inHi, outLo, outHi)` `lerp(a, b, t)` | range re-mapping and linear interpolation |
| `<pluginId>.<name>(…)` | a plugin's function, e.g. `scale.tare()` — see [Plugin](#plugin--a-step-a-plugin-provides). Blocks the program up to the function's timeout (default 250 ms); a failure (not running, timeout, error) stops the program with the plugin's message. |

#### Properties

Read-only system values, resolved live whenever an expression reads them (and
only then). They can be read anywhere an expression is allowed and in `{…}`
text interpolation, but never assigned — the validator rejects a Set Variable,
loop, vision or HTTP target named like one (`readOnlyProperty`).

| Property | Value |
|---|---|
| `$robot.x` `y` `z` `rx` `ry` `rz` | current TCP pose |
| `$robot.targetX` … `targetRz` | commanded target |
| `$robot.moving` `homed` `faulted` `driverConnected` | 1/0 |
| `$robot.speedS` `accelS` `decelS` `speedJ` `accelJ` `decelJ` | current motion defaults |
| `$robot.speedOverride` | percent |
| `$robot.joint1` `joint2x` `joint2z` `joint4` | joint readouts |
| `$program.runCount` `stepIndex` `stepCount` `elapsedMs` `loopDepth` | the current run |
| `$time.now` (unix ms) `hour` `minute` `second` `dayOfWeek` (0 = Sunday) `dayOfYear` | local wall clock |
| `$aux.<deviceId>.<axisIndex>.position` `$aux.<deviceId>.moving` | aux axes (steps, 1/0) |
| `$<pluginId>.<name>` | a running plugin's live property, e.g. `$scale.weight` (unknown until the plugin sets it) |

A name that is a variable or IO value wins over a property of the same name.

#### Errors

An unknown `$name` stops the program with an error — a typo'd offset must never
move the robot somewhere unintended. A **syntax error** (unbalanced parentheses,
a stray token such as `1 2` or `#`, an unknown function, a function given the
wrong number of arguments) also stops the program, with the position of the
problem; older builds silently fell back to the field's literal value.
`ValidateBuiltProgram` reports both before a run.

#### Expressions as a variable's initial value

A number or boolean variable can carry `valueExpression` instead of a literal
`value`. It is evaluated **once at program start**, and again each time a routine
is entered, against the variables declared *above* it plus IO — variables
initialise in declaration order, so it cannot see one declared below. Any
non-zero result makes a boolean `True`.

`value` should still be written alongside it: it is what a build that does not
understand `valueExpression` uses, and the fallback for a runtime failure other
than a syntax error or an unknown `$name` — both of those error the program at
start, the same as anywhere else.

#### Computed variables

A variable with `isComputed: true` is a named formula — a user-defined property.
Its `valueExpression` is evaluated **every time** `$name` is read, against the
live variables, lists, IO and properties, exactly like `$robot.x`; it has no
stored value (`value` is ignored). It works everywhere an expression is read:
step fields, conditions, list indexes, `{…}` and bare `$name` text
interpolation, `EvaluateExpression`.

```json
{ "name": "area",  "isComputed": true, "valueExpression": "$w * $h" }
{ "name": "ready", "isComputed": true, "valueExpression": "$area > 100 and $stb.in1", "isBoolean": true, "displayOnMonitor": true }
```

- **Reading** — nested computed references are fine (`$volume` = `$area * $d`).
  Unlike an initial value, a formula can use variables declared below it: it runs
  when read, not at start. A formula that ends up reading itself stops the
  program with an expression error (code `computedCycle`) instead of recursing.
  An unknown `$name` inside the formula stops the program like anywhere else.
- **Writing** — refused. A Set Variable, loop index, forEach value/index,
  vision/HTTP output variable or stopwatch naming a computed variable stops the
  program with "Cannot assign computed variable …".
- **`isGlobal`** — the program that declares it registers the formula in the
  global store on start (the latest declaring program wins); every program can
  then read it, even one that does not declare it. A global formula is evaluated
  against global variables, IO and properties only — never a reader's locals. A
  program's own computed variable of the same name wins over a global one.
- **`isBoolean`** — the monitor shows true/false; the formula still yields 1/0.
- **`displayOnMonitor`** — shows the formula's current result; `NaN` when it
  fails to evaluate.
- **Symbols** — `GetExpressionSymbols` lists them with `kind: "computed"`, the
  `expression`, `isBoolean`, `isGlobal` and a live `value` while an executor
  holds the program (global ones are evaluated from the store otherwise);
  without a program, the registered global computed variables are listed.
- **Invalid combinations** — `isPersistent`, `isString`, `isImage`,
  `isStopwatch` and list `items` make no sense for a formula; at run time the
  variable is still treated as computed.

Validation (`ValidateBuiltProgram`, all errors): the formula is parsed and every
reference resolved like any other expression (`expressionSyntax`,
`unknownVariable`, `unknownProperty`, …; an empty formula is `missingField`), plus:

| Code | When |
|---|---|
| `computedCycle` | computed variables whose formulas depend on each other in a loop (reported on every variable in the loop) |
| `computedVariable` | a write target — Set Variable, loop index, forEach value/index, vision/HTTP output variable, stopwatch — that is a computed variable |
| `computedGlobalScope` | a global computed formula that references a non-global program variable, a list, or a non-global computed variable |
| `computedKindConflict` | `isComputed` together with `isPersistent` / `isString` / `isImage` / `isStopwatch` / `items` |

### Variable interpolation

Text fields — `statusMessage`/`statusWarning`/`statusError`, `saveImagePath`,
`pointNameExpr`, and `variableExpr` on a string variable — substitute variable
references into the surrounding text:

| Form | Meaning |
|---|---|
| `$name` / `{$name}` | Scalar → value, string → value, list → a count worded by element type: `"N items"`, `"N points"`, `"N objects"`. |
| `$name[expr]` / `{$name[expr]}` | One element, rendered by element type — a `Number` as the value, a `Boolean` as `True`/`False`, a `Point` as `(x=…, y=…, …)`, a `Record` as `(row=0, col=1, …)`. `expr` is itself an expression, and an empty list renders `(empty)`. |
| `$name[expr].z` / `{$name[expr].z}` | One named field of that element — a point's axis, or a record's field. Unknown fields render `0`. |
| `{expr}` | Any math expression, e.g. `{$index + 1}`, `{$row * 3 + $col}`, `{round($mm, 1)}` or `{$robot.z}`. Braced form only — properties and dotted names need braces (`$robot.x` bare reads as `$robot` followed by text). |

The braces are purely a delimiter — the `$` is required inside them just as it is
everywhere else. They exist for two reasons. They let a reference sit directly
against other text: `{$prefix}{$index}` → `bin3`, where `$prefix_2` would instead
read the `_2` as part of the variable name. And they bound a full expression,
which the bare `$` form has no way to terminate.

Anything that resolves to neither a known variable nor a valid expression is left
in the text as written rather than substituted. A braced body containing a name
written *without* its `$` is left alone for the same reason: to the evaluator a
bare word is not a lookup but the value `0`, so substituting it would quietly
produce a wrong answer instead of an obvious one. Function names followed by `(`,
`true`/`false` and `and`/`or`/`not` are not bare names.

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

### Stop or record on condition (MoveL, MoveJ, JumpL, JumpJ, AuxMove, GripperAction)

Any of these move blocks can watch a condition while the move is in flight —
"move with force" is the common case: push until a load cell reads a limit, then
stop there. All four fields are optional; no `moveStopCondition` rows = off.

| Field | Meaning |
|---|---|
| `moveStopCondition` | A ConditionGroup (same shape as `waitCondition`), evaluated every executor tick while the move runs, against live IO. |
| `moveStopMode` | `"stop"` (default) or `"record"`. **stop**: the move ends where it is the first time the condition is true and the program continues with the next step — no pause, no error. **record**: the move runs on to its target; only the position at the first true evaluation is captured. |
| `moveStopPositionVariableName` | Where to write the position at the moment the condition first became true. Robot moves: a **Points list** variable, replaced with exactly one element (the live TCP pose), so `$var[0]` / `$var[0].z` read it. Aux moves: a **number** variable receiving the axis position in its physical unit (mm / deg), or raw steps when the axis has no unit. |
| `moveStopFlagVariableName` | A number / boolean variable set when the step completes: `1` if the condition fired during the move, `0` if not. Created on first use, like `waitTimeoutVariableName`. |

A force limit is just the condition `$io.Arduino_Nano.Load_Cell >= 2000` — load
cells read in grams, and a nano or pin name with spaces is reachable through its
underscore alias (see [expressions-and-variables.md](expressions-and-variables.md) §2).

Rules:
- A `MoveL` with a stop condition never blends, and a blended run stops gathering
  before one; a `JumpL` with a stop condition takes the stepped (non-blended)
  path. The watch on a Jump spans all three legs: in stop mode the remaining legs
  are skipped; in record mode the Jump finishes normally.
- `AuxMove` only watches when `auxWaitForDone` is true (the validator warns
  otherwise). An aux stop is a smooth stop at the move's own deceleration.
- A user Stop / Continue mid-move keeps the watch (a fired record-mode watch does
  not fire again after Continue).
- An unknown variable or a syntax error in the condition ends the program with
  `Stop condition on <step>: …`, like a while-loop condition.

The old `MoveWithForce` block (press along an axis until a load cell reads a target)
was removed in favour of this; a program still holding one loads it as an `Unknown`
step, which the executor skips.

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
| `AuxMove` | `auxSteps` **or** `auxDistance`+`auxUnit`; `auxVelocity`, `auxAccel`, `auxDecel`; `auxWaitForDone`; `auxAbsolute`; `moveStop*` | Indexed move. Sign of steps/distance = direction. `auxWaitForDone` (default true) blocks the program until the motor finishes; false = fire-and-forget. `auxAbsolute` moves to an absolute position instead of relative. Takes the [stop-condition fields](#stop-or-record-on-condition-movel-movej-jumpl-jumpj-auxmove-gripperaction) when it waits. |
| `GripperAction` | `gripperAction` (`Open` / `Close` / `GripTo`), `gripTargetMm`, `auxVelocity`, `auxAccel`; `moveStop*` | Drive a gripper-role axis to its open / closed gap or to `gripTargetMm`. Takes the stop-condition fields — e.g. close until a load cell reads a grip force. |
| `AuxContinuous` | `auxVelocity`, `auxAccel` | Start continuous jogging at a velocity until stopped. |
| `AuxStop` | `auxDecel`, `auxImmediate` | Stop the axis; `auxImmediate` halts without a decel ramp. |
| `AuxEnable` | `auxEnable` (bool) | Enable or disable the stepper drivers. |

---

## Vision & camera

### RunVision
Runs a vision program on the current camera frame and writes its results into
program variables. A vision program is image layers plus a list of building-block
tools (find a line / corner / circle, blobs, pattern and contour match, ArUco, fixtures,
calipers, distances and angles, pixel counts, intensity, edges, polygons, barcodes); the
model, every tool, its values and how fixtures move regions are in
[vision-tools.md](vision-tools.md).

| Field | Meaning |
|---|---|
| `visionProgramId` / `visionProgramName` | Which vision program to run. |
| `passVariable` | Optional: set to the program's overall pass (1/0) — every enabled tool with `includeInPass` passed. |
| `toolOutputs` | `[{ toolId, value, variable }]`: one tool result into one variable (below). |
| `outputFrame` | Frame of the positions written (`points`, `pose`, item and line coordinates, and a tool's `x`/`y` values): `"pixel"` (the default), `"normalized"` (0–1) or `"robot"` (mm on the camera's calibration plane; points get `z = planeZ`). Other values — widths, radii, distances — stay in pixels. `"robot"` needs a calibrated camera ([camera-calibration.md](camera-calibration.md)): validation reports `cameraNotCalibrated`, and at run time the step fails if the camera has no calibration or now runs at a different resolution than it was calibrated at. Any other value is `badOutputFrame`. |
| `waitTimeoutMs` | Max time to wait for a fresh vision result (default 30 000 ms; `0` or less waits forever). Expiry ends the program with an error. |

`value` picks what is written:

| `value` | Variable | Written |
|---|---|---|
| any key of the tool's `values` (`angle`, `count`, `percent`, `distance`, `radius`, …) | Number (String for text such as a barcode's `value`) | The value; `x` / `y` converted by `outputFrame`. |
| `passed` | Boolean | 1 when the tool's status is ok and every limit is met. |
| `found` | Boolean | The tool's `found` value. |
| `status` | String | `ok`, `fail`, `notRun` or `error`. |
| `cells` | Record list | One record per grid cell, row-major: `row`, `col`, `index` (= `row * cols + col`), `value` (the tool's main value: `percent`, the chosen `statistic`, `density`, `count`), `passed` (1/0). |
| `items` | Record list | The tool's list records (blobs, matches, markers, codes…), numeric fields only (`x`, `y` converted). |
| `points` | Point list | The tool's points (edge points, corners, caliper edges…). |
| `lines` | Record list | `{ x1, y1, x2, y2 }` per line. |
| `pose` | Record list | One record `{ x, y, z, angle }` when the tool found its object, else empty — read `$pose[0].x`. |

A tool with no result (disabled) or that did not run empties its lists and writes 0 for
`passed` / `found` / `count`; other values are left unchanged. Variables named in
`toolOutputs` and `passVariable` are created by the step if not declared. Validation
warns (`unknownVisionTool`) when a `toolId` is not in the vision program, and reports a
`toolOutputs` entry with no tool, value or variable (`missingField`).

```
$whiteCells.length            → number of cells
$whiteCells[0].value          → main value of the top-left cell
$whiteCells[$i].passed == 1   → condition on the cell a forEach loop is on
$partPose[0].angle            → a located part's angle
```

When a tool's fixture is not found, the tool does not run that frame (status `notRun`,
`passed` 0, its cells empty) rather than measure at a guessed position. If a tool throws,
the result carries an `errors` entry (`"<toolId>: <message>"`) and its status is `error`.

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

## Plugins

### Plugin — a step a plugin provides

Runs one of the steps an installed plugin declares in its manifest
([plugins.md](plugins.md) §6). The program waits (without blocking anything else)
until the plugin answers, then writes the mapped outputs and moves on.

| Field | Meaning |
|---|---|
| `pluginId` | The plugin's manifest `id` (e.g. `"scale"`). |
| `pluginStepId` | The manifest step `id` (e.g. `"weigh"`). |
| `pluginParams` | `{ key: text }`, one entry per param; how the text is read depends on the param's type (below). A missing or blank entry uses the manifest default; a `required` param without a default stops the program. |
| `pluginOutputs` | `[{ key, variableName }]` — which outputs to write, and where. |
| `pluginTimeoutMs` | Overrides the manifest step's `timeoutMs` (0 = no timeout). |

| Param type | Text | Sent to the plugin |
|---|---|---|
| `number` | an expression (`5`, `$count * 2`) | number |
| `boolean` | an expression (`true`, `$a > 3`); non-zero is true | boolean |
| `string` | a template (`Bin {$i}`) | string |
| `enum` | one of the param's `options`, taken literally | string |
| `point` | a saved point name (`P1`), `grid:<grid name>[row, col]` or `grid:<grid name>[index]`, `stack:<stack name>[index]` (indices are expressions), `$pts[$i]`, or a template naming a point — like a move's `pointNameExpr`; the local frame is **not** applied | `{x,y,z,rx,ry,rz}` |
| `list` | a list variable name (`history` or `$history`) | array (numbers, or objects for points/records) |
| `image` | an image variable name | base64 JPEG string |
| `variable` | a variable name | the name itself |

| Output type | Written to |
|---|---|
| `number` / `boolean` | a number or boolean variable (created when undeclared) |
| `string` | a text variable |
| `point` | a points list variable — replaced by a one-element list, read as `$var[0].x` |
| `list` | a list variable (a declared list keeps its element type; a new one takes numbers / points / records from the data) |
| `image` | an image variable |

An output the plugin does not return is skipped. A value that does not fit its
variable stops the program (`pluginOutputType`). While the step runs the monitor
shows `<plugin name>: <step label>`, followed by `: <message>` (and `(n%)`) when
the plugin reports progress. The program stops with an error when the plugin is
not running (`Plugin '<id>' is not running`), when it answers with an error
(`<plugin name>: <step label>: <message>`), or on timeout
(`Plugin step '<label>' timed out after N ms`; the plugin gets `step.cancel`).
A Stop while the step is outstanding cancels it (`step.cancel`, reason
`stopped`); Continue sends it again. Background programs run Plugin steps too.

---

## Program editor v2 blocks

New blocks from the program-editor order (design: [program-language.md](program-language.md)
§4). The step types, fields and enum strings are fixed (see the catalog and the
goldens); each section is filled in by the package that implements the block
(the letter in brackets), with small, section-scoped edits. A run that reaches a
block whose package has not landed yet ends with "… not implemented on this
controller yet".

### Note — note or section header (A)

`type: "Note"`. Not executed and not counted as a step. The text is in `comment`;
`noteStyle` is `"note"` (default) or `"section"` (a header; the editor can
collapse the steps up to the next section).

### LoopControl — break / next iteration (B)

`type: "LoopControl"`, `loopControl: "break"` (default) | `"continue"`. **Break** leaves the
nearest enclosing loop; the step after the loop runs next. **Next iteration** ends this pass:
a count loop counts it, a for-each moves to the next element, a while loop re-checks its
condition. Branches and Try bodies between the step and the loop are simply left. It never
crosses a routine boundary: a Break in a routine called from a loop is an error at run time
and `loopControlOutsideLoop` (error) in the validator.

### Return (B)

`type: "Return"`. In a routine: leave it now; its out / inOut parameters are copied back (see
CallRoutine — parameters). In a main or background program: the program finishes `Complete`.

### EndProgram (B)

`type: "EndProgram"`, optional `statusMessage` (a template). Finishes the program `Complete`
from any depth (inside loops, routines, Try), with the message as the final status text. A
background program ends only itself.

### RaiseError (B)

`type: "RaiseError"`, `errorMessage` (template, required), `errorCode` (identifier, default
`userError`). Fails with that code and message: an enclosing Try that catches the code takes
over, otherwise the program ends `Error` with the message. A code of your own is always
catchable; raising a fatal catalog code (e.g. `motionFault`) is a validator warning, as no Try
can catch it.

### Switch (B)

`type: "Switch"`, `switchExpr` (expression, required), `switchCases: [{id, values: [expr…],
steps}]`, `switchDefault` (steps). `switchExpr` is evaluated once; the first case with a value
equal to it (`==` within 1e-9) runs, otherwise the default; no fall-through. Numbers only —
test text with `If $s.contains(…)`. Validator: `duplicateCaseValue` (warning) when two cases
share a literal value, `missingField` for a case with no value, `emptyBranch` when no case and
no default has steps.

### Try / Catch (B)

`type: "Try"`, `trySteps`, `catchSteps`, `tryRetries` (default 0), `tryRetryDelayMs` (default
0), `catchCodes` (empty = every catchable code), `errorVariableName` / `errorCodeVariableName`
(text variables; created if not declared).

When a step inside `trySteps` fails with a **catchable** code — at any depth, inside loops,
branches and called routines:

1. everything inside the Try is abandoned (a routine that did not finish copies no outputs);
2. while retries are left, the body runs again from the top after `tryRetryDelayMs`;
3. then the error message and code go into the two variables and `catchSteps` run; an empty
   catch ignores the error. The program carries on after the Try either way.

A code not in `catchCodes` goes on to the next enclosing Try. A failure inside `catchSteps`
also goes outward. Each catch is logged (`[Caught <code>] <message> — retry 1 of 2`).

**Catchable codes:** `waitTimeout`, `visionTimeout`, `visionFailed`, `httpFailed`,
`httpTimeout`, `receiveTimeout`, `pluginFailed`, `pluginTimeout`, `pluginNotRunning`,
`promptTimeout`, `pointNotFound`, `arcDegenerate`, `indexOutOfRange`, `userError` and any
RaiseError code of your own. **Fatal, never caught:** motion faults and joint limits, driver
disconnects, e-stop, a user Stop, unknown variables, expression syntax errors and writes to
computed variables — these are authoring or hardware problems a Try must not hide. What the
existing steps fail with:

| Step | Catchable failure |
|---|---|
| Wait (condition, `waitTimeoutAction: "error"`) | `waitTimeout` |
| RunVision | `visionTimeout` (no result in time), `visionFailed` (processor lost, output frame not available) |
| CaptureImage / SaveImage | `visionFailed` (no camera frame) |
| HttpRequest (waits for the reply) | `httpFailed` (network error, non-2xx status), `httpTimeout`. An empty or non-JSON reply is not a failure; fire-and-continue requests never fail. |
| HttpReceive | `receiveTimeout` |
| Plugin step, plugin functions in expressions | `pluginFailed`, `pluginTimeout`, `pluginNotRunning` |
| MoveL / MoveJ / JumpL / JumpJ with a `pointNameExpr` target | `pointNotFound` (the template names no saved point, or the points list is empty) |

A program that ends `Error` reports `errorCode` in its status (when the failure has one), and
its `currentStepId` is the step that failed.

Validator: `emptyBranch` (no try steps), negative retries or delay, catch codes that are not
identifiers (error) or are fatal (warning), and `paramOutputType` when an error variable is
declared as something other than text.

### Wait — on timeout (B)

A condition Wait with a timeout takes `waitTimeoutAction`: `"continue"` (default — carry on;
the timed-out flag says what happened) or `"error"` — fail with `waitTimeout` ("Wait …: timed
out after N ms waiting for <condition>"), which a Try can catch. The timed-out flag is written
either way. A condition that holds on the tick the timeout falls due counts as met.

### CallRoutine — parameters (B)

A routine's variables become parameters with `paramMode`: `"in"`, `"out"` or `"inOut"`. The
call gives `routineArgs: {param: expr}` and `routineOutputs: [{key: param, variableName}]`.

- Each `in` / `inOut` argument is evaluated in the **caller's** scope before the routine's
  variables are initialised (so `n = $n + 1` reads the caller's `$n`). A text parameter takes a
  template, a boolean one stores 0 / 1, a list parameter takes a list variable (`$pts`, copied).
  A parameter with no argument keeps its declared value (`paramMissing`, warning).
- When the routine ends — its last step, or a Return — each mapped `out` / `inOut` parameter is
  copied into the caller's variable (an undeclared target is created with the parameter's
  kind). An `in` parameter is never copied back.
- Parameters live in the shared variable scope (as routine variables always have), so name them
  distinctly from the caller's variables.
- A routine is a boundary: GoToLabel and Break / Next inside it never reach the caller's labels
  or loops (validator: `unknownLabel`, `loopControlOutsideLoop`).

Validator: `unknownParam` (an argument or output naming no parameter, or an output from an `in`
parameter; an argument to an `out` parameter is a warning), `paramMissing`, `paramOutputType`
(an output into a variable of a kind that cannot hold it), and the argument expressions and
templates themselves.

### SetOutput — toggle and expression value (B)

`outputAction: "set"` (default) | `"toggle"`. Toggle flips the output's current state (as
`$stb.out2`, `$relay.3` or the IO device pin reads it). Set takes `expressions.outputValue` when
present (non-zero = on), otherwise the literal `outputValue`. A pulse always reverts to the
opposite of the state the step set, so a toggle pulse flips and flips back.

### Debugging — run from step, breakpoints, step mode (B)

`ExecuteBuiltProgram { name, startAtStepId?, breakpoints?: [stepId], stepMode?: bool }`:

- **Breakpoints** pause the program *before* the step (status `Stopped`, `pauseReason:
  "breakpoint"`, `pausedAtStepId`). Continue (`StartProgram` on the paused program) runs on; a
  breakpoint in a loop pauses on every pass.
- **Step mode** pauses before every step (`pauseReason: "step"`). `StepBuiltProgram {}` runs the
  paused step and pauses before the next one; on a running program it pauses before its next
  step. Continue leaves step mode. Notes and disabled steps never pause, and a multi-tick step
  (Wait, Jump legs, vision) is never paused half-way.
- **`SetBreakpoints { stepIds }`** replaces the running program's breakpoints → `{ count }`.
- **Run from step** builds the frame stack down to the step: enclosing loops start at their
  first pass (count index 0, for-each element 0, a while condition not checked), an If / Switch
  branch, Try body or catch is entered without evaluating anything, and the steps before it do
  not run (variables start at their declared values). A step inside a routine, a disabled step
  or a CNC block is refused (`ok: false`, "Cannot run from this step: …").

Background programs ignore all of this. A plain start clears the debug state.

### SavePosition (C)

### SetSpeedFactor (C)

### MoveC — arc move (C)

### ListOp — list operations (C)

### Prompt — operator dialog (C)

---

## Notes for tooling (app helpers / website)

Every block is a `ProgramStep` with a `type` and the subset of fields above.
Programs are fetched via the `GetBuiltPrograms` WebSocket command and saved via
`SaveBuiltProgram` (see `docs/websocket-api.md`). Because the field set is a
single flat model keyed by `type`, this reference can be generated into a
structured catalog (block → fields → types/defaults) for in-app help or a
downloadable web page without duplicating the definitions.
