---
name: robot-programming
description: Build, edit, validate and run programs on an ASTRO robot through the robot MCP server. Use when asked to write a robot program, inspect robot state or points, move or home the robot, or debug a program that behaves wrong at run time.
---

# Robot programming

The `robot` MCP server talks to SimpleRobotController over WebSocket. Source and
setup: `SimpleRobotController/mcp/README.md`. Full command reference:
`SimpleRobotController/docs/websocket-api.md`. Semantics of every block:
`SimpleRobotController/docs/program-blocks.md`.

## Order of operations

1. **`robot_status`** first, always. It tells you the pose, whether the robot is homed
   or faulted, *which endpoint you are connected to*, and whether the motion gate is
   open. Do not skip it and then be surprised by a refusal.
2. **`robot_points`** before writing any move — `pointName` must match a taught point
   exactly, and the robot may have far fewer points than existing programs assume.
3. **`robot_step_schema`** before writing steps you have not written before. Call it
   with no arguments for the type list, then with `types=['MoveL','Loop']` for fields.
   Guessing field names produces steps the executor silently ignores.
4. **Editing an existing program: `robot_get_program` first.** `robot_save_program`
   replaces the program wholesale. Fetch, modify, save — never reconstruct from memory.
5. **`robot_save_program`** validates and refuses on errors. It does **not** run anything.
6. **`robot_run_program`** only when the operator has asked for it and is watching.

## The `$` rule — the one that bites

Expressions require the `$` sigil. A bare word is **not** a variable lookup; it
evaluates to `0`.

```
"$baseSpeed * 2"     → 200 when baseSpeed is 100
"baseSpeed * 2"      → 0        ← silently wrong, not an error
```

Braces are a *delimiter*, not an alternative syntax — the `$` is still required
inside them. `{$index + 1}` interpolates; `{index + 1}` is left in the text verbatim.
Use braces when a reference has to butt up against surrounding text
(`"{$binPrefix}{$index}"`), or to wrap arithmetic inside a string.

Braces are optional in a numeric field; the evaluator ignores them either way.

The validator catches both mistakes. Trust it over your reading of the expression.

## What an expression can say

```
arithmetic   + - * /                        "$base * 2 - $offset"
comparison   == != < <= > >=                "$count > 5"        → 1 or 0
logic        and or not   (also && || !)    "$ready and not $fault"
grouping     (…)                            "($a > 1) and ($b > 2)"
```

Precedence, tightest first: `* /`, `+ -`, comparison, `not`, `and`, `or`. So
`"$count > 5 and $count < 10"` needs no parentheses, and `"not $a > 5"` means
`not ($a > 5)`.

Comparison and logic yield `1` or `0` — exactly how a boolean is stored — so a
comparison can be assigned to a boolean variable with no conversion. Any non-zero
value counts as true on the way in. Chained comparisons are left-associative as in
C, *not* mathematical: `"1 < 2 < 3"` is `(1 < 2) < 3`, which is `1`.

`==` compares within `1e-9`, the same tolerance an `If` condition row uses, so
`0.1 + 0.2 == 0.3` is true.

## Move targets

A move step resolves its target in this precedence order:

`gridPoint` → `stackPoint` → `varPointName` (legacy) → `pointNameExpr` → `pointName` → current TCP

That last fallback matters: **a move with no target is legal and simply does not
move.** It is a warning, not an error.

`pointNameExpr` resolves two ways, decided by shape:

| Expression | Resolves to |
|---|---|
| `$pts[$i]`, `{$pts[0]}` — *only* an indexed points variable | those coordinates directly |
| `$target`, `{$binPrefix}{$index}`, `bin{$i + 1}` — anything else | text naming a saved point |

The coordinate form is anchored at both ends. `bin$pts[0]` is text being assembled,
not a coordinate. Both forms re-resolve on every execution, so assigning the
referenced variables retargets the move — that is the whole point of them.

## Program shape

```json
{
  "name": "Pick and place",
  "description": "",
  "steps": [ { "id": "...", "type": "MoveL", "pointName": "Home" } ],
  "variables": [ { "id": "...", "name": "i", "value": 0 } ],
  "isRoutine": false,
  "isBackground": false
}
```

- `name` is the key — saving over an existing name replaces it.
- Every step needs a unique `id`. Any stable string works.
- `isRoutine: true` hides it from the program list; reach it with a `CallRoutine` step.
- `isBackground: true` runs it in parallel with the main program, and **motion, tool
  and homing steps are skipped**. The validator treats motion in a background program
  as an error.
- Variables referenced by `SetVariable` must be declared in `variables`.
- A variable's shape comes from which field it carries: `value` (number/boolean),
  `stringValue` + `isString`, or `items` + `elementType` (a list).
- A number or boolean can start from an expression instead of a literal:
  `valueExpression`. It is evaluated once at program start — and again on each
  routine entry — against the variables declared **above** it plus IO, since
  variables initialise in declaration order. Always set `value` too; it is the
  fallback if the expression cannot be evaluated.

### List variables

There is one list type. `items` holds the elements and `elementType` says what they
are — `Number`, `Boolean`, `Point`, or `Record`. Every element is a record of named
numbers whatever the type, because a point is just a record with a known field set
and a number is one with a single reserved key:

| elementType | element shape                | read as                              |
|-------------|------------------------------|--------------------------------------|
| `Number`    | `{"value": 5}`               | `$v[0]`                              |
| `Boolean`   | `{"value": 1}`               | `$v[0]` → `1`/`0`; prints True/False |
| `Point`     | `{"x":1,"y":2, ... ,"rz":6}` | `$v[0].x`, or `$v[0][2]` for z       |
| `Record`    | `{"coverage": 42.5, ...}`    | `$v[0].coverage`                     |

`$v.length` (or `.count`) works on all four. Only a `Point` list can be a move target.

`Number` and `Boolean` elements *are* the value, so `$v[0]` resolves on its own and
anything written after it is left unconsumed — `$flags[0].foo` and `$flags[0][2]`
both just read the element. On a `Point` or `Record` list a bare `$v[0]` reads `0`
instead, and positional `$v[0][2]` only means an axis on a `Point` list.

A boolean list is stored as `0`/`1` and reaches expressions that way, so it drops
straight into a condition. It renders as `True`/`False` in a status message.

`Number` and `Boolean` lists are the ones you author by hand — the other two normally
start `[]` and are filled at run time by `RunVision`. Record fields are numbers only,
so a boolean *field* inside a record is still `0`/`1`.

Older programs carry `values`, `points`, or `objects` instead. Those still load and
mean the same thing; write `items` + `elementType` in anything new.

Nested steps live in `loopSteps`, `ifSteps`, `elseSteps`, and
`elseIfBranches[].steps`. They are full steps and need their own unique ids.

## Vision grids and object variables

A vision **zone** can carry a `grid` of `{ rows, cols }`. A color coverage
inspection on that zone is then measured once per cell instead of once overall.
Only color coverage reads the grid — the other inspection types ignore it.

The grid lives on the zone, which the app's vision editor owns. You do not create
it from a program; you consume its results in `colorOutputs`:

```json
{ "inspectionId": "...", "coverageVar": "cov", "passedVar": "allOk",
  "cellsVar": "cells", "cellsPassedVar": "nOk" }
```

- `coverageVar` is still the **whole-zone** percentage.
- `passedVar` on a gridded zone means **every cell passed**, not the zone average.
- `cellsVar` needs a **`Record` list**: declare it as `{"id": "...", "name": "cells",
  "items": [], "elementType": "Record"}`. It is filled row-major with one record per
  cell — fields `row`, `col`, `index`, `coverage`, `passed` (1/0).

Read a record with `$cells[$i].coverage`; count them with `$cells.length`. An
out-of-range index or unknown field reads `0`, not an error. Fields are numbers
only, so a boolean is `0`/`1`.

To act per cell, `forEach` over the object list. It gives you an **index**, not a
value — a record is not a number:

```json
{ "type": "Loop", "loopMode": "forEach",
  "forEachVariableName": "cells", "forEachIndexVariableName": "i",
  "loopSteps": [ { "type": "IfCondition", "condition": { "combinator": "ALL",
    "items": [ { "left": "$cells[$i].passed", "operator": "==", "right": "0" } ] } } ] }
```

## Units

mm and degrees. Linear speed mm/s, accel mm/s². Joint speed deg/s, accel deg/s².
Aux axis is steps/s unless `auxUnit` is set to `"mm"` or `"deg"`.

## Safety posture

Motion tools (`robot_move`, `robot_home`, `robot_run_program`, `robot_set_output`)
refuse unless the operator has set `RMS_MCP_ALLOW_MOTION=1`. If one refuses, **tell
the operator how to enable it — do not route around it** via `robot_raw_command` or
by shelling out to a WebSocket client.

Before anything moves:

- Confirm `wasHomed` is true and `faulted` is false in `robot_status`.
- Prefer moving to a **taught point** over raw coordinates. Coordinates you derived
  yourself have not been checked against the cell for collisions.
- Say what will move and roughly where before issuing the command.

`robot_stop` always works, gate or no gate.

## Debugging a program that misbehaves

- Run `robot_validate_program` on it first — a missing `$` or a typo'd step type
  accounts for most "it runs but does nothing" reports.
- `robot_status.programs` carries progress for the running program;
  `robot_raw_command` with `GetProgramLogs` gets the log.
- Mark a variable `displayOnMonitor: true` to watch its live value.
- A step that appears to do nothing is often an unconfigured one the app saved with
  default scaffolding fields (`outputNumber`, `waitMs`, `loopCount` all present on a
  step whose type ignores them). The validator flags these as warnings.
