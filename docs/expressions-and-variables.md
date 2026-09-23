# Expressions, variables, properties, validation and revisions

The contract between the controller and the app for the program-editor
features added in September 2026. Wire additions are all additive: nothing
existing is renamed or reshaped.

## 1. Step fields

| Field | Type | Meaning |
|---|---|---|
| `enabled` | bool, optional | `false` = executor skips the step (still counted for progress, logged as `[Skipped — disabled] …`). Absent/`true` = runs. |
| `comment` | string, optional | Free text shown under the step in the editor. Never executed. |

## 2. Expression language

Expressions are numeric (doubles). Booleans are 1/0. Existing syntax stays valid.

```
Literals      3.14  -5  true  false
Variables     $name                 program, global, persistent variables
IO            $stb.in1 $stb.out2 $relay.3 $nano.<board>.<pin>
Properties    $robot.x  $program.runCount  $time.hour     (read-only, section 3)
Lists         $list.length  $list.count  $list[i]  $pts[i].x  $pts[i][0]  $recs[i].field
Arithmetic    +  -  *  /  %  ^        (% = remainder, ^ = power, right-assoc)
Comparison    == != < <= > >=         (yield 1/0; == is within 1e-9)
Logic         and or not  && || !
Conditional   cond ? a : b            (lowest precedence, right-assoc)
Grouping      ( … )
```

Precedence, tightest first: `^` · unary `-`/`not` · `* / %` · `+ -` ·
comparison · `and` · `or` · `? :`.

### Functions (case-insensitive)

| Function | Notes |
|---|---|
| `abs(x)` `sign(x)` `sqrt(x)` `pow(x,y)` | |
| `min(a,b,…)` `max(a,b,…)` `clamp(x,lo,hi)` | variadic min/max |
| `round(x)` `round(x,digits)` `floor(x)` `ceil(x)` `trunc(x)` | round half away from zero |
| `mod(a,b)` | same as `%` |
| `sin(deg)` `cos(deg)` `tan(deg)` `asin(x)` `acos(x)` `atan(x)` `atan2(y,x)` | degrees in and out |
| `deg(rad)` `rad(deg)` `hypot(x,y)` `dist(x1,y1,x2,y2)` `dist3(x1,y1,z1,x2,y2,z2)` | |
| `if(cond,a,b)` | same as `cond ? a : b` |
| `len($list)` `sum($list)` `avg($list)` `minOf($list)` `maxOf($list)` | number lists |
| `rand()` `rand(lo,hi)` | uniform |
| `map(x,inLo,inHi,outLo,outHi)` `lerp(a,b,t)` | |

Unknown function → parse error `unknownFunction`. Wrong arity → `badArity`.

### Errors

`UnknownVariableException` (exists) stays fatal at run time. Syntax errors now
throw `ExpressionParseException { Message, Position }` instead of silently
evaluating to a fallback; the validator (section 5) reports them before a run.
`ExpressionEvaluator.TryParse(expr, out error)` checks syntax only.

## 3. Properties (read-only system variables)

Resolved live at evaluation time; never assignable. Names are case-insensitive.

| Property | Value |
|---|---|
| `$robot.x/y/z/rx/ry/rz` | current TCP pose |
| `$robot.targetX/…/targetRz` | commanded target |
| `$robot.moving` `$robot.homed` `$robot.faulted` `$robot.driverConnected` | 1/0 |
| `$robot.speedS $robot.accelS $robot.decelS $robot.speedJ $robot.accelJ $robot.decelJ` | current motion defaults |
| `$robot.speedOverride` | percent |
| `$robot.joint1 $robot.joint2x $robot.joint2z $robot.joint4` | joint readouts |
| `$program.runCount` `$program.stepIndex` `$program.stepCount` `$program.elapsedMs` `$program.loopDepth` | current run |
| `$time.now` (unix ms) `$time.hour` `$time.minute` `$time.second` `$time.dayOfWeek` (0=Sun) `$time.dayOfYear` | wall clock |
| `$aux.<deviceId>.<axisIndex>.position` `$aux.<deviceId>.moving` | aux axes |

Commands (section 4) expose the list so the app can offer them in pickers.

## 4. New commands

| Command | Params | Response (merged into the ack) |
|---|---|---|
| `ValidateBuiltProgram` | `program` (full BuiltProgram object, may be unsaved) | `problems: [{ stepId, stepPath, field?, severity: "error"\|"warning", code, message }]` |
| `EvaluateExpression` | `expression`, `programName?` | `ok`, `value` (number), `error?`, `isBoolean?` — evaluated against the named running program's variables (or globals + IO + properties when none) |
| `GetExpressionSymbols` | `programName?` | `variables: [{name, kind: "number"\|"boolean"\|"string"\|"image"\|"list", elementType?, isGlobal, isPersistent, value?}]`, `properties: [{name, description, type}]`, `functions: [{name, signature, description}]`, `io: [{name, description}]` |
| `GetBuiltProgramRevisions` | `name` | `revisions: [{ id, savedUnixMs, stepCount, variableCount, note? }]` newest first |
| `GetBuiltProgramRevision` | `name`, `id` | `program` (BuiltProgram JSON) |
| `RestoreBuiltProgramRevision` | `name`, `id` | saves that revision as current (which itself creates a new revision); returns `program` |

`SaveBuiltProgram` is unchanged but now stores a revision of the previous
content before overwriting when the content differs (section 6).

### Validation codes

`unknownPoint unknownTool unknownLocal unknownRoutine unknownVisionProgram
unknownGrid unknownStack unknownLabel duplicateLabel unknownVariable
unknownProperty expressionSyntax emptyLoop emptyBranch missingField
routineRecursion disabledStep(warning) unreachableStep(warning)
unusedVariable(warning)`.

## 5. Validation rules

Walk every step recursively (including `steps`, `elseSteps`, `elseIfBranches`,
routine bodies by id then name, with a visited set for recursion). For every
expression field (`expressions[*]`, `valueExpression`, `condition` groups,
`whileCondition`, list index expressions) run `TryParse` and resolve every
`$name` against: program variables, globals declared in this program, IO names
(pattern), properties (section 3), loop index variables and forEach item
variables in scope. Referenced points/tools/locals/grids/stacks/vision programs
must exist in their repositories.

## 6. Revisions

Stored beside the program: `builtPrograms/.revisions/<safe-name>/<unixms>.json`.
A revision is written by `SaveBuiltProgram` when the new JSON differs from the
current file, and by `RestoreBuiltProgramRevision`. Keep the newest 30 per
program; delete the folder with the program. `id` is the unix-ms file stem.
