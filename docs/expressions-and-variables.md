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

Precedence, tightest first: `^` · unary `-` · `* / %` · `+ -` ·
comparison · `not` · `and` · `or` · `? :`. A leading `not` keeps its existing
place above comparison (`not $a > 5` is `not ($a > 5)`, as it always was); a
`not` written as an operand (`$a == not $b`) binds like unary `-`.

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

**Plugin functions** ([plugins.md](plugins.md) §5): a running plugin's functions
are called with a dotted name, `<pluginId>.<name>(…)` — `scale.tare()`,
`scale.toOz($g)` (case-insensitive, numeric arguments, numeric result; booleans
come back as 1/0). A dotted name is any `ident(.ident)+` immediately followed by
`(`; it always parses, and is resolved when it is evaluated, so the parse cache
never depends on which plugins are installed. At evaluation:

- no such plugin → `unknownFunction`; the plugin exists but has no such function →
  `unknownPluginFunction`; wrong argument count (the manifest's `minArgs`/`maxArgs`) → `badArity`;
- the plugin is not running → `pluginNotRunning`; no reply within the function's
  `timeoutMs` (default 250, max 5000) → `pluginFunctionTimeout`; the plugin answered
  `ok:false` → `pluginFunctionFailed`.

All of these error the program with the plugin's message (they are
`ExpressionParseException`s — `PluginFunctionCallException` for the last three —
so every path that reports an expression error reports them). The call blocks the
program loop up to the timeout: prefer a plugin **property** for anything read
every tick. `References`/`ReferencedNames` report the call as a function named
`pluginId.name`.

### Errors

`UnknownVariableException` (exists) stays fatal at run time. Syntax errors now
throw `ExpressionParseException { Message, Position }` instead of silently
evaluating to a fallback; the validator (section 5) reports them before a run.
`ExpressionEvaluator.TryParse(expr, out error)` (and the overload with
`out position`) checks syntax only. `ExpressionParseException.Code` is
`expressionSyntax`, `unknownFunction` or `badArity`. An empty expression is valid
and evaluates to 0; a bare word without `$` is valid and evaluates to 0 (legacy —
the validator warns).

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
| `$camera.<cameraId>.calibrated` | 1 when the camera has a camera-to-robot calibration ([camera-calibration.md](camera-calibration.md)) |
| `$<pluginId>.<name>` | a running plugin's live property, e.g. `$scale.weight` ([plugins.md](plugins.md)) |

Plugin properties sit in the chain computed variables → plugin properties →
system properties (a plugin id can never be one of the roots above). A property
the plugin has not set, or of a plugin that is not connected, is unknown
(`UnknownVariableException` at run time). They work in computed variables too.

Commands (section 4) expose the list so the app can offer them in pickers.

## 4. New commands

| Command | Params | Response (merged into the ack) |
|---|---|---|
| `ValidateBuiltProgram` | `program` (full BuiltProgram object, may be unsaved) | `problems: [{ stepId, stepPath, field?, severity: "error"\|"warning", code, message }]` |
| `EvaluateExpression` | `expression`, `programName?` | `ok`, `value` (number), `error?`, `isBoolean?` — evaluated against the named running program's variables (or globals + IO + properties when none) |
| `GetExpressionSymbols` | `programName?` | `variables: [{name, kind: "number"\|"boolean"\|"string"\|"image"\|"list", elementType?, isGlobal, isPersistent, value?}]`, `properties: [{name, description, type, pluginId?, value?}]`, `functions: [{name, signature, description, pluginId?}]`, `io: [{name, description}]`, `plugins: [{id, name, running, functions: [{name, fullName, signature, description, minArgs, maxArgs, timeoutMs}], properties: [{name, fullName, description, type, value}]}]` — plugin functions/properties also appear in `functions`/`properties` under their full name (`scale.tare`, `scale.weight`) with `pluginId` |
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

Also emitted: `unknownFunction` and `badArity` (the parse-error codes of section 2),
`readOnlyProperty` (a property or IO name used as a write target — Set Variable,
loop/vision/HTTP output variables — or declared as a variable), `unknownProgram`
(Start/Stop/WaitForBackground naming a missing program) and
`unknownStepType(warning)` (a step type the controller does not know; it is skipped),
`cameraNotCalibrated` (RunVision `outputFrame: "robot"` for a vision program whose
camera has no calibration) and `badOutputFrame` (an `outputFrame` other than
`pixel`, `normalized` or `robot`).

Plugins ([plugins.md](plugins.md) §6), checked when the controller has a plugin
manager: `unknownPlugin` (Plugin step `pluginId` not installed),
`unknownPluginStep` (no such `pluginStepId`), `pluginNotRunning(warning)` (the
plugin is installed but not running — for a Plugin step, an `id.fn(…)` call or a
`$id.prop` reference), `pluginParamMissing` (a `required` param with no text and no
default), `pluginParamEnum` (enum text not in `options`), `pluginOutputType` (an
output mapped to a variable that cannot hold it: number/boolean → number or boolean
variable, string → text variable, point → points list, list → a list, image → an
image variable; undeclared targets are fine for number/boolean/list/image; also a
warning for a mapping to an output the step does not have) and
`unknownPluginFunction` (`x.y(…)` where `x` is a plugin without function `y`;
`badArity` uses the manifest's `minArgs`/`maxArgs`). `$id.prop` for an installed
plugin that neither declares nor has set `prop` is `unknownProperty`. A `grid:` /
`stack:` point param naming a missing grid/stack is `unknownGrid`/`unknownStack`.
Problems inside templates (`statusMessage`, `saveImagePath`, string Set Variable,
text conditions) are warnings: at run time an unresolved reference there is left
as written rather than failing. `stepPath` uses the JSON field names
(`steps[2].loopSteps[0]`, `steps[1].elseIfBranches[0].steps[3]`); a routine body is
checked once, under the first call that reaches it: `steps[4].routine(Pick).steps[0]`.
Program-level problems (variables) have `stepId: null` and `stepPath: "variables[i]"`.

`EvaluateExpression` failures also carry `code` (`expressionSyntax`,
`unknownFunction`, `badArity`, `unknownVariable`, and for plugin calls
`unknownPluginFunction`, `pluginNotRunning`, `pluginFunctionTimeout`,
`pluginFunctionFailed`) and `position` (-1 when not a syntax error). The named program's variables are used while the foreground
executor holds it (running, paused, or finished and not yet reset).

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

## 7. Computed variables (user-defined properties)

A variable with `isComputed: true` is a named formula: its `valueExpression` is
evaluated every time the variable is read, against the live variables, IO and
properties, exactly like `$robot.*`. It has no stored value and cannot be
assigned.

| Rule | Detail |
|---|---|
| Model | `ProgramVariable { isComputed: true, valueExpression: "<expr>", isBoolean?, isGlobal?, displayOnMonitor? }`. `value` is ignored; `isPersistent`, `isString`, `isImage`, `isStopwatch`, `items` are invalid with `isComputed`. |
| Read | `$name` resolves the formula each time. Nested computed references are fine; a cycle is a validation error (`computedCycle`) and at run time evaluates to a program error rather than recursing. |
| Write | Any assignment (Set Variable, loop/vision/HTTP output targets) → validation code `computedVariable` (error); at run time the write is refused with a program error. |
| Global | `isGlobal` computed variables are registered in the global store by the program that declares them and readable from every program, evaluated against globals + IO + properties only. A global computed formula that references a non-global program variable → `computedGlobalScope` (error). |
| Boolean | `isBoolean` renders the value as true/false on the monitor; the formula still yields 1/0. |
| Monitor | `displayOnMonitor` works; the value shown is the formula's current result. An evaluation error shows as `NaN`. |
| Symbols | `GetExpressionSymbols` lists them with `kind: "computed"` and an extra `expression` field; live `value` when the program holds them. |
| Validation | Formula parsed and every reference resolved like any other expression (`expressionSyntax`, `unknownVariable`, …), plus the three codes above. Declaring a computed variable with a stored-kind flag → `computedKindConflict`. |
