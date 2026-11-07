# AGENT-PROGRAMMING — writing SimpleRobot programs (context for AI agents)

Purpose: everything an agent (Claude etc.) needs to write **correct** programs for
SimpleRobotController, as **Simple Robot Script** text (`.srs`) or as **block-program JSON**
(`BuiltProgram`). Dense by design; headings are stable so you can grep them
(`## RULES`, `## SRS:`, `## EXPR:`, `#### STEP MoveL`, …).

Sources of truth (this file is derived from them; when they disagree the code wins):
`RobotControl/Programs/StepCatalog.cs` (every step + field), `ErrorCodes.cs`, `RobotControl/Language/*`
(parser/printer), `docs/program-text.md`, `docs/program-blocks.md`, `docs/expressions-and-variables.md`,
`docs/program-language.md` (§4 semantics, "as built" lists), `docs/vision-tools.md`, `docs/plugins.md`,
`docs/websocket-api.md`, `mcp/README.md`. The step index, the per-step field reference and the
error-code table below are **generated from the catalog** (`AgentDocTests`; regenerate with
`scripts/update-agent-doc.ps1`), and every complete example (a block starting `#!srs 1`) is parsed by
that test, so they cannot drift.

Contents: [RULES](#rules) · [SAFETY](#safety) · [MODEL](#model-programs-steps-ids) ·
[WORKFLOW](#workflow-author--verify--save) · [SRS](#srs-lexical-rules) · [VARIABLES](#vars-declarations-types-scopes) ·
[EXPRESSIONS](#expr-the-expression-language) · [TARGETS](#motion-move-targets-and-modifiers) ·
[FLOW](#flow-semantics) · [RESTRICTIONS](#restrictions) · [ERRORS](#errors-runtime-codes-and-try) ·
[PROBLEMS](#problems-parse-and-validation-codes) · [ROUND TRIP](#roundtrip-names-ids-with-off) ·
[JSON](#json-block-program-shape) · [PITFALLS](#pitfalls) · [VERIFY](#verify-how-to-check-a-program) ·
[EXAMPLES](#examples) · [STEP INDEX](#step-index-generated) · [FIELDS](#step-fields-generated)

---

## RULES

The short list. Each is expanded below.

1. **Never run motion unless the user asked for it in this conversation and is watching.** Writing,
   parsing, validating and saving a program moves nothing; `ExecuteBuiltProgram` / `StartProgram` /
   `robot_run_program` / `robot_move` / `robot_home` / `robot_set_output` do. See [SAFETY](#safety).
2. **Verify every program before you hand it over**: `ParseProgramText` with `validate: true` (or
   `robot_program_text_set` with `dry_run: true`) must report **zero error problems**. See [VERIFY](#verify-how-to-check-a-program).
3. **Read the robot first**: `robot_points` (or `GetPoints`), tools, grids, vision programs,
   routines. Names in text must match what exists (exact, then case-insensitive, then underscore form).
4. **`$` on every variable reference** in expressions and templates. A bare word evaluates to `0`.
5. **One statement per line; blocks close with `end`.** Indentation is cosmetic (the formatter re-indents).
6. **Positional expressions need parentheses** unless they are a single `$var` token:
   `SetSpeedL ($base * 2)`, `SetOutput stb 2 ($n > 3)`. Named values (`speed=$base * 2`) do not.
7. **A plain number stores a literal; `(…)` or anything non-numeric stores an expression**
   (`speed=(200)` is the expression `200`).
8. **Declare every variable you read** (`var`, `computed`, `param`). Loop index/value variables
   (`as $i`, `for $v, $i`) need no declaration.
9. **String variables take quoted templates** (`$label = "Bin {$i}"`); number/bool variables take
   expressions (`$ok = $n > 3`). Mixing them is the parse error `stringRequired`.
10. **Background programs skip motion, speed, tool, local, homing and thread steps** (the
    controller just skips them at run time; the MCP save check reports them as errors). Put motion
    in the main program.
11. **Routines share the caller's variable scope**; give parameters names the caller does not use.
12. **Try catches only catchable codes**; motion faults, joint limits, e-stop, user Stop, unknown
    variables, expression syntax errors and computed-variable writes are fatal.
13. **Keep `@with {…}` lines exactly as they are** when editing text you read from the robot.
14. **Editing an existing program**: read it (`robot_program_text_get` / `FormatProgramText`), edit,
    write it back with the saved program as `base` (MCP does this for you) so step ids are kept.
15. **Ids, not names, are the stored link.** Text resolves names to ids; in JSON write the id
    (`pointId`, `toolId`, `routineId`, …) plus the name.

## SAFETY

- Programs are data; running them moves a machine with real mass. **Do not start a program, move,
  home, or actuate an output unless the user explicitly asked for that action now.** "Write a
  program" is not "run it".
- MCP motion tools are gated (`RMS_MCP_ALLOW_MOTION=1`). If one refuses, tell the user how to enable
  it; never route around the gate (`robot_raw_command`, a hand-written WebSocket client, …).
  `robot_stop` always works.
- **Verify on a scratch controller, not the production one.** A scratch controller is a separate
  process with its own data folder and port; it never touches the robot's `:9000` instance:
  `SimpleRobotController.exe --port 9125 --data <empty folder>` (working directory = the build
  folder). Seed it with the points/tools/grids the program uses (see [VERIFY](#verify-how-to-check-a-program)),
  send only `ParseProgramText` / `FormatProgramText` / `ValidateBuiltProgram` / `Save*`, and stop it
  by PID afterwards. With no driver board attached it still parses and validates everything.
- Before any motion the user requested: `robot_status` must show `wasHomed: true` and `faulted:
  false`; prefer taught points over coordinates you computed; say what will move and where.
- Coordinates you derive (vision, offsets, `set=`) are unchecked for collisions. Approach with a
  `JumpL`/`JumpJ` lift height (`z=`) above everything in the cell.

## MODEL: programs, steps, ids

- **BuiltProgram** `{ id, name, description, steps[], variables[], isRoutine, isBackground,
  killBackgroundOnStop (default true), editorMode ("blocks"|"text"), displayPageHtml?, displayPageMode?,
  lastUpdatedUnixMs }`. `name` is the key: saving a program with an existing name replaces it.
- **Kinds**: main program; **routine** (`isRoutine: true`; hidden from the list; run only by
  `call` / `CallRoutine`; may declare `param` variables); **background** (`isBackground: true`; started
  with `StartBackground`, runs in parallel, skips the steps whose catalog `background` is false).
- **ProgramStep**: one flat JSON object keyed by `type` (a `StepType` name). Every step has `id`
  (unique string, any format; the controller uses GUIDs), `type`, optional `name`, `comment`, `enabled`
  (`false` = skipped with its whole body), `expressions` (`{exprKey: "expression"}` overriding a literal).
  Only the fields of that type are read; others are ignored (validator warns about leftovers).
- **Nested bodies**: `loopSteps`, `ifSteps`, `elseIfBranches[].steps`, `elseSteps`,
  `switchCases[].steps`, `switchDefault`, `trySteps`, `catchSteps` (`cncProgramSteps` is legacy). Nested
  steps are full steps with their own unique ids.
- **References are by id** with the name kept for display: `pointId`+`pointName`, `toolId`+`toolName`,
  `localId`+`localName`, `routineId`+`routineName`, `backgroundProgramId`+`backgroundProgramName`,
  `visionProgramId`+`visionProgramName`, `labelId`+`labelName`, `gridPoint.gridId`, `stackPoint.stackId`,
  device ids (`outputDeviceId`, `loadCellDeviceId`, `auxDeviceId`), camera ids (`captureImageCameraId`,
  `saveImageCameraId`), vision tool ids (`toolOutputs[].toolId`), plugin `pluginId`+`pluginStepId`.
  Exception: grids/stacks still name their base point (`basePointName`).
- **Units**: mm and degrees; linear speed mm/s, accel/decel mm/s²; joint moves deg/s, deg/s²
  (websocket-api.md "Units"; the catalog's unit column shows mm/s for every `speed`); time fields in ms
  (text accepts `500ms`, `1.5s`); aux axes steps/s unless `unit=mm|deg`; load cells read grams.
- **Repositories** a program can refer to: points, tools, locals (user frames), grids (2-D: base point
  + row/col offsets + rotation, `rowCount`/`colCount`), stacks (1-D: base + index × offset, `maxCount`
  wraps), routines, background programs, vision programs (and their tools), IO devices, cameras,
  plugins.

## WORKFLOW: author → verify → save

| Goal | MCP tool (`mcp/robot_mcp.py`) | WebSocket command (`ws://<host>:9000/control`) |
|---|---|---|
| See the cell | `robot_status`, `robot_points`, `robot_io`, `robot_programs` | `GetStatus`, `GetPoints`, `GetTools`, `GetLocals`, `GetGrids`, `GetStacks`, `GetVisionPrograms`, `GetBuiltPrograms`, `GetCameras` |
| Step/field reference | `robot_step_schema` | `GetStepCatalog` (this file's generated tables) |
| Read a program as text | `robot_program_text_get(name, include_ids?, header?)` | `FormatProgramText {program, includeIds?, includeHeader?}` → `{text, languageVersion, map}` |
| Check text | `robot_program_text_set(name, text, dry_run=true)` | `ParseProgramText {text, base?, validate?: true}` → `{ok, program?, problems, map}` |
| Save text | `robot_program_text_set(name, text[, is_routine, is_background, allow_untaught])` | `ParseProgramText` then `SaveBuiltProgram <program>` |
| Check / save JSON | `robot_validate_program`, `robot_save_program(program, dry_run?, allow_untaught?)` | `ValidateBuiltProgram {program}` → `{problems}`, `SaveBuiltProgram <program>` |
| Targeted JSON edit | `robot_modify_program(name, edits, dry_run?)` | — |
| Evaluate an expression | — | `EvaluateExpression {expression, programName?}`, `GetExpressionSymbols {programName?}` |
| Run (only when asked) | `robot_run_program` (gated) | `ExecuteBuiltProgram {name}` |

- Text is usually easier and safer than JSON: the controller resolves names to ids, fills defaults and
  keeps step ids. Use JSON when a tool needs it or for a field the text has no words for.
- `robot_program_text_set` parses on the controller (against the saved program as `base`, so ids are
  kept), validates, refuses on errors (`force` skips validation, never parse errors), and lists
  `droppedFields` after a save. `allow_untaught: true` saves programs whose points/grids/routines are
  missing and reports them as `pending` (tell the user what to teach).
- `ParseProgramText`'s `ok` **is the ack's `ok`**: `ok: false` means "the text did not parse", not
  "the command failed"; read `problems` either way.
- Saving never runs anything. `SaveBuiltProgram` archives the previous version (30 revisions).

## SRS: lexical rules

| Element | Rule |
|---|---|
| File | Optional first line `#!srs 1` (the formatter writes it). Then declarations, then statements. |
| Line | One statement per line. A line continues while `(` `[` `{` is open or after a trailing `,`. A blank line ends a continuation. |
| Comment | `#` to end of line (outside strings). |
| Strings | `"…"`; escapes `\"` `\\` `\n` `\t` `\r`; other backslashes kept (`"C:\data"`). `{$x}` / `{expr}` braces are kept and filled in at run time. |
| Numbers | `3`, `-2.5`, `1e3`; millisecond fields also take `500ms`, `1.5s`. |
| Booleans | `on` `off` `true` `false`. |
| Names | Points, tools, routines, devices, …: bare if an identifier (`Pick1`), else quoted (`"Pick 1"`) or underscore form (`Pick_1`). |
| Variables | `$name` everywhere a variable is read or written (`result=$v`, `for $v in …`). |
| Positional values | After the keyword, separated by spaces (or commas): name, string, number, `on`/`off`, `$var` (with `[…]`/`.field`), or `(expression)`. |
| Named values | `name=value`, comma-separated; the value runs to the next top-level comma, so it may be any expression. |
| Keywords | Case-insensitive; the formatter writes the canonical spelling. Every StepType name is also a keyword (`IfCondition`, `SetVariable`, …) used with `@with` for steps the sugar cannot express. |

## SRS: program layout and declarations

```
#!srs 1
@program {"name": "Pick and place", "isRoutine": false}   # only in exported/imported files
var count = 0                     # Parts placed          ← trailing comment = the variable's description
var ok: bool = off
var label: string = "Bin {$i}"
var startZ = $robot.z             # non-literal → valueExpression, evaluated once at start
var pts: list point = ((x=1, y=2, z=3), (x=4, rz=90))   # zero axes omitted
var vals: list number = (1, 2.5, 3)
var flags: list bool = (on, off)
var cells: list record = ((row=0, col=1), (row=1, col=0))
var photo: image
var lap: stopwatch
computed area = $w * $h           # formula, re-evaluated on every read; cannot be assigned
computed ready: bool = $area > 100 and $stb.in1
global persistent input monitor var runs = 0   # modifiers, in this order
param in height = 20              # routine parameters: in | out | inout
param out found: bool = off
## Section header
SetTool Gripper
```

- Declaration grammar: `{global|persistent|input|monitor} (var|computed|param in|param out|param inout) NAME [: TYPE] [= INIT]`.
- Types: `number` (default), `bool`, `string`, `image`, `stopwatch`, `list number|bool|point|record`.
- `@program {…}` is read only when parsing **without** a base program (import). With a base, program
  metadata (name, kind) is kept from the base and the header is ignored.
- Declarations may appear anywhere; the formatter moves them to the top.

## SRS: comments, notes and attributes

| Text | Becomes |
|---|---|
| `## Title` | Note step, `noteStyle: "section"`, text in `comment` |
| `# …` paragraph followed by a blank line, `end` or end of file | Note step (`noteStyle` note) |
| `# …` lines directly above a statement | that step's `comment` (lines joined with `\n`) |
| `MoveL P1  # text` (trailing comment on a statement) | added to that step's `comment`; **the formatter prints it on its own line above the statement** |
| `@name "…"` | the next step's `name` |
| `@off` | the next step is disabled (`enabled: false`; skipped with its body) |
| `@id "…"` | the next step's `id` (wins over reconciliation; printed only with `includeIds`) |
| `@with {json}` | JSON merged over the next step's fields (objects merge, `null` removes) |

## SRS: statements by group

All forms parse; each line is one step. `…` = body lines.

```
# Motion
MoveL Pick1                                    # saved point
MoveJ "Pick 1", speed=50, accel=100, decel=100
MoveL here, offset=(z=20)                      # no target: relative to the current pose
MoveL $pts[$i]                                 # Points-list element (coordinates)
MoveL "bin{$i}"                                # template naming a saved point
MoveL $target                                  # string variable naming a saved point
JumpJ grid Tray[$r, $c], z=40                  # grid cell (row, col), 0-based
JumpL grid Tray[$n], z=40                      # grid cell by index (row-major; grid needs colCount)
MoveL stack Bins[$i]                           # stack slot
MoveL Pick1, blend=5                           # blend=on | blend=5 | blend=$r | blend=off
MoveL Pick1, offset=(x=1, z=$h), set=(z=30), toolOffset=(z=-5), local=Fixture1
MoveL Pick1, until=$io.Nano.Load >= 2000, untilMode=record, untilPos=$hitPts, untilFlag=$hit
JumpL Place1, z=50, zStart=40, zEnd=60
MoveC Place1, via=Pick1, speed=50, tolerance=0.1
ThreadMove distance=10, pitch=0.5, peck=on, peckDepth=2
RunHoming
SetJointPosition J3, 90                        # J1..J4; declares a value, no motion
CncProgram "part.dxf", safeZ=5                 # the toolpath itself travels in @with {"cncSpec": …}
SavePosition $pts, mode=append                 # replace | append | insert | set (index=…)
SavePosition point Pick1                       # overwrite or create a saved point

# Motion settings (persist for later moves of this program)
SetSpeedL 200, accel=500, decel=500
SetSpeedJ 50
SetBlendRadius 5
SetSpeedFactor 50                              # % of operator override, 1-100
SetTool Gripper
SetLocal Fixture1
ClearLocal

# I/O, load cells, aux axes
SetOutput stb 1 on                             # card [device] number value; stb outputs 1-4
SetOutput relay 3 off, pulse=250ms, blocking=on
SetOutput io "Arduino Nano" 4 on
SetOutput stb 2 toggle
SetOutput stb 2 ($count > 3)                   # non-zero = on
Wait 500ms
Wait $delay                                    # ms
Wait until $stb.in1 and $ok, timeout=5s, timedOut=$late, onTimeout=error
Tare device="Arduino Nano", pin=2
ReadLoadCell $grams, device="Arduino Nano", pin=2, unit=g
StartRecording $rec, device="Arduino Nano", pin=2
StopRecording device="Arduino Nano", pin=2
ExportData csv "logs/run-{$time_ms}.csv"
  "weight" = $grams
  "series" = list $rec
  "photo" = image $photo
end
AuxMove device=Aux, axis=0, unit=mm, distance=10, velocity=20, absolute=on
AuxContinuous device=Aux, axis=0, velocity=-10
AuxStop device=Aux, axis=0, immediate=on
AuxEnable on, device=Aux, axis=0
GripperAction close, until=$io.Nano.Grip >= 300  # open | close | to 25
ConveyorAction forward                         # forward | reverse | stop
RotaryIndex +1, count=3

# Flow
if $count > 5 and $ok
  …
elif $mode == 2                                # "else if" also accepted
  …
else
  …
end
repeat 3 as $i                                 # count loop; $i = 0,1,2
  …
end
repeat forever                                 # until stopped / break / exit
  …
end
while $stb.in1 == 0
  …
end
for $v, $i in $vals                            # for $v in $list · for _, $i in $pts
  …
end
switch $bin
case 1, 2
  …
default
  …
end
try retry=2, delay=500ms
  …
catch $msg, code=$code, only=(waitTimeout, httpFailed)
  …
end
break
continue
return
exit "Done after {$count} parts"
raise "No part at slot {$i}", code=noPart
label retry
goto retry
call Pick
call Pick(height=20, count=$n), out found=$ok
pause
status "Placed {$count}"                       # status warning "…" · status error "…"
stopwatch start $lap                           # start | stop | reset

# Variables and lists
$count = $count + 1
$label = "Bin {$i}"
$vals.append($grams)
$vals.insert(0, $grams)
$vals.removeAt($i)
$vals.clear()
$vals[$i] = $grams
$pts[$i].z = 10
$pts.append(x=$robot.x, y=$robot.y, z=5)
$pts.append()                                  # append the current pose (Points list)
$cells.sort(by=value, desc)
$vals.reverse()
$copy.copyFrom($vals)

# Vision, communication, background, plugins
RunVision "Detect Parts", frame=robot, pass=$visionOk
  "Top edge".angle -> $edgeAngle               # tool name (or id) . value -> $variable
  Board.cells -> $cells
end
CaptureImage $photo, camera=Top
SaveImage "shots/{$time_ms}.jpg", camera="Side Cam"
StartBackground Conveyor
StopBackground Conveyor
WaitForBackground Conveyor
HttpRequest "http://192.168.4.10:8080/move", wait=on, timeout=5s
  send "board" = list $cells
  send "photo" = image $photo
  send "n" = $count
  receive "move" -> $move
end
HttpReceive "checkers", timeout=60s            # other machines POST /webhook/checkers
  receive "x" -> $x
end
prompt "Load the tray", title="Setup", buttons=("Continue", "Skip"), result=$choice
prompt "How many parts?", input=number, default=10, min=1, max=50, value=$qty, timeout=60s, onTimeout=0
plugin scale.weigh, timeout=5s
  samples = $n * 2                             # unquoted = expression (number/boolean params)
  unit = "g"                                   # quoted = exact text (string/enum/point params)
  grams -> $w
end
unknown "MoveWithForce"                        # a step type this controller does not know
```

- Row bodies (`RunVision`, `HttpRequest`, `HttpReceive`, `ExportData`, `plugin`) take rows and `end`
  **only when they have rows**; with no rows the step is just its first line.
- A condition is one expression; its top-level `and`/`or` become separate condition rows in JSON.
- Enum words: stopwatch `start|stop|reset`, gripper `open|close|to`, conveyor `forward|reverse|stop`;
  every other enum uses its JSON value (`mode=append`, `unit=kg`, `frame=robot`).

## VARS: declarations, types, scopes

| Text | JSON `ProgramVariable` |
|---|---|
| `var n = 3` | `{"name":"n","value":3}` |
| `var n = $robot.z` | `{"name":"n","value":0,"valueExpression":"$robot.z"}` |
| `var ok: bool = on` | `{"name":"ok","value":1,"isBoolean":true}` |
| `var s: string = "Bin {$i}"` | `{"name":"s","isString":true,"stringValue":"Bin {$i}"}` |
| `var pts: list point = ((x=1, z=3))` | `{"name":"pts","elementType":"Point","items":[{"x":1,"y":0,"z":3,"rx":0,"ry":0,"rz":0}]}` |
| `var v: list number = (1, 2)` | `{"name":"v","elementType":"Number","items":[{"value":1},{"value":2}]}` |
| `var b: list bool = (on, off)` | `{"elementType":"Boolean","items":[{"value":1},{"value":0}]}` |
| `var r: list record` | `{"elementType":"Record","items":[]}` |
| `var img: image` / `var t: stopwatch` | `"isImage": true` / `"isStopwatch": true` (value = elapsed ms) |
| `computed a = $w * $h` | `{"isComputed":true,"valueExpression":"$w * $h"}` (+ `isBoolean` for `: bool`) |
| `global` / `persistent` / `input` / `monitor` | `isGlobal` / `isPersistent` / `isInput` / `displayOnMonitor` |
| `param in` / `param out` / `param inout` | `paramMode: "in" \| "out" \| "inOut"` |
| trailing `# text` | `description` |

- **Scopes**: a program's variables are its own; `global` variables are shared by every running
  program (declare the global in each program that uses it); routines **share the caller's scope**
  (their variables, parameters included, live in the same table — no true locals).
- **Initialisation**: in declaration order at program start, and again each time a routine is entered.
  A `valueExpression` sees only variables declared above it (plus IO/properties). `persistent`
  values are saved when the program finishes and restored next run.
- **Lists**: one list type; `elementType` Number | Boolean | Point | Record; every element is a record
  of named numbers (`value` for Number/Boolean; `x y z rx ry rz` for Point). Records hold numbers only
  (booleans as 0/1, no strings). Read `$v[i]` (Number/Boolean), `$pts[i].x` or `$pts[i][2]` (Point),
  `$recs[i].field` (Record); `$list.length` / `$list.count`. Out-of-range index or unknown field reads
  `0` in expressions. Only a **Point** list can be a move target.
- **Strings** are not expression values: they live in string variables and templates. Text helpers
  return numbers: `$s.contains("x")`, `.startsWith`, `.endsWith` (1/0), `.indexOf("x")` (−1 absent),
  `.toNumber()` (0 if not a number), `.length`.
- **Implicitly created**: output variables of RunVision (`toolOutputs`, `pass`), HTTP receive,
  Try `catch $msg, code=$code`, Prompt `result`, plugin outputs, stop-condition flag/position and Wait
  `timedOut` are created if undeclared — but **declare them anyway** so expressions that read them
  validate and their type is what you intend (`string` for catch variables).
- **Loop variables** (`repeat … as $i`, `for $v, $i in $list`) need no declaration.
- **Computed variables** cannot be written (Set Variable, loop variable, output target → error
  `computedVariable`), cannot be `persistent`/`string`/`image`/`stopwatch`/list; a cycle is
  `computedCycle`; a global computed formula may read only globals/IO/properties.
- **Input variables**: a display page may seed `input` scalars/bools/strings when it starts the program.

## EXPR: the expression language

Numeric (doubles); booleans are 1/0; any non-zero is true.

```
Literals      3.14  -5  1e3  true  false
Variables     $name                            (program, global, persistent, routine params, loop vars)
IO            $stb.in1..in4  $stb.out1..out4  $relay.<n>  $io.<device>.<pin>   ($io.Arduino_Nano.Load_Cell)
Properties    $robot.*  $program.*  $time.*  $aux.*  $camera.*  $<plugin>.<prop>   (read-only)
Lists         $l.length  $l.count  $l[i]  $pts[i].x  $pts[i][0]  $recs[i].field
Arithmetic    + - * / % ^            (% remainder with the sign of the left side; ^ power, right-assoc; /0 and %0 give 0)
Comparison    == != < <= > >=        (1/0; == within 1e-9; chained comparisons are C-style)
Logic         and or not   && || !
Conditional   cond ? a : b           (right-assoc, lowest precedence)
Grouping      ( … )                  ({…} braces are ignored in numeric fields)
```

Precedence, tightest first: `^` · unary `-` · `* / %` · `+ -` · comparison · `not` · `and` · `or` · `?:`.
`not $a > 5` = `not ($a > 5)`; `-2 ^ 2` = `-4`. Both sides of `and`/`or` and both branches of `?:` are
always evaluated (an unknown variable fails whichever way the condition goes).

### EXPR: functions (case-insensitive; angles in degrees)

| Function | Notes |
|---|---|
| `abs(x)` `sign(x)` `sqrt(x)` `pow(x,y)` | |
| `min(a,b,…)` `max(a,b,…)` `clamp(x,lo,hi)` | variadic |
| `round(x)` `round(x,digits)` `floor(x)` `ceil(x)` `trunc(x)` | half away from zero; negative digits round to tens… |
| `mod(a,b)` | = `a % b` |
| `sin` `cos` `tan` `asin` `acos` `atan` `atan2(y,x)` | degrees in and out |
| `deg(rad)` `rad(deg)` `hypot(x,y)` `dist(x1,y1,x2,y2)` `dist3(x1,y1,z1,x2,y2,z2)` | |
| `if(cond,a,b)` | = `cond ? a : b` |
| `len($l)` `sum($l)` `avg($l)` `minOf($l)` `maxOf($l)` | bare list reference; point/record lists name the field: `sum($pts.x)` |
| `rand()` `rand(lo,hi)` | uniform [0,1) / [lo,hi) |
| `map(x,inLo,inHi,outLo,outHi)` `lerp(a,b,t)` | |
| `inBox(x,y,z,x1,y1,z1,x2,y2,z2)` `inCircle(x,y,cx,cy,r)` | 1/0, edges inside |
| `<pluginId>.<fn>(…)` | plugin function (blocks up to its timeout, default 250 ms; failures error the program) |

Unknown function → `unknownFunction`; wrong argument count → `badArity`.

### EXPR: properties (read-only; never assign)

| Property | Value |
|---|---|
| `$robot.x y z rx ry rz` | current TCP pose (mm/deg) |
| `$robot.targetX … targetRz` | commanded target |
| `$robot.moving homed faulted driverConnected` | 1/0 |
| `$robot.speedS accelS decelS speedJ accelJ decelJ` · `$robot.speedOverride` | motion defaults · override % |
| `$robot.joint1 joint2x joint2z joint4` | joint readouts |
| `$program.runCount stepIndex stepCount elapsedMs loopDepth speedFactor` | current run |
| `$time.now` (unix ms) `hour minute second dayOfWeek` (0 = Sunday) `dayOfYear` · `$time_ms` | wall clock |
| `$aux.<deviceId>.<axis>.position` · `$aux.<deviceId>.moving` | aux axes |
| `$camera.<cameraId>.calibrated` | 1 when the camera has a robot calibration |
| `$<pluginId>.<name>` | a running plugin's live property (unknown until set) |

A variable or IO name wins over a property of the same name. Writing to a property or IO name →
validation `readOnlyProperty`.

### EXPR: templates (text fields)

Template fields (`status`, `exit`, `raise`, prompt message/title, `SaveImage`/`ExportData` paths,
string `$s = "…"`, string routine args, target names) interpolate:
`$name` / `{$name}` (value; a list renders "N items/points/objects"), `{$list[expr]}` (element),
`{$pts[$i].z}` (field), `{expr}` (any expression: `{$i + 1}`, `{round($g, 1)}`, `{$robot.z}`).
Properties and dotted names **need braces** (`{$robot.x}`). Anything that does not resolve is left as
written (template problems are warnings, not errors). `{x + 1}` without `$` is left verbatim.

## MOTION: move targets and modifiers

Target resolution order (first set wins): `gridPoint` → `stackPoint` → `varPointName` (legacy) →
`pointNameExpr` → `pointName`/`pointId` → **nothing = current pose** (then offsets make a relative move).

| Text target | JSON |
|---|---|
| `Pick1` / `"Pick 1"` / `Pick_1` | `pointId` + `pointName` |
| `$pts[$i]` (exactly an indexed Point-list reference) | `pointNameExpr: "$pts[$i]"` → those coordinates |
| `"bin{$i}"`, `$target` (anything else) | `pointNameExpr` → text naming a saved point; none → `pointNotFound` (catchable) |
| `grid Tray[$r, $c]` | `gridPoint {gridId, rowIndex, colIndex}` + `expressions.gridRowIndex/gridColIndex` |
| `grid Tray[$n]` | `gridPoint {gridId, gridIndex, useGridIndex: true}` + `expressions.gridGridIndex` (row = n / colCount) |
| `stack Bins[$i]` | `stackPoint {stackId, index}` + `expressions.stackIndex` |
| `here` | no target fields |

Modifiers (each axis may be an expression): `offset=(x=…, …)` → `offsetX…offsetRZ` (added),
`set=(…)` → `overrideX…overrideRZ` (replaces the axis outright), `toolOffset=(…)` → `toolOffsetX…RZ`
(tool frame), `local=Name` → `localId`/`localName` (this move in that frame). `speed accel decel`
override the program defaults for one move. `blend=on|off|<radius>|$r` → `blend` + `blendRadius`
(consecutive blended `MoveL`s become one path). Jumps: `z=` (`jumpZ`, lift height for both legs),
`zStart=`, `zEnd=`. Stop condition (MoveL/MoveJ/JumpL/JumpJ/AuxMove/GripperAction):
`until=<cond>` (`moveStopCondition`), `untilMode=stop|record`, `untilPos=$PointsList` (aux: number),
`untilFlag=$v` (1 if it fired). A move with a stop condition never blends.
Program moves are scaled by the operator speed override × `SetSpeedFactor`.

## FLOW: semantics

- **Loops**: `repeat n` (`loopMode` count; `loopCount` 0 = forever; `as $i` = 0-based index),
  `for $v, $i in $list` (forEach; on a Point/Record list `$v` receives the **index**; read the element
  as `$list[$i].field`), `while cond` (checked before each pass). `break` leaves the nearest loop;
  `continue` starts its next pass. Neither crosses a routine boundary.
- **If**: `if / elif / else / end`; first true branch runs.
- **Switch**: `switchExpr` evaluated once; first case with an equal value (`==` within 1e-9) runs;
  no fall-through; numbers only (test text with `if $s.contains("x")`).
- **Try**: on a catchable failure anywhere inside (loops, branches, called routines): abandon the body;
  while retries remain wait `delay` and rerun the body from the top; then write `$msg`/`$code`
  (string variables) and run `catch`; an empty catch swallows the error. Execution continues after
  `end`. A code not in `only=(…)` or a failure inside `catch` propagates to the next enclosing Try,
  else ends the program `Error`.
- **raise "msg", code=x**: fails with code `x` (default `userError`) — catchable. `status error "…"`
  only reports, it does **not** stop the program.
- **return**: leave a routine (copying its out-params back), or finish a main program. **exit "msg"**:
  finish the program `Complete` from any depth (a background program ends only itself).
- **label / goto**: jump within the same program/routine only (routine boundary). Duplicate labels are an error.
- **pause**: pauses; the operator continues or stops from the monitor.
- **call Routine(a=expr, …), out p=$var**: in/inOut args are evaluated in the **caller's** scope
  before the routine's variables initialise; a text param takes a template, a list param a list
  variable (copied). On return (last step or `return`) each mapped out/inOut param is copied to the
  caller's variable (created if undeclared). An in-param without an argument keeps its declared value
  (`paramMissing` warning). An unfinished routine (error caught outside) copies nothing.
- **Wait**: `Wait 500ms` / `Wait $ms`; `Wait until cond, timeout=…, timedOut=$v, onTimeout=continue|error`
  (`error` → catchable `waitTimeout`; the flag is written either way; timeout empty/0 = forever).
- **Prompt**: shows a dialog and waits; `result=$v` gets the button index from 0 (number var) or the
  label (string var); `input=number|text` + `value=$v`, `default=`, `min=`, `max=`; `timeout=` with
  `onTimeout=<button index>` presses that button, without it the timeout fails `promptTimeout`.
  Answered via `AnswerPrompt {promptId, button, value?}`. 1–4 buttons.
- **Background**: `StartBackground P` runs program P in parallel; `StopBackground P`;
  `WaitForBackground P` blocks until it finishes. Exchange data through `global` variables.
  `killBackgroundOnStop` (default true) stops them when the main program stops.
- **SetOutput**: `on/off`, `toggle` (flips the current state), `(expr)` (non-zero = on);
  `pulse=…` flips back after the time (`blocking=on` waits for it).
- **SavePosition**: stores the pose in the **active local frame**; `point X` overwrites/creates a
  saved point (points history records it).
- **SetSpeedFactor**: 1–100 % of the operator override for this program's later moves; resets to 100 at the end.
- **Disabled steps** (`@off`) are skipped with their whole body; Notes never execute.

## RESTRICTIONS

- **Background programs skip** (silently, logged): `MoveL MoveJ JumpL JumpJ MoveC ThreadMove RunHoming
  SetTool SetSpeedL SetSpeedJ SetSpeedFactor SetLocal ClearLocal` (the catalog `background` flag; see
  [STEP INDEX](#step-index-generated)). They can do IO, waits, variables, vision, HTTP, prompts,
  plugins, aux axes. `SavePosition point …` in a background program is a warning
  (`pointWriteInBackground`). Background programs ignore breakpoints.
- **Routine boundaries**: `goto` cannot reach a label outside the routine (`unknownLabel`);
  `break`/`continue` cannot reach a loop outside it (`loopControlOutsideLoop`, also a run-time error);
  a routine calling itself (directly or not) is `routineRecursion`.
- **What Try can catch**: see [ERRORS](#errors-runtime-codes-and-try). Fatal codes are never caught;
  `badListOperation` (a list step that does not fit its list) is fatal too.
- **MoveC**: no stop condition (`unsupportedOnStep` error); tool offset and blending are ignored
  (warnings); start/via/end collinear → `arcDegenerate` (catchable); via cannot be a grid/stack cell.
- **Lists**: only a Point list is a move target; `$pts.append()` with no fields appends the current
  pose (Point lists only); `fields` on a Number list → `badListOperation`.
- **RunVision `frame=robot`** needs a calibrated camera (`cameraNotCalibrated` error otherwise);
  only coordinates are converted (widths/radii stay in pixels).
- **Prompt**: 1–4 buttons. **SetSpeedFactor**: 1–100. **SetJointPosition**: J1–J4.
- **Strings**: no string expressions; compare text with `.contains` / `.startsWith` / `.endsWith`.
- **Switch**: numeric only.

## ERRORS: runtime codes and Try

`catch` gets the message in `$msg` and the code in `$code`; `only=(a, b)` filters codes (empty = every
catchable code). Generated from `ErrorCodes.cs`:

<!-- BEGIN GENERATED errors (AgentDocTests) -->
| Code | Meaning | Try can catch |
|---|---|---|
| `waitTimeout` | Wait timed out | yes |
| `visionTimeout` | Vision timed out | yes |
| `visionFailed` | Vision failed | yes |
| `httpFailed` | HTTP request failed | yes |
| `httpTimeout` | HTTP request timed out | yes |
| `receiveTimeout` | HTTP receive timed out | yes |
| `pluginFailed` | Plugin step failed | yes |
| `pluginTimeout` | Plugin step timed out | yes |
| `pluginNotRunning` | Plugin not running | yes |
| `promptTimeout` | Prompt timed out | yes |
| `pointNotFound` | Point not found | yes |
| `arcDegenerate` | Arc points are in a line | yes |
| `indexOutOfRange` | List index out of range | yes |
| `userError` | Raised error | yes |
| `motionFault` | Motion fault | **no — fatal** |
| `jointLimit` | Joint limit | **no — fatal** |
| `driverDisconnected` | Driver disconnected | **no — fatal** |
| `emergencyStop` | Emergency stop | **no — fatal** |
| `userStop` | Stopped by user | **no — fatal** |
| `unknownVariable` | Unknown variable | **no — fatal** |
| `expressionSyntax` | Expression error | **no — fatal** |
| `computedVariable` | Write to a computed variable | **no — fatal** |
| `badListOperation` | List step does not fit the list | **no — fatal** |
| any other identifier (`raise …, code=myCode`) | your own RaiseError code | yes |
<!-- END GENERATED errors -->

What raises what: Wait `onTimeout=error` → `waitTimeout`; RunVision → `visionTimeout` / `visionFailed`;
CaptureImage/SaveImage with no frame → `visionFailed`; HttpRequest (waiting) → `httpFailed`
(network, non-2xx) / `httpTimeout`; HttpReceive → `receiveTimeout`; Plugin steps and plugin functions
→ `pluginFailed` / `pluginTimeout` / `pluginNotRunning`; Prompt timeout without `onTimeout` →
`promptTimeout`; template/`$var` move target naming no point (or an empty Points list) →
`pointNotFound`; MoveC collinear → `arcDegenerate`; ListOp/SavePosition index outside the list →
`indexOutOfRange`; `raise` → its code (`userError` by default).

## PROBLEMS: parse and validation codes

`TextProblem = {line, col, endLine, endCol, severity, code, message, stepId?, field?, source: "parse"|"validate"}`
(1-based; columns in UTF-16 units; `endCol` exclusive). A bad line is skipped and parsing continues,
so all problems come back at once.

| Parse code (`source: "parse"`) | Meaning / fix |
|---|---|
| `syntax` | not valid here (message says what was expected) |
| `unknownKeyword` | statement word does not exist (message suggests one) |
| `unknownArgument` / `duplicateArgument` / `missingArgument` | named value the step lacks / given twice / required one missing |
| `badValue` | wrong value kind: text for a number, enum word not in the options, an unparenthesised expression in a positional slot |
| `expressionSyntax` / `unknownFunction` / `badArity` | inside an expression |
| `missingEnd` / `unexpectedEnd` / `misplacedClause` | block structure (`else`/`case`/`catch` in the wrong place) |
| `declarationKind` | e.g. `persistent computed` |
| `stringRequired` | string variable assigned an expression, or number variable assigned quoted text |
| `duplicateId` | two `@id`s equal |
| `ambiguousName` (error) | an underscore form matches two names — quote the real name |
| `unknownPoint` `unknownTool` `unknownLocal` `unknownGrid` `unknownStack` `unknownRoutine` `unknownProgram` `unknownVisionProgram` `unknownVisionTool` `unknownDevice` `unknownCamera` `unknownLabel` (warnings) | name does not resolve **yet**; the name is kept (the validator then reports the missing point/tool/… as an **error**) |
| `unknownStepType` (warning) | `unknown "X"` — a step type this controller does not know |
| `unknownField` (warning) | an `@with` key that is not a field (dropped) |

Validation (`source: "validate"`, from `ValidateBuiltProgram`; errors block Run and MCP saves):
`unknownPoint unknownTool unknownLocal unknownRoutine unknownVisionProgram unknownGrid unknownStack
unknownProgram unknownLabel duplicateLabel unknownVariable unknownProperty readOnlyProperty
expressionSyntax unknownFunction badArity emptyLoop emptyBranch missingField routineRecursion
loopControlOutsideLoop unknownParam paramOutputType unsupportedOnStep computedCycle computedVariable
computedGlobalScope computedKindConflict cameraNotCalibrated badOutputFrame unknownPlugin
unknownPluginStep pluginParamMissing pluginParamEnum pluginOutputType unknownPluginFunction`; warnings:
`disabledStep unreachableStep unusedVariable untaughtGrid unknownStepType unknownVisionTool
paramMissing duplicateCaseValue pointWriteInBackground pluginNotRunning`. Missing references
(`unknownPoint`, `unknownGrid`, …) are errors at validation: save with `allow_untaught` when the user
will teach them later.

## ROUNDTRIP: names, ids, @with, @off

- **Names → ids.** Matching order: exact → ignoring case → underscore form (case-sensitive, then
  not). The underscore form replaces every run of non-identifier characters with one `_`:
  `"Pick 1"` → `Pick_1`, `"Bin - left"` → `Bin_left`, `"Arduino Nano"` → `Arduino_Nano` (the same alias
  `$io.Arduino_Nano.…` uses). Works for points, tools, locals, grids, stacks, routines, background
  programs, vision programs and their tools (`Top_edge.angle -> $a`), devices, cameras and `goto`
  labels. Ambiguous underscore form → `ambiguousName`: write the quoted name. A label declaration stays
  quoted (`label "Try again"`), `goto Try_again` refers to it.
- **The formatter** prints a name bare if it is an identifier, else its underscore form when that
  reads back as the same item and is not a keyword/reserved word, else quoted (also quoted when the
  item is unknown or starts with a digit).
- **Unknown names are warnings**: a program can be written before its points are taught. For id-only
  references (devices, cameras, grids, stacks, vision tools) an unknown name is kept as the id text.
- **Ids are kept across edits** when you parse with `base` = the program being edited: steps are
  matched list by list (type + primary reference, LCS, moved steps, changed fields). New steps get
  fresh GUIDs. `@id "…"` pins an id. Variables match by name. Condition rows, else-if branches and
  switch cases keep their ids.
- **`@with {…}`** carries fields the text has no words for (old editor leftovers, a CNC toolpath,
  a RunVision `waitTimeoutMs`, …), merged over the statement (objects merge, `null` removes). The
  formatter writes a field it cannot express into `@with` rather than lose it. **Leave `@with` lines
  untouched.** You may add one yourself for a field that has no text form, e.g.
  `@with {"waitTimeoutMs": 10000}` above a `RunVision`.
- **`@off`** = `enabled: false` (step and body skipped; validator warns `disabledStep` and checks
  nothing inside). Remove the line to re-enable.
- **Guarantee**: `Normalize(Parse(Format(P))) == Normalize(P)`, ids included; `Format` is
  idempotent (gofmt-like: two-space indent, declarations first, canonical keyword spelling).
  Normalize only makes behaviour-neutral changes (legacy fields to current form, empty lists dropped,
  names refreshed from ids, implied defaults written, editor leftovers removed).

## JSON: block-program shape

```
BuiltProgram  { id, name, description, steps: ProgramStep[], variables: ProgramVariable[],
                isRoutine, isBackground, killBackgroundOnStop, editorMode? }
ProgramStep   { id, type, name?, comment?, enabled?, expressions?: {exprKey: "expr"}, …type fields }
ConditionGroup{ combinator: "ALL" | "ANY", items: [ { id, expr, join?: "AND" | "OR" } ] }
                (legacy items {left, operator, right} still load; write expr)
ElseIfBranch  { id, condition: ConditionGroup, steps }
SwitchCase    { id, values: ["1", "$n + 1"], steps }
GridPointRef  { gridId, rowIndex?, colIndex?, gridIndex?, useGridIndex }
StackPointRef { stackId, index? }
Row (send)    { key, expr } | { key, listVar } | { key, imageVar }      (jsonOutbound, exportColumns)
Mapping (in)  { key, variableName }                                    (jsonInbound, httpReceiveInbound, pluginOutputs, routineOutputs)
VisionOutput  { toolId, value, variable }                              (toolOutputs)
routineArgs / pluginParams / listFields   { name: "expression or template" }
```

- Field names, kinds, defaults and expression keys per step: [FIELDS](#step-fields-generated).
  A composite text value maps to several JSON fields (`target` → `pointId pointName pointNameExpr
  gridPoint stackPoint`; `offset` → `offsetX…offsetRZ`).
- **Expression-capable fields** (an "Expr key" in the tables): put the literal in the field and/or an
  expression in `expressions[exprKey]`; the expression wins at run time. Examples:
  `"expressions": {"speed": "$base * 2", "overrideX": "$parts[$i].x", "gridGridIndex": "$i"}`.
- Write ids for references (`pointId` from `GetPoints`, `toolId` from `GetTools`, …) **and** the name.
  A name without an id is resolved by name when the program is normalized/formatted.
- Every step and every condition item / branch / case needs a unique `id`.
- A list variable must carry `"items": []` (an empty array) and `elementType`; without `items` it is a scalar.
- A reference id that does not exist on this robot (e.g. a `pointId` copied from another robot) is
  kept: the step still resolves by name, but the formatter shows it as `@with {"pointId": "…"}`.
  Drop foreign ids, or let `ParseProgramText` fill the real ones.
- Field names are case-sensitive in your JSON; a misspelt field is silently dropped (MCP reports it as `droppedFields`).
- Enum values are case-sensitive strings exactly as the catalog lists them (`"Open"`, `"RunForward"`,
  `"Start"`, `"Warning"`, `"forEach"`).

## PITFALLS

- `speed=baseSpeed` (no `$`) → reads 0; the validator only **warns** (`expressionSyntax` warning
  "a bare word evaluates to 0"). Treat that warning as an error. `$nope` (undeclared) is an error.
- `SetSpeedL $base * 2` → `badValue`; write `SetSpeedL ($base * 2)`. Same for any positional expression.
- `$label = $i` on a string variable → `stringRequired`; write `$label = "{$i}"`.
- Trailing `# comment` on a statement is the step's comment and moves above it when formatted; on a
  `var` line it is the description.
- A grid by index (`grid Tray[$n]`) needs the grid's `colCount`; rows/cols are 0-based.
- `for $v in $pts` on a Point list gives `$v` = index, not a point; use `for _, $i in $pts` and `$pts[$i]`.
- `status error "…"` does not stop anything; use `raise` (catchable) or `exit`.
- `catch $msg` needs `$msg` to be a **string** variable (`paramOutputType` otherwise). Declare `var msg: string`.
- Routine variables re-initialise on every call and share the caller's scope: a routine variable named
  like a caller variable overwrites it.
- `Wait until $x` with no `timeout` waits forever; add `timeout=…, onTimeout=error` inside a `try` if it may never come true.
- `JumpL … z=` is the lift height (absolute Z of the lift and lower legs), not a relative lift. A
  Jump without `z=` (or `zStart=`) fails at run time ("JumpZ must be set") and the validator does
  not catch it — always give `z=`.
- A move with no target and no offsets does not move (a warning, not an error).
- Prompt `result=$choice` is the button **index from 0**, not 1.
- Background programs silently skip motion; a "monitor" that moves the robot will not.
- Don't reconstruct an existing program from memory and save it wholesale: read it, edit, write back
  (ids, `@with`, comments and disabled steps survive).
- `ok: false` from `ParseProgramText` means parse errors, not a transport failure; `ok: true` can
  still carry validation **errors** — check `problems`.
- `MoveJ`/`SetSpeedJ` speeds are joint speeds; `SetSpeedFactor` can only slow down (1–100 %).
- Text has no `timeout=` for `RunVision` (that design never shipped); use `@with {"waitTimeoutMs": …}`.

## VERIFY: how to check a program

1. **Parse + validate** (no motion):
   - WebSocket: `ParseProgramText {text, validate: true}` (add `base: <saved program>` when editing).
     Clean = `ok: true` and **no problem with `severity: "error"`**. Fix each problem at its
     `line:col`. Warnings are acceptable only if understood (e.g. `unknownPoint` for a point the user
     will teach — then the save needs `allow_untaught`).
   - MCP: `robot_program_text_set(name, text, dry_run=true)` → `parsed`, `errors`, `textProblems`,
     validation problems; nothing is saved.
   - JSON: `ValidateBuiltProgram {program}` / `robot_validate_program`, or `robot_save_program(…, dry_run=true)`.
2. **Canonical form**: `FormatProgramText {program: <parsed program>}` → compare with your text. If
   it differs only in layout, prefer the formatter's text (it is what the app shows).
3. **Round trip** before replacing a saved program: `robot_program_text_get` → edit → `robot_program_text_set`
   with `dry_run=true`; check `newProblems` is empty.
4. **Evaluate expressions** when unsure: `EvaluateExpression {expression, programName?}`.
5. **Scratch controller** for programs that reference things the real robot lacks, or when the robot
   is not yours to touch:
   - Build: `dotnet build RobotControl -c Debug -o <scratch>\ctrl` (never run from `RobotControl\bin`).
   - Start: `<scratch>\ctrl\SimpleRobotController.exe --port 9125 --data <scratch>\data` (cwd `<scratch>\ctrl`; remember the PID).
   - Seed (moves nothing): `TeachPoint {name}` then `EditPoint {name, x, y, z, rz}`; `CreateTool {name, x…rz}`;
     `CreateLocal {name, x…rz}`; `SaveGrid {id, name, basePointName, rowOffsetX/Y/Z, colOffsetX/Y/Z, rowCount, colCount, rotation}`;
     `SaveStack {…}`; `SaveVisionProgram {id, name, cameraId, layers, tools}`; routines/background
     programs via `ParseProgramText` (with an `@program` header) + `SaveBuiltProgram`. `frame=robot`
     additionally needs `<data>\cameraCalibrations\<cameraId>.json` present at start-up.
   - Send only parse/format/validate/save commands; **never** `ExecuteBuiltProgram`, `StartProgram`,
     `Home`, `MoveL`, outputs. Stop the process by PID when done.
6. **Unit test** (repo): `dotnet test RobotControl.Tests --filter AgentDocTests` parses every complete
   example in this file offline.

## EXAMPLES

The examples are written against this cell (seed it on a scratch controller to reproduce): points
`Home Infeed TrayOrigin CameraClear PickZ Drop`; tool `Gripper`; grid `Tray` (base `TrayOrigin`,
3 rows × 4 cols, 30 mm pitch); vision program `Find Parts` (camera `CAM_0`, calibrated) with a blob
tool `Parts`; routine `PlaceInSlot` (example 5a); background program `DoorMonitor` (example 6a).
STB output 1 = vacuum, output 2 = conveyor nudge, output 4 = amber light; inputs 1 = part present,
2 = door closed, 3 = vacuum sensor. Every block below parses with zero problems and is already in
the formatter's canonical form (checked by `AgentDocTests` and on a scratch controller).

### Example 1 — pick and place into a grid

Nested count loops over rows and columns; `grid Tray[$r, $c]` is 0-based.

```srs
#!srs 1
var rows = 3  # Tray rows
var cols = 4  # Tray columns
var placed = 0  # Parts placed this run

## Setup
SetTool Gripper
SetSpeedL 200, accel=800, decel=800
MoveJ Home

## Fill the tray, row by row
repeat $rows as $r
  repeat $cols as $c
    JumpL Infeed, z=60
    # vacuum on
    SetOutput stb 1 on
    Wait 300ms
    JumpL grid Tray[$r, $c], z=60, speed=150
    # release
    SetOutput stb 1 off
    Wait 200ms
    $placed = $placed + 1
  end
end
MoveJ Home
status "Placed {$placed} parts"
```

### Example 2 — vision-guided pick

`frame=robot` converts blob centres to robot mm (needs a calibrated camera). `set=` replaces X/Y
of the taught `PickZ` point (which supplies Z and orientation) with each blob's position.

```srs
#!srs 1
var ok: bool = off  # Vision passed
var parts: list record  # Blob centres, robot mm
var picked = 0

SetTool Gripper
MoveJ CameraClear
RunVision Find_Parts, frame=robot, pass=$ok
  Parts.items -> $parts
end
if not $ok or $parts.length == 0
  status warning "No parts found"
  exit
end
for _, $i in $parts
  JumpL PickZ, z=60, set=(x=$parts[$i].x, y=$parts[$i].y)
  SetOutput stb 1 on
  Wait 300ms
  JumpL Drop, z=60
  SetOutput stb 1 off
  $picked = $picked + 1
end
MoveJ CameraClear
status "Picked {$picked} of {$parts.length}"
```

### Example 3 — error handling with retry

Three attempts (1 + `retry=2`) to see a part, nudging the conveyor each time; then a catchable
`raise` with a code of its own ends the program unless an outer `try` handles `infeedEmpty`.

```srs
#!srs 1
var msg: string  # Error message from the catch
var code: string  # Error code from the catch
var late: bool = off

## Wait for a part, nudging the conveyor between tries
try retry=2, delay=500ms
  # nudge the conveyor
  SetOutput stb 2 on, pulse=400ms
  Wait until $stb.in1, timeout=3s, timedOut=$late, onTimeout=error
catch $msg, code=$code, only=(waitTimeout)
  status error "No part arrived ({$code}): {$msg}"
  raise "Infeed empty after 3 tries", code=infeedEmpty
end
JumpL Infeed, z=60
SetOutput stb 1 on
```

The same program as block JSON (what `ParseProgramText` returns, trimmed; `AgentDocTests` checks
that it formats to the text above). `JumpL` carries only `pointName` here, which resolves by name;
a saved program should also carry the robot's real `pointId` (from `GetPoints`) — `ParseProgramText`
fills it in for you.

```json
{
  "id": "prog-retry",
  "name": "Wait for part",
  "description": "",
  "isRoutine": false,
  "isBackground": false,
  "variables": [
    { "id": "v-msg", "name": "msg", "value": 0, "isString": true, "description": "Error message from the catch" },
    { "id": "v-code", "name": "code", "value": 0, "isString": true, "description": "Error code from the catch" },
    { "id": "v-late", "name": "late", "value": 0, "isBoolean": true }
  ],
  "steps": [
    { "id": "s1", "type": "Note", "noteStyle": "section", "comment": "Wait for a part, nudging the conveyor between tries" },
    { "id": "s2", "type": "Try", "tryRetries": 2, "tryRetryDelayMs": 500, "catchCodes": ["waitTimeout"],
      "errorVariableName": "msg", "errorCodeVariableName": "code",
      "trySteps": [
        { "id": "s3", "type": "SetOutput", "comment": "nudge the conveyor",
          "outputCard": "stb", "outputNumber": 2, "outputValue": true, "pulseMs": 400 },
        { "id": "s4", "type": "Wait", "waitMode": "condition",
          "waitCondition": { "combinator": "ALL", "items": [ { "id": "c1", "expr": "$stb.in1" } ] },
          "waitTimeoutMs": 3000, "waitTimeoutVariableName": "late", "waitTimeoutAction": "error" }
      ],
      "catchSteps": [
        { "id": "s5", "type": "StatusUpdate", "statusSeverity": "Error", "statusError": "No part arrived ({$code}): {$msg}" },
        { "id": "s6", "type": "RaiseError", "errorMessage": "Infeed empty after 3 tries", "errorCode": "infeedEmpty" }
      ] },
    { "id": "s7", "type": "JumpL", "pointName": "Infeed", "jumpZ": 60 },
    { "id": "s8", "type": "SetOutput", "outputCard": "stb", "outputNumber": 1, "outputValue": true }
  ]
}
```

### Example 4 — operator prompt

Button index from 0 drives a `switch`; a number input with a timeout that presses button 0.
`grid Tray[$i]` uses the row-major index (the grid has `colCount` 4).

```srs
#!srs 1
var choice = 0  # Button index, from 0
var qty = 12  # Parts to place

prompt "Load the tray, then choose", title="Setup", buttons=("Start", "Home first", "Cancel"), result=$choice
switch $choice
case 0
  status "Starting"
case 1
  MoveJ Home
default
  exit "Cancelled by operator"
end
prompt "How many parts?", input=number, default=12, min=1, max=12, value=$qty, timeout=60s, onTimeout=0
repeat $qty as $i
  JumpL Infeed, z=60
  SetOutput stb 1 on
  Wait 300ms
  JumpL grid Tray[$i], z=60
  SetOutput stb 1 off
end
status "Placed {$qty} parts"
```

### Example 5a — a routine with in and out parameters

Save it as a routine (the `@program` header does that on import; with MCP:
`robot_program_text_set("PlaceInSlot", text, is_routine=true)`).

```srs
#!srs 1
@program {"name":"PlaceInSlot","description":"","isRoutine":true,"isBackground":false,"killBackgroundOnStop":true}
param in slot = 0  # Tray cell index (row-major, from 0)
param in liftZ = 60  # Jump height
param out placed: bool = off  # The part left the gripper

JumpL grid Tray[$slot], z=$liftZ
SetOutput stb 1 off
Wait 200ms
$placed = $stb.in3 == 0
```

### Example 5b — calling the routine

Arguments are evaluated in the caller's scope; `out placed=$ok` copies the routine's `$placed`
into the caller's `$ok` when it returns. Parameter names differ from the caller's variables
(shared scope).

```srs
#!srs 1
var ok: bool = off  # Set by the routine's out parameter
var count = 0

repeat 12 as $i
  JumpL Infeed, z=60
  SetOutput stb 1 on
  Wait 300ms
  call PlaceInSlot(slot=$i, liftZ=50), out placed=$ok
  if not $ok
    raise "Part stuck in the gripper at slot {$i}", code=partStuck
  end
  $count = $count + 1
end
status "Placed {$count} parts"
```

### Example 6a — background IO monitor

A background program (no motion allowed) that mirrors the door switch into a global flag and a
light. Save with `is_background=true` (or the header).

```srs
#!srs 1
@program {"name":"DoorMonitor","description":"","isRoutine":false,"isBackground":true,"killBackgroundOnStop":true}
global var doorOpen: bool = off  # Shared with the main program

repeat forever
  $doorOpen = $stb.in2 == 0
  # amber light follows the door
  SetOutput stb 4 $doorOpen
  Wait 50ms
end
```

### Example 6b — main program using the monitor

```srs
#!srs 1
global var doorOpen: bool = off  # Written by DoorMonitor

StartBackground DoorMonitor
repeat 12 as $i
  if $doorOpen
    status warning "Door open - waiting"
    Wait until not $doorOpen
  end
  JumpL Infeed, z=60
  SetOutput stb 1 on
  Wait 300ms
  JumpL grid Tray[$i], z=60
  SetOutput stb 1 off
end
StopBackground DoorMonitor
```

---

## STEP INDEX (generated)

Every step type, its text keyword, catalog category, body kind and whether a background program runs it.

<!-- BEGIN GENERATED steps (AgentDocTests) -->
| Type | Text keyword | Category | Body | Runs in background |
|---|---|---|---|---|
| `MoveL` | `MoveL` | motion |  | **no — skipped** |
| `MoveJ` | `MoveJ` | motion |  | **no — skipped** |
| `JumpL` | `JumpL` | motion |  | **no — skipped** |
| `JumpJ` | `JumpJ` | motion |  | **no — skipped** |
| `MoveC` | `MoveC` | motion |  | **no — skipped** |
| `ThreadMove` | `ThreadMove` | motion |  | **no — skipped** |
| `RunHoming` | `RunHoming` | motion |  | **no — skipped** |
| `SetJointPosition` | `SetJointPosition` | motion |  | yes |
| `CncProgram` | `CncProgram` | motion |  | yes |
| `SavePosition` | `SavePosition` | motion |  | yes |
| `SetSpeedL` | `SetSpeedL` | motionSettings |  | **no — skipped** |
| `SetSpeedJ` | `SetSpeedJ` | motionSettings |  | **no — skipped** |
| `SetBlendRadius` | `SetBlendRadius` | motionSettings |  | yes |
| `SetSpeedFactor` | `SetSpeedFactor` | motionSettings |  | **no — skipped** |
| `SetTool` | `SetTool` | motionSettings |  | **no — skipped** |
| `SetLocal` | `SetLocal` | motionSettings |  | **no — skipped** |
| `ClearLocal` | `ClearLocal` | motionSettings |  | **no — skipped** |
| `IfCondition` | `if` | flow | branches | yes |
| `Switch` | `switch` | flow | cases | yes |
| `Loop` | `repeat` `for` `while` | flow | steps | yes |
| `LoopControl` | `break` `continue` | flow |  | yes |
| `Try` | `try` | flow | tryCatch | yes |
| `RaiseError` | `raise` | flow |  | yes |
| `CallRoutine` | `call` | flow |  | yes |
| `Return` | `return` | flow |  | yes |
| `EndProgram` | `exit` | flow |  | yes |
| `Label` | `label` | flow |  | yes |
| `GoToLabel` | `goto` | flow |  | yes |
| `PauseProgram` | `pause` | flow |  | yes |
| `Note` | `# …` / `## …` | flow |  | yes |
| `SetVariable` | `$var = …` | variables |  | yes |
| `ListOp` | `$list.append(…)` `$list[i] = …` … | variables |  | yes |
| `StopwatchControl` | `stopwatch` | variables |  | yes |
| `Wait` | `Wait` | variables |  | yes |
| `SetOutput` | `SetOutput` | io |  | yes |
| `Tare` | `Tare` | io |  | yes |
| `ReadLoadCell` | `ReadLoadCell` | io |  | yes |
| `StartRecording` | `StartRecording` | io |  | yes |
| `StopRecording` | `StopRecording` | io |  | yes |
| `ExportData` | `ExportData` | io | mapping | yes |
| `AuxMove` | `AuxMove` | io |  | yes |
| `AuxContinuous` | `AuxContinuous` | io |  | yes |
| `AuxStop` | `AuxStop` | io |  | yes |
| `AuxEnable` | `AuxEnable` | io |  | yes |
| `GripperAction` | `GripperAction` | io |  | yes |
| `ConveyorAction` | `ConveyorAction` | io |  | yes |
| `RotaryIndex` | `RotaryIndex` | io |  | yes |
| `RunVision` | `RunVision` | vision | mapping | yes |
| `CaptureImage` | `CaptureImage` | vision |  | yes |
| `SaveImage` | `SaveImage` | vision |  | yes |
| `StartBackground` | `StartBackground` | background |  | yes |
| `StopBackground` | `StopBackground` | background |  | yes |
| `WaitForBackground` | `WaitForBackground` | background |  | yes |
| `HttpRequest` | `HttpRequest` | comms | mapping | yes |
| `HttpReceive` | `HttpReceive` | comms | mapping | yes |
| `StatusUpdate` | `status` | comms |  | yes |
| `Prompt` | `prompt` | comms |  | yes |
| `Plugin` | `plugin` | plugins | mapping | yes |
<!-- END GENERATED steps -->

## STEP FIELDS (generated)

Per step: the text syntax and every field. **Text**: `pos n` = n-th positional value after the
keyword, `name=` = named value, `body`/`rows` = indented lines, `@with only` = no text form.
**JSON field(s)**: the ProgramStep properties it stores into (composites list several). **Kind**:
catalog kind (`number integer bool enum text template expr condition varWrite varRead listVar
pointRef toolRef localRef routineRef programRef visionRef deviceRef cameraRef labelRef pluginRef
target axes6 mappingOut rows exprMap textList steps branches cases json`), with the variable kinds
accepted in parentheses. **Default**: the JSON default when absent. **Allowed**: enum values (the
text word when it differs), range. **Expr key**: the `expressions` key that may replace the literal.
**Notes**: required, when the field applies (`showIf`), help.

<!-- BEGIN GENERATED fields (AgentDocTests) -->
### Fields: Motion

#### STEP MoveL — Move Linear

Move to a target in a straight line. Background: skipped. Body: none.  
Text: `MoveL <target>[, speed=…][, blend=…][, offset=(…)][, until=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `pointId` `pointName` `pointNameExpr` `gridPoint` `stackPoint` | target |  |  |  | `gridRowIndex` `gridColIndex` `gridGridIndex` `stackIndex` | A saved point, a $points[i] element, a template naming a point, a grid or stack cell — or nothing for the current position (use with offsets). |
| `speed=` | `speed` | number | mm/s |  | min 0 | `speed` | Overrides the program's speed for this move only. |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |
| `blend=` | `blend` | bool |  | false |  |  | Round the corner into the next move instead of stopping. |
| `blendRadius=` | `blendRadius` | number | mm |  | min 0 | `blendRadius` | when blend ∈ {true}; Empty = the radius set by Set Blend Radius. |
| `offset=` | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | axes6 | mm / deg |  |  | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | Added to the target (relative move when there is no target). |
| `set=` | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | axes6 | mm / deg |  |  | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | Replaces an axis of the target outright. |
| `toolOffset=` | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | axes6 | mm / deg |  |  | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | Applied in the tool frame at execution time. |
| `local=` | `localId` `localName` | localRef |  |  |  |  | Run this move in a local frame instead of the active one. |
| `until=` | `moveStopCondition` | condition |  |  |  |  | Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000. |
| `untilMode=` | `moveStopMode` | enum |  | "stop" | `stop` `record` |  |  |
| `untilPos=` | `moveStopPositionVariableName` | varWrite (list:Point) |  |  |  |  | A Points list set to the one pose where the condition fired. |
| `untilFlag=` | `moveStopFlagVariableName` | varWrite (boolean, number) |  |  |  |  | Set to 1 if the condition fired during the move, else 0. |

#### STEP MoveJ — Move Joint

Move to a target by joint interpolation. Background: skipped. Body: none.  
Text: `MoveJ <target>[, speed=…][, offset=(…)][, until=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `pointId` `pointName` `pointNameExpr` `gridPoint` `stackPoint` | target |  |  |  | `gridRowIndex` `gridColIndex` `gridGridIndex` `stackIndex` | A saved point, a $points[i] element, a template naming a point, a grid or stack cell — or nothing for the current position (use with offsets). |
| `speed=` | `speed` | number | mm/s |  | min 0 | `speed` | Overrides the program's speed for this move only. |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |
| `offset=` | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | axes6 | mm / deg |  |  | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | Added to the target (relative move when there is no target). |
| `set=` | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | axes6 | mm / deg |  |  | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | Replaces an axis of the target outright. |
| `toolOffset=` | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | axes6 | mm / deg |  |  | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | Applied in the tool frame at execution time. |
| `local=` | `localId` `localName` | localRef |  |  |  |  | Run this move in a local frame instead of the active one. |
| `until=` | `moveStopCondition` | condition |  |  |  |  | Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000. |
| `untilMode=` | `moveStopMode` | enum |  | "stop" | `stop` `record` |  |  |
| `untilPos=` | `moveStopPositionVariableName` | varWrite (list:Point) |  |  |  |  | A Points list set to the one pose where the condition fired. |
| `untilFlag=` | `moveStopFlagVariableName` | varWrite (boolean, number) |  |  |  |  | Set to 1 if the condition fired during the move, else 0. |

#### STEP JumpL — Jump Linear

Lift, move linearly over the target, then lower. Background: skipped. Body: none.  
Text: `JumpL <target>, z=…[, zStart=…][, zEnd=…][, speed=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `pointId` `pointName` `pointNameExpr` `gridPoint` `stackPoint` | target |  |  |  | `gridRowIndex` `gridColIndex` `gridGridIndex` `stackIndex` | A saved point, a $points[i] element, a template naming a point, a grid or stack cell — or nothing for the current position (use with offsets). |
| `z=` | `jumpZ` | number | mm |  |  | `jumpZ` | Z used for the lift and lower legs. |
| `zStart=` | `jumpZStart` | number | mm |  |  | `jumpZStart` | Overrides the lift leg only. |
| `zEnd=` | `jumpZEnd` | number | mm |  |  | `jumpZEnd` | Overrides the lower leg only. |
| `speed=` | `speed` | number | mm/s |  | min 0 | `speed` | Overrides the program's speed for this move only. |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |
| `blend=` | `blend` | bool |  | false |  |  | Round the corner into the next move instead of stopping. |
| `blendRadius=` | `blendRadius` | number | mm |  | min 0 | `blendRadius` | when blend ∈ {true}; Empty = the radius set by Set Blend Radius. |
| `offset=` | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | axes6 | mm / deg |  |  | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | Added to the target (relative move when there is no target). |
| `set=` | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | axes6 | mm / deg |  |  | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | Replaces an axis of the target outright. |
| `toolOffset=` | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | axes6 | mm / deg |  |  | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | Applied in the tool frame at execution time. |
| `local=` | `localId` `localName` | localRef |  |  |  |  | Run this move in a local frame instead of the active one. |
| `until=` | `moveStopCondition` | condition |  |  |  |  | Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000. |
| `untilMode=` | `moveStopMode` | enum |  | "stop" | `stop` `record` |  |  |
| `untilPos=` | `moveStopPositionVariableName` | varWrite (list:Point) |  |  |  |  | A Points list set to the one pose where the condition fired. |
| `untilFlag=` | `moveStopFlagVariableName` | varWrite (boolean, number) |  |  |  |  | Set to 1 if the condition fired during the move, else 0. |

#### STEP JumpJ — Jump Joint

Lift, move by joint interpolation over the target, then lower. Background: skipped. Body: none.  
Text: `JumpJ <target>, z=…[, zStart=…][, zEnd=…][, speed=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `pointId` `pointName` `pointNameExpr` `gridPoint` `stackPoint` | target |  |  |  | `gridRowIndex` `gridColIndex` `gridGridIndex` `stackIndex` | A saved point, a $points[i] element, a template naming a point, a grid or stack cell — or nothing for the current position (use with offsets). |
| `z=` | `jumpZ` | number | mm |  |  | `jumpZ` |  |
| `zStart=` | `jumpZStart` | number | mm |  |  | `jumpZStart` |  |
| `zEnd=` | `jumpZEnd` | number | mm |  |  | `jumpZEnd` |  |
| `speed=` | `speed` | number | mm/s |  | min 0 | `speed` | Overrides the program's speed for this move only. |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |
| `offset=` | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | axes6 | mm / deg |  |  | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | Added to the target (relative move when there is no target). |
| `set=` | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | axes6 | mm / deg |  |  | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | Replaces an axis of the target outright. |
| `toolOffset=` | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | axes6 | mm / deg |  |  | `toolOffsetX` `toolOffsetY` `toolOffsetZ` `toolOffsetRX` `toolOffsetRY` `toolOffsetRZ` | Applied in the tool frame at execution time. |
| `local=` | `localId` `localName` | localRef |  |  |  |  | Run this move in a local frame instead of the active one. |
| `until=` | `moveStopCondition` | condition |  |  |  |  | Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000. |
| `untilMode=` | `moveStopMode` | enum |  | "stop" | `stop` `record` |  |  |
| `untilPos=` | `moveStopPositionVariableName` | varWrite (list:Point) |  |  |  |  | A Points list set to the one pose where the condition fired. |
| `untilFlag=` | `moveStopFlagVariableName` | varWrite (boolean, number) |  |  |  |  | Set to 1 if the condition fired during the move, else 0. |

#### STEP MoveC — Move Arc

Move along a circular arc through a via point to the end target. Background: skipped. Body: none.  
Text: `MoveC <end>, via=<point>[, speed=…][, tolerance=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `pointId` `pointName` `pointNameExpr` `gridPoint` `stackPoint` | target |  |  |  | `gridRowIndex` `gridColIndex` `gridGridIndex` `stackIndex` | A saved point, a $points[i] element, a template naming a point, a grid or stack cell — or nothing for the current position (use with offsets). |
| `via=` | `viaPointId` `viaPointName` `viaPointNameExpr` | target |  |  |  |  | **required**; A point the arc passes through (saved point, $points[i] element or template). |
| `speed=` | `speed` | number | mm/s |  | min 0 | `speed` | Overrides the program's speed for this move only. |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |
| `tolerance=` | `arcTolerance` | number | mm | 0.1 | min 0.001 |  | Largest allowed gap between the true arc and its straight segments. |
| `offset=` | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | axes6 | mm / deg |  |  | `offsetX` `offsetY` `offsetZ` `offsetRX` `offsetRY` `offsetRZ` | Added to the target (relative move when there is no target). |
| `set=` | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | axes6 | mm / deg |  |  | `overrideX` `overrideY` `overrideZ` `overrideRX` `overrideRY` `overrideRZ` | Replaces an axis of the target outright. |
| `local=` | `localId` `localName` | localRef |  |  |  |  | Run this move in a local frame instead of the active one. |

#### STEP ThreadMove — Thread Move

Move the tool in Z while rotating RZ to follow a thread. Background: skipped. Body: none.  
Text: `ThreadMove distance=…, pitch=…[, peck=on, peckDepth=…, peckRetract=…][, exitHeight=…][, reverseOut=on]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `distance=` | `threadDistance` | number | mm |  |  | `threadDistance` | **required** |
| `pitch=` | `threadPitch` | number | mm |  | min 0 | `threadPitch` | **required** |
| `peck=` | `threadPeck` | bool |  | false |  |  |  |
| `peckDepth=` | `threadPeckDepth` | number | mm |  | min 0 | `threadPeckDepth` | when threadPeck ∈ {true} |
| `peckRetract=` | `threadPeckRetract` | number | mm |  | min 0 | `threadPeckRetract` | when threadPeck ∈ {true}; Empty = retract fully to the start each peck. |
| `exitHeight=` | `threadExitHeight` | number | mm |  |  | `threadExitHeight` |  |
| `reverseOut=` | `threadReverseOut` | bool |  | false |  |  |  |
| `speed=` | `speed` | number | mm/s |  | min 0 | `speed` | Overrides the program's speed for this move only. |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |

#### STEP RunHoming — Run Homing

Run the full homing sequence and wait for it to finish. Background: skipped. Body: none.  
Text: `RunHoming`

No fields.

#### STEP SetJointPosition — Set Joint Position

Declare one joint's value with no motion (manual homing). Background: runs. Body: none.  
Text: `SetJointPosition J<n>, <value>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `jointIndex` | integer |  | 0 | `0` `1` `2` `3` |  |  |
| pos 2 | `jointValue` | number | deg / mm |  |  | `jointValue` | **required** |

#### STEP CncProgram — CNC Program

Follow a toolpath generated from a DXF or SVG file. Background: runs. Body: none.  
Text: `CncProgram "<file>"[, safeZ=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `cncDxfFile` | text |  |  |  |  |  |
| `safeZ=` | `cncSafeZ` | number | mm |  |  |  |  |
| `@with` only | `cncSpec` | json |  |  |  |  | Edited in the CNC builder. |

#### STEP SavePosition — Save Position

Store the current position in a Points list or a saved point. Background: runs. Body: none.  
Text: `SavePosition $<list>[, mode=replace|append|insert|set][, index=…][, source=actual|target] | SavePosition point <name>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `to=` | `savePositionTo` | enum |  | "variable" | `variable` `point` |  |  |
| pos 1 | `variableName` | listVar (list:Point) |  |  |  |  | when savePositionTo ∈ {variable} |
| `point=` | `pointId` `pointName` | pointRef |  |  |  |  | when savePositionTo ∈ {point}; Overwritten (or created) with the current position; recorded in the points history. |
| `mode=` | `savePositionMode` | enum |  | "replace" | `replace` `append` `insert` `set` |  | when savePositionTo ∈ {variable} |
| `index=` | `savePositionIndex` | integer |  |  | min 0 | `savePositionIndex` | when savePositionMode ∈ {insert, set} |
| `source=` | `savePositionSource` | enum |  | "actual" | `actual` `target` |  |  |

### Fields: Motion Settings

#### STEP SetSpeedL — Set Speed (Linear)

Set the linear move speed, accel and decel. Background: skipped. Body: none.  
Text: `SetSpeedL <speed>[, accel=…][, decel=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `speed` | number | mm/s |  | min 0 | `speed` | **required** |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |

#### STEP SetSpeedJ — Set Speed (Joint)

Set the joint move speed, accel and decel. Background: skipped. Body: none.  
Text: `SetSpeedJ <speed>[, accel=…][, decel=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `speed` | number | mm/s |  | min 0 | `speed` | **required** |
| `accel=` | `accel` | number | mm/s² |  | min 0 | `accel` |  |
| `decel=` | `decel` | number | mm/s² |  | min 0 | `decel` |  |

#### STEP SetBlendRadius — Set Blend Radius

Set the default corner radius for later blended moves. Background: runs. Body: none.  
Text: `SetBlendRadius <mm>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `blendRadius` | number | mm |  | min 0 | `blendRadius` | **required** |

#### STEP SetSpeedFactor — Set Speed Factor

Scale this program's move speeds by a percentage. Background: skipped. Body: none.  
Text: `SetSpeedFactor <percent>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `speedFactor` | number | % | 100 | min 1; max 100 | `speedFactor` | **required**; Never exceeds the operator's speed override; resets to 100 when the program ends. |

#### STEP SetTool — Set Tool

Change the active tool used by later moves. Background: skipped. Body: none.  
Text: `SetTool <tool>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `toolId` `toolName` | toolRef |  |  |  |  | None = no tool offset. |

#### STEP SetLocal — Set Local

Activate a local coordinate frame for later moves. Background: skipped. Body: none.  
Text: `SetLocal <local>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `localId` `localName` | localRef |  |  |  |  | **required** |

#### STEP ClearLocal — Clear Local

Return later moves to the world frame. Background: skipped. Body: none.  
Text: `ClearLocal`

No fields.

### Fields: Logic & Flow

#### STEP IfCondition — If Condition

Run steps only when a condition holds, with else-if and else branches. Background: runs. Body: branches.  
Text: `if <condition> … [elif <condition> …] [else …] end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `condition` | condition |  |  |  |  | **required** |
| body | `ifSteps` | steps |  |  |  |  |  |
| `elif` blocks | `elseIfBranches` | branches |  |  |  |  |  |
| body | `elseSteps` | steps |  |  |  |  |  |

#### STEP Switch — Switch

Run the case whose value equals an expression. Background: runs. Body: cases.  
Text: `switch <expr> case <v>[, <v>…] … [default …] end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `switchExpr` | expr |  |  |  |  | **required**; Evaluated once; compared with each case value. |
| `case` blocks | `switchCases` | cases |  |  |  |  |  |
| body | `switchDefault` | steps |  |  |  |  |  |

#### STEP Loop — Loop

Repeat steps a number of times, for each list element, or while a condition holds. Background: runs. Body: steps.  
Text: `repeat <n>|forever [as $i] … end | for $v[, $i] in $list … end | while <condition> … end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `mode=` | `loopMode` | enum |  | "count" | `count` `forEach` `while` |  |  |
| pos 1 | `loopCount` | integer |  | 1 | min 0 | `loopCount` | when loopMode ∈ {count}; 0 = forever. |
| `in=` | `forEachVariableName` | listVar (list) |  |  |  |  | when loopMode ∈ {forEach} |
| `value=` | `forEachValueVariableName` | varWrite (number, boolean) |  |  |  |  | when loopMode ∈ {forEach}; Number/Boolean lists: the element. Other lists: the index. |
| `as=` | `forEachIndexVariableName` | varWrite (number) |  |  |  |  | when loopMode ∈ {count, forEach} |
| `while=` | `loopWhileCondition` | condition |  |  |  |  | when loopMode ∈ {while} |
| body | `loopSteps` | steps |  |  |  |  |  |

#### STEP LoopControl — Break / Next

Leave the enclosing loop, or skip to its next iteration. Background: runs. Body: none.  
Text: `break | continue`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `loopControl` | enum |  | "break" | `break` `continue` |  |  |

#### STEP Try — Try

Run steps; on a runtime error retry them or run the catch steps. Background: runs. Body: tryCatch.  
Text: `try [retry=…][, delay=…] … [catch [$msg][, code=$code][, only=(…)] …] end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `retry=` | `tryRetries` | integer |  | 0 | min 0 |  |  |
| `delay=` | `tryRetryDelayMs` | integer | ms | 0 | min 0 |  |  |
| `only=` | `catchCodes` | textList |  |  | catchable codes or your own raise codes |  | Empty = every catchable error. A Raise Error code may be typed in. |
| `message=` | `errorVariableName` | varWrite (string) |  |  |  |  |  |
| `code=` | `errorCodeVariableName` | varWrite (string) |  |  |  |  |  |
| body | `trySteps` | steps |  |  |  |  |  |
| body | `catchSteps` | steps |  |  |  |  |  |

#### STEP RaiseError — Raise Error

Fail with a message — caught by an enclosing Try, otherwise ends the program. Background: runs. Body: none.  
Text: `raise "<message>"[, code=<identifier>]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `errorMessage` | template |  |  |  |  | **required** |
| `code=` | `errorCode` | text |  | "userError" |  |  | An identifier a Try can filter on. |

#### STEP CallRoutine — Call Routine

Run a routine, then continue. Background: runs. Body: none.  
Text: `call <routine>[(param=…, …)][, out param=$var]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `routineId` `routineName` | routineRef |  |  |  |  | **required** |
| rows / sugar | `routineArgs` | exprMap |  |  |  |  | One expression per in / in-out parameter (a template for a text parameter), evaluated before the call. |
| `out=` | `routineOutputs` | mappingOut |  |  |  |  | Out / in-out parameters copied into your variables when the routine returns. |

#### STEP Return — Return

Leave the routine (or end the program). Background: runs. Body: none.  
Text: `return`

No fields.

#### STEP EndProgram — End Program

Finish the program now, from anywhere. Background: runs. Body: none.  
Text: `exit ["<message>"]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `statusMessage` | template |  |  |  |  |  |

#### STEP Label — Label

Mark a place Go To Label can jump to. Background: runs. Body: none.  
Text: `label <name>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `labelName` | text |  |  |  |  | **required** |
| `@with` only | `labelId` | text |  |  |  |  |  |

#### STEP GoToLabel — Go To Label

Jump to a Label. Background: runs. Body: none.  
Text: `goto <label>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `labelId` `labelName` | labelRef |  |  |  |  | **required** |

#### STEP PauseProgram — Pause Program

Pause — the operator continues or stops from the monitor. Background: runs. Body: none.  
Text: `pause`

No fields.

#### STEP Note — Note

A note or section header — not executed. Background: runs. Body: none.  
Text: `# text | ## Section`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| sugar | `noteStyle` | enum |  | "note" | `note` `section` |  |  |
| pos 1 | `comment` | text |  |  |  |  |  |

### Fields: Variables & Timing

#### STEP SetVariable — Set Variable

Assign a value or expression to a variable. Background: runs. Body: none.  
Text: `$<name> = <expression | "template">`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `variableName` | varWrite (number, boolean, string) |  |  |  |  | **required** |
| pos 2 | `variableExpr` | expr |  |  |  |  | **required**; An expression; a template for a text variable. |

#### STEP ListOp — List

Append, insert, remove, set, sort or copy list elements. Background: runs. Body: none.  
Text: `$<list>.append(…) | .insert(i, …) | .removeAt(i) | .clear() | [i] = … | [i].field = … | .sort(by=…, desc) | .reverse() | .copyFrom($other)`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `variableName` | listVar (list) |  |  |  |  | **required** |
| `action=` | `listAction` | enum |  | "append" | `append` `insert` `removeAt` `clear` `set` `setField` `sort` `reverse` `copyFrom` |  | **required** |
| `index=` | `listIndex` | integer |  |  | min 0 | `listIndex` | when listAction ∈ {insert, removeAt, set, setField} |
| `value=` | `listValueExpr` | expr |  |  |  |  | when listAction ∈ {append, insert, set}; For Number and Boolean lists. |
| `fields=` | `listFields` | exprMap |  |  |  |  | when listAction ∈ {append, insert, set, setField}; For Point and Record lists. Append with no fields on a Points list appends the current position. |
| `from=` | `listSourceVariable` | listVar (list) |  |  |  |  | when listAction ∈ {copyFrom} |
| `by=` | `listSortField` | text |  |  |  |  | when listAction ∈ {sort}; Empty = the element value. |
| `desc=` | `listSortDescending` | bool |  | false |  |  | when listAction ∈ {sort} |

#### STEP StopwatchControl — Stopwatch

Start, stop or reset a stopwatch variable. Background: runs. Body: none.  
Text: `stopwatch start|stop|reset $<name>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `stopwatchAction` | enum |  | "Start" | `Start`(text `start`) `Stop`(text `stop`) `Reset`(text `reset`) |  | **required** |
| pos 2 | `stopwatchVariableName` | varWrite (stopwatch) |  |  |  |  | **required** |

#### STEP Wait — Wait

Wait for a time, or until a condition holds. Background: runs. Body: none.  
Text: `Wait <ms> | Wait until <condition>[, timeout=…][, timedOut=$v][, onTimeout=continue|error]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `mode=` | `waitMode` | enum |  | "duration" | `duration` `condition` |  |  |
| pos 1 | `waitMs` | integer | ms |  | min 0 | `waitMs` | when waitMode ∈ {duration} |
| `until=` | `waitCondition` | condition |  |  |  |  | when waitMode ∈ {condition} |
| `timeout=` | `waitTimeoutMs` | integer | ms |  | min 0 |  | when waitMode ∈ {condition}; Empty or 0 = wait forever. |
| `timedOut=` | `waitTimeoutVariableName` | varWrite (boolean, number) |  |  |  |  | when waitMode ∈ {condition} |
| `onTimeout=` | `waitTimeoutAction` | enum |  | "continue" | `continue` `error` |  | when waitMode ∈ {condition} |

### Fields: I/O & Aux Axes

#### STEP SetOutput — Set Output

Turn a digital output on or off, toggle it or pulse it. Background: runs. Body: none.  
Text: `SetOutput <stb|relay|io> [<device>] <number> <on|off|toggle|(expr)>[, pulse=…][, blocking=on]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `outputCard` | enum |  | "stb" | `stb` `relay` `io` |  | **required** |
| pos 2 | `outputDeviceId` | deviceRef |  |  |  |  | when outputCard ∈ {io} |
| pos 3 | `outputNumber` | integer |  |  | min 0 |  | **required** |
| `action=` | `outputAction` | enum |  | "set" | `set` `toggle` |  |  |
| pos 4 | `outputValue` | bool |  | true |  | `outputValue` | when outputAction ∈ {set}; An expression: non-zero = on. |
| `pulse=` | `pulseMs` | integer | ms |  | min 0 |  | Set for this long, then flip back. Empty = hold. |
| `blocking=` | `pulseBlocking` | bool |  | false |  |  |  |

#### STEP Tare — Tare Load Cell

Zero a load cell. Background: runs. Body: none.  
Text: `Tare device=…, pin=…`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `device=` | `loadCellDeviceId` | deviceRef |  |  |  |  |  |
| `pin=` | `loadCellPin` | integer |  |  | min 0 |  |  |

#### STEP ReadLoadCell — Read Load Cell

Read a load cell into a variable. Background: runs. Body: none.  
Text: `ReadLoadCell $<var>, device=…, pin=…[, unit=g]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `variableName` | varWrite (number) |  |  |  |  | **required** |
| `device=` | `loadCellDeviceId` | deviceRef |  |  |  |  |  |
| `pin=` | `loadCellPin` | integer |  |  | min 0 |  |  |
| `unit=` | `loadCellUnit` | enum |  | "g" | `g` `kg` `N` `lb` `oz` |  |  |

#### STEP StartRecording — Start Recording

Record a load cell into a list variable. Background: runs. Body: none.  
Text: `StartRecording $<list>, device=…, pin=…`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `recordVariableName` | listVar (list:Record) |  |  |  |  | **required** |
| `device=` | `loadCellDeviceId` | deviceRef |  |  |  |  |  |
| `pin=` | `loadCellPin` | integer |  |  | min 0 |  |  |

#### STEP StopRecording — Stop Recording

Stop a recording and store its samples. Background: runs. Body: none.  
Text: `StopRecording device=…, pin=…`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `device=` | `loadCellDeviceId` | deviceRef |  |  |  |  |  |
| `pin=` | `loadCellPin` | integer |  |  | min 0 |  |  |

#### STEP ExportData — Export Data

Write variables to a CSV or JSON file. Background: runs. Body: mapping.  
Text: `ExportData csv|json "<path>" … end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `exportFormat` | enum |  | "csv" | `csv` `json` |  |  |
| pos 2 | `exportPath` | template |  |  |  |  | **required** |
| rows / sugar | `exportColumns` | rows |  |  |  |  |  |

#### STEP AuxMove — Aux Move

Move an aux axis by a distance or a number of steps. Background: runs. Body: none.  
Text: `AuxMove device=…, axis=…, distance=…|steps=…[, unit=mm|deg][, velocity=…][, absolute=on][, wait=off][, until=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |
| `unit=` | `auxUnit` | enum |  |  | `mm` `deg` |  | Empty = raw steps. |
| `distance=` | `auxDistance` | number |  |  |  | `auxDistance` | In the unit above. |
| `steps=` | `auxSteps` | integer |  |  |  | `auxSteps` | Used when no unit is set; the sign is the direction. |
| `velocity=` | `auxVelocity` | number |  |  | min 0 | `auxVelocity` |  |
| `accel=` | `auxAccel` | number |  |  | min 0 | `auxAccel` |  |
| `decel=` | `auxDecel` | number |  |  | min 0 | `auxDecel` |  |
| `absolute=` | `auxAbsolute` | bool |  | false |  |  |  |
| `wait=` | `auxWaitForDone` | bool |  | true |  |  |  |
| `until=` | `moveStopCondition` | condition |  |  |  |  | Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000. |
| `untilMode=` | `moveStopMode` | enum |  | "stop" | `stop` `record` |  |  |
| `untilPos=` | `moveStopPositionVariableName` | varWrite (number) |  |  |  |  | Receives the axis position (mm / deg, or steps). |
| `untilFlag=` | `moveStopFlagVariableName` | varWrite (boolean, number) |  |  |  |  | Set to 1 if the condition fired during the move, else 0. |

#### STEP AuxContinuous — Aux Continuous Run

Run an aux axis continuously until an Aux Stop. Background: runs. Body: none.  
Text: `AuxContinuous device=…, axis=…, velocity=…[, accel=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |
| `unit=` | `auxUnit` | enum |  |  | `mm` `deg` |  |  |
| `velocity=` | `auxVelocity` | number |  |  |  | `auxVelocity` | The sign is the direction. |
| `accel=` | `auxAccel` | number |  |  | min 0 | `auxAccel` |  |

#### STEP AuxStop — Aux Stop

Stop aux axis motion — ramped or immediate. Background: runs. Body: none.  
Text: `AuxStop device=…, axis=…[, decel=…][, immediate=on]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |
| `decel=` | `auxDecel` | number |  |  | min 0 | `auxDecel` |  |
| `immediate=` | `auxImmediate` | bool |  | false |  |  |  |

#### STEP AuxEnable — Aux Motor Enable

Enable or disable the motor drivers of an aux card. Background: runs. Body: none.  
Text: `AuxEnable device=…, axis=…, on|off`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |
| pos 1 | `auxEnable` | bool |  | true |  |  |  |

#### STEP GripperAction — Gripper

Open, close or grip to a jaw gap. Background: runs. Body: none.  
Text: `GripperAction open|close|to <mm>[, device=…, axis=…][, velocity=…][, until=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `gripperAction` | enum |  | "Open" | `Open`(text `open`) `Close`(text `close`) `GripTo`(text `to`) |  | **required** |
| `to=` | `gripTargetMm` | number | mm |  | min 0 | `gripTargetMm` | when gripperAction ∈ {GripTo} |
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |
| `velocity=` | `auxVelocity` | number | mm/s |  | min 0 | `auxVelocity` |  |
| `accel=` | `auxAccel` | number | mm/s² |  | min 0 | `auxAccel` |  |
| `until=` | `moveStopCondition` | condition |  |  |  |  | Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000. |
| `untilMode=` | `moveStopMode` | enum |  | "stop" | `stop` `record` |  |  |
| `untilPos=` | `moveStopPositionVariableName` | varWrite (number) |  |  |  |  | Receives the axis position (mm / deg, or steps). |
| `untilFlag=` | `moveStopFlagVariableName` | varWrite (boolean, number) |  |  |  |  | Set to 1 if the condition fired during the move, else 0. |

#### STEP ConveyorAction — Conveyor

Run a conveyor forward or reverse, or stop it. Background: runs. Body: none.  
Text: `ConveyorAction forward|reverse|stop[, device=…, axis=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `conveyorAction` | enum |  | "RunForward" | `RunForward`(text `forward`) `RunReverse`(text `reverse`) `Stop`(text `stop`) |  | **required** |
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |

#### STEP RotaryIndex — Rotary Index

Index a rotary axis a number of positions. Background: runs. Body: none.  
Text: `RotaryIndex +1|-1[, count=…][, device=…, axis=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `rotaryDirection` | integer |  | 1 | `1` `-1` |  |  |
| `count=` | `rotaryCount` | integer |  | 1 | min 1 |  |  |
| `device=` | `auxDeviceId` | deviceRef |  |  |  |  | The aux IO card. |
| `axis=` | `auxAxisIndex` | integer |  | 0 | min 0 |  |  |

### Fields: Vision

#### STEP RunVision — Run Vision

Run a vision program and write its results into variables. Background: runs. Body: mapping.  
Text: `RunVision <program>[, frame=pixel|normalized|robot][, pass=$v] … end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `visionProgramId` `visionProgramName` | visionRef |  |  |  |  | **required** |
| `frame=` | `outputFrame` | enum |  | "pixel" | `pixel` `normalized` `robot` |  |  |
| `pass=` | `passVariable` | varWrite (boolean, number) |  |  |  |  |  |
| rows / sugar | `toolOutputs` | mappingOut |  |  |  |  | Rows of tool · value → variable. |

#### STEP CaptureImage — Capture Image

Store the current camera frame in an image variable. Background: runs. Body: none.  
Text: `CaptureImage $<image>[, camera=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `captureImageVariableName` | varWrite (image) |  |  |  |  | **required** |
| `camera=` | `captureImageCameraId` | cameraRef |  |  |  |  |  |

#### STEP SaveImage — Save Image

Save a camera snapshot to a file. Background: runs. Body: none.  
Text: `SaveImage "<path>"[, camera=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `saveImagePath` | template |  |  |  |  | **required** |
| `camera=` | `saveImageCameraId` | cameraRef |  |  |  |  |  |

### Fields: Background

#### STEP StartBackground — Start Background

Start a background program in parallel. Background: runs. Body: none.  
Text: `StartBackground <program>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `backgroundProgramId` `backgroundProgramName` | programRef |  |  |  |  | **required** |

#### STEP StopBackground — Stop Background

Stop a background program. Background: runs. Body: none.  
Text: `StopBackground <program>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `backgroundProgramId` `backgroundProgramName` | programRef |  |  |  |  | **required** |

#### STEP WaitForBackground — Wait for Background

Wait until a background program finishes. Background: runs. Body: none.  
Text: `WaitForBackground <program>`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `backgroundProgramId` `backgroundProgramName` | programRef |  |  |  |  | **required** |

### Fields: Communication

#### STEP HttpRequest — HTTP Request

POST JSON to a server and read values from the reply. Background: runs. Body: mapping.  
Text: `HttpRequest "<url>"[, wait=on][, timeout=…] send … receive … end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `jsonUrl` | text |  |  |  |  | **required** |
| `wait=` | `jsonWaitForResponse` | bool |  | true |  |  |  |
| `timeout=` | `jsonTimeoutMs` | integer | ms |  | min 0 |  |  |
| `send=` | `jsonOutbound` | rows |  |  |  |  |  |
| `receive=` | `jsonInbound` | mappingOut |  |  |  |  |  |

#### STEP HttpReceive — HTTP Receive

Wait for an HTTP POST to a named webhook. Background: runs. Body: mapping.  
Text: `HttpReceive "<name>"[, timeout=…] receive … end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `httpReceiveName` | text |  |  |  |  | **required**; Other machines POST to /webhook/<name>. |
| `timeout=` | `httpReceiveTimeoutMs` | integer | ms |  | min 0 |  |  |
| `receive=` | `httpReceiveInbound` | mappingOut |  |  |  |  |  |

#### STEP StatusUpdate — Status Update

Publish a message, warning or error to the monitor. Background: runs. Body: none.  
Text: `status [info|warning|error] "<message>"`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| sugar | `statusSeverity` | enum |  | "Info" | `Info` `Warning` `Error` |  |  |
| pos 1 | `statusMessage` | template |  |  |  |  |  |
| `warning=` | `statusWarning` | template |  |  |  |  |  |
| `error=` | `statusError` | template |  |  |  |  |  |

#### STEP Prompt — Prompt

Ask the operator: show a message with buttons and an optional input. Background: runs. Body: none.  
Text: `prompt "<message>"[, title=…][, buttons=(…)][, input=number|text][, result=$v][, value=$v][, timeout=…]`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `promptMessage` | template |  |  |  |  | **required** |
| `title=` | `promptTitle` | template |  |  |  |  |  |
| `buttons=` | `promptButtons` | textList |  | ["OK"] | min 1; max 4 |  |  |
| `result=` | `promptResultVariableName` | varWrite (number, string) |  |  |  |  | A number variable gets the button index (from 0); a text variable gets its label. |
| `input=` | `promptInput` | enum |  | "none" | `none` `number` `text` |  |  |
| `default=` | `promptDefault` | expr |  |  |  |  | when promptInput ∈ {number, text} |
| `min=` | `promptMin` | number |  |  |  |  | when promptInput ∈ {number} |
| `max=` | `promptMax` | number |  |  |  |  | when promptInput ∈ {number} |
| `value=` | `promptInputVariableName` | varWrite (number, string) |  |  |  |  | when promptInput ∈ {number, text} |
| `timeout=` | `promptTimeoutMs` | integer | ms | 0 | min 0 |  | 0 = wait forever. |
| `onTimeout=` | `promptTimeoutButton` | integer |  |  | min 0; max 3 |  | Button index chosen on timeout. Empty = fail (catchable). |

### Fields: Plugins

#### STEP Plugin — Plugin

A step provided by an installed plugin. Background: runs. Body: mapping.  
Text: `plugin <id>.<step>[, timeout=…] <param> = … <output> -> $v end`

| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |
|---|---|---|---|---|---|---|---|
| pos 1 | `pluginId` `pluginStepId` | pluginRef |  |  |  |  | **required** |
| rows / sugar | `pluginParams` | exprMap |  |  |  |  | Typed by the plugin manifest. |
| rows / sugar | `pluginOutputs` | mappingOut |  |  |  |  |  |
| `timeout=` | `pluginTimeoutMs` | integer | ms |  | min 0 |  | Empty = the manifest default; 0 = none. |
<!-- END GENERATED fields -->
