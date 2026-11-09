# Simple Robot Script (.srs) — the text view of a program

Every built program can be read and written as text. **Simple Robot Script** (SRS, file
extension `.srs`) is a second view of exactly the same program the Blocks editor shows: one
statement per step, block bodies closed with `end`, the same `$variables` and expressions.
Switching between Blocks and Text never loses anything — not a field, not a step id.

```srs
#!srs 1
var count = 0              # Parts placed this run
var parts: list point

## Pick and place
SetTool Gripper
for _, $i in $parts
  JumpL $parts[$i], z=40
  GripperAction close
  JumpL grid Tray[$count], z=40
  GripperAction open
  $count = $count + 1
end
status "Placed {$count} parts"
```

- The text is never stored: the program is still the JSON the controller runs. The controller
  formats it (`FormatProgramText`) and parses it back (`ParseProgramText`), resolving point,
  tool, routine and device **names** to their stable **ids**.
- Statements use the block names (`MoveL`, `SetOutput`, `RunVision` …), so what you learn in
  Blocks carries over; flow and assignment use the familiar lowercase words (`if`, `repeat`,
  `$x = …`). Keywords are case-insensitive; the formatter writes the canonical spelling.
- The formatter is canonical (like gofmt): it re-indents with two spaces and orders things one
  way, so `Format(Parse(text))` is stable.

Contents: [Rules](#rules) · [Program layout](#program-layout) · [Declarations](#declarations) ·
[Comments and attributes](#comments-notes-and-attributes) · [Statements](#statements) ·
[Names and ids](#names-and-ids) · [@with and the round trip](#with-and-the-round-trip) ·
[Problems](#problems) · [Commands and tools](#commands-and-tools) · [Grammar](#grammar) ·
[Argument reference](#argument-reference)

---

## Rules

| Element | Rule |
|---|---|
| Line | One statement per line. A line continues while a `(`, `[` or `{` is open, or after a trailing `,`. A blank line ends a continuation (an unclosed bracket is reported there). |
| Comment | `#` to the end of the line, outside strings. |
| Version | The first line `#!srs 1`. The formatter writes it; the parser accepts text without it. |
| Strings | `"…"` with the escapes `\"` `\\` `\n` `\t` `\r` (any other backslash is kept as written, so `"C:\data"` works). Template braces `{$x}` are kept as written and filled in at run time. |
| Numbers | `3`, `-2.5`, `1e3`. Fields in milliseconds also take `500ms` and `1.5s`. |
| Booleans | `on` `off` `true` `false` — the same 1 / 0 everywhere, inside expressions too (`$done = on`). |
| Names | A point, tool, routine, device … whose name is a plain identifier is written bare (`Pick1`); anything else in quotes (`"Pick 1"`) or in its underscore form (`Pick_1`) — see [Names and ids](#names-and-ids). |
| Variables | `$name` — in expressions and wherever a variable is written to (`result=$v`, `for $v in …`). |
| Positional values | Right after the keyword, separated by spaces: a name, a string, a number, `on`/`off`, a `$var` (with `[…]` / `.field`), or an expression **in parentheses**: `SetSpeedL ($base * 2)`. |
| Named values | `name=value`, comma-separated: `MoveL Pick1, speed=200, blend=5`. A named value runs to the next top-level comma, so it can be any expression: `speed=$base * 2`. |
| Literal or expression | A plain number is stored as the number; anything else as an expression that overrides it (the fx toggle in Blocks). Parentheses always mean an expression: `speed=(200)` stores the *expression* `200`. |

## Program layout

```srs
#!srs 1
@program {"name": "Pick and place", "isRoutine": false}   # only in exported files
var count = 0                                             # declarations first
## Setup                                                  # then the steps
SetTool Gripper
```

`@program {…}` carries the program's name, kind and display page in an exported `.srs` file.
It is read only when importing (parsing without a base program); otherwise the program's
metadata stays as it is and is edited in the editor header.

## Declarations

```srs
var count = 0                     # Parts placed      ← a trailing comment is the description
var ok: bool = off
var label: string = "Bin {$i}"
var startZ = $robot.z             # an expression → computed once at program start
var pts: list point = ((x=1, y=2, z=3), (x=4, rz=90))   # zero axes are left out
var vals: list number = (1, 2.5, 3)
var flags: list bool = (on, off)
var cells: list record = ((row=0, col=1), (row=1, col=0))
var photo: image
var lap: stopwatch
computed area = $w * $h           # a formula, evaluated every time it is read
computed ready: bool = $area > 100 and $stb.in1
global persistent input monitor var runs = 0   # modifiers, in this order
param in height = 20              # routine parameters: in | out | inout
param out found: bool = off
```

| Word | Field |
|---|---|
| `global` | shared with every running program (`isGlobal`) |
| `persistent` | kept between runs (`isPersistent`) |
| `input` | a display page may set it at start (`isInput`) |
| `monitor` | shown on the monitor (`displayOnMonitor`) |
| `param in` / `param out` / `param inout` | routine parameter (`paramMode`) |
| `computed` | formula variable; cannot also be persistent, a string, image, stopwatch or list |

Types: `number` (the default), `bool`, `string`, `image`, `stopwatch`, `list number | bool |
point | record`. A value that is not a plain literal (`var startZ = $robot.z`) becomes the
variable's start expression.

## Comments, notes and attributes

```srs
## Pick                      ← a section header (Note step, style "section")

# Check the tray first.      ← a comment paragraph followed by a blank line,
# Then pick.                   `end` or the end of the file: a Note step

# Approach slowly            ← comment lines directly above a statement: its comment
@name "Approach"             ← the step's name
@off                         ← the step is disabled (skipped when the program runs)
MoveL Pick1, speed=50
```

| Attribute | Meaning |
|---|---|
| `@name "…"` | the step's name |
| `@off` | disabled (`enabled: false`) |
| `@id "…"` | the step's id — written only when ids are asked for; it always wins |
| `@with {…}` | fields that have no words in the text, as JSON — see [below](#with-and-the-round-trip) |

Attributes apply to the next statement (or the next comment paragraph, for a Note). A comment at
the end of a statement line is added to that step's comment.

## Statements

### Motion

```srs
MoveL Pick1                                   # a saved point (by name)
MoveJ "Pick 1", speed=50, accel=100, decel=100
MoveL here, offset=(z=20)                     # no target: relative to where the robot is
MoveL $pts[$i]                                # an element of a Points list
MoveL "bin{$i}"                               # a template that names a saved point
MoveL $target                                 # a variable holding a point name
JumpJ grid Tray[$r, $c], z=40                 # a grid cell (row, col) …
JumpL grid Tray[$n], z=40                     # … or by index
MoveL stack Bins[$i]                          # a stack slot
MoveL Pick1, blend=5                          # blend=on | blend=5 | blend=$r | blend=off
MoveL Pick1, offset=(x=1, z=$h), set=(z=30), toolOffset=(z=-5), local=Fixture1
MoveL Pick1, until=$io.Nano.Load >= 2000, untilMode=record, untilPos=$hitPts, untilFlag=$hit
JumpL Place1, z=50, zStart=40, zEnd=60
MoveC Place1, via=Pick1, speed=50, tolerance=0.1
ThreadMove distance=10, pitch=0.5, peck=on, peckDepth=2
RunHoming
SetJointPosition J3, 90
CncProgram "part.dxf", safeZ=5                # the toolpath itself travels in @with
SavePosition $pts, mode=append                # replace | append | insert | set
SavePosition point Pick1                      # overwrite (or create) a saved point
```

`offset=` adds to the target, `set=` replaces an axis outright, `toolOffset=` applies in the
tool frame. Axes are `x y z rx ry rz`; each value may be an expression.

### Motion settings

```srs
SetSpeedL 200, accel=500, decel=500
SetSpeedJ 50
SetBlendRadius 5
SetSpeedFactor 50
SetTool Gripper
SetLocal Fixture1
ClearLocal
```

### I/O, load cells, aux axes

```srs
SetOutput stb 1 on                            # card [device] number value
SetOutput relay 3 off, pulse=250ms, blocking=on
SetOutput io "Arduino Nano" 4 on
SetOutput stb 2 toggle
SetOutput stb 2 ($count > 3)                  # non-zero = on
Wait 500ms
Wait $delay
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
GripperAction close, until=$io.Nano.Grip >= 300     # open | close | to 25
ConveyorAction forward                              # forward | reverse | stop
RotaryIndex +1, count=3
```

### Flow

```srs
if $count > 5 and $ok
  …
elif $mode == 2              # "else if" works too
  …
else
  …
end

repeat 3 as $i               # count loop; "repeat forever" = run until stopped or a break of its own
  …
end
while $stb.in1 == 0
  …
end
for $v, $i in $vals          # for $v in $list · for _, $i in $pts
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

break                        # leave the loop
continue                     # next iteration
return
exit "Done after {$count} parts"
raise "No part at slot {$i}", code=noPart
label retry
goto retry
call Pick
call Pick(height=20, count=$n), out found=$ok
pause
status "Placed {$count}"     # status warning "…" / status error "…"
stopwatch start $lap         # start | stop | reset
```

A condition is an ordinary expression; its top-level `and` / `or` become the separate
condition rows the Blocks editor shows.

### Variables and lists

```srs
$count = $count + 1          # Set Variable
$label = "Bin {$i}"          # a string variable takes text in quotes (a template)
$vals.append($grams)
$vals.insert(0, $grams)
$vals.removeAt($i)
$vals.clear()
$vals[$i] = $grams
$pts[$i].z = 10
$pts.append(x=$robot.x, y=$robot.y, z=5)
$pts.append()                # appends the current position
$cells.sort(by=value, desc)
$vals.reverse()
$copy.copyFrom($vals)
```

### Vision, communication, background, plugins

```srs
RunVision "Detect Parts", frame=robot, pass=$visionOk
  "Top edge".angle -> $edgeAngle       # tool (by its name, else its id) . value -> variable
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
HttpReceive "checkers", timeout=60s
  receive "x" -> $x
end
prompt "Load the tray, then continue", title="Setup", buttons=("Continue", "Skip"), result=$choice
prompt "How many parts?", input=number, default=10, min=1, max=50, value=$qty, timeout=60s
plugin scale.weigh, timeout=5s
  samples = $n * 2                     # unquoted: an expression
  unit = "g"                           # quoted: the exact text
  grams -> $w
end
unknown "MoveWithForce"                # a step type this controller does not know
```

Row bodies (`RunVision`, `HttpRequest`, `HttpReceive`, `ExportData`, `plugin`) close with
`end` when they have rows; a step with no rows is just its first line.

## Names and ids

- **Names resolve to ids.** `MoveL Pick1` stores the point's id and its name; a renamed point
  keeps working and the text shows its new name. Names match exactly first, then ignoring case.
- **Names with spaces: the underscore form.** A name that is not an identifier can be written
  either quoted (`"Pick 1"`) or as an identifier in its *underscore form*: every run of
  characters an identifier cannot hold (spaces, `-`, `/`, `.`, accents, emoji …) becomes one `_`.
  `"Pick 1"` is `Pick_1`, `"Bin - left"` is `Bin_left`, `"Arduino Nano"` is `Arduino_Nano`
  (the same spelling `$io.Arduino_Nano.…` already uses). This works for every name the text
  resolves: points (targets, `via=`, `SavePosition point`), tools, locals, grids, stacks,
  routines (`call`), background programs, vision programs and their tools (`Top_edge.angle -> $a`),
  devices, cameras and `goto` labels.
  - **Matching order:** an exact name → the name ignoring case → the underscore form,
    case-sensitive first, then ignoring case (so `pick_1` finds `"Pick 1"`, like `pick1` finds
    `Pick1`). An exact name always wins: with points `"Pick 1"` and `Pick_1`, `Pick_1` is the
    second one.
  - **Ambiguity is an error** (`ambiguousName`): when no name matches exactly and the underscore
    form fits two or more names (`"Pick 1"` and `"Pick-1"` are both `Pick_1`), the text must use
    the quoted name. A quoted real name always resolves, so quoting is the way out.
  - **Printing:** the formatter writes a name bare when it is an identifier, else its underscore
    form when that is an identifier, is not a keyword or reserved value word (`end`, `here`, `on`,
    `_`, `MoveL`, …) and reads back as the same item, else quoted. So `"Pick 1"` prints as
    `Pick_1`, but prints quoted when a `Pick_1` or a `"Pick-1"` also exists, when it starts with
    a digit (`"1st pick"`), or when the point is not in the repository (an unknown name or a
    dangling id — the underscore form would not read back as the same name).
  - A label's own declaration (`label "Try again"`) stays quoted — it defines the name;
    `goto Try_again` refers to it.
  - Only the text uses the underscore form; names stored in program JSON match as written.
- **Unknown names are warnings, not errors**, so you can write a program before teaching its
  points: the name is kept (the validator reports it until the point exists). For things linked
  by id only (devices, cameras, grids, stacks, vision tools) the text you typed is kept as written.
- **Ids are kept across edits.** Text has no ids, so each parse is matched against the program
  being edited, list by list: steps keep their ids when they line up (same type and the same
  point / routine / variable / label …), when only a field changed, when the target changed
  between unchanged neighbours, and when a step was moved. Condition rows, else-if branches,
  switch cases, a label's link id and literals hidden behind an expression are kept too.
  Anything new gets a fresh id. `@id "…"` pins an id explicitly.
- **Labels.** `goto NAME` links to the label of that name in the same program; renaming a label
  in the text keeps its links.

## @with and the round trip

The text has words for everything the editor normally sets. A step can still carry other fields
— left over from an older editor, set by a script, or a toolpath edited in the CNC builder.
Those ride along in an `@with {…}` line above the step, as JSON that is merged over what the
statement says (objects merge, `null` removes a value):

```srs
@with {"cncSpec": {"file": "cube.dxf", "safeZ": 5, "paths": [[0, 0, 10, 0]]}}
CncProgram "cube.dxf", safeZ=5
```

Leave `@with` lines as they are; they are how a program survives a round trip through text
unchanged. The formatter checks every statement it writes by reading it back, so a field it could
not put into words always lands in `@with` rather than being lost.

The guarantee, tested on every step type, every field and hundreds of random programs:
`Normalize(Parse(Format(P))) == Normalize(P)`, ids included. *Normalize* only makes changes that
do not alter what the program does: legacy fields are rewritten to their current form
(`varPointName` → `pointNameExpr`, legacy condition rows → expressions, image rows →
`jsonOutbound`), empty lists are dropped, names are refreshed from their ids, implied defaults
are written out (loop mode, wait mode, output card …), and defaults the old editor stamped on
every step (`outputNumber`, `waitMs`, `loopCount` on steps that never read them) are removed.

## Problems

Parse problems come back with 1-based `line`/`col` (UTF-16 code units, as in JavaScript),
`endLine`/`endCol` (exclusive), a `severity` and a `code`. A line with a problem is skipped and
parsing carries on, so every problem in the text is reported at once.

| Code | Meaning |
|---|---|
| `syntax` | not valid here (with what was expected) |
| `unknownKeyword` | a statement word that does not exist (with "did you mean …") |
| `unknownArgument` / `duplicateArgument` / `missingArgument` | a named value the step does not have / given twice / a required one missing |
| `badValue` | the wrong kind of value: text where a number belongs, an option that does not exist, an expression without parentheses in a positional slot |
| `expressionSyntax` / `unknownFunction` / `badArity` | from the expression checker, pointing inside the expression |
| `missingEnd` / `unexpectedEnd` / `misplacedClause` | block structure: a block without `end`, a stray `end`, `else`/`case`/`catch` in the wrong place |
| `declarationKind` | e.g. `persistent computed` |
| `stringRequired` | a string variable assigned an expression, or a number variable assigned text |
| `duplicateId` | two steps with the same `@id` |
| `unknownPoint`, `unknownTool`, `unknownRoutine`, `unknownGrid`, `unknownDevice`, `unknownLabel` … | warnings: the name does not resolve (yet) |
| `unknownField` | warning: an `@with` key that is not a field (dropped) |
| `ambiguousName` | a name in its underscore form (`Pick_1`) fits more than one item (`"Pick 1"`, `"Pick-1"`) — write the quoted name |

With validation on, the program validator's problems are added too (`source: "validate"`),
placed on the step's line and, where the problem names a field, on that value.

## Commands and tools

```jsonc
FormatProgramText { program, includeIds?: false, includeHeader?: false }
→ { text, languageVersion, map: [ { stepId, stepPath, line, endLine } ] }

ParseProgramText  { text, base?: BuiltProgram, validate?: true }
→ { ok, program?, problems: [ TextProblem ], map }
```

- `ok` is false when there is any error-severity *parse* problem; `program` is then absent.
  Validation errors do not clear `ok` (they block Run, not switching views). Note that the
  result's own `ok` is the ack's `ok` on the wire — read it as "the text parsed", not "the
  command failed".
- `base` is the program being edited: its metadata is kept and its ids are reconciled. Without
  a base (importing a file) the `@program` header gives the metadata and every id is new.
- `map` places each step on its lines (a block's `endLine` is its `end`) — for run highlighting,
  breakpoints and jumping from a problem to its line.
- MCP (`mcp/robot_mcp.py`): `robot_program_text_get(name)` and
  `robot_program_text_set(name, text)` read and write a program as text; the set validates and
  saves like `robot_save_program`.

## Grammar

```ebnf
file        = [ "#!srs" INT NL ] { line } ;
line        = ( decl | attr | stmt | comment | blank ) [ "#" text ] NL ;   (* a line continues while a bracket is open or after "," *)
attr        = "@" ( "name" STRING | "off" | "id" STRING | "with" JSON | "program" JSON ) ;
comment     = "#" text | "##" text ;                    (* paragraph → Note; "##" → section Note *)

decl        = { "global" | "persistent" | "input" | "monitor" }
              ( "var" | "computed" | "param" ( "in" | "out" | "inout" ) ) NAME [ ":" type ] [ "=" init ] ;
type        = "number" | "bool" | "string" | "image" | "stopwatch" | "list" ( "number" | "bool" | "point" | "record" ) ;
init        = EXPR | STRING | "(" [ value { "," value } ] ")" ;

stmt        = if | loop | switch | try | assign | listop | flow | keyword-stmt ;
if          = "if" COND NL body { ( "elif" | "else if" ) COND NL body } [ "else" NL body ] "end" ;
loop        = ( "repeat" [ ( "forever" | POS ) ] [ "as" VAR ]
              | "while" COND
              | "for" ( VAR | "_" ) [ "," VAR ] "in" VAR ) [ "," args ] NL body "end" ;
switch      = "switch" EXPR NL { "case" EXPR { "," EXPR } NL body } [ "default" NL body ] "end" ;
try         = "try" [ args ] NL body [ "catch" [ VAR ] [ "," args ] NL body ] "end" ;
assign      = VAR "=" ( EXPR | STRING )
            | VAR "[" EXPR "]" [ "." IDENT ] "=" ( EXPR | "(" IDENT "=" EXPR { "," IDENT "=" EXPR } ")" ) ;
listop      = VAR "." ( "append" | "insert" | "removeAt" | "clear" | "sort" | "reverse" | "copyFrom" ) "(" [ arglist ] ")" ;
flow        = "break" | "continue" | "return" | "pause"
            | "exit" [ STRING ] | "raise" STRING [ "," args ] | "label" NAME | "goto" NAME
            | "call" NAME [ "(" IDENT "=" value { "," IDENT "=" value } ")" ] { "," "out" IDENT "=" VAR }
            | "status" [ "info" | "warning" | "error" ] [ STRING ] [ "," args ]
            | "stopwatch" ( "start" | "stop" | "reset" ) VAR
            | "prompt" STRING [ "," args ]
            | "plugin" NAME "." NAME [ "," args ] [ rows ]
            | "unknown" STRING ;
keyword-stmt= KEYWORD { POS } [ [ "," ] args ] [ rows ] ;   (* KEYWORD: a step type name, e.g. MoveL *)
rows        = NL { row NL } "end" ;                          (* RunVision, HttpRequest, HttpReceive, ExportData, plugin *)
row         = [ "send" ] KEY "=" ( EXPR | "list" VAR | "image" VAR )
            | [ "receive" ] KEY "->" VAR
            | KEY "." KEY "->" VAR ;                         (* RunVision: tool.value -> $var *)
args        = named { "," named } ;
named       = IDENT "=" value ;
value       = EXPR | STRING | BOOL | NUMBER [ "ms" | "s" ] | tuple | list | target ;
tuple       = "(" IDENT "=" EXPR { "," IDENT "=" EXPR } ")" ;
list        = "(" NAME { "," NAME } ")" ;
target      = "here" | NAME | STRING | VARREF | ( "grid" | "stack" ) NAME "[" EXPR [ "," EXPR ] "]" ;
POS         = NAME | STRING | NUMBER [ unit ] | BOOL | VARREF | "(" EXPR ")" | target | "J" INT | "toggle" ;
NAME        = IDENT | STRING ;
VAR         = "$" IDENT ;
VARREF      = VAR { "[" EXPR "]" | "." IDENT } ;
COND, EXPR  = the expression language (docs/expressions-and-variables.md), up to the next top-level "," or the end of the line ;
```

Every step type name is also a keyword (`IfCondition`, `Loop`, `SetVariable`, `Note` …): the
formatter uses the bare name plus `@with` for a step it cannot otherwise write.

## Argument reference

Generated from the step catalog (`GetStepCatalog`); *position n* is the n-th value after the
keyword, `name=` a named value.

<!-- BEGIN GENERATED ARGUMENTS (DocTableTests) -->
#### Motion

**Move Linear** — Move to a target in a straight line.  
`MoveL <target>[, speed=…][, blend=…][, offset=(…)][, until=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Target | target |  |  |
| `speed=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |
| `blend=` | Blend into next move | `on` / `off` |  | off |
| `blendRadius=` | Blend radius | number or `(expr)` | mm |  |
| `offset=` | Offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `set=` | Override | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `toolOffset=` | Tool offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `local=` | Local frame | local name |  |  |
| `until=` | Stop condition | condition |  |  |
| `untilMode=` | When true | `stop` \| `record` |  | stop |
| `untilPos=` | Position variable | `$variable` |  |  |
| `untilFlag=` | Fired flag | `$variable` |  |  |

**Move Joint** — Move to a target by joint interpolation.  
`MoveJ <target>[, speed=…][, offset=(…)][, until=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Target | target |  |  |
| `speed=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |
| `offset=` | Offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `set=` | Override | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `toolOffset=` | Tool offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `local=` | Local frame | local name |  |  |
| `until=` | Stop condition | condition |  |  |
| `untilMode=` | When true | `stop` \| `record` |  | stop |
| `untilPos=` | Position variable | `$variable` |  |  |
| `untilFlag=` | Fired flag | `$variable` |  |  |

**Jump Linear** — Lift, move linearly over the target, then lower.  
`JumpL <target>, z=…[, zStart=…][, zEnd=…][, speed=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Target | target |  |  |
| `z=` | Lift height | number or `(expr)` | mm |  |
| `zStart=` | Lift height (start) | number or `(expr)` | mm |  |
| `zEnd=` | Lift height (end) | number or `(expr)` | mm |  |
| `speed=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |
| `blend=` | Blend into next move | `on` / `off` |  | off |
| `blendRadius=` | Blend radius | number or `(expr)` | mm |  |
| `offset=` | Offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `set=` | Override | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `toolOffset=` | Tool offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `local=` | Local frame | local name |  |  |
| `until=` | Stop condition | condition |  |  |
| `untilMode=` | When true | `stop` \| `record` |  | stop |
| `untilPos=` | Position variable | `$variable` |  |  |
| `untilFlag=` | Fired flag | `$variable` |  |  |

**Jump Joint** — Lift, move by joint interpolation over the target, then lower.  
`JumpJ <target>, z=…[, zStart=…][, zEnd=…][, speed=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Target | target |  |  |
| `z=` | Lift height | number or `(expr)` | mm |  |
| `zStart=` | Lift height (start) | number or `(expr)` | mm |  |
| `zEnd=` | Lift height (end) | number or `(expr)` | mm |  |
| `speed=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |
| `offset=` | Offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `set=` | Override | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `toolOffset=` | Tool offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `local=` | Local frame | local name |  |  |
| `until=` | Stop condition | condition |  |  |
| `untilMode=` | When true | `stop` \| `record` |  | stop |
| `untilPos=` | Position variable | `$variable` |  |  |
| `untilFlag=` | Fired flag | `$variable` |  |  |

**Move Arc** — Move along a circular arc through a via point to the end target.  
`MoveC <end>, via=<point>[, speed=…][, tolerance=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Target | target |  |  |
| `via=` | Via point | target |  |  |
| `speed=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |
| `tolerance=` | Arc tolerance | number | mm | 0.1 |
| `offset=` | Offset | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `set=` | Override | `(x=…, y=…, z=…, rx=…, ry=…, rz=…)` | mm / deg |  |
| `local=` | Local frame | local name |  |  |

**Thread Move** — Move the tool in Z while rotating RZ to follow a thread.  
`ThreadMove distance=…, pitch=…[, peck=on, peckDepth=…, peckRetract=…][, exitHeight=…][, reverseOut=on]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `distance=` | Distance | number or `(expr)` | mm |  |
| `pitch=` | Pitch | number or `(expr)` | mm |  |
| `peck=` | Peck | `on` / `off` |  | off |
| `peckDepth=` | Peck depth | number or `(expr)` | mm |  |
| `peckRetract=` | Peck retract | number or `(expr)` | mm |  |
| `exitHeight=` | Exit height | number or `(expr)` | mm |  |
| `reverseOut=` | Reverse out | `on` / `off` |  | off |
| `speed=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |

**Run Homing** — Run the full homing sequence and wait for it to finish.  
`RunHoming`


**Set Joint Position** — Declare one joint's value with no motion (manual homing).  
`SetJointPosition J<n>, <value>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Joint | number |  | 0 |
| position 2 | Value | number or `(expr)` | deg / mm |  |

**CNC Program** — Follow a toolpath generated from a DXF or SVG file.  
`CncProgram "<file>"[, safeZ=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | File | text |  |  |
| `safeZ=` | Safe Z | number | mm |  |

**Save Position** — Store the current position in a Points list or a saved point.  
`SavePosition $<list>[, mode=replace\|append\|insert\|set][, index=…][, source=actual\|target] \| SavePosition point <name>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `to=` | Save to | `variable` \| `point` |  | variable |
| position 1 | Points variable | `$list` |  |  |
| `point=` | Saved point | point name |  |  |
| `mode=` | Mode | `replace` \| `append` \| `insert` \| `set` |  | replace |
| `index=` | Index | number or `(expr)` |  |  |
| `source=` | Position | `actual` \| `target` |  | actual |


#### Motion Settings

**Set Speed (Linear)** — Set the linear move speed, accel and decel.  
`SetSpeedL <speed>[, accel=…][, decel=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |

**Set Speed (Joint)** — Set the joint move speed, accel and decel.  
`SetSpeedJ <speed>[, accel=…][, decel=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `decel=` | Deceleration | number or `(expr)` | mm/s² |  |

**Set Blend Radius** — Set the default corner radius for later blended moves.  
`SetBlendRadius <mm>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Blend radius | number or `(expr)` | mm |  |

**Set Speed Factor** — Scale this program's move speeds by a percentage.  
`SetSpeedFactor <percent>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Speed factor | number or `(expr)` | % | 100 |

**Set Tool** — Change the active tool used by later moves.  
`SetTool <tool>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Tool | tool name |  |  |

**Set Local** — Activate a local coordinate frame for later moves.  
`SetLocal <local>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Local frame | local name |  |  |

**Clear Local** — Return later moves to the world frame.  
`ClearLocal`



#### Logic & Flow

**If Condition** — Run steps only when a condition holds, with else-if and else branches.  
`if <condition> … [elif <condition> …] [else …] end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Condition | condition |  |  |

**Switch** — Run the case whose value equals an expression.  
`switch <expr> case <v>[, <v>…] … [default …] end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Value | expression |  |  |

**Loop** — Repeat steps a number of times, for each list element, or while a condition holds.  
`repeat <n>\|forever [as $i] … end \| for $v[, $i] in $list … end \| while <condition> … end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `mode=` | Repeat | `count` \| `forEach` \| `while` |  | count |
| position 1 | Count | number or `(expr)` |  | 1 |
| `in=` | List | `$list` |  |  |
| `value=` | Value variable | `$variable` |  |  |
| `as=` | Index variable | `$variable` |  |  |
| `while=` | While | condition |  |  |

**Break / Next** — Leave the enclosing loop, or skip to its next iteration.  
`break \| continue`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Action | `break` \| `continue` |  | break |

**Try** — Run steps; on a runtime error retry them or run the catch steps.  
`try [retry=…][, delay=…] … [catch [$msg][, code=$code][, only=(…)] …] end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `retry=` | Retries | number |  | 0 |
| `delay=` | Retry delay | duration (`500ms`, `2s`) | ms | 0 |
| `only=` | Catch only | `(a, b, …)` |  |  |
| `message=` | Message variable | `$variable` |  |  |
| `code=` | Code variable | `$variable` |  |  |

**Raise Error** — Fail with a message — caught by an enclosing Try, otherwise ends the program.  
`raise "<message>"[, code=<identifier>]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Message | `"text {$var}"` |  |  |
| `code=` | Code | text |  | userError |

**Call Routine** — Run a routine, then continue.  
`call <routine>[(param=…, …)][, out param=$var]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Routine | routine name |  |  |
| rows / sugar | Arguments | exprMap |  |  |
| `out=` | Outputs | mappingOut |  |  |

**Return** — Leave the routine (or end the program).  
`return`


**End Program** — Finish the program now, from anywhere.  
`exit ["<message>"]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Message | `"text {$var}"` |  |  |

**Label** — Mark a place Go To Label can jump to.  
`label <name>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Name | text |  |  |

**Go To Label** — Jump to a Label.  
`goto <label>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Label | label name |  |  |

**Pause Program** — Pause — the operator continues or stops from the monitor.  
`pause`


**Note** — A note or section header — not executed.  
`# text \| ## Section`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| rows / sugar | Style | `note` \| `section` |  | note |
| position 1 | Text | text |  |  |


#### Variables & Timing

**Set Variable** — Assign a value or expression to a variable.  
`$<name> = <expression \| "template">`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Variable | `$variable` |  |  |
| position 2 | Value | expression |  |  |

**List** — Append, insert, remove, set, sort or copy list elements.  
`$<list>.append(…) \| .insert(i, …) \| .removeAt(i) \| .clear() \| [i] = … \| [i].field = … \| .sort(by=…, desc) \| .reverse() \| .copyFrom($other)`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | List | `$list` |  |  |
| `action=` | Action | `append` \| `insert` \| `removeAt` \| `clear` \| `set` \| `setField` \| `sort` \| `reverse` \| `copyFrom` |  | append |
| `index=` | Index | number or `(expr)` |  |  |
| `value=` | Value | expression |  |  |
| `fields=` | Fields | exprMap |  |  |
| `from=` | Copy from | `$list` |  |  |
| `by=` | Sort by field | text |  |  |
| `desc=` | Descending | `on` / `off` |  | off |

**Stopwatch** — Start, stop or reset a stopwatch variable.  
`stopwatch start\|stop\|reset $<name>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Action | `start` \| `stop` \| `reset` |  | Start |
| position 2 | Stopwatch | `$variable` |  |  |

**Wait** — Wait for a time, or until a condition holds.  
`Wait <ms> \| Wait until <condition>[, timeout=…][, timedOut=$v][, onTimeout=continue\|error]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `mode=` | Wait for | `duration` \| `condition` |  | duration |
| position 1 | Duration | duration (`500ms`, `2s`) | ms |  |
| `until=` | Until | condition |  |  |
| `timeout=` | Timeout | duration (`500ms`, `2s`) | ms |  |
| `timedOut=` | Timed-out flag | `$variable` |  |  |
| `onTimeout=` | On timeout | `continue` \| `error` |  | continue |


#### I/O & Aux Axes

**Set Output** — Turn a digital output on or off, toggle it or pulse it.  
`SetOutput <stb\|relay\|io> [<device>] <number> <on\|off\|toggle\|(expr)>[, pulse=…][, blocking=on]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Card | `stb` \| `relay` \| `io` |  | stb |
| position 2 | Device | device name |  |  |
| position 3 | Output | number |  |  |
| `action=` | Action | `set` \| `toggle` |  | set |
| position 4 | On | `on` / `off` or `(expr)` |  | on |
| `pulse=` | Pulse | duration (`500ms`, `2s`) | ms |  |
| `blocking=` | Wait for pulse | `on` / `off` |  | off |

**Tare Load Cell** — Zero a load cell.  
`Tare device=…, pin=…`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `device=` | Device | device name |  |  |
| `pin=` | Pin | number |  |  |

**Read Load Cell** — Read a load cell into a variable.  
`ReadLoadCell $<var>, device=…, pin=…[, unit=g]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Variable | `$variable` |  |  |
| `device=` | Device | device name |  |  |
| `pin=` | Pin | number |  |  |
| `unit=` | Unit | `g` \| `kg` \| `N` \| `lb` \| `oz` |  | g |

**Start Recording** — Record a load cell into a list variable.  
`StartRecording $<list>, device=…, pin=…`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Into list | `$list` |  |  |
| `device=` | Device | device name |  |  |
| `pin=` | Pin | number |  |  |

**Stop Recording** — Stop a recording and store its samples.  
`StopRecording device=…, pin=…`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `device=` | Device | device name |  |  |
| `pin=` | Pin | number |  |  |

**Export Data** — Write variables to a CSV or JSON file.  
`ExportData csv\|json "<path>" … end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Format | `csv` \| `json` |  | csv |
| position 2 | Path | `"text {$var}"` |  |  |
| rows / sugar | Columns | rows |  |  |

**Aux Move** — Move an aux axis by a distance or a number of steps.  
`AuxMove device=…, axis=…, distance=…\|steps=…[, unit=mm\|deg][, velocity=…][, absolute=on][, wait=off][, until=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |
| `unit=` | Unit | `mm` \| `deg` |  |  |
| `distance=` | Distance | number or `(expr)` |  |  |
| `steps=` | Steps | number or `(expr)` |  |  |
| `velocity=` | Velocity | number or `(expr)` |  |  |
| `accel=` | Acceleration | number or `(expr)` |  |  |
| `decel=` | Deceleration | number or `(expr)` |  |  |
| `absolute=` | Absolute | `on` / `off` |  | off |
| `wait=` | Wait until done | `on` / `off` |  | on |
| `until=` | Stop condition | condition |  |  |
| `untilMode=` | When true | `stop` \| `record` |  | stop |
| `untilPos=` | Position variable | `$variable` |  |  |
| `untilFlag=` | Fired flag | `$variable` |  |  |

**Aux Continuous Run** — Run an aux axis continuously until an Aux Stop.  
`AuxContinuous device=…, axis=…, velocity=…[, accel=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |
| `unit=` | Unit | `mm` \| `deg` |  |  |
| `velocity=` | Velocity | number or `(expr)` |  |  |
| `accel=` | Acceleration | number or `(expr)` |  |  |

**Aux Stop** — Stop aux axis motion — ramped or immediate.  
`AuxStop device=…, axis=…[, decel=…][, immediate=on]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |
| `decel=` | Deceleration | number or `(expr)` |  |  |
| `immediate=` | Immediate | `on` / `off` |  | off |

**Aux Motor Enable** — Enable or disable the motor drivers of an aux card.  
`AuxEnable device=…, axis=…, on\|off`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |
| position 1 | Enabled | `on` / `off` |  | on |

**Gripper** — Open, close or grip to a jaw gap.  
`GripperAction open\|close\|to <mm>[, device=…, axis=…][, velocity=…][, until=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Action | `open` \| `close` \| `to` |  | Open |
| `to=` | Jaw gap | number or `(expr)` | mm |  |
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |
| `velocity=` | Speed | number or `(expr)` | mm/s |  |
| `accel=` | Acceleration | number or `(expr)` | mm/s² |  |
| `until=` | Stop condition | condition |  |  |
| `untilMode=` | When true | `stop` \| `record` |  | stop |
| `untilPos=` | Position variable | `$variable` |  |  |
| `untilFlag=` | Fired flag | `$variable` |  |  |

**Conveyor** — Run a conveyor forward or reverse, or stop it.  
`ConveyorAction forward\|reverse\|stop[, device=…, axis=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Action | `forward` \| `reverse` \| `stop` |  | RunForward |
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |

**Rotary Index** — Index a rotary axis a number of positions.  
`RotaryIndex +1\|-1[, count=…][, device=…, axis=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Direction | number |  | 1 |
| `count=` | Count | number |  | 1 |
| `device=` | Device | device name |  |  |
| `axis=` | Axis | number |  | 0 |


#### Vision

**Run Vision** — Run a vision program and write its results into variables.  
`RunVision <program>[, frame=pixel\|normalized\|robot][, pass=$v] … end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Vision program | vision program name |  |  |
| `frame=` | Coordinates | `pixel` \| `normalized` \| `robot` |  | pixel |
| `pass=` | Pass variable | `$variable` |  |  |
| rows / sugar | Tool outputs | mappingOut |  |  |

**Capture Image** — Store the current camera frame in an image variable.  
`CaptureImage $<image>[, camera=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Image variable | `$variable` |  |  |
| `camera=` | Camera | camera name |  |  |

**Save Image** — Save a camera snapshot to a file.  
`SaveImage "<path>"[, camera=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Path | `"text {$var}"` |  |  |
| `camera=` | Camera | camera name |  |  |


#### Background

**Start Background** — Start a background program in parallel.  
`StartBackground <program>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Background program | program name |  |  |

**Stop Background** — Stop a background program.  
`StopBackground <program>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Background program | program name |  |  |

**Wait for Background** — Wait until a background program finishes.  
`WaitForBackground <program>`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Background program | program name |  |  |


#### Communication

**HTTP Request** — POST JSON to a server and read values from the reply.  
`HttpRequest "<url>"[, wait=on][, timeout=…] send … receive … end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | URL | text |  |  |
| `wait=` | Wait for the reply | `on` / `off` |  | on |
| `timeout=` | Timeout | duration (`500ms`, `2s`) | ms |  |
| `send=` | Send | rows |  |  |
| `receive=` | Receive | mappingOut |  |  |

**HTTP Receive** — Wait for an HTTP POST to a named webhook.  
`HttpReceive "<name>"[, timeout=…] receive … end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Name | text |  |  |
| `timeout=` | Timeout | duration (`500ms`, `2s`) | ms |  |
| `receive=` | Receive | mappingOut |  |  |

**Status Update** — Publish a message, warning or error to the monitor.  
`status [info\|warning\|error] "<message>"`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| rows / sugar | Severity | `Info` \| `Warning` \| `Error` |  | Info |
| position 1 | Message | `"text {$var}"` |  |  |
| `warning=` | Warning | `"text {$var}"` |  |  |
| `error=` | Error | `"text {$var}"` |  |  |

**Prompt** — Ask the operator: show a message with buttons and an optional input.  
`prompt "<message>"[, title=…][, buttons=(…)][, input=number\|text][, result=$v][, value=$v][, timeout=…]`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Message | `"text {$var}"` |  |  |
| `title=` | Title | `"text {$var}"` |  |  |
| `buttons=` | Buttons | `(a, b, …)` |  | OK |
| `result=` | Button variable | `$variable` |  |  |
| `input=` | Input | `none` \| `number` \| `text` |  | none |
| `default=` | Default | expression |  |  |
| `min=` | Minimum | number |  |  |
| `max=` | Maximum | number |  |  |
| `value=` | Input variable | `$variable` |  |  |
| `timeout=` | Timeout | duration (`500ms`, `2s`) | ms | 0 |
| `onTimeout=` | On timeout press | number |  |  |


#### Plugins

**Plugin** — A step provided by an installed plugin.  
`plugin <id>.<step>[, timeout=…] <param> = … <output> -> $v end`

| In text | Field | Value | Unit | Default |
|---|---|---|---|---|
| position 1 | Plugin step | pluginRef |  |  |
| rows / sugar | Parameters | exprMap |  |  |
| rows / sugar | Outputs | mappingOut |  |  |
| `timeout=` | Timeout | duration (`500ms`, `2s`) | ms |  |

<!-- END GENERATED ARGUMENTS -->
