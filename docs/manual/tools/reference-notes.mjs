// Hand-written part of the instruction reference. gen-reference.mjs merges it with the step
// catalog (data/step-catalog.json, from GetStepCatalog): the catalog supplies names, syntax,
// fields, units and defaults; this file supplies the prose a catalog cannot hold.
//
// Per step type:
//   notes    — restrictions and behaviour worth knowing (Markdown inline)
//   errors   — runtime error codes the step can fail with: [code, when]
//   example  — a short Simple Robot Script example (declarations + statements). It must parse;
//              shots.cjs renders the same text in the Blocks editor for the screenshot.
//   seeAlso  — other step types
//   chapter  — the concept chapter (manifest id) that explains the step in context

const MOTION_FATAL = [
  ["motionFault", "The motion system reports a fault (fatal)."],
  ["jointLimit", "The target is outside a joint's soft limits (fatal)."],
  ["driverDisconnected", "The motor driver is not connected (fatal)."],
];
const EXPR = [
  ["unknownVariable", "An expression names a variable that does not exist (fatal)."],
  ["expressionSyntax", "An expression does not parse (fatal)."],
];
const TARGET_NOTES = [
  "The target is resolved from the first of: a grid cell, a stack slot, a `$list[i]` element or template (`pointNameExpr`), a saved point. With no target the move starts from the current position, so `offset=` makes it relative.",
  "`offset=` is added to the target, `set=` replaces an axis outright, `toolOffset=` is applied in the tool frame; each axis may be an expression.",
  "Program moves are scaled by the operator's speed override and by Set Speed Factor.",
  "Not run in background programs (motion is skipped there).",
];
const STOP_NOTES = [
  "A stop condition (`until=`) is checked every controller tick while the move runs. `untilMode=stop` ends the move where it is and continues with the next step (no error); `record` lets the move finish and stores the pose where the condition first held in `untilPos=` (a one-element Points list). `untilFlag=` gets 1 if it fired, else 0.",
];

export default {
  // ── Motion ──────────────────────────────────────────────────────────────
  MoveL: {
    chapter: "motion",
    notes: [
      "The tool centre point travels a straight line in Cartesian space.",
      "`blend=on` (or `blend=<radius>`) rounds the corner into the next blend-enabled move instead of stopping; consecutive blended moves run as one continuous path. Without a radius the value from Set Blend Radius is used.",
      "A Move Linear with a stop condition never blends.",
      ...TARGET_NOTES, ...STOP_NOTES,
    ],
    errors: [["pointNotFound", "A template or `$list[i]` target names no saved point, or the Points list is empty."], ...MOTION_FATAL, ...EXPR],
    example: `MoveL Approach, speed=150
MoveL Pick1, offset=(z=20), blend=5
MoveL Pick1`,
    seeAlso: ["MoveJ", "JumpL", "MoveC", "SetSpeedL", "SetBlendRadius"],
  },
  MoveJ: {
    chapter: "motion",
    notes: [
      "All joints start and stop together; the tool path is generally *not* a straight line. Usually the fastest way between two poses in free space.",
      "Move Joint does not blend.",
      ...TARGET_NOTES, ...STOP_NOTES,
    ],
    errors: [["pointNotFound", "A template or `$list[i]` target names no saved point."], ...MOTION_FATAL, ...EXPR],
    example: `MoveJ Home
MoveJ Pick1, speed=50, accel=100, decel=100`,
    seeAlso: ["MoveL", "JumpJ", "SetSpeedJ"],
  },
  JumpL: {
    chapter: "motion",
    notes: [
      "Three legs: lift straight up to the height `z=`, travel over the target, lower onto the target. The heights are absolute Z values, not offsets. `zStart=` / `zEnd=` set the lift and lower heights separately (the traverse runs from one to the other); `z=` is required unless both are given.",
      "Blending applies to the traverse leg.",
      "With a stop condition the jump takes the stepped (non-blended) path and the watch spans all three legs; in stop mode the remaining legs are skipped.",
      ...TARGET_NOTES.slice(0, 3), TARGET_NOTES[3],
    ],
    errors: [["pointNotFound", "A template or `$list[i]` target names no saved point."], ...MOTION_FATAL, ...EXPR],
    example: `JumpL Place1, z=60
JumpL grid Tray[2, 1], z=40, zEnd=50`,
    seeAlso: ["JumpJ", "MoveL", "GripperAction"],
  },
  JumpJ: {
    chapter: "motion",
    notes: [
      "Like Jump Linear, but each leg is a joint-interpolated move — faster, with a curved path between the lift and lower points.",
      ...TARGET_NOTES.slice(0, 3), TARGET_NOTES[3],
    ],
    errors: [["pointNotFound", "A template or `$list[i]` target names no saved point."], ...MOTION_FATAL, ...EXPR],
    example: `JumpJ stack Bins[0], z=50
JumpJ Pick1, z=50, speed=80`,
    seeAlso: ["JumpL", "MoveJ"],
  },
  MoveC: {
    chapter: "motion",
    notes: [
      "Follows the circle through the start (where the robot is), the via point and the end, in any plane.",
      "The arc is cut into equal chords that stay within `tolerance=` of the true circle (and span at most 2°), sent as one continuous path; the robot stops exactly on the end.",
      "RX / RY / RZ change linearly from the start's to the end's; the via point's orientation is not used.",
      "`offset=`, `set=` and `local=` apply to both the via and the end. Stop conditions are not supported; tool offset and blending are ignored.",
      "Not run in background programs.",
    ],
    errors: [["arcDegenerate", "Start, via and end are on one line, or two of them are the same."], ["pointNotFound", "The via or end names no saved point."], ...MOTION_FATAL],
    example: `MoveL Pick1
MoveC Place1, via=Via1, speed=80, tolerance=0.1`,
    seeAlso: ["MoveL", "SetSpeedL"],
  },
  ThreadMove: {
    chapter: "motion",
    notes: [
      "Moves the tool `distance=` along Z while turning RZ by one revolution per `pitch=` mm — screwing in or tapping.",
      "`peck=on` advances in `peckDepth=` steps, retracting by `peckRetract=` between pecks (empty = back to the start each peck). `exitHeight=` and `reverseOut=on` control how the tool leaves.",
      "Not run in background programs.",
    ],
    errors: [...MOTION_FATAL],
    example: `MoveL Pick1, offset=(z=5)
ThreadMove distance=-12, pitch=1.25, reverseOut=on`,
    seeAlso: ["MoveL", "SetTool"],
  },
  RunHoming: {
    chapter: "motion",
    notes: ["Runs the full homing sequence and waits until it finishes.", "Not run in background programs."],
    errors: [...MOTION_FATAL],
    example: `if not $robot.homed
  RunHoming
end`,
    seeAlso: ["SetJointPosition"],
  },
  SetJointPosition: {
    chapter: "motion",
    notes: ["Declares the current value of one joint without moving it (manual homing). `J1`…`J4`."],
    errors: [],
    example: `SetJointPosition J4, 0`,
    seeAlso: ["RunHoming"],
  },
  CncProgram: {
    chapter: "motion",
    notes: [
      "Follows a toolpath generated from a DXF or SVG file. The toolpath itself is edited in the CNC builder and travels in the text as an `@with {…}` line.",
      "At run time the toolpath expands into linear moves (and thread moves). Run from step cannot start inside it.",
    ],
    errors: [...MOTION_FATAL],
    example: `CncProgram "part.dxf", safeZ=5`,
    seeAlso: ["MoveL", "ThreadMove"],
  },
  SavePosition: {
    chapter: "points",
    notes: [
      "Stores where the robot is (\"Here\"), expressed in the program's active local frame, so a later move under the same local lands on the same spot.",
      "Into a Points list: `mode=replace` (the list becomes exactly this pose), `append`, `insert` or `set` (at `index=`). A list that does not exist is created.",
      "`SavePosition point <name>` overwrites a saved point (or creates it); the change is recorded in the points history.",
      "`source=target` stores the last commanded target instead of the measured position.",
    ],
    errors: [["indexOutOfRange", "`index=` is outside the list (catchable)."], ["badListOperation", "The variable is not a Points list (fatal)."], ["pointNotFound", "A point linked by id no longer exists."]],
    example: `var taught: list point
SavePosition $taught, mode=append
SavePosition point Pick1`,
    seeAlso: ["ListOp", "MoveL"],
  },

  // ── Motion settings ─────────────────────────────────────────────────────
  SetSpeedL: {
    chapter: "motion",
    notes: ["Sets the default speed, acceleration and deceleration for later linear moves (Move Linear, Jump Linear legs, arcs). A move's own `speed=` overrides it for that move only.", "Not run in background programs."],
    errors: EXPR,
    example: `SetSpeedL 200, accel=800, decel=800`,
    seeAlso: ["SetSpeedJ", "SetSpeedFactor", "MoveL"],
  },
  SetSpeedJ: {
    chapter: "motion",
    notes: ["Sets the default speed, acceleration and deceleration for later joint moves.", "Not run in background programs."],
    errors: EXPR,
    example: `SetSpeedJ 60, accel=200`,
    seeAlso: ["SetSpeedL", "MoveJ"],
  },
  SetBlendRadius: {
    chapter: "motion",
    notes: ["Sets the default corner radius used by later moves that blend without giving their own radius."],
    errors: EXPR,
    example: `SetBlendRadius 5
MoveL Approach, blend=on`,
    seeAlso: ["MoveL"],
  },
  SetSpeedFactor: {
    chapter: "motion",
    notes: [
      "Every later move of this program runs at *operator override × factor / 100* — single moves, blended runs, jump legs, arcs and a move resumed after a stop.",
      "It can only slow a program down: values are clamped to 1–100 %. Manual moves and jogging are never affected.",
      "Readable as `$program.speedFactor`; back to 100 when the run ends.",
      "Not run in background programs.",
    ],
    errors: EXPR,
    example: `SetSpeedFactor 25     # slow first cycle
MoveL Pick1
SetSpeedFactor 100`,
    seeAlso: ["SetSpeedL", "SetSpeedJ"],
  },
  SetTool: {
    chapter: "points",
    notes: ["Makes a tool (TCP offset) active for later moves; points are reached with the tool's tip. None = no tool offset.", "Not run in background programs."],
    errors: [],
    example: `SetTool Gripper
MoveL Pick1`,
    seeAlso: ["SetLocal", "MoveL"],
  },
  SetLocal: {
    chapter: "points",
    notes: ["Activates a local coordinate frame: later targets and offsets are interpreted in it.", "A single move can use another frame with `local=`.", "Not run in background programs."],
    errors: [],
    example: `SetLocal Fixture1
MoveL here, offset=(x=10, y=10)
ClearLocal`,
    seeAlso: ["ClearLocal", "SetTool"],
  },
  ClearLocal: {
    chapter: "points",
    notes: ["Returns later moves to the world (base) frame.", "Not run in background programs."],
    errors: [],
    example: `ClearLocal`,
    seeAlso: ["SetLocal"],
  },

  // ── Logic & flow ────────────────────────────────────────────────────────
  IfCondition: {
    chapter: "flow",
    notes: [
      "Branches are tested top to bottom; the first whose condition holds runs, else the `else` branch.",
      "A condition is an ordinary expression; non-zero is true. Its top-level `and` / `or` become the separate condition rows the Blocks editor shows.",
    ],
    errors: EXPR,
    example: `var count = 0
if $count >= 12
  status "Tray full"
elif $count > 8
  status warning "Almost full"
else
  $count = $count + 1
end`,
    seeAlso: ["Switch", "Loop"],
  },
  Switch: {
    chapter: "flow",
    notes: [
      "The value is evaluated once; the first case with an equal value (within 1e-9) runs, otherwise `default`. There is no fall-through.",
      "Numbers only — test text with `if $s.contains(…)`.",
    ],
    errors: EXPR,
    example: `var grade = 1
switch $grade
case 1
  MoveL Place1
case 2, 3
  MoveL stack Bins[0]
default
  raise "Unknown grade {$grade}"
end`,
    seeAlso: ["IfCondition"],
  },
  Loop: {
    chapter: "flow",
    notes: [
      "`repeat n` runs the body n times (`as $i` counts from 0); `repeat forever` (count 0) runs until stopped or left with `break`.",
      "`for $v, $i in $list` walks a list. Number and Boolean lists give the element in `$v`; Point and Record lists give the index, so read the element as `$list[$i]`. Write `for _, $i in $list` when you only want the index.",
      "`while <condition>` re-checks the condition before every pass.",
    ],
    errors: EXPR,
    example: `var vals: list number = (1, 2, 3)
var sum = 0
repeat 3 as $i
  MoveL grid Tray[$i], offset=(z=10)
end
for $v, $i in $vals
  $sum = $sum + $v
end
while $stb.in1 == 0
  Wait 10ms
end`,
    seeAlso: ["LoopControl", "IfCondition"],
  },
  LoopControl: {
    chapter: "flow",
    notes: [
      "`break` leaves the nearest enclosing loop; `continue` ends this pass (a count loop counts it, a for-each moves to the next element, a while re-checks its condition).",
      "Branches and Try bodies between the step and the loop are simply left. It never crosses a routine boundary.",
    ],
    errors: [],
    example: `repeat forever
  if $stb.in2
    break
  end
  Wait 50ms
end`,
    seeAlso: ["Loop"],
  },
  Try: {
    chapter: "errors",
    notes: [
      "When a step inside the body fails with a catchable code — at any depth, including called routines — the body is abandoned; while retries are left it runs again from the top after `delay=`; then the message and code go into the catch variables and the catch steps run.",
      "An empty catch ignores the error. The program continues after the Try either way.",
      "`only=(…)` limits which codes are caught; others go to the next enclosing Try. A failure inside the catch also goes outward.",
      "Fatal codes (motion faults, e-stop, user stop, unknown variables, syntax errors) are never caught.",
    ],
    errors: [],
    example: `var msg: string
var code: string
try retry=2, delay=500ms
  Wait until $stb.in1, timeout=2s, onTimeout=error
catch $msg, code=$code
  status warning "Gave up: {$msg}"
end`,
    seeAlso: ["RaiseError", "Wait"],
  },
  RaiseError: {
    chapter: "errors",
    notes: ["Fails with your message and code (default `userError`). An enclosing Try that catches the code takes over; otherwise the program ends with Error.", "A code of your own is always catchable."],
    errors: [["userError", "The default code — or any identifier you give with `code=`."]],
    example: `var count = 0
if $count == 0
  raise "No parts found", code=noParts
end`,
    seeAlso: ["Try", "EndProgram"],
  },
  CallRoutine: {
    chapter: "routines",
    notes: [
      "Runs a routine program, then continues with the next step.",
      "`in` / `inout` arguments are evaluated in the caller's scope before the routine starts; when it ends (last step or Return), mapped `out` / `inout` parameters are copied into the caller's variables.",
      "A routine is a boundary: Go To Label and Break / Next inside it never reach the caller.",
    ],
    errors: EXPR,
    example: `var ok: bool = off
call PickPart(slot=3), out picked=$ok`,
    seeAlso: ["Return"],
  },
  Return: {
    chapter: "routines",
    notes: ["In a routine: leave it now (out parameters are copied back). In a main or background program: finish with Complete."],
    errors: [],
    example: `if not $stb.in1
  return
end`,
    seeAlso: ["CallRoutine", "EndProgram"],
  },
  EndProgram: {
    chapter: "flow",
    notes: ["Finishes the program with Complete from any depth (loops, routines, Try), with the message as the final status text. A background program ends only itself."],
    errors: [],
    example: `var count = 0
exit "Done after {$count} parts"`,
    seeAlso: ["Return", "RaiseError"],
  },
  Label: {
    chapter: "flow",
    notes: ["Marks a place Go To Label can jump to. The name must be unique in the program."],
    errors: [],
    example: `label retry
Wait until $stb.in1, timeout=5s
goto retry`,
    seeAlso: ["GoToLabel"],
  },
  GoToLabel: {
    chapter: "flow",
    notes: ["Jumps to a label in the same program (or the same routine). Prefer loops where they fit — they read better and keep the structure visible."],
    errors: [],
    example: `label again
MoveL Pick1
goto again`,
    seeAlso: ["Label", "Loop"],
  },
  PauseProgram: {
    chapter: "running",
    notes: ["Pauses the program (status Stopped) with its place kept; the operator continues or stops it from the monitor."],
    errors: [],
    example: `status "Check the gripper, then press Continue"
pause`,
    seeAlso: ["Prompt"],
  },
  Note: {
    chapter: "structure",
    notes: ["Not executed and not counted as a step. `## text` is a section header the editor can collapse; a comment paragraph followed by a blank line is a note."],
    errors: [],
    example: `## Pick
# Approach from above, then grip.

MoveL Pick1`,
    seeAlso: [],
  },

  // ── Variables & timing ──────────────────────────────────────────────────
  SetVariable: {
    chapter: "data",
    notes: [
      "Evaluates the expression and stores it. A text variable takes a template in quotes: `$label = \"Bin {$i}\"`.",
      "Comparisons yield 1 / 0, so they can be stored in a boolean directly.",
      "Properties (`$robot.x`…), IO names and computed variables cannot be assigned.",
    ],
    errors: [["computedVariable", "The target is a computed variable (fatal)."], ...EXPR],
    example: `var count = 0
var label: string
$count = $count + 1
$label = "Bin {$count}"`,
    seeAlso: ["ListOp"],
  },
  ListOp: {
    chapter: "data",
    notes: [
      "Number / Boolean lists take a value; Point / Record lists take fields (`x=…, y=…`). `$pts.append()` with no fields appends the current position.",
      "Everything is evaluated before the list changes, so a failing step leaves the list as it was. The declared list is never changed — a re-run starts from it.",
    ],
    errors: [["indexOutOfRange", "The index is outside the list (catchable)."], ["badListOperation", "The step does not fit the list: fields on a Number list, a value for a Points list, an incompatible copy (fatal)."], ...EXPR],
    example: `var vals: list number
var pts: list point
$vals.append(5)
$vals.sort(by=value, desc)
$pts.append(x=$robot.x, y=$robot.y, z=50)
$pts[0].z = 30`,
    seeAlso: ["SetVariable", "SavePosition", "Loop"],
  },
  StopwatchControl: {
    chapter: "data",
    notes: ["A stopwatch variable holds elapsed milliseconds while it runs; `stop` freezes it, `reset` sets it to 0."],
    errors: [],
    example: `var lap: stopwatch
stopwatch start $lap
MoveL Pick1
stopwatch stop $lap
status "Move took {$lap} ms"`,
    seeAlso: ["Wait"],
  },
  Wait: {
    chapter: "io",
    notes: [
      "`Wait 500ms` pauses for a time. `Wait until <condition>` holds until the condition is true, checked every tick.",
      "With `timeout=`, `timedOut=$v` is set to 1 when time ran out. `onTimeout=continue` (default) carries on; `onTimeout=error` fails with `waitTimeout`, which a Try can catch.",
      "A condition that holds on the tick the timeout falls due counts as met.",
    ],
    errors: [["waitTimeout", "Condition wait timed out with `onTimeout=error` (catchable)."], ...EXPR],
    example: `var late: bool = off
Wait 250ms
Wait until $stb.in1 == 1, timeout=5s, timedOut=$late`,
    seeAlso: ["Try", "SetOutput"],
  },

  // ── I/O & aux ───────────────────────────────────────────────────────────
  SetOutput: {
    chapter: "io",
    notes: [
      "Cards: `stb` (driver board outputs), `relay` (USB relay channels), `io` (an IO device such as an Arduino Nano — name the device).",
      "The value may be `on`, `off`, `toggle` or an expression in parentheses (non-zero = on).",
      "`pulse=` sets the output for that long, then flips it back. `blocking=on` waits for the pulse to finish; otherwise the program moves on and the flip-back happens in the background.",
    ],
    errors: EXPR,
    example: `SetOutput stb 1 on
SetOutput relay 2 on, pulse=250ms, blocking=on
SetOutput io Arduino_Nano 7 toggle`,
    seeAlso: ["Wait"],
  },
  Tare: {
    chapter: "io",
    notes: ["Zeroes a load cell: its current raw reading becomes the tare offset."],
    errors: [],
    example: `Tare device=Arduino_Nano, pin=2`,
    seeAlso: ["ReadLoadCell"],
  },
  ReadLoadCell: {
    chapter: "io",
    notes: ["Reads the calibrated weight into a variable in the chosen unit (grams by default). In expressions a load cell can also be read live as `$io.<device>.<pin name>` (grams)."],
    errors: [],
    example: `var grams = 0
ReadLoadCell $grams, device=Arduino_Nano, pin=2, unit=g`,
    seeAlso: ["Tare", "StartRecording"],
  },
  StartRecording: {
    chapter: "io",
    notes: ["Starts capturing a load cell. The samples arrive in the list when Stop Recording runs, as records `{t, v, r}`. The recording also shows in the app's Recordings browser."],
    errors: [],
    example: `var rec: list record
StartRecording $rec, device=Arduino_Nano, pin=2
MoveL Pick1, offset=(z=-5)
StopRecording device=Arduino_Nano, pin=2`,
    seeAlso: ["StopRecording", "ExportData"],
  },
  StopRecording: {
    chapter: "io",
    notes: ["Stops the recording this program started on that load cell and fills its list variable."],
    errors: [],
    example: `StopRecording device=Arduino_Nano, pin=2`,
    seeAlso: ["StartRecording"],
  },
  ExportData: {
    chapter: "comms",
    notes: [
      "Writes one row per column (`\"name\" = value`) to a CSV or JSON file on the controller. A value can be an expression, `list $var` or `image $var`.",
      "The path is a template (`{$time_ms}` etc.); a relative path is relative to the controller's folder. Missing folders are created.",
    ],
    errors: [],
    example: `var grams = 0
var rec: list record
ExportData csv "logs/run-{$time_ms}.csv"
  "weight" = $grams
  "series" = list $rec
end`,
    seeAlso: ["StartRecording", "HttpRequest"],
  },
  AuxMove: {
    chapter: "io",
    notes: [
      "Moves an auxiliary stepper axis by `distance=` in `unit=` (mm or deg) or by raw `steps=`; the sign is the direction. `absolute=on` moves to a position instead.",
      "`wait=on` (default) holds the program until the motor finishes; `wait=off` fires and forgets.",
      "Takes a stop condition only when it waits.",
    ],
    errors: EXPR,
    example: `AuxMove device=Aux_Stepper, axis=0, unit=mm, distance=25, velocity=20
AuxMove device=Aux_Stepper, axis=0, steps=-400, until=$stb.in2`,
    seeAlso: ["AuxStop", "AuxContinuous", "GripperAction"],
  },
  AuxContinuous: {
    chapter: "io",
    notes: ["Runs an aux axis at a constant velocity (sign = direction) until an Aux Stop."],
    errors: [],
    example: `AuxContinuous device=Aux_Stepper, axis=2, velocity=-10
Wait until $stb.in3
AuxStop device=Aux_Stepper, axis=2`,
    seeAlso: ["AuxStop"],
  },
  AuxStop: {
    chapter: "io",
    notes: ["Stops an aux axis with a ramp (`decel=`), or at once with `immediate=on`."],
    errors: [],
    example: `AuxStop device=Aux_Stepper, axis=2, immediate=on`,
    seeAlso: ["AuxContinuous", "AuxMove"],
  },
  AuxEnable: {
    chapter: "io",
    notes: ["Enables or disables the motor drivers of an aux card. A disabled motor can be turned by hand."],
    errors: [],
    example: `AuxEnable on, device=Aux_Stepper, axis=0`,
    seeAlso: ["AuxMove"],
  },
  GripperAction: {
    chapter: "io",
    notes: [
      "Drives a gripper-role aux axis to its configured open or closed gap, or to a jaw gap in mm (`to 25`).",
      "With a stop condition the jaws stop when it holds — for example close until a load cell reads a grip force.",
    ],
    errors: EXPR,
    example: `GripperAction open
GripperAction close, until=$io.Arduino_Nano.Input_1 >= 300
GripperAction to 25`,
    seeAlso: ["AuxMove", "JumpL"],
  },
  ConveyorAction: {
    chapter: "io",
    notes: ["Runs a conveyor-role aux axis forward or in reverse, or stops it."],
    errors: [],
    example: `ConveyorAction forward, device=Aux_Stepper, axis=2
Wait until $stb.in1
ConveyorAction stop, device=Aux_Stepper, axis=2`,
    seeAlso: ["RotaryIndex", "AuxContinuous"],
  },
  RotaryIndex: {
    chapter: "io",
    notes: ["Turns a rotary-role aux axis by `count` positions in the given direction (+1 or −1)."],
    errors: [],
    example: `RotaryIndex +1, count=3, device=Aux_Stepper, axis=3`,
    seeAlso: ["ConveyorAction"],
  },

  // ── Vision ──────────────────────────────────────────────────────────────
  RunVision: {
    chapter: "vision",
    notes: [
      "Runs a vision program on a fresh camera frame and writes tool results into variables, one row per `tool.value -> $var`. Values include any key of the tool's values (`angle`, `count`, `x`…), `passed`, `found`, `status`, and the lists `points`, `items`, `cells`, `lines`, `pose`.",
      "`frame=` sets the coordinates of positions written: `pixel` (default), `normalized` (0–1) or `robot` (mm on the camera's calibration plane — needs a calibrated camera).",
      "`pass=$v` gets 1 when every tool that counts toward the pass passed. Variables named in rows are created if not declared.",
    ],
    errors: [["visionTimeout", "No fresh result in time (default 30 s) (catchable)."], ["visionFailed", "The vision processor is lost, or the robot frame is not available (catchable)."]],
    example: `var ok: bool = off
var edgeAngle = 0
var cells: list record
RunVision Detect_Parts, frame=normalized, pass=$ok
  Top_edge.angle -> $edgeAngle
  Slots.cells -> $cells
end`,
    seeAlso: ["CaptureImage", "SaveImage", "Try"],
  },
  CaptureImage: {
    chapter: "vision",
    notes: ["Stores the current camera frame (JPEG) in an image variable — to send over HTTP, hand to a plugin, export or show on a display page."],
    errors: [["visionFailed", "The camera has no frame (catchable)."]],
    example: `var photo: image
CaptureImage $photo, camera=Top_Camera`,
    seeAlso: ["SaveImage", "HttpRequest"],
  },
  SaveImage: {
    chapter: "vision",
    notes: ["Saves a camera snapshot to a file on the controller. The path is a template: `\"shots/{$time_ms}.jpg\"`."],
    errors: [["visionFailed", "The camera has no frame (catchable)."]],
    example: `SaveImage "shots/part-{$time_ms}.jpg", camera=Top_Camera`,
    seeAlso: ["CaptureImage"],
  },

  // ── Background ──────────────────────────────────────────────────────────
  StartBackground: {
    chapter: "background",
    notes: ["Starts a background program running in parallel with this one. Background programs skip motion, tool and homing steps."],
    errors: [],
    example: `StartBackground Conveyor_Watch`,
    seeAlso: ["StopBackground", "WaitForBackground"],
  },
  StopBackground: {
    chapter: "background",
    notes: ["Stops a running background program."],
    errors: [],
    example: `StopBackground Conveyor_Watch`,
    seeAlso: ["StartBackground"],
  },
  WaitForBackground: {
    chapter: "background",
    notes: ["Holds this program until the background program finishes."],
    errors: [],
    example: `StartBackground Conveyor_Watch
WaitForBackground Conveyor_Watch`,
    seeAlso: ["StartBackground"],
  },

  // ── Communication ───────────────────────────────────────────────────────
  HttpRequest: {
    chapter: "comms",
    notes: [
      "POSTs a JSON object built from the `send` rows (`\"key\" = expression`, `list $var`, `image $var`) and, with `wait=on`, reads `receive \"key\" -> $var` values from the JSON reply.",
      "An empty or non-JSON reply is not a failure. With `wait=off` the request never fails.",
    ],
    errors: [["httpFailed", "Network error or a non-2xx status (catchable)."], ["httpTimeout", "No reply in time (catchable)."]],
    example: `var count = 0
var reply = 0
HttpRequest "http://192.168.4.10:8080/report", wait=on, timeout=5s
  send "count" = $count
  receive "next" -> $reply
end`,
    seeAlso: ["HttpReceive", "ExportData"],
  },
  HttpReceive: {
    chapter: "comms",
    notes: ["Waits for another machine to POST JSON to `/webhook/<name>` on the controller, then copies `receive` keys into variables."],
    errors: [["receiveTimeout", "Nothing arrived in time (catchable)."]],
    example: `var x = 0
HttpReceive "orders", timeout=60s
  receive "x" -> $x
end`,
    seeAlso: ["HttpRequest"],
  },
  StatusUpdate: {
    chapter: "comms",
    notes: ["Publishes a message to the monitor and the run log: `status \"…\"`, `status warning \"…\"`, `status error \"…\"`. The text is a template. An error status does not stop the program."],
    errors: [],
    example: `var count = 0
status "Placed {$count} parts"
status warning "Tray almost full"`,
    seeAlso: ["Prompt", "RaiseError"],
  },
  Prompt: {
    chapter: "comms",
    notes: [
      "Shows the operator a dialog with 1–4 buttons and optionally a number or text input, and waits. A number result variable gets the button index (from 0), a text one its label.",
      "With `timeout=` and `onTimeout=<button>` the button is pressed on timeout; without a button the prompt fails `promptTimeout`.",
      "The dialog appears in the app on any screen. A Stop keeps it on screen but refuses answers; Continue shows it again.",
    ],
    errors: [["promptTimeout", "Timed out with no timeout button (catchable)."]],
    example: `var choice = 0
var qty = 0
prompt "Load the tray, then continue", title="Setup", buttons=("Continue", "Skip"), result=$choice
prompt "How many parts?", input=number, default=10, min=1, max=50, value=$qty`,
    seeAlso: ["StatusUpdate", "PauseProgram"],
  },

  // ── Plugins ─────────────────────────────────────────────────────────────
  Plugin: {
    chapter: "plugins",
    notes: [
      "Runs a step an installed plugin declares in its manifest. Parameter rows: unquoted = an expression, quoted = exact text. Output rows `key -> $var` write the plugin's results.",
      "The program waits (without blocking anything else) until the plugin answers. Background programs can run plugin steps.",
    ],
    errors: [["pluginNotRunning", "The plugin is not running (catchable)."], ["pluginFailed", "The plugin answered with an error (catchable)."], ["pluginTimeout", "No answer in time (catchable)."]],
    example: `var w = 0
plugin scale.weigh, timeout=5s
  samples = 5
  unit = "g"
  grams -> $w
end`,
    seeAlso: ["HttpRequest"],
  },
};
