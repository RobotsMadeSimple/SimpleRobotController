// Builds the checkers programs on a controller:
//  - grid "Checkers Board" (id checkers-board, 8x8, base point "Checkers Board origin") — created
//    only when missing, as a placeholder for the grid-teach wizard to fill in
//  - routines "Checkers Grip" / "Checkers Release" (only when missing — edit them for your tool)
//  - "Checkers Game (camera)": end-to-end game vs a human, the camera spotting the human's move
//  - "Checkers API Boilerplate": its TODO placeholders replaced with real grid moves
// Squares 0..31 → board cell row = floor(s/4), col = 2*(s%4) + 1 - row%2 (the plugin's numbering).
//   node checkers-game.mjs json [visionProgramId] [orientation]   print the game program JSON (no controller needed)
//   CTRL=host:port node checkers-game.mjs [build|save] [visionProgramId] [orientation]
// orientation = where black's side is in the camera image (detectMove): rotated90 (right — the
// side-on camera in the real photos, default), normal (top), rotated180 (bottom), rotated270 (left).
// Crowning + captured-piece stacks (README "Checker silos"): two Silo-role aux axes, one per colour,
// each both SUPPLY a crown (SiloAction Present) and RECEIVE captured checkers (SiloAction Accept —
// the stack lowers one checker so the next lands at the rim). The steps link a silo by its aux
// device id + axis index, baked in here:
//   SILO_DEVICE (default AUX_STEPPER_001), SILO_ROBOT_AXIS (black, default 0), SILO_HUMAN_AXIS (white, default 1)
// Taught points (nothing here creates them): "Checkers Silo Robot" = the black silo's rim pose,
// "Checkers Silo Human" = the white silo's rim pose. The robot picks/drops black checkers at the
// first, white checkers at the second. Load a few spare checkers per tube (NOT full) so there is room
// for captures; an overfull tube's Accept is caught (siloFull) and the checker is dropped on top.
// The robot clears BOTH colours: its own jumps (white) and — because detectMove now reads your move
// with your jumped pieces still on the board — the black pieces you jump (leave them on the board).
import { writeFileSync } from "node:fs";
import { Ctrl } from "../../../../live-tests/harness.mjs";

const GRID_ID = "checkers-board", GRID_NAME = "Checkers Board", ORIGIN = "Checkers Board origin";
// Teaching note shown on the grid so whoever sets it up knows where to jog. Cell [row, col] must land
// on the centre of the board square at that row/column; the base point is the far corner square (row 0,
// col 0). Teach the base there, then the other corners so every square centre is covered.
const GRID_DESC = "The 8x8 playing board. Grid cell [row, col] lands on the centre of the board square at that row and column. The base point 'Checkers Board origin' is the corner square (row 0, col 0): jog to the centre of that corner square to teach it, then teach the opposite corners so every square centre is covered. The checkers plugin addresses dark squares 0-31 through these cells.";
const SILO_ROBOT_POINT = "Checkers Silo Robot";   // black silo rim: robot picks its crowns here, drops captured BLACK checkers here
const SILO_HUMAN_POINT = "Checkers Silo Human";   // white silo rim: human picks crowns here, robot drops captured WHITE checkers here
const SILO = {
  device: process.env.SILO_DEVICE || "AUX_STEPPER_001",
  robotAxis: Number(process.env.SILO_ROBOT_AXIS ?? 0),
  humanAxis: Number(process.env.SILO_HUMAN_AXIS ?? 1),
};
// The vision program checkers-vision.mjs builds (pass another id to use your own).
const VISION = { visionProgramId: process.argv[3] || "checkers-vision", visionProgramName: "Checkers Board (camera)" };
const ORIENTATION = process.argv[4] || "rotated90";

let n = 0; const id = p => `ckg-${p}-${++n}`;
const cond = (...exprs) => ({ combinator: "ALL", items: exprs.map(e => ({ id: id("c"), expr: e })) });
const setv = (variableName, variableExpr) => ({ id: id("set"), type: "SetVariable", variableName, variableExpr });
const status = (statusMessage, statusSeverity = "Info") => ({ id: id("st"), type: "StatusUpdate", statusMessage, statusSeverity });
const wait = (ms, expr) => ({ id: id("wait"), type: "Wait", waitMs: ms, ...(expr ? { expressions: { waitMs: expr } } : {}) });
const call = name => ({ id: id("call"), type: "CallRoutine", routineName: name });
const iff = (exprs, ifSteps, elseSteps = []) => ({ id: id("if"), type: "IfCondition", condition: cond(...exprs), ifSteps, elseSteps });
const whileLoop = (name, exprs, loopSteps) => ({ id: id("loop"), type: "Loop", name, loopMode: "while", loopWhileCondition: cond(...exprs), loopSteps });
const plugin = (stepId, name, params, outputs) => ({ id: id("pl"), type: "Plugin", name, pluginId: "checkers", pluginStepId: stepId,
  pluginParams: params, pluginOutputs: Object.entries(outputs).map(([key, variableName]) => ({ key, variableName })) });

// A board square (an expression) as a cell of the taught grid, optionally lifted.
const cell = (sqExpr, name, lift) => ({ id: id("mv"), type: "MoveL", name,
  gridPoint: { gridId: GRID_ID, rowIndex: 0, colIndex: 0, useGridIndex: false },
  expressions: { gridRowIndex: `floor(${sqExpr} / 4)`, gridColIndex: `2 * mod(${sqExpr}, 4) + 1 - mod(floor(${sqExpr} / 4), 2)`,
    ...(lift ? { offsetZ: "$liftMm" } : {}) } });
const siloRim = (point, name, lift) => ({ id: id("mv"), type: "MoveL", name, pointName: point, ...(lift ? { expressions: { offsetZ: "$liftMm" } } : {}) });
const pickAt = (sq, what) => [cell(sq, `Above ${what}`, true), cell(sq, `Down onto ${what}`), call("Checkers Grip"), cell(sq, `Lift ${what}`, true)];
const placeAt = (sq, what) => [cell(sq, `Above ${what}`, true), cell(sq, `Down onto ${what}`), call("Checkers Release"), cell(sq, `Lift off ${what}`, true)];
// Put a captured checker on its colour's motorized stack: lower the stack one part (SiloAction
// Accept; catchable siloFull/siloNotHomed just drop it on top), then place at the taught rim pose.
const toStack = (axis, point, colour) => [
  tryStep(`Make room on the ${colour} stack`,
    [silo("Accept", axis, `Lower the ${colour} stack by one checker`)],
    [status(`The ${colour} stack is full or not homed — dropping the checker on top`, "Warning")],
    ["siloFull", "siloNotHomed"]),
  siloRim(point, `Above the ${colour} stack`, true),
  siloRim(point, `Down onto the ${colour} stack`),
  call("Checkers Release"),
  siloRim(point, `Up from the ${colour} stack`, true),
];
const park = () => siloRim(SILO_ROBOT_POINT, "Park clear of the camera (above my stack)", true);
// Silo steps (docs/aux-silo.md) and the crowning checker's pick at the robot silo's rim.
const silo = (siloAction, axis, name) => ({ id: id("silo"), type: "SiloAction", name, siloAction, auxDeviceId: SILO.device, auxAxisIndex: axis });
const siloPt = (name, lift) => siloRim(SILO_ROBOT_POINT, name, lift);
const tryStep = (name, trySteps, catchSteps, catchCodes) => ({ id: id("try"), type: "Try", name, trySteps, catchSteps, catchCodes,
  errorVariableName: "siloError" });
// Square + one checker height: the crown goes on top of the kinged piece.
const crownAt = (sq, name) => { const m = cell(sq, name); m.expressions.offsetZ = "$checkerMm"; return m; };
const crownRobot = () => tryStep("Crown my piece from my silo", [
  silo("Present", SILO.robotAxis, "Lift a black checker to the rim"),
  siloPt("Above my silo", true), siloPt("Down onto the top checker"), call("Checkers Grip"), siloPt("Lift the crown", true),
  cell("$toSq", "Above the new king", true), crownAt("$toSq", "Down onto the new king"), call("Checkers Release"),
  cell("$toSq", "Lift off the new king", true),
], [status("My silo could not give a checker ({$siloError}) — please crown my piece on square {$toSq}", "Warning")],
  ["siloEmpty", "siloNotHomed"]);
const crownHuman = () => [
  // Offer the human a white checker from their silo (no silo / empty: just ask).
  tryStep("Offer a white checker", [silo("Present", SILO.humanAxis, "Lift a white checker to the rim")], [], ["siloEmpty", "siloNotHomed"]),
  status("Crown your piece on square {$hTo}: stack a white checker on it (take the top one from your silo)", "Warning"),
];
const prepareSilos = () => [
  iff(["$siloRefill == 1"],
    [silo("Refill", SILO.robotAxis, "Black silo just loaded: full + home"), silo("Refill", SILO.humanAxis, "White silo just loaded: full + home")],
    [silo("Home", SILO.robotAxis, "Home the black silo"), silo("Home", SILO.humanAxis, "Home the white silo")]),
];
// The vision program's tools "white" and "black" (gridded 8x8) fill the two cell lists.
const look = () => ({ id: id("vis"), type: "RunVision", name: "Look at the board", waitTimeoutMs: 5000, ...VISION, outputFrame: "normalized",
  toolOutputs: [{ toolId: "white", value: "cells", variable: "whiteCells" }, { toolId: "black", value: "cells", variable: "blackCells" }] });
const detect = () => plugin("detectMove", "Compare the board with the game",
  { whiteCells: "$whiteCells", blackCells: "$blackCells", minCoverage: "$minCoverage", orientation: ORIENTATION, bothMeans: "white" },
  { from: "hFrom", to: "hTo", changed: "changed", moveText: "seenText" });

const v = (name, value, description, extra = {}) => ({ id: `ckg-var-${name}`, name, value, description, ...extra });
const num = (name, value, description, mon = false) => v(name, value, description, mon ? { displayOnMonitor: true } : {});
const str = (name, description) => v(name, 0, description, { isString: true, stringValue: "", displayOnMonitor: true });
const list = (name, elementType, description) => v(name, 0, description, { items: [], elementType });

function game() {
  const variables = [
    num("liftMm", 30, "Height above a square (and the stacks) to travel at (mm)"),
    num("checkerMm", 3.9, "Height of one checker (mm) — a crown is placed this far above the square"),
    num("siloRefill", 0, "1 = both silos were just loaded full: refill (count = capacity) instead of only homing them"),
    str("siloError", "Why a silo could not give a checker"),
    num("speed", 150, "Travel speed (mm/s)"),
    num("readMs", 700, "Pause between camera reads while waiting for your move (ms)"),
    num("stableReads", 3, "Your move must read the same this many times in a row (hand clear of the board)"),
    num("minCoverage", 0, "0 = use each vision tool's own pass/fail; else the min cell value for a piece"),
    num("maxPlies", 300, "Stop after this many moves"),
    list("whiteCells", "Record", "White pieces per board cell (camera): {row, col, index, value, passed}"),
    list("blackCells", "Record", "Black pieces per board cell (camera): {row, col, index, value, passed}"),
    list("captured", "Number", "Squares the robot just jumped — white checkers cleared to the white stack"),
    num("fromSq", -1, "Robot move: from square", true), num("toSq", -1, "Robot move: to square", true),
    v("kinged", 0, "The robot's piece was crowned", { isBoolean: true }),
    v("gameOver", 0, "Game over", { isBoolean: true, displayOnMonitor: true }),
    num("capSq", -1, "Captured square being cleared"),
    list("hCaptured", "Number", "Squares you just jumped — black checkers cleared to the black stack"),
    num("hCapSq", -1, "Your captured square being cleared"),
    num("hFrom", -1, "Your move as seen: from (-1 = none)", true), num("hTo", -1, "Your move as seen: to", true),
    num("changed", 0, "Squares that differ from the game", true),
    num("lastFrom", -1, "Previous read's from"), num("lastTo", -1, "Previous read's to"),
    num("seen", 0, "Reads in a row showing the same move", true),
    v("legal", 0, "Your move was legal", { isBoolean: true }),
    num("humanDone", 0, "Your move has been taken"),
    v("hKinged", 0, "Your piece was crowned", { isBoolean: true }),
    num("plies", 0, "Moves played", true),
    str("moveText", "Last move"), str("seenText", "What the camera sees"), str("winner", "Winner"),
  ];
  const robotTurn = [
    plugin("aiMove", "Robot picks its move", {}, { fromSquare: "fromSq", toSquare: "toSq", captured: "captured", kinged: "kinged", gameOver: "gameOver", moveText: "moveText", winner: "winner" }),
    iff(["$fromSq >= 0"], [
      status("Robot: {$moveText}"),
      { id: id("caps"), type: "Loop", name: "Clear each jumped white checker to the white stack", loopMode: "forEach",
        forEachVariableName: "captured", forEachValueVariableName: "capSq", loopSteps: [...pickAt("$capSq", "the jumped checker"), ...toStack(SILO.humanAxis, SILO_HUMAN_POINT, "white")] },
      ...pickAt("$fromSq", "my piece"),
      ...placeAt("$toSq", "its new square"),
      iff(["$kinged == 1"], [crownRobot()]),
      park(),
      setv("plies", "$plies + 1"),
    ]),
  ];
  const humanTurn = [
    setv("humanDone", "0"), setv("seen", "0"), setv("lastFrom", "-1"), setv("lastTo", "-1"),
    status("Your move (white). Slide your checker and take your hand off the board — leave any checker you jumped on the board; I'll clear it."),
    whileLoop("Watch the board until your move is clear", ["$humanDone == 0"], [
      wait(700, "$readMs"),
      look(),
      detect(),
      // Same legal move as the previous read → one more in a row; a new one → 1; none → 0.
      setv("seen", "($hFrom >= 0 and $hFrom == $lastFrom and $hTo == $lastTo) * $seen + ($hFrom >= 0)"),
      setv("lastFrom", "$hFrom"), setv("lastTo", "$hTo"),
      iff(["$seen >= $stableReads"], [
        plugin("applyMove", "Take your move", { from: "$hFrom", to: "$hTo" }, { legal: "legal", captured: "hCaptured", kinged: "hKinged", gameOver: "gameOver", moveText: "moveText", winner: "winner" }),
        setv("humanDone", "1"),
        setv("plies", "$plies + 1"),
        status("You: {$moveText}"),
        { id: id("hcaps"), type: "Loop", name: "Clear each checker you jumped to the black stack", loopMode: "forEach",
          forEachVariableName: "hCaptured", forEachValueVariableName: "hCapSq", loopSteps: [...pickAt("$hCapSq", "the checker you jumped"), ...toStack(SILO.robotAxis, SILO_ROBOT_POINT, "black")] },
        iff(["$hKinged == 1"], crownHuman()),
      ]),
    ]),
  ];
  const steps = [
    status("Checkers vs the camera: the robot plays black (rows 0-2) and moves first; you play white."),
    { id: id("spd"), type: "SetSpeedL", speed: 150, accel: 1000, decel: 1000, expressions: { speed: "$speed" } },
    call("Checkers Release"),
    park(),
    ...prepareSilos(),
    plugin("newGame", "New game, black first", { firstPlayer: "black" }, { gameOver: "gameOver", moveText: "moveText" }),
    setv("changed", "1"),
    whileLoop("Wait for the starting position", ["$changed > 0"], [
      look(), detect(),
      iff(["$changed > 0"], [status("Set up the board: {$changed} square(s) differ from the starting position", "Warning"), wait(1000, "$readMs")]),
    ]),
    status("Board ready."),
    whileLoop("Play", ["$gameOver == 0", "$plies < $maxPlies"], [
      ...robotTurn,
      iff(["$gameOver == 0"], humanTurn),
    ]),
    iff(["$gameOver == 1"], [status("Game over: {$winner} wins.")], [status("Stopped after {$plies} moves.", "Warning")]),
    park(),
  ];
  return { name: "Checkers Game (camera)", description: "End-to-end checkers vs a human: the robot (black) plays the plugin's moves on the taught 'Checkers Board' grid, clears every jumped checker to its colour's motorized stack (white to 'Checkers Silo Human', black to 'Checkers Silo Robot'), crowns its kings with a checker from its black stack, and waits for the camera to see your move — leave the pieces you jump on the board and the robot removes them — before replying.", steps, variables };
}

const routine = (name, on) => ({ name, description: `Checkers ${on ? "grip" : "release"} for the end effector — edit to suit your tool (output 1 by default).`,
  steps: [{ id: id("out"), type: "SetOutput", outputNumber: 1, outputValue: on }, wait(300)], variables: [] });

// The HTTP-server boilerplate: replace its TODO status placeholders with real moves.
function boilerplate(orig) {
  const p = JSON.parse(JSON.stringify(orig));
  const fix = steps => steps.flatMap(s => {
    if (s.loopSteps) s.loopSteps = fix(s.loopSteps);
    if (s.ifSteps) s.ifSteps = fix(s.ifSteps);
    if (s.elseSteps) s.elseSteps = fix(s.elseSteps);
    if (s.id === "chk003") // hops: pick at the start, place at the end (no need to touch the hop squares)
      return [...pickAt("$fromSq", "the moving piece"), ...placeAt("$toSq", "its new square")];
    if (s.id === "chk004") return [...pickAt("$remove[$cap]", "the jumped checker"), ...toStack(SILO.humanAxis, SILO_HUMAN_POINT, "white")];
    if (s.id === "chk006") return [status("Please crown the piece on square {$toSq}", "Warning")];
    return [s];
  });
  p.steps = fix(p.steps);
  p.variables = p.variables.filter(x => x.name !== "hop");   // the hop loop is gone
  if (!p.variables.some(x => x.name === "liftMm")) p.variables.push(num("liftMm", 30, "Height above a square to travel at (mm)"));
  p.steps.push(park());
  p.description = "Plays against the CheckersServer HTTP API; moves pieces on the taught 'Checkers Board' grid and clears jumped checkers to the motorized white stack ('Checkers Silo Human').";
  return p;
}

const mode = process.argv[2] ?? "build";
if (mode === "json") { process.stdout.write(JSON.stringify({ game: game(), grip: routine("Checkers Grip", true), release: routine("Checkers Release", false) })); process.exit(0); }
const c = new Ctrl(); await c.open();
const J = x => typeof x === "string" ? JSON.parse(x) : x;
const progs = J((await c.cmd("GetBuiltPrograms")).programs);
const g = game();
const orig = progs.find(p => p.name === "Checkers API Boilerplate");
// The boilerplate is converted once; after that only the game program is (re)built.
const converted = !orig || JSON.stringify(orig).includes("ckg-");
if (converted) console.log("Checkers API Boilerplate: already converted (or missing) — left as is");
const bp = converted ? null : boilerplate(orig);
const r1 = routine("Checkers Grip", true), r2 = routine("Checkers Release", false);
if (!converted) writeFileSync("checkers-boilerplate-before.json", JSON.stringify(orig, null, 1));
if (mode === "save") {
  if (!J((await c.cmd("GetGrids")).grids).some(x => x.id === GRID_ID))
    console.log("grid:", (await c.cmd("SaveGrid", { id: GRID_ID, name: GRID_NAME, description: GRID_DESC, basePointName: ORIGIN, rowOffsetX: 0, rowOffsetY: 0, rowOffsetZ: 0, colOffsetX: 0, colOffsetY: 0, colOffsetZ: 0, rowCount: 8, colCount: 8, rotation: 0 })).ok);
  for (const r of [r1, r2]) if (!progs.some(p => p.name === r.name)) console.log("routine", r.name, (await c.cmd("SaveBuiltProgram", r)).ok);
}
for (const p of [g, bp].filter(Boolean)) {
  const val = await c.cmd("ValidateBuiltProgram", { program: p });
  const probs = (val.problems ?? []).map(x => `${x.severity}:${x.code}:${x.message}`);
  console.log(p.name, "steps", p.steps.length, "problems:", [...new Set(probs)].join(" | ") || "none");
  if (mode === "save") console.log("  saved:", (await c.cmd("SaveBuiltProgram", p)).ok);
}
c.close();
