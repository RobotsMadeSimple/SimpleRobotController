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
import { writeFileSync } from "node:fs";
import { Ctrl } from "../../../../live-tests/harness.mjs";

const GRID_ID = "checkers-board", GRID_NAME = "Checkers Board", ORIGIN = "Checkers Board origin";
const DISCARD = "Checkers Discard";
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
const discard = (name, lift) => ({ id: id("mv"), type: "MoveL", name, pointName: DISCARD, ...(lift ? { expressions: { offsetZ: "$liftMm" } } : {}) });
const pickAt = (sq, what) => [cell(sq, `Above ${what}`, true), cell(sq, `Down onto ${what}`), call("Checkers Grip"), cell(sq, `Lift ${what}`, true)];
const placeAt = (sq, what) => [cell(sq, `Above ${what}`, true), cell(sq, `Down onto ${what}`), call("Checkers Release"), cell(sq, `Lift off ${what}`, true)];
const toDiscard = () => [discard("Above the discard pile", true), discard("Down to the discard pile"), call("Checkers Release"), discard("Up from the discard pile", true)];
const park = () => discard("Park clear of the camera (above the discard pile)", true);
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
    num("liftMm", 30, "Height above a square (and the discard pile) to travel at (mm)"),
    num("speed", 150, "Travel speed (mm/s)"),
    num("readMs", 700, "Pause between camera reads while waiting for your move (ms)"),
    num("stableReads", 3, "Your move must read the same this many times in a row (hand clear of the board)"),
    num("minCoverage", 0, "0 = use each vision tool's own pass/fail; else the min cell value for a piece"),
    num("maxPlies", 300, "Stop after this many moves"),
    list("whiteCells", "Record", "White pieces per board cell (camera): {row, col, index, value, passed}"),
    list("blackCells", "Record", "Black pieces per board cell (camera): {row, col, index, value, passed}"),
    list("captured", "Number", "Squares the robot just jumped (cleared to the discard pile)"),
    num("fromSq", -1, "Robot move: from square", true), num("toSq", -1, "Robot move: to square", true),
    v("kinged", 0, "The robot's piece was crowned", { isBoolean: true }),
    v("gameOver", 0, "Game over", { isBoolean: true, displayOnMonitor: true }),
    num("capSq", -1, "Captured square being cleared"),
    num("hFrom", -1, "Your move as seen: from (-1 = none)", true), num("hTo", -1, "Your move as seen: to", true),
    num("changed", 0, "Squares that differ from the game", true),
    num("lastFrom", -1, "Previous read's from"), num("lastTo", -1, "Previous read's to"),
    num("seen", 0, "Reads in a row showing the same move", true),
    v("legal", 0, "Your move was legal", { isBoolean: true }),
    num("humanDone", 0, "Your move has been taken"),
    num("plies", 0, "Moves played", true),
    str("moveText", "Last move"), str("seenText", "What the camera sees"), str("winner", "Winner"),
  ];
  const robotTurn = [
    plugin("aiMove", "Robot picks its move", {}, { fromSquare: "fromSq", toSquare: "toSq", captured: "captured", kinged: "kinged", gameOver: "gameOver", moveText: "moveText", winner: "winner" }),
    iff(["$fromSq >= 0"], [
      status("Robot: {$moveText}"),
      { id: id("caps"), type: "Loop", name: "Clear each jumped piece to the discard pile", loopMode: "forEach",
        forEachVariableName: "captured", forEachValueVariableName: "capSq", loopSteps: [...pickAt("$capSq", "the jumped piece"), ...toDiscard()] },
      ...pickAt("$fromSq", "my piece"),
      ...placeAt("$toSq", "its new square"),
      iff(["$kinged == 1"], [status("Please crown my piece on square {$toSq} (stack a black piece on it)", "Warning")]),
      park(),
      setv("plies", "$plies + 1"),
    ]),
  ];
  const humanTurn = [
    setv("humanDone", "0"), setv("seen", "0"), setv("lastFrom", "-1"), setv("lastTo", "-1"),
    status("Your move (white). Take your hand off the board when done; remove any piece you jumped."),
    whileLoop("Watch the board until your move is clear", ["$humanDone == 0"], [
      wait(700, "$readMs"),
      look(),
      detect(),
      // Same legal move as the previous read → one more in a row; a new one → 1; none → 0.
      setv("seen", "($hFrom >= 0 and $hFrom == $lastFrom and $hTo == $lastTo) * $seen + ($hFrom >= 0)"),
      setv("lastFrom", "$hFrom"), setv("lastTo", "$hTo"),
      iff(["$seen >= $stableReads"], [
        plugin("applyMove", "Take your move", { from: "$hFrom", to: "$hTo" }, { legal: "legal", gameOver: "gameOver", moveText: "moveText", winner: "winner" }),
        setv("humanDone", "1"),
        setv("plies", "$plies + 1"),
        status("You: {$moveText}"),
      ]),
    ]),
  ];
  const steps = [
    status("Checkers vs the camera: the robot plays black (rows 0-2) and moves first; you play white."),
    { id: id("spd"), type: "SetSpeedL", speed: 150, accel: 1000, decel: 1000, expressions: { speed: "$speed" } },
    call("Checkers Release"),
    park(),
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
  return { name: "Checkers Game (camera)", description: "End-to-end checkers vs a human: the robot (black) plays the plugin's moves on the taught 'Checkers Board' grid, clears jumped pieces to 'Checkers Discard', and waits for the camera to see your move (stable for a few reads) before replying.", steps, variables };
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
    if (s.id === "chk004") return [...pickAt("$remove[$cap]", "the jumped piece"), ...toDiscard()];
    if (s.id === "chk006") return [status("Please crown the piece on square {$toSq}", "Warning")];
    return [s];
  });
  p.steps = fix(p.steps);
  p.variables = p.variables.filter(x => x.name !== "hop");   // the hop loop is gone
  if (!p.variables.some(x => x.name === "liftMm")) p.variables.push(num("liftMm", 30, "Height above a square to travel at (mm)"));
  p.steps.push(park());
  p.description = "Plays against the CheckersServer HTTP API; moves pieces on the taught 'Checkers Board' grid and clears jumped pieces to 'Checkers Discard'.";
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
    console.log("grid:", (await c.cmd("SaveGrid", { id: GRID_ID, name: GRID_NAME, basePointName: ORIGIN, rowOffsetX: 0, rowOffsetY: 0, rowOffsetZ: 0, colOffsetX: 0, colOffsetY: 0, colOffsetZ: 0, rowCount: 8, colCount: 8, rotation: 0 })).ok);
  for (const r of [r1, r2]) if (!progs.some(p => p.name === r.name)) console.log("routine", r.name, (await c.cmd("SaveBuiltProgram", r)).ok);
}
for (const p of [g, bp].filter(Boolean)) {
  const val = await c.cmd("ValidateBuiltProgram", { program: p });
  const probs = (val.problems ?? []).map(x => `${x.severity}:${x.code}:${x.message}`);
  console.log(p.name, "steps", p.steps.length, "problems:", [...new Set(probs)].join(" | ") || "none");
  if (mode === "save") console.log("  saved:", (await c.cmd("SaveBuiltProgram", p)).ok);
}
c.close();
