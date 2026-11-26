// Builds "Checkers Vision Live": a vision test loop with no motion. It keeps running the board
// vision program and loads what the camera sees into the checkers plugin (setBoard from camera
// cells), so the plugin's board GUI shows the camera's reading live. Kings read as men.
//
// A dedicated vision program (checkers-hand, built by checkers-hand.mjs) decides whether a hand is
// over the board, from whole-board signals that a hand changes a lot: edge density and grey stdDev
// (a smooth hand over the busy board drops both) and skin-colour % (a hand raises it). While a hand
// is seen the board HOLDS its last reading (the setBoard step is skipped); the "Board clear" lamp
// and the hold are both driven by that one hand signal, not by the piece counts.
// Its display page shows the plugin board beside the camera frame, the piece counts and Run/Stop.
//   node checkers-vision-live.mjs json [visionProgramId] [orientation]   print the program JSON
//   CTRL=host:port node checkers-vision-live.mjs [build|save] [visionProgramId] [orientation]
// orientation = where black's side is in the camera image (as in checkers-game.mjs).
import { Ctrl } from "../../../../live-tests/harness.mjs";

const VISION = { visionProgramId: process.argv[3] || "checkers-vision", visionProgramName: "Checkers Board (camera)" };
const HAND = { visionProgramId: "checkers-hand", visionProgramName: "Checkers Hand Watch (camera)" };
const ORIENTATION = process.argv[4] || "rotated90";
const NAME = "Checkers Vision Live";

let n = 0; const id = p => `ckl-${p}-${++n}`;
const v = (name, value, description, extra = {}) => ({ id: `ckl-var-${name}`, name, value, description, ...extra });
const num = (name, value, description, mon = false) => v(name, value, description, mon ? { displayOnMonitor: true } : {});
const list = (name, elementType, description) => v(name, 0, description, { items: [], elementType });
const setv = (variableName, variableExpr) => ({ id: id("set"), type: "SetVariable", variableName, variableExpr });
const cond = (...exprs) => ({ combinator: "ALL", items: exprs.map(e => ({ id: id("c"), expr: e })) });
const iff = (exprs, ifSteps, elseSteps = []) => ({ id: id("if"), type: "IfCondition", condition: cond(...exprs), ifSteps, elseSteps });

// The display page: a 12-column widget grid (the app compiles it to HTML when the page opens).
const widget = (wid, type, x, y, w, h, config) => ({ id: `ckl-w-${wid}`, type, x, y, w, h, config });
const displayPage = (cameraId) => ({
  cols: 12, rowHeight: 44, theme: "light",
  widgets: [
    // Header strip: run state, then the one primary action (Run) and Stop. No title widget —
    // the app's header already names the program.
    widget("state", "status", 0, 0, 8, 2, {}),
    widget("run", "button", 8, 0, 2, 2, { label: "Run", action: "run", inputs: "" }),
    widget("stop", "button", 10, 0, 2, 2, { label: "Stop", action: "stop" }),
    // Hero: the board as the plugin sees it, drawn turned like the camera sees it (the board
    // page's ui param) so the two line up side by side.
    widget("board", "plugin", 0, 2, 6, 16, { pluginId: "checkers", entry: "ui/index.html", title: "Checkers board", pluginName: "Checkers Opponent",
      params: { orientation: ORIENTATION }, paramsText: "" }),
    widget("camera", "camera", 6, 2, 6, 8, { cameraId, label: "Camera", fit: "cover" }),
    widget("white", "value", 6, 10, 3, 4, { variable: "$whiteSeen", label: "White seen", unit: "/ 12" }),
    widget("black", "value", 9, 10, 3, 4, { variable: "$blackSeen", label: "Black seen", unit: "/ 12" }),
    // The board only updates once nothing is over it (the hand detector reads clear).
    // The lamp lights in the "ok" colour, so it reads "clear" (lit) rather than "hand" (lit).
    widget("hand", "boolean", 6, 14, 3, 4, { variable: "$handSeen == 0", label: "Board clear" }),
    widget("reads", "value", 9, 14, 3, 4, { variable: "$reads", label: "Frames read" }),
  ],
});

function program(cameraId) {
  const variables = [
    num("readMs", 300, "Pause between camera reads (ms)"),
    num("minCoverage", 0, "0 = use each vision tool's own pass/fail; else the min cell value for a piece"),
    num("settleReads", 3, "Reads in a row that must agree before the board updates (a moving hand never does)"),
    num("maxStray", -1, "Off (-1): the board hold is driven by the hand detector, not by the plugin's own light-square count"),
    // Hand detector (dedicated checkers-hand vision program). Primary signal is skin-colour % over
    // the board: ~0 on a clear board, higher with a hand in the way. Tune handSkinMin after watching
    // the live edgeDensity / flatStdDev / skinPct values with the board clear, then with a hand over it.
    num("handSkinMin", 8, "Skin-colour % over the board above which a hand counts as present (tune live)"),
    num("edgeDensity", 0, "Hand watch: edge density % over the board (a hand lowers it)", true),
    num("flatStdDev", 0, "Hand watch: grey stdDev over the board (a hand lowers it)", true),
    num("skinPct", 0, "Hand watch: skin-colour % over the board (a hand raises it)", true),
    num("handSeen", 0, "1 while a hand is over the board (the board holds its last reading)", true),
    list("whiteCells", "Record", "White pieces per board cell (camera): {row, col, index, value, passed}"),
    list("blackCells", "Record", "Black pieces per board cell (camera): {row, col, index, value, passed}"),
    num("whiteSeen", 0, "White pieces the camera sees", true),
    num("blackSeen", 0, "Black pieces the camera sees", true),
    num("reads", 0, "Camera reads so far", true),
    v("camera", 0, "The camera frame of the latest read", { isImage: true, displayOnMonitor: true }),
  ];
  const look = { id: id("vis"), type: "RunVision", name: "Look at the board", waitTimeoutMs: 5000, ...VISION, outputFrame: "normalized",
    toolOutputs: [{ toolId: "white", value: "cells", variable: "whiteCells" }, { toolId: "black", value: "cells", variable: "blackCells" }] };
  const lookHand = { id: id("hand"), type: "RunVision", name: "Is a hand over the board?", waitTimeoutMs: 5000, ...HAND, outputFrame: "normalized",
    toolOutputs: [
      { toolId: "edges", value: "density", variable: "edgeDensity" },
      { toolId: "flat", value: "stdDev", variable: "flatStdDev" },
      { toolId: "skin", value: "percent", variable: "skinPct" },
    ] };
  // One place decides "hand present": edit this expression (and handSkinMin) to use whichever of the
  // three signals separates best for this camera.
  const decideHand = iff(["$skinPct > $handSkinMin"], [setv("handSeen", "1")], [setv("handSeen", "0")]);
  const snap = { id: id("cap"), type: "CaptureImage", name: "Keep the frame for the display page", captureImageVariableName: "camera", captureImageCameraId: cameraId };
  const show = { id: id("pl"), type: "Plugin", name: "Show it on the plugin board", pluginId: "checkers", pluginStepId: "setBoard",
    pluginParams: { whiteCells: "$whiteCells", blackCells: "$blackCells", minCoverage: "$minCoverage", orientation: ORIENTATION, bothMeans: "white", turn: "black",
      settleReads: "$settleReads", maxStrayCells: "$maxStray" },
    pluginOutputs: [{ key: "whiteCount", variableName: "whiteSeen" }, { key: "blackCount", variableName: "blackSeen" }] };
  // Only load the reading onto the board when no hand is over it — otherwise the board holds.
  const showIfClear = iff(["$handSeen == 0"], [show]);
  const steps = [{
    id: id("loop"), type: "Loop", name: "Keep reading the board", loopMode: "while",
    loopWhileCondition: { combinator: "ALL", items: [{ id: id("c"), expr: "1 == 1" }] },
    loopSteps: [look, lookHand, decideHand, snap, showIfClear,
      { id: id("set"), type: "SetVariable", variableName: "reads", variableExpr: "$reads + 1" },
      { id: id("wait"), type: "Wait", waitMs: 300, expressions: { waitMs: "$readMs" } }],
  }];
  return {
    name: NAME,
    description: "Vision test, no motion: reads the board with the camera over and over and shows the reading on the checkers plugin's board (kings read as men). A dedicated hand-watch vision program holds the board while a hand is over it. Open its display page to watch; Stop when done.",
    steps, variables, displayPageMode: "widgets", displayPageLayout: displayPage(cameraId),
  };
}

const mode = process.argv[2] ?? "build";
if (mode === "json") { process.stdout.write(JSON.stringify(program(""))); process.exit(0); }
const c = new Ctrl(); await c.open();
const J = x => typeof x === "string" ? JSON.parse(x) : x;
const vision = J((await c.cmd("GetVisionPrograms")).programs).find(x => x.id === VISION.visionProgramId);
const p = program(vision?.cameraId ?? "");
const val = await c.cmd("ValidateBuiltProgram", { program: p });
const probs = (val.problems ?? []).map(x => `${x.severity}:${x.code}:${x.message}`);
console.log(p.name, `(camera "${vision?.cameraId ?? ""}")`, "problems:", [...new Set(probs)].join(" | ") || "none");
if (mode === "save") {
  const old = J((await c.cmd("GetBuiltPrograms")).programs).find(x => x.name === NAME);
  console.log("  saved:", (await c.cmd("SaveBuiltProgram", old ? { ...p, id: old.id } : p)).ok);
}
c.close();
