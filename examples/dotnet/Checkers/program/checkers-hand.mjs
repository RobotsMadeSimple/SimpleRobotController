// Saves the vision program "Checkers Hand Watch" (id checkers-hand): a dedicated detector that
// tells whether a hand (or anything that is not the board) is over the playing area, so the
// Checkers Vision Live loop can hold the board while a hand is in the way instead of guessing from
// piece counts. Built only from generic vision tools (docs/vision-tools.md), over the SAME board
// quad as checkers-vision, so it needs no separate region teaching:
//
//  - layer "gray" (grey channel) and the raw colour camera
//  - edgePixels "edges": edge density over the whole board. The board is busy (square grid + piece
//    rings), so a smooth hand laid over it DROPS the density.
//  - intensity  "flat":  grey stdDev over the whole board. The board is high-contrast
//    (black/white squares, cream/black pieces); a uniform hand over it DROPS the stdDev.
//  - pixelCount "skin":  % of the board that is skin-coloured (colour image). A clear board is
//    ~0; a hand RAISES it.
//
// The live loop reads whichever of these separates a hand best for this camera (set when the loop
// is built) — it does not have to use all three. Redraw/retune when the camera moves, same as the
// board program.
//   node checkers-hand.mjs json [cameraId]                  print the program JSON
//   CTRL=host:port node checkers-hand.mjs save [cameraId]   save it (camera: the given id, else the
//                        saved hand program's, else the checkers-vision program's, else the first camera)
import { Ctrl } from "../../../../live-tests/harness.mjs";

const ID = "checkers-hand";
const NAME = "Checkers Hand Watch (camera)";

// The board playing area, normalized — the same quad checkers-vision's "white"/"black" grids use.
// Keep in step with checkers-vision.json if that quad is re-drawn for a new camera position.
const BOARD_QUAD = [
  { x: 0.36531, y: 0.40278 },
  { x: 0.65547, y: 0.40056 },
  { x: 0.94000, y: 0.85917 },
  { x: 0.11602, y: 0.85611 },
];
const region = () => ({ shape: "quad", points: BOARD_QUAD.map(p => ({ ...p })) });
const whole = { rows: 1, cols: 1, inset: 0.08 }; // one cell over the area, frame trimmed off

export function visionProgram(cameraId = "") {
  return {
    id: ID, name: NAME, cameraId,
    description: "Is a hand over the board? Edge density, grey stdDev and skin-colour % over the " +
      "whole playing area; a hand drops the first two and raises the last. Used by Checkers Vision Live.",
    layers: [
      { id: "gray", name: "Gray", input: "camera", filters: [{ id: "g1", type: "channel", enabled: true, channel: "gray" }] },
    ],
    tools: [
      { id: "edges", name: "Edge density", type: "edgePixels", enabled: true, input: "gray",
        region: region(), grid: { ...whole }, includeInPass: false,
        params: { cannyLow: 50, cannyHigh: 150 } },
      { id: "flat", name: "Grey stdDev", type: "intensity", enabled: true, input: "gray",
        regionFrom: "edges", grid: { ...whole }, includeInPass: false,
        params: { statistic: "stdDev" } },
      { id: "skin", name: "Skin colour %", type: "pixelCount", enabled: true, input: "camera",
        regionFrom: "edges", grid: { ...whole }, includeInPass: false,
        params: { mode: "color", colors: [
          { r: 210, g: 150, b: 120, tolerance: 28 },
          { r: 165, g: 110, b: 95,  tolerance: 28 },
        ] } },
    ],
  };
}

const args = process.argv.slice(2);
const [mode = "json", cameraArg = ""] = args.filter(a => !a.startsWith("--"));

if (mode === "json") {
  console.log(JSON.stringify(visionProgram(cameraArg), null, 2));
} else if (mode === "save") {
  if (!process.env.CTRL) { console.error("set CTRL=host:port (the controller to save to)"); process.exit(2); }
  const c = new Ctrl(); await c.open();
  const J = x => typeof x === "string" ? JSON.parse(x) : x;
  let cameraId = cameraArg;
  if (!cameraId) {
    const progs = J((await c.cmd("GetVisionPrograms")).programs) ?? [];
    cameraId = (progs.find(p => p.id === ID) ?? progs.find(p => p.id === "checkers-vision"))?.cameraId;
  }
  if (!cameraId) cameraId = (J((await c.cmd("GetCameras")).cameras) ?? [])[0]?.id ?? "";
  await c.cmd("SaveVisionProgram", visionProgram(cameraId));
  const back = (J((await c.cmd("GetVisionPrograms")).programs) ?? []).find(p => p.id === ID);
  c.close();
  const tools = back?.tools ?? [];
  if (tools.length !== 3) {
    console.log("SAVED BUT THE CONTROLLER DID NOT KEEP ALL 3 TOOLS — it may be running a build without these vision tools; update it and save again");
    process.exit(1);
  }
  console.log(`saved "${NAME}" (${ID}) on camera "${cameraId || "(none)"}" with tools ${tools.map(t => t.id).join(", ")}`);
} else {
  console.error("usage: node checkers-hand.mjs json|save [cameraId]"); process.exit(2);
}
