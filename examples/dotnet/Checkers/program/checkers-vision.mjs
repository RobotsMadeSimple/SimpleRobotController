// Builds the vision program "Checkers Board (camera)" (id checkers-vision) for a camera looking
// at the board from the side, with black pieces on black squares:
//  - zone "Board": a Skewed (perspective) 8x8 grid over the 64 squares — row 0 = black's side,
//    aligned to the "Squares" frame below
//  - frame "Squares" (method checker, on the Board zone): fits the 8x8 checkerboard to the
//    squares themselves each frame, so the grid snaps onto them and follows a nudged camera
//    (no reference to capture: a checker frame on its own zone always snaps that zone)
//  - colour "white": cream pieces, per cell
//  - texture "black": black pieces by their ring detail, per cell (colour can't separate black
//    pieces from black squares). detectMove's bothMeans=white handles white pieces passing it too.
// Tuned on two real photos (1280 and 1920 px: all 24 pieces read on both, grid edge ratio 13–18).
// The default zone is where the board sat in the second photo; redraw it roughly over the
// squares in the app (within ~a fifth of a square of each corner) and press "Snap zone to squares".
//   node checkers-vision.mjs json [cameraId]             print the program JSON
//   CTRL=host:port node checkers-vision.mjs save [cameraId]   save it (camera: the given id, else
//                                                          the old "Checkers Board" program's, else the first camera)
import { Ctrl } from "../../../../live-tests/harness.mjs";

const ID = "checkers-vision", NAME = "Checkers Board (camera)";
const ZONE = [[0.3653, 0.4028], [0.6555, 0.4006], [0.9400, 0.8592], [0.1160, 0.8561]];   // TL TR BR BL

export function visionProgram(cameraId = "") {
  return {
    id: ID, name: NAME, cameraId,
    description: "Angled view of the checkers board: checker-grid alignment, colour for white pieces, texture for black pieces.",
    zones: [{ id: "board", name: "Board", grid: { rows: 8, cols: 8 }, alignTo: "squares",
              geometry: { shape: "Quad", x: 0, y: 0, width: 1, height: 1, rotation: 0, cx: 0.5, cy: 0.5, radius: 0.25, points: ZONE } }],
    inspections: [], polygonInspections: [], arucoInspections: [], lineInspections: [], barcodeInspections: [],
    cornerInspections: [], edgeInspections: [],
    frameInspections: [{ id: "squares", name: "Squares", enabled: true, zoneId: "board", method: "checker",
                         gridRows: 8, gridCols: 8, minEdgeRatio: 4, referenceCorners: null }],
    colorInspections: [{ id: "white", name: "White pieces", enabled: true, zoneId: "board",
                         colors: [{ id: "cream", r: 232, g: 230, b: 200, tolerance: 20 }], minCoverage: 30, maxCoverage: null }],
    textureInspections: [{ id: "black", name: "Black pieces", enabled: true, zoneId: "board", cellInset: 25, minScore: 4, maxScore: null }],
    inspectionOrder: ["squares", "white", "black"],
  };
}

const mode = process.argv[2] || "json";
if (mode === "json") {
  console.log(JSON.stringify(visionProgram(process.argv[3] || ""), null, 2));
} else if (mode === "save") {
  const c = new Ctrl(); await c.open();
  const J = x => typeof x === "string" ? JSON.parse(x) : x;
  let cameraId = process.argv[3];
  if (!cameraId) {
    const progs = J((await c.cmd("GetVisionPrograms")).programs) ?? [];
    cameraId = progs.find(p => p.name === "Checkers Board")?.cameraId;
  }
  if (!cameraId) cameraId = (J((await c.cmd("GetCameras")).cameras) ?? [])[0]?.id ?? "";
  const prog = visionProgram(cameraId);
  await c.cmd("SaveVisionProgram", prog);
  const back = (J((await c.cmd("GetVisionPrograms")).programs) ?? []).find(p => p.id === ID);
  const kept = back && back.textureInspections?.length === 1 && back.frameInspections?.[0]?.method === "checker"
    && back.frameInspections[0].gridRows === 8;
  console.log(kept ? `saved "${NAME}" (${ID}) on camera "${cameraId || "(none)"}"`
                   : "SAVED BUT THE CONTROLLER DROPPED THE NEW FIELDS — it is running a build without texture/checker support; update it and save again");
  c.close();
  if (!kept) process.exit(1);
} else {
  console.error("usage: node checkers-vision.mjs json|save [cameraId]"); process.exit(2);
}
