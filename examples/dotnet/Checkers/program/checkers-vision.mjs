// Saves the vision program "Checkers Board (camera)" (id checkers-vision, defined in
// checkers-vision.json) for a camera looking at the board from the side, with black pieces
// on black squares. Built only from generic vision tools (docs/vision-tools.md):
//  - layers "Gray" and "Fine detail" (gray minus a 17 px blur: slow shading drops out)
//  - findLine "top" / "right" / "left": the board's outer edges (white table → black frame);
//    findLine "bottom": the squares' bottom edge (the board's own is often cut off)
//  - makeFixture "board" (fourLines): the four corners, a perspective fixture
//  - pixelCount "white" (cream pieces) and intensity "black" (stdDev of fine detail: black
//    pieces' rings), both an 8x8 grid on the squares, fixtured to "board"
// Checked on two real photos by RobotControl.Tests/VisionCheckersRealPhotoTests: all 24
// pieces read on both, and the grid carried from one camera view to the other.
// The regions are drawn on the default camera view (the second photo). For another view,
// redraw the four edge regions and the squares region in the app, then Set reference on Board.
//   node checkers-vision.mjs json [cameraId]                  print the program JSON
//   CTRL=host:port node checkers-vision.mjs save [cameraId]   save it (camera: the given id, else
//                                                             the old "Checkers Board" program's, else the first camera)
import { readFileSync } from "node:fs";
import { Ctrl } from "../../../../live-tests/harness.mjs";

const PROGRAM = JSON.parse(readFileSync(new URL("./checkers-vision.json", import.meta.url), "utf8"));
const ID = PROGRAM.id, NAME = PROGRAM.name;

export function visionProgram(cameraId = "") {
  return { ...structuredClone(PROGRAM), cameraId };
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
    cameraId = progs.find(p => p.id === ID || p.name === "Checkers Board")?.cameraId;
  }
  if (!cameraId) cameraId = (J((await c.cmd("GetCameras")).cameras) ?? [])[0]?.id ?? "";
  await c.cmd("SaveVisionProgram", visionProgram(cameraId));
  const back = (J((await c.cmd("GetVisionPrograms")).programs) ?? []).find(p => p.id === ID);
  const kept = back && back.tools?.length === PROGRAM.tools.length && back.tools.some(t => t.type === "makeFixture");
  console.log(kept ? `saved "${NAME}" (${ID}) on camera "${cameraId || "(none)"}"`
                   : "SAVED BUT THE CONTROLLER DROPPED THE TOOLS — it is running a build without building-block vision tools; update it and save again");
  c.close();
  if (!kept) process.exit(1);
} else {
  console.error("usage: node checkers-vision.mjs json|save [cameraId]"); process.exit(2);
}
