// Saves the vision program "Checkers Board (camera)" (id checkers-vision) for a camera looking
// at the board from the side, with black pieces on black squares. Built only from generic
// vision tools (docs/vision-tools.md). Two versions, same id and outputs (checkers-game.mjs and
// the detectMove step read tools "white" and "black", each an 8x8 grid of cells):
//
//  fixed (default, checkers-vision.json) — the grid stays where you put it:
//  - layers "Gray" and "Fine detail" (gray minus a 17 px blur: slow shading drops out)
//  - pixelCount "white" (cream pieces): a skewed (four-corner) 8x8 grid on the squares, no fixture
//  - intensity "black" (stdDev of fine detail: black pieces' rings): regionFrom "white", so it
//    uses the same four corners — set them once, on "White pieces"
//  Set the grid in the app: Vision editor → White pieces → Area → Draw, drag the four corners
//  onto the corners of the squares (1,1 = row 1, column 1), Save. Redo it when the camera moves.
//
//  tracked (--tracked, checkers-vision-tracked.json) — the grid follows the board:
//  - findLine "top" / "right" / "left": the board's outer edges (white table → black frame);
//    findLine "bottom": the squares' bottom edge (the board's own is often cut off)
//  - makeFixture "board" (fourLines): the four corners, a perspective fixture
//  - "white" and "black" as above, fixtured to "board"
//
// Both are checked on two real photos by RobotControl.Tests/VisionCheckersRealPhotoTests. Their
// regions are drawn on the default camera view (the second photo).
//   node checkers-vision.mjs json [cameraId] [--tracked]                  print the program JSON
//   CTRL=host:port node checkers-vision.mjs save [cameraId] [--tracked]   save it (camera: the given id,
//                       else the saved program's, else the old "Checkers Board" program's, else the first camera)
import { readFileSync } from "node:fs";
import { Ctrl } from "../../../../live-tests/harness.mjs";

const args = process.argv.slice(2);
const TRACKED = args.includes("--tracked");
const [mode = "json", cameraArg = ""] = args.filter(a => !a.startsWith("--"));
const FILE = TRACKED ? "./checkers-vision-tracked.json" : "./checkers-vision.json";
const PROGRAM = JSON.parse(readFileSync(new URL(FILE, import.meta.url), "utf8"));
const ID = PROGRAM.id, NAME = PROGRAM.name;

export function visionProgram(cameraId = "") {
  return { ...structuredClone(PROGRAM), cameraId };
}

if (mode === "json") {
  console.log(JSON.stringify(visionProgram(cameraArg), null, 2));
} else if (mode === "save") {
  if (!process.env.CTRL) { console.error("set CTRL=host:port (the controller to save to)"); process.exit(2); }
  const c = new Ctrl(); await c.open();
  const J = x => typeof x === "string" ? JSON.parse(x) : x;
  let cameraId = cameraArg;
  if (!cameraId) {
    const progs = J((await c.cmd("GetVisionPrograms")).programs) ?? [];
    cameraId = (progs.find(p => p.id === ID) ?? progs.find(p => p.name === "Checkers Board"))?.cameraId;
  }
  if (!cameraId) cameraId = (J((await c.cmd("GetCameras")).cameras) ?? [])[0]?.id ?? "";
  await c.cmd("SaveVisionProgram", visionProgram(cameraId));
  const back = (J((await c.cmd("GetVisionPrograms")).programs) ?? []).find(p => p.id === ID);
  c.close();
  const tools = back?.tools ?? [];
  if (tools.length !== PROGRAM.tools.length || (TRACKED && !tools.some(t => t.type === "makeFixture"))) {
    console.log("SAVED BUT THE CONTROLLER DROPPED THE TOOLS — it is running a build without building-block vision tools; update it and save again");
    process.exit(1);
  }
  console.log(`saved "${NAME}" (${ID}, ${TRACKED ? "tracked" : "fixed grid"}) on camera "${cameraId || "(none)"}"`);
  // An older controller ignores regionFrom: "black" then keeps its own copy of the same corners.
  const shared = PROGRAM.tools.filter(t => t.regionFrom);
  if (shared.some(t => !tools.find(x => x.id === t.id)?.regionFrom))
    console.log("note: this controller does not know regionFrom yet — each grid keeps its own corners (move both, or update the controller)");
} else {
  console.error("usage: node checkers-vision.mjs json|save [cameraId] [--tracked]"); process.exit(2);
}
