// Draws the manual's diagrams into docs/manual/figures/*.svg and lists them in manifest.json.
//   node docs/manual/tools/figures.mjs
// Plain SVG only — rect, line, polyline, polygon, path, circle, text — with presentation
// attributes (no <style>, no CSS classes, no foreignObject, no markers, no external fonts), so the
// same files render in Chromium (PDF) and in react-native-svg (the app's docs viewer).
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const OUT = path.join(ROOT, "figures");
fs.mkdirSync(OUT, { recursive: true });

// ── Palette and primitives ─────────────────────────────────────────────────
const C = {
  ink: "#1e293b", sub: "#475569", mute: "#94a3b8", line: "#cbd5e1", grid: "#e2e8f0", bg: "#ffffff", panel: "#f8fafc",
  blue: "#2563eb", blueL: "#dbeafe", blueXL: "#eff6ff",
  green: "#16a34a", greenL: "#dcfce7", orange: "#ea580c", orangeL: "#ffedd5",
  red: "#dc2626", redL: "#fee2e2", purple: "#7c3aed", purpleL: "#ede9fe", teal: "#0d9488", tealL: "#ccfbf1",
  amber: "#d97706", amberL: "#fef3c7",
};
const FONT = "Helvetica, Arial, sans-serif";
const MONO = "Menlo, Consolas, monospace";
const esc = s => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
const n = v => Math.round(v * 10) / 10;

function text(x, y, s, o = {}) {
  const lines = String(s).split("\n");
  const size = o.size ?? 13, lh = o.lh ?? size * 1.3;
  const y0 = o.middle ? y - ((lines.length - 1) * lh) / 2 + size * 0.35 : y;
  return lines.map((l, i) => `<text x="${n(x)}" y="${n(y0 + i * lh)}" font-family="${o.mono ? MONO : FONT}" font-size="${size}"` +
    ` font-weight="${o.weight ?? "normal"}" fill="${o.fill ?? C.ink}" text-anchor="${o.anchor ?? "start"}"` +
    (o.italic ? ` font-style="italic"` : "") + `>${esc(l)}</text>`).join("");
}
const rect = (x, y, w, h, o = {}) => `<rect x="${n(x)}" y="${n(y)}" width="${n(w)}" height="${n(h)}" rx="${o.r ?? 8}" ry="${o.r ?? 8}" fill="${o.fill ?? C.bg}" stroke="${o.stroke ?? C.line}" stroke-width="${o.sw ?? 1.5}"${o.dash ? ` stroke-dasharray="${o.dash}"` : ""}/>`;
const line = (x1, y1, x2, y2, o = {}) => `<line x1="${n(x1)}" y1="${n(y1)}" x2="${n(x2)}" y2="${n(y2)}" stroke="${o.stroke ?? C.sub}" stroke-width="${o.sw ?? 1.5}"${o.dash ? ` stroke-dasharray="${o.dash}"` : ""}/>`;
const circle = (x, y, r, o = {}) => `<circle cx="${n(x)}" cy="${n(y)}" r="${r}" fill="${o.fill ?? C.bg}" stroke="${o.stroke ?? C.ink}" stroke-width="${o.sw ?? 1.5}"${o.dash ? ` stroke-dasharray="${o.dash}"` : ""}/>`;
const pathD = (d, o = {}) => `<path d="${d}" fill="${o.fill ?? "none"}" stroke="${o.stroke ?? C.sub}" stroke-width="${o.sw ?? 1.5}"${o.dash ? ` stroke-dasharray="${o.dash}"` : ""} stroke-linejoin="round" stroke-linecap="round"/>`;
const poly = (pts, o = {}) => `<polygon points="${pts.map(p => p.map(n).join(",")).join(" ")}" fill="${o.fill ?? C.sub}" stroke="${o.stroke ?? "none"}" stroke-width="${o.sw ?? 1}"/>`;
const polyline = (pts, o = {}) => `<polyline points="${pts.map(p => p.map(n).join(",")).join(" ")}" fill="none" stroke="${o.stroke ?? C.sub}" stroke-width="${o.sw ?? 1.5}"${o.dash ? ` stroke-dasharray="${o.dash}"` : ""} stroke-linejoin="round"/>`;

/** Arrowhead at (x, y) pointing along angle a (radians). */
const head = (x, y, a, o = {}) => {
  const L = o.size ?? 9, W = (o.size ?? 9) * 0.55;
  return poly([[x, y], [x - L * Math.cos(a) + W * Math.sin(a), y - L * Math.sin(a) - W * Math.cos(a)], [x - L * Math.cos(a) - W * Math.sin(a), y - L * Math.sin(a) + W * Math.cos(a)]], { fill: o.stroke ?? C.sub });
};
function arrow(x1, y1, x2, y2, o = {}) {
  const a = Math.atan2(y2 - y1, x2 - x1), L = (o.size ?? 9) * 0.8;
  return line(x1, y1, x2 - L * Math.cos(a), y2 - L * Math.sin(a), o) + head(x2, y2, a, o) + (o.both ? head(x1, y1, a + Math.PI, o) : "");
}
/** Orthogonal arrow through waypoints [[x,y]…]; head on the last segment. */
function elbow(pts, o = {}) {
  const [p, q] = [pts[pts.length - 2], pts[pts.length - 1]];
  const a = Math.atan2(q[1] - p[1], q[0] - p[0]), L = (o.size ?? 9) * 0.8;
  const body = pts.slice(0, -1).concat([[q[0] - L * Math.cos(a), q[1] - L * Math.sin(a)]]);
  return polyline(body, o) + head(q[0], q[1], a, o);
}
function box(x, y, w, h, label, o = {}) {
  const size = o.size ?? 13, subSize = o.subSize ?? 11, subLh = subSize * 1.3;
  const subLines = o.sub ? String(o.sub).split("\n").length : 0;
  const total = size * 1.2 + (subLines ? 4 + subLines * subLh : 0);
  const top = y + (h - total) / 2;
  return rect(x, y, w, h, { fill: o.fill ?? C.blueXL, stroke: o.stroke ?? C.blue, r: o.r ?? 8, sw: o.sw, dash: o.dash }) +
    text(x + w / 2, top + size * 0.6, label, { anchor: "middle", middle: true, size, weight: o.weight ?? "bold", fill: o.color ?? C.ink, mono: o.mono }) +
    (o.sub ? text(x + w / 2, top + size * 1.2 + 4 + (subLines * subLh) / 2, o.sub, { anchor: "middle", middle: true, size: subSize, lh: subLh, fill: C.sub, mono: o.subMono }) : "");
}
function diamond(cx, cy, w, h, label, o = {}) {
  return poly([[cx, cy - h / 2], [cx + w / 2, cy], [cx, cy + h / 2], [cx - w / 2, cy]], { fill: o.fill ?? C.amberL, stroke: o.stroke ?? C.amber, sw: 1.5 }) +
    text(cx, cy, label, { anchor: "middle", middle: true, size: o.size ?? 12, weight: "bold", mono: o.mono });
}
const pill = (x, y, w, h, label, o = {}) => box(x, y, w, h, label, { r: h / 2, ...o });
const label = (x, y, s, o = {}) => text(x, y, s, { size: 11, fill: o.fill ?? C.sub, ...o });
const code = (x, y, s, o = {}) => text(x, y, s, { mono: true, size: o.size ?? 12, fill: o.fill ?? C.ink, ...o });

const figures = [];
function fig(id, caption, w, h, body) {
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">` +
    `<rect x="0" y="0" width="${w}" height="${h}" fill="${C.bg}"/>${body}</svg>\n`;
  fs.writeFileSync(path.join(OUT, `${id}.svg`), svg);
  figures.push({ id, file: `figures/${id}.svg`, caption, width: w, height: h });
}

// ── 1. Ecosystem ───────────────────────────────────────────────────────────
fig("ecosystem", "One program, two views: the app edits it, the controller stores, checks and runs it.", 760, 330, [
  rect(16, 16, 230, 298, { fill: C.panel }), text(131, 42, "Simple Robot app", { anchor: "middle", weight: "bold", size: 14 }),
  box(36, 62, 190, 56, "Blocks editor", { sub: "drag, drop, fill in fields" }),
  box(36, 134, 190, 56, "Text editor", { sub: "Simple Robot Script (.srs)" }),
  box(36, 206, 190, 46, "Monitor", { fill: C.greenL, stroke: C.green }),
  label(131, 284, "web · Android · desktop", { anchor: "middle" }),
  arrow(256, 120, 330, 120, { both: true, stroke: C.blue }), label(293, 108, "WebSocket", { anchor: "middle" }), label(293, 138, ":9000", { anchor: "middle" }),
  rect(336, 16, 220, 298, { fill: C.panel }), text(446, 42, "SimpleRobotController", { anchor: "middle", weight: "bold", size: 14 }),
  box(356, 62, 180, 40, "Program (JSON)", { fill: C.purpleL, stroke: C.purple }),
  box(356, 114, 180, 40, "Format / Parse text", { size: 12 }),
  box(356, 166, 180, 40, "Validator", { size: 12 }),
  box(356, 218, 180, 40, "Executor", { size: 12, fill: C.greenL, stroke: C.green }),
  label(446, 284, "step catalog · error codes", { anchor: "middle" }),
  arrow(566, 238, 610, 238, { stroke: C.green }),
  rect(616, 16, 128, 298, { fill: C.panel }), text(680, 42, "Hardware", { anchor: "middle", weight: "bold", size: 14 }),
  ...["Robot arm", "Driver I/O", "IO devices", "Aux axes", "Cameras", "Plugins"].map((s, i) => box(630, 58 + i * 40, 100, 30, s, { size: 11, fill: C.bg, stroke: C.line, weight: "normal" })),
].join(""));

// ── 2. Round trip ──────────────────────────────────────────────────────────
fig("round-trip", "Blocks and text are two views of the same stored program. Switching views never loses a field or a step id.", 760, 220, [
  box(290, 80, 180, 60, "Program JSON", { fill: C.purpleL, stroke: C.purple, sub: "what the controller runs" }),
  box(20, 80, 160, 60, "Blocks", { sub: "one card per step" }),
  box(580, 80, 160, 60, "Text (.srs)", { sub: "one line per step" }),
  arrow(180, 100, 290, 100, { stroke: C.blue }), arrow(290, 122, 180, 122, { stroke: C.blue }),
  label(235, 92, "edit", { anchor: "middle" }), label(235, 140, "show", { anchor: "middle" }),
  arrow(470, 100, 580, 100, { stroke: C.blue }), arrow(580, 122, 470, 122, { stroke: C.blue }),
  label(525, 92, "Format", { anchor: "middle", size: 10 }), label(525, 140, "Parse", { anchor: "middle", size: 10 }),
  code(380, 40, "Normalize(Parse(Format(P))) == Normalize(P)", { anchor: "middle", fill: C.purple }),
  label(380, 188, "names in the text resolve to stable ids — a renamed point keeps working", { anchor: "middle" }),
].join(""));

// ── 3. Program anatomy ─────────────────────────────────────────────────────
fig("program-anatomy", "Program kinds: a main program calls routines and starts background programs.", 760, 300, [
  rect(20, 20, 330, 260, { fill: C.blueXL, stroke: C.blue }), text(36, 46, "Main program", { weight: "bold", size: 15 }),
  label(36, 66, "runs from the program list or a display page"),
  rect(36, 80, 298, 56, { fill: C.bg }), code(48, 100, "var count = 0"), code(48, 118, "var parts: list point"), label(318, 98, "declarations", { anchor: "end" }),
  rect(36, 146, 298, 120, { fill: C.bg }),
  code(48, 166, "SetTool Gripper"), code(48, 184, "repeat 12 as $i"), code(64, 202, "call PickPart(slot=$i)", { fill: C.purple }),
  code(64, 220, "JumpL grid Tray[$i], z=40"), code(48, 238, "end"), code(48, 256, "StartBackground Conveyor_Watch", { fill: C.teal }),
  label(318, 164, "steps", { anchor: "end" }),
  rect(420, 20, 320, 120, { fill: C.purpleL, stroke: C.purple }), text(436, 46, "Routine", { weight: "bold", size: 15 }),
  label(436, 66, "hidden from the list; runs only when called"), code(436, 90, "param in slot = 0"), code(436, 108, "param out picked: bool"), code(436, 126, "… steps … return"),
  rect(420, 160, 320, 120, { fill: C.tealL, stroke: C.teal }), text(436, 186, "Background program", { weight: "bold", size: 15 }),
  label(436, 206, "runs in parallel; no motion, tool or homing"), code(436, 230, "repeat forever"), code(452, 248, "Wait until $stb.in1 == 0"), code(436, 266, "end"),
  elbow([[250, 202], [380, 202], [380, 80], [420, 80]], { stroke: C.purple }), label(388, 140, "call", { fill: C.purple }),
  elbow([[300, 252], [390, 252], [390, 220], [420, 220]], { stroke: C.teal }), label(342, 246, "start", { fill: C.teal }),
].join(""));

// ── 4. Variable scopes ─────────────────────────────────────────────────────
fig("variable-scopes", "Where a variable lives: the global store is shared by every running program; a program's variables (and its routines' parameters) belong to its run.", 760, 330, [
  rect(16, 16, 520, 298, { fill: C.amberL, stroke: C.amber }), text(32, 42, "Global store", { weight: "bold", size: 15 }), label(32, 60, "global var · global computed — every running program reads and writes the same value"),
  rect(36, 76, 230, 222, { fill: C.blueXL, stroke: C.blue }), text(50, 100, "Main program run", { weight: "bold" }),
  code(50, 124, "var count"), code(50, 142, "persistent var runs"), code(50, 160, "computed area"),
  rect(50, 176, 202, 108, { fill: C.purpleL, stroke: C.purple }), text(62, 196, "called routine", { weight: "bold", size: 12 }),
  code(62, 216, "param in slot"), code(62, 234, "param out picked"), label(62, 256, "shares the run's variable scope —"), label(62, 270, "name parameters distinctly"),
  rect(286, 76, 230, 222, { fill: C.tealL, stroke: C.teal }), text(300, 100, "Background program run", { weight: "bold" }),
  code(300, 124, "var pulses"), label(300, 146, "its own variables, separate"), label(300, 162, "from the main program's"),
  rect(300, 180, 202, 60, { fill: C.bg, stroke: C.line, dash: "4 3" }), label(312, 200, "loop / for-each variables"), label(312, 216, "are ordinary program variables"),
  rect(556, 16, 188, 140, { fill: C.panel }), text(570, 42, "Read-only", { weight: "bold" }),
  code(570, 66, "$robot.x  $time.hour"), code(570, 86, "$program.runCount"), code(570, 106, "$stb.in1  $io.Nano.Pin"), label(570, 130, "properties and IO —"), label(570, 144, "read anywhere, never assigned"),
  rect(556, 174, 188, 140, { fill: C.panel }), text(570, 200, "On disk", { weight: "bold" }),
  label(570, 222, "persistent: saved when the"), label(570, 238, "run finishes, restored at"), label(570, 254, "the next start"),
].join(""));

// ── 5. Frames ──────────────────────────────────────────────────────────────
{
  const ox = 70, oy = 270, th = 15 * Math.PI / 180; // local frame turned +15° (counter-clockwise on paper)
  const lx = 230, ly = 200, c15 = Math.cos(th), s15 = Math.sin(th);
  const rz = -th; // screen angle of x′
  const L = (u, v) => [lx + u * c15 - v * s15, ly - u * s15 - v * c15];
  const pX = L(90, 0), pY = L(0, 90), P = L(70, 50);
  fig("frames", "Coordinate frames. Left (top view): the world frame and a local frame (Fixture1) offset and turned 15°; a point taught in the local frame moves with it. Right (side view): the tool frame puts the TCP at the gripper tip.", 760, 320, [
    rect(16, 16, 420, 288, { fill: C.panel }), text(30, 40, "World and local frames (top view)", { weight: "bold" }),
    arrow(ox, oy, ox + 140, oy, { stroke: C.ink }), arrow(ox, oy, ox, oy - 200, { stroke: C.ink }),
    text(ox + 146, oy + 4, "X", { weight: "bold" }), text(ox - 4, oy - 206, "Y", { weight: "bold", anchor: "middle" }), label(ox - 8, oy + 18, "world origin"),
    arrow(lx, ly, pX[0], pX[1], { stroke: C.orange }), arrow(lx, ly, pY[0], pY[1], { stroke: C.orange }),
    text(pX[0] + 6, pX[1] + 4, "x′", { fill: C.orange, weight: "bold" }), text(pY[0] - 4, pY[1] - 6, "y′", { fill: C.orange, weight: "bold" }),
    pathD(`M ${lx + 40} ${ly} A 40 40 0 0 0 ${lx + 40 * Math.cos(rz)} ${ly + 40 * Math.sin(rz)}`, { stroke: C.orange }), label(lx + 46, ly - 2, "rz 15°", { fill: C.orange }),
    line(ox, oy, lx, ly, { dash: "4 4", stroke: C.mute }), label(150, 252, "local origin (x, y)", { fill: C.mute }),
    circle(P[0], P[1], 5, { fill: C.blue, stroke: C.blue }), text(P[0] + 8, P[1] - 6, "Pick1 (taught in Fixture1)", { size: 11, fill: C.blue }),
    code(230, 296, "SetLocal Fixture1 … ClearLocal", { size: 11, fill: C.sub }),
    rect(452, 16, 292, 288, { fill: C.panel }), text(466, 40, "Tool frame (side view)", { weight: "bold" }),
    rect(560, 60, 80, 40, { fill: C.grid, stroke: C.sub, r: 4 }), label(600, 84, "arm", { anchor: "middle" }),
    rect(575, 100, 50, 14, { fill: C.sub, stroke: C.sub, r: 2 }), label(632, 112, "flange (no tool)"),
    rect(585, 114, 30, 70, { fill: C.blueL, stroke: C.blue, r: 3 }), line(588, 184, 588, 222, { stroke: C.blue, sw: 4 }), line(612, 184, 612, 222, { stroke: C.blue, sw: 4 }),
    circle(600, 222, 5, { fill: C.red, stroke: C.red }), text(612, 240, "TCP — tool tip", { fill: C.red, size: 12, weight: "bold" }),
    arrow(530, 107, 530, 222, { both: true, stroke: C.orange }), label(522, 168, "z 85", { anchor: "end", fill: C.orange }),
    code(466, 270, "SetTool Gripper", { size: 11, fill: C.sub }), label(466, 288, "targets are reached with the TCP"),
  ].join(""));
}

// ── 6. Target modifiers ────────────────────────────────────────────────────
fig("target-modifiers", "How a move target is built: the base target, then offset, then override, all in the active (or step) local frame; the tool offset applies in the tool frame at the end.", 780, 210, [
  box(16, 60, 140, 70, "Target", { sub: "point · $list[i]\ngrid · stack · here", subSize: 10 }),
  box(186, 60, 130, 70, "+ offset=", { mono: true, sub: "adds per axis", fill: C.greenL, stroke: C.green }),
  box(346, 60, 130, 70, "set=", { mono: true, sub: "replaces an axis", fill: C.amberL, stroke: C.amber }),
  box(506, 60, 120, 70, "local=", { mono: true, sub: "frame for this move", fill: C.orangeL, stroke: C.orange }),
  box(656, 60, 108, 70, "toolOffset=", { mono: true, size: 12, sub: "in the tool frame", fill: C.purpleL, stroke: C.purple }),
  arrow(156, 95, 186, 95), arrow(316, 95, 346, 95), arrow(476, 95, 506, 95), arrow(626, 95, 656, 95),
  code(16, 170, "MoveL Pick1, offset=(z=20), set=(rz=90), local=Fixture1, toolOffset=(z=-5)", { size: 12 }),
  label(16, 192, "Pick1 z=40 → offset gives z=60 → rz forced to 90 → resolved in Fixture1 → TCP backed off 5 mm along the tool's z"),
].join(""));

// ── 7. Grid and stack ──────────────────────────────────────────────────────
{
  const bx = 70, by = 70, s = 46, rows = 3, cols = 4;
  const cells = [];
  for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
    const x = bx + c * s, y = by + r * s;
    cells.push(rect(x, y, s - 6, s - 6, { fill: r === 1 && c === 2 ? C.blueL : C.bg, stroke: r === 1 && c === 2 ? C.blue : C.line, r: 6 }));
    cells.push(text(x + (s - 6) / 2, y + (s - 6) / 2, `${r * cols + c}`, { anchor: "middle", middle: true, size: 13, weight: "bold", fill: C.sub }));
    cells.push(text(x + (s - 6) / 2, y + s - 12, `${r},${c}`, { anchor: "middle", size: 9, fill: C.mute }));
  }
  fig("grid-stack", "A grid addresses cells by [row, col] or by index (row-major: index = row × columns + col). A stack addresses slots base + index × offset.", 760, 300, [
    rect(16, 16, 420, 268, { fill: C.panel }), text(30, 40, "Grid  Tray  (3 rows × 4 columns)", { weight: "bold" }),
    ...cells,
    circle(bx + 7, by + 7, 4, { fill: C.red, stroke: C.red }), label(bx - 4, by - 8, "base point (cell 0,0)", { fill: C.red }),
    arrow(bx + 20, by + 20 + 3 * s + 6, bx + 20 + s, by + 20 + 3 * s + 6, { stroke: C.orange }), label(bx + 20, by + 3 * s + 46, "column offset", { fill: C.orange }),
    arrow(bx + 4 * s + 10, by + 20, bx + 4 * s + 10, by + 20 + s, { stroke: C.orange }), label(bx + 4 * s + 16, by + 50, "row offset", { fill: C.orange }),
    code(300, 230, "grid Tray[1, 2]", { fill: C.blue }), code(300, 250, "grid Tray[6]", { fill: C.blue }), label(300, 268, "same cell"),
    rect(452, 16, 292, 268, { fill: C.panel }), text(466, 40, "Stack  Bins  (offset z 12, max 5)", { weight: "bold" }),
    ...[0, 1, 2, 3, 4].map(i => rect(540, 240 - i * 36, 120, 28, { fill: i === 2 ? C.blueL : C.bg, stroke: i === 2 ? C.blue : C.line, r: 4 }) + text(600, 258 - i * 36, `index ${i}`, { anchor: "middle", size: 12, fill: C.sub })),
    circle(530, 254, 4, { fill: C.red, stroke: C.red }), label(522, 258, "base", { anchor: "end", fill: C.red }),
    arrow(680, 254, 680, 218, { stroke: C.orange }), label(686, 240, "+12 z", { fill: C.orange }),
    code(466, 70, "MoveL stack Bins[2]", { fill: C.blue }), label(466, 88, "index past max wraps round"),
  ].join(""));
}

// ── 8. Linear vs joint ─────────────────────────────────────────────────────
fig("linear-vs-joint", "Move Linear keeps the tool on a straight line; Move Joint moves every joint proportionally, so the tool follows a curve — usually faster in open space.", 700, 260, [
  rect(16, 16, 668, 228, { fill: C.panel }),
  circle(120, 180, 7, { fill: C.ink }), text(120, 206, "start", { anchor: "middle", size: 12 }),
  circle(580, 90, 7, { fill: C.ink }), text(580, 70, "Pick1", { anchor: "middle", size: 12, weight: "bold" }),
  arrow(127, 178, 572, 92, { stroke: C.blue, sw: 2.5 }), code(300, 120, "MoveL Pick1", { fill: C.blue, weight: "bold" }),
  pathD("M 127 186 C 260 250, 470 230, 574 98", { stroke: C.orange, sw: 2.5, dash: "7 5" }), head(574, 98, -1.0, { stroke: C.orange }),
  code(380, 232, "MoveJ Pick1", { fill: C.orange, weight: "bold" }),
  label(36, 44, "top view of the tool tip path"),
].join(""));

// ── 9. Speed profile ───────────────────────────────────────────────────────
fig("speed-profile", "A move ramps up at accel, cruises at speed, and ramps down at decel. A short move never reaches full speed (dashed). Program speeds are scaled by the speed override × Set Speed Factor.", 700, 260, [
  line(60, 210, 660, 210, { stroke: C.ink }), line(60, 210, 60, 30, { stroke: C.ink }), head(660, 210, 0, { stroke: C.ink }), head(60, 30, -Math.PI / 2, { stroke: C.ink }),
  text(664, 230, "time", { anchor: "end", size: 12 }), text(52, 36, "speed", { anchor: "end", size: 12 }),
  polyline([[60, 210], [180, 70], [470, 70], [600, 210]], { stroke: C.blue, sw: 3 }),
  polyline([[60, 210], [130, 128], [200, 210]], { stroke: C.orange, sw: 2, dash: "6 4" }),
  line(60, 70, 180, 70, { dash: "3 3", stroke: C.mute }), label(66, 64, "speed (mm/s)"),
  label(80, 128, "accel", { fill: C.blue, weight: "bold" }), label(540, 140, "decel", { fill: C.blue, weight: "bold" }), label(300, 62, "cruise", { fill: C.blue, weight: "bold" }),
  label(206, 200, "short move", { fill: C.orange }),
  code(360, 250, "SetSpeedL 200, accel=800, decel=800", { anchor: "middle", fill: C.sub }),
].join(""));

// ── 10. Blending ───────────────────────────────────────────────────────────
fig("blending", "Without blending the robot stops exactly on each point. With a blend radius it rounds the corner inside the radius and never stops.", 720, 260, [
  rect(16, 16, 336, 228, { fill: C.panel }), text(30, 40, "MoveL Approach", { mono: true, size: 12, weight: "bold" }), text(30, 58, "MoveL Pick1", { mono: true, size: 12, weight: "bold" }),
  polyline([[50, 210], [190, 90], [320, 200]], { stroke: C.blue, sw: 3 }), circle(190, 90, 7, { fill: C.red, stroke: C.red }), label(200, 84, "stops here (speed 0)", { fill: C.red }),
  circle(50, 210, 5, { fill: C.ink }), circle(320, 200, 5, { fill: C.ink }),
  rect(368, 16, 336, 228, { fill: C.panel }), text(382, 40, "MoveL Approach, blend=10", { mono: true, size: 12, weight: "bold" }), text(382, 58, "MoveL Pick1", { mono: true, size: 12, weight: "bold" }),
  polyline([[402, 210], [542, 90], [672, 200]], { stroke: C.line, sw: 2, dash: "5 4" }),
  circle(542, 90, 46, { fill: "none", stroke: C.orange, dash: "4 3" }),
  pathD("M 402 210 L 507 120 Q 542 90 577 120 L 672 200", { stroke: C.blue, sw: 3 }),
  line(542, 90, 588, 90, { stroke: C.orange }), label(560, 84, "r", { fill: C.orange, weight: "bold" }),
  label(500, 156, "rounded corner,", { fill: C.blue }), label(500, 170, "no stop", { fill: C.blue }),
  circle(402, 210, 5, { fill: C.ink }), circle(672, 200, 5, { fill: C.ink }),
].join(""));

// ── 11. Arc through a via point ────────────────────────────────────────────
{
  const cx = 300, cy = 230, R = 170, a0 = Math.PI * 0.95, a1 = Math.PI * 0.05, am = Math.PI * 0.55;
  const P = a => [cx + R * Math.cos(a), cy - R * Math.sin(a)];
  const segs = [];
  const N = 10;
  for (let i = 0; i <= N; i++) segs.push(P(a0 + (a1 - a0) * i / N));
  const [s, v, e] = [P(a0), P(am), P(a1)];
  fig("arc-via", "Move Arc follows the circle through the start, the via point and the end. It is cut into chords within tolerance= of the true arc and run as one continuous path.", 620, 270, [
    pathD(`M ${n(s[0])} ${n(s[1])} A ${R} ${R} 0 0 1 ${n(e[0])} ${n(e[1])}`, { stroke: C.line, sw: 8 }),
    polyline(segs, { stroke: C.blue, sw: 2.5 }), ...segs.map(p => circle(p[0], p[1], 2.5, { fill: C.blue, stroke: C.blue })),
    circle(s[0], s[1], 7, { fill: C.ink }), text(s[0] - 10, s[1] + 24, "start (where the robot is)", { size: 12 }),
    circle(v[0], v[1], 7, { fill: C.orange, stroke: C.orange }), text(v[0], v[1] - 16, "via=Via1", { anchor: "middle", mono: true, size: 12, fill: C.orange, weight: "bold" }),
    circle(e[0], e[1], 7, { fill: C.ink }), text(e[0] - 20, e[1] + 24, "end  Place1", { size: 12 }),
    line(cx, cy, cx, cy, {}), circle(cx, cy, 3, { fill: C.mute, stroke: C.mute }), label(cx + 6, cy - 4, "centre (computed)", { fill: C.mute }),
    label(440, 60, "grey band: true arc", { fill: C.mute }), label(440, 76, "blue: chords, each within", { fill: C.blue }), label(440, 92, "tolerance (default 0.1 mm)", { fill: C.blue }),
    code(20, 262, "MoveC Place1, via=Via1, speed=80, tolerance=0.1", { fill: C.sub }),
  ].join(""));
}

// ── 12. Jump ───────────────────────────────────────────────────────────────
fig("jump", "Jump (side view): lift straight up to zStart, traverse to above the target at zEnd, lower onto the target. z= sets both heights; the heights are absolute Z values.", 720, 280, [
  line(40, 240, 690, 240, { stroke: C.ink }), label(690, 258, "X / Y", { anchor: "end" }), line(40, 240, 40, 30, { stroke: C.ink }), label(48, 38, "Z"),
  rect(80, 226, 80, 14, { fill: C.grid, stroke: C.line, r: 2 }), rect(540, 226, 80, 14, { fill: C.grid, stroke: C.line, r: 2 }),
  circle(120, 214, 6, { fill: C.ink }), text(120, 270, "start", { anchor: "middle", size: 12 }),
  circle(580, 214, 6, { fill: C.ink }), text(580, 270, "target Place1", { anchor: "middle", size: 12 }),
  arrow(120, 208, 120, 74, { stroke: C.blue, sw: 2.5 }), arrow(126, 70, 572, 100, { stroke: C.blue, sw: 2.5 }), arrow(580, 106, 580, 206, { stroke: C.blue, sw: 2.5 }),
  line(40, 70, 120, 70, { dash: "3 3", stroke: C.orange }), label(46, 64, "zStart (lift)", { fill: C.orange }),
  line(580, 100, 680, 100, { dash: "3 3", stroke: C.orange }), label(600, 94, "zEnd (lower)", { fill: C.orange }),
  label(130, 150, "1 lift", { fill: C.blue, weight: "bold" }), label(330, 74, "2 traverse", { fill: C.blue, weight: "bold" }), label(588, 160, "3 lower", { fill: C.blue, weight: "bold" }),
  code(200, 140, "JumpL Place1, zStart=80, zEnd=60", { fill: C.sub }), label(200, 158, "JumpL: straight legs · JumpJ: joint-interpolated legs", { fill: C.sub }),
].join(""));

// ── 13. Stop condition ─────────────────────────────────────────────────────
fig("stop-condition", "A move with a stop condition: the condition is checked every tick. stop ends the move where it is; record lets it finish and stores the pose where the condition first held.", 760, 300, [
  rect(16, 16, 728, 268, { fill: C.panel }),
  line(70, 250, 720, 250, { stroke: C.ink }), label(720, 268, "time", { anchor: "end" }),
  line(70, 250, 70, 40, { stroke: C.ink }),
  text(80, 46, "load cell (g)", { size: 12, fill: C.red, weight: "bold" }),
  polyline([[70, 240], [300, 238], [360, 200], [420, 130], [470, 80]], { stroke: C.red, sw: 2.5 }),
  line(70, 140, 720, 140, { dash: "5 4", stroke: C.red }), label(600, 134, "until= load >= 2000", { fill: C.red }),
  line(410, 50, 410, 250, { dash: "3 3", stroke: C.ink }), label(414, 62, "condition first true"),
  polyline([[70, 190], [410, 190]], { stroke: C.blue, sw: 3 }), polyline([[410, 190], [410, 190]], {}),
  text(250, 182, "stop: move ends here, next step runs", { size: 12, fill: C.blue, anchor: "middle" }),
  polyline([[410, 214], [650, 214]], { stroke: C.green, sw: 3, dash: "7 4" }), circle(410, 214, 6, { fill: C.green, stroke: C.green }),
  text(530, 232, "record: keeps moving to the target", { size: 12, fill: C.green, anchor: "middle" }),
  label(418, 210, "pose stored in untilPos=", { fill: C.green }),
  code(80, 276, "MoveL Pick1, offset=(z=-10), until=$io.Arduino_Nano.Load_Cell >= 2000, untilMode=stop, untilFlag=$hit", { size: 11, fill: C.sub }),
].join(""));

// ── 14. IO pulse timing ────────────────────────────────────────────────────
function lane(y, h, label_, pts, color) {
  return text(20, y + h / 2 + 4, label_, { size: 12, weight: "bold" }) + polyline(pts.map(([x, v]) => [x, y + (v ? 0 : h)]), { stroke: color, sw: 2.5 });
}
fig("io-pulse", "Set Output with pulse=250ms. blocking=on holds the program until the output has flipped back; without it the next step starts at once and the flip-back happens in the background.", 760, 300, [
  text(20, 30, "blocking=on", { mono: true, weight: "bold", fill: C.blue }),
  lane(44, 30, "output", [[170, 0], [260, 0], [260, 1], [460, 1], [460, 0], [720, 0]], C.blue),
  rect(170, 90, 90, 30, { fill: C.grid, r: 4 }), text(215, 110, "step n−1", { anchor: "middle", size: 11 }),
  rect(260, 90, 200, 30, { fill: C.blueL, stroke: C.blue, r: 4 }), text(360, 110, "SetOutput … pulse (waits)", { anchor: "middle", size: 11 }),
  rect(460, 90, 120, 30, { fill: C.grid, r: 4 }), text(520, 110, "next step", { anchor: "middle", size: 11 }),
  text(20, 110, "program", { size: 12, weight: "bold" }),
  arrow(260, 136, 460, 136, { both: true, stroke: C.orange }), label(360, 152, "250 ms", { anchor: "middle", fill: C.orange }),
  text(20, 186, "blocking=off", { mono: true, weight: "bold", fill: C.teal }),
  lane(200, 30, "output", [[170, 0], [260, 0], [260, 1], [460, 1], [460, 0], [720, 0]], C.teal),
  rect(170, 246, 90, 30, { fill: C.grid, r: 4 }), text(215, 266, "step n−1", { anchor: "middle", size: 11 }),
  rect(260, 246, 40, 30, { fill: C.tealL, stroke: C.teal, r: 4 }), text(280, 266, "Set", { anchor: "middle", size: 11 }),
  rect(300, 246, 130, 30, { fill: C.grid, r: 4 }), text(365, 266, "next step", { anchor: "middle", size: 11 }),
  rect(430, 246, 120, 30, { fill: C.grid, r: 4 }), text(490, 266, "…", { anchor: "middle", size: 11 }),
  text(20, 266, "program", { size: 12, weight: "bold" }),
].join(""));

// ── 15. Wait until with timeout ────────────────────────────────────────────
fig("io-wait", "Wait until <condition>, timeout=5s. If the input comes on in time the program continues; otherwise timedOut= is set to 1 and onTimeout decides: continue (default) or fail with waitTimeout.", 760, 280, [
  text(20, 30, "A — condition met", { weight: "bold", fill: C.green }),
  lane(44, 30, "$stb.in1", [[160, 0], [380, 0], [380, 1], [720, 1]], C.green),
  rect(160, 92, 220, 28, { fill: C.amberL, stroke: C.amber, r: 4 }), text(270, 111, "Wait until $stb.in1", { anchor: "middle", size: 11, mono: true }),
  rect(380, 92, 150, 28, { fill: C.grid, r: 4 }), text(455, 111, "next step", { anchor: "middle", size: 11 }), label(540, 111, "timedOut = 0", { fill: C.green }),
  text(20, 160, "B — timeout", { weight: "bold", fill: C.red }),
  lane(174, 30, "$stb.in1", [[160, 0], [720, 0]], C.red),
  rect(160, 222, 400, 28, { fill: C.amberL, stroke: C.amber, r: 4 }), text(360, 241, "Wait until $stb.in1 … 5 s", { anchor: "middle", size: 11, mono: true }),
  line(560, 214, 560, 262, { stroke: C.red, sw: 2 }), label(566, 232, "timedOut = 1", { fill: C.red }), label(566, 248, "continue  |  error → waitTimeout", { fill: C.red }),
  arrow(160, 268, 560, 268, { both: true, stroke: C.orange, size: 7 }), label(360, 264, "timeout", { anchor: "middle", fill: C.orange }),
].join(""));

// ── Flowchart helpers ──────────────────────────────────────────────────────
const yes = (x, y) => label(x, y, "yes", { fill: C.green, weight: "bold" });
const no = (x, y) => label(x, y, "no", { fill: C.red, weight: "bold" });

// ── 16. If ─────────────────────────────────────────────────────────────────
fig("flow-if", "if / elif / else: conditions are tested top to bottom; the first that holds runs its branch, then execution continues after end.", 720, 330, [
  pill(300, 14, 120, 32, "if", { mono: true }),
  arrow(360, 46, 360, 70),
  diamond(360, 100, 200, 60, "$count >= 12", { mono: true }),
  arrow(460, 100, 540, 100), yes(474, 92), box(540, 80, 160, 40, "branch 1", { fill: C.greenL, stroke: C.green }),
  arrow(360, 130, 360, 160), no(366, 150),
  diamond(360, 190, 200, 60, "$count > 8", { mono: true }),
  arrow(460, 190, 540, 190), yes(474, 182), box(540, 170, 160, 40, "branch 2 (elif)", { fill: C.greenL, stroke: C.green }),
  arrow(360, 220, 360, 250), no(366, 240),
  box(280, 250, 160, 36, "else branch", { fill: C.grid, stroke: C.sub }),
  polyline([[620, 120], [620, 140], [700, 140], [700, 305]]), polyline([[620, 210], [620, 228], [700, 228]]),
  polyline([[360, 286], [360, 305]]),
  pill(160, 290, 120, 30, "after end", { fill: C.bg, stroke: C.sub, weight: "normal" }), arrow(700, 305, 280, 305),
  code(20, 80, "if $count >= 12", { size: 11 }), code(20, 96, "  …", { size: 11 }), code(20, 112, "elif $count > 8", { size: 11 }), code(20, 128, "  …", { size: 11 }),
  code(20, 144, "else", { size: 11 }), code(20, 160, "  …", { size: 11 }), code(20, 176, "end", { size: 11 }),
].join(""));

// ── 17. Switch ─────────────────────────────────────────────────────────────
fig("flow-switch", "switch evaluates its value once and runs the first case with an equal value (no fall-through), else default.", 720, 250, [
  box(270, 16, 180, 40, "switch $grade", { mono: true }),
  arrow(360, 56, 360, 84),
  line(90, 84, 450, 84, { stroke: C.sub }),
  ...[["case 1", 90], ["case 2, 3", 270], ["default", 450], ["", 630]].slice(0, 3).map(([s, x]) => arrow(x, 84, x, 112) + box(x - 75, 112, 150, 44, s, { mono: true, fill: s === "default" ? C.grid : C.greenL, stroke: s === "default" ? C.sub : C.green })),
  label(100, 104, "== 1", { fill: C.green }), label(280, 104, "== 2 or 3", { fill: C.green }), label(460, 104, "no case matched", { fill: C.sub }),
  ...[90, 270, 450].map(x => line(x, 156, x, 196)), line(90, 196, 450, 196), arrow(270, 196, 270, 216),
  pill(210, 216, 120, 28, "after end", { fill: C.bg, stroke: C.sub, weight: "normal" }),
  label(545, 130, "values compared", { fill: C.sub }), label(545, 146, "within 1e-9", { fill: C.sub }), label(545, 162, "(numbers only)", { fill: C.sub }),
].join(""));

// ── 18. Loops ──────────────────────────────────────────────────────────────
function loopChart(x0, title, cond, body, foot) {
  return [
    text(x0 + 110, 30, title, { anchor: "middle", weight: "bold", mono: true }),
    diamond(x0 + 110, 90, 190, 64, cond, { mono: true, size: 11 }),
    arrow(x0 + 110, 122, x0 + 110, 150), yes(x0 + 116, 142),
    box(x0 + 40, 150, 140, 40, body, { size: 12, fill: C.greenL, stroke: C.green }),
    elbow([[x0 + 40, 170], [x0 + 6, 170], [x0 + 6, 90], [x0 + 15, 90]]),
    elbow([[x0 + 205, 90], [x0 + 218, 90], [x0 + 218, 236], [x0 + 170, 236]]), no(x0 + 196, 82),
    pill(x0 + 50, 222, 120, 28, "after end", { fill: C.bg, stroke: C.sub, weight: "normal" }),
    arrow(x0 + 110, 44, x0 + 110, 58),
    label(x0 + 110, 274, foot, { anchor: "middle" }),
  ].join("");
}
fig("flow-loops", "The three loop forms. break leaves the loop at once; continue skips to the next pass.", 760, 290, [
  loopChart(10, "repeat 12 as $i", "$i < 12 ?", "body; $i += 1", "count from 0 · repeat forever = no limit"),
  loopChart(265, "while <cond>", "condition ?", "body", "checked before every pass"),
  loopChart(520, "for $v, $i in $list", "elements left ?", "$v, $i = next", "Point / Record lists: $v gets the index"),
].join(""));

// ── 19. Try / retry / catch ────────────────────────────────────────────────
fig("flow-try", "try retry=2: a catchable failure abandons the body and runs it again until the retries are used up; then the catch steps run with the message and code. Fatal errors and codes not in only=(…) go outward.", 760, 420, [
  pill(310, 10, 140, 30, "try retry=2", { mono: true }), arrow(380, 40, 380, 60),
  box(300, 60, 160, 40, "run body", { fill: C.greenL, stroke: C.green }),
  arrow(380, 100, 380, 124),
  diamond(380, 154, 170, 60, "failed?"),
  elbow([[465, 154], [690, 154], [690, 360]]), no(474, 146),
  arrow(380, 184, 380, 210), yes(386, 202),
  diamond(380, 240, 210, 60, "catchable and\nin only=(…)?", { size: 11 }),
  elbow([[275, 240], [150, 240], [150, 300]]), no(240, 232),
  box(60, 300, 180, 64, "goes outward", { sub: "next Try, else the\nprogram ends with Error", subSize: 10, fill: C.redL, stroke: C.red }),
  arrow(380, 270, 380, 290), yes(386, 284),
  diamond(380, 316, 150, 50, "retries left?", { size: 11 }),
  elbow([[455, 316], [520, 316], [520, 80], [460, 80]], { stroke: C.orange }), yes(470, 308), label(526, 200, "wait delay=,", { fill: C.orange }), label(526, 214, "run again", { fill: C.orange }),
  arrow(380, 341, 380, 366), no(386, 358),
  box(290, 366, 200, 46, "catch $msg, code=$code", { mono: true, size: 11, sub: "run the catch steps", fill: C.amberL, stroke: C.amber }),
  pill(630, 360, 120, 32, "after end", { fill: C.bg, stroke: C.sub, weight: "normal" }), arrow(490, 389, 630, 380),
].join(""));

// ── 20. Error propagation ──────────────────────────────────────────────────
fig("error-propagation", "A catchable error travels outward from the failing step — through loops, branches and called routines — to the nearest Try that catches its code. With none, the program ends with Error.", 760, 270, [
  rect(16, 16, 728, 238, { fill: C.panel }), text(30, 40, "Main program", { weight: "bold" }),
  rect(40, 54, 560, 184, { fill: C.amberL, stroke: C.amber }), code(52, 74, "try  only=(visionFailed)", { size: 12 }),
  rect(64, 86, 520, 140, { fill: C.blueXL, stroke: C.blue }), code(76, 106, "repeat 12", { size: 12 }),
  rect(88, 118, 480, 96, { fill: C.purpleL, stroke: C.purple }), code(100, 138, "call PickPart  (routine)", { size: 12 }),
  box(112, 154, 200, 44, "RunVision", { sub: "fails: visionFailed", fill: C.redL, stroke: C.red }),
  elbow([[312, 176], [620, 176], [620, 74], [600, 74]], { stroke: C.red, sw: 2 }), label(626, 130, "caught by", { fill: C.red }), label(626, 144, "the Try", { fill: C.red }),
  label(340, 200, "routine abandoned (no outputs copied), loop left", { fill: C.sub }),
  label(622, 200, "Fatal codes", { fill: C.ink, weight: "bold" }), label(622, 214, "(motionFault,", { fill: C.ink }), label(622, 228, "userStop …) are", { fill: C.ink }), label(622, 242, "never caught", { fill: C.ink }),
].join(""));

// ── 21. Routine parameters ─────────────────────────────────────────────────
fig("routine-params", "Calling a routine: in / inout arguments are evaluated in the caller, then copied into the parameters; when the routine ends (or returns), out / inout parameters are copied back.", 760, 260, [
  rect(16, 16, 300, 228, { fill: C.blueXL, stroke: C.blue }), text(30, 42, "Caller", { weight: "bold" }),
  code(30, 76, "var ok: bool"), code(30, 96, "var n = 3"),
  code(30, 140, "call PickPart(slot=$n + 1),", { size: 12, fill: C.purple }), code(46, 158, "out picked=$ok", { size: 12, fill: C.purple }),
  code(30, 210, "if not $ok …"),
  rect(444, 16, 300, 228, { fill: C.purpleL, stroke: C.purple }), text(458, 42, "Routine  PickPart", { weight: "bold" }),
  code(458, 76, "param in  slot = 0"), code(458, 96, "param out picked: bool = off"),
  code(458, 140, "JumpL stack Bins[$slot], z=40"), code(458, 160, "GripperAction close"), code(458, 180, "$picked = on"), code(458, 210, "end of routine / return"),
  arrow(316, 100, 444, 80, { stroke: C.green, sw: 2 }), label(326, 70, "1  slot ← 4", { fill: C.green, weight: "bold" }), label(326, 120, "evaluated in", { fill: C.green }), label(326, 134, "the caller", { fill: C.green }),
  arrow(444, 205, 316, 180, { stroke: C.orange, sw: 2 }), label(330, 224, "2  ok ← picked", { fill: C.orange }),
].join(""));

// ── 22. Run state machine ──────────────────────────────────────────────────
fig("run-states", "Program states. Stop, a Pause Program step, a breakpoint or step mode all pause the run (Stopped) with its place kept; Continue resumes, Step runs one step and pauses again.", 760, 320, [
  pill(30, 140, 110, 40, "Ready", { fill: C.grid, stroke: C.sub }),
  pill(250, 140, 130, 40, "Running", { fill: C.greenL, stroke: C.green }),
  pill(500, 40, 130, 40, "Stopped", { fill: C.amberL, stroke: C.amber, sub: "" }),
  pill(500, 160, 130, 40, "Complete", { fill: C.blueL, stroke: C.blue }),
  pill(500, 250, 130, 40, "Error", { fill: C.redL, stroke: C.red }),
  arrow(140, 160, 250, 160, { stroke: C.green }), label(150, 152, "Run / Start", { fill: C.green }),
  arrow(370, 144, 500, 66, { stroke: C.amber }), label(330, 96, "Stop · pause step", { fill: C.amber }), label(330, 110, "breakpoint · step mode", { fill: C.amber }),
  arrow(500, 74, 380, 152, { stroke: C.green }), label(420, 136, "Continue", { fill: C.green }),
  pathD("M 630 52 C 720 20, 730 100, 632 72", { stroke: C.amber }), head(632, 72, Math.PI * 0.9, { stroke: C.amber }), label(650, 18, "Step: one step,", { fill: C.amber }), label(650, 32, "then pause again", { fill: C.amber }),
  arrow(380, 170, 500, 178, { stroke: C.blue }), label(392, 212, "last step · exit · return", { fill: C.blue }),
  arrow(370, 176, 500, 262, { stroke: C.red }), label(330, 252, "uncaught error", { fill: C.red }),
  elbow([[565, 290], [565, 310], [85, 310], [85, 180]], { stroke: C.sub, dash: "5 4" }), label(150, 304, "Reset (from Stopped, Complete or Error)", { fill: C.sub }),
].join(""));

// ── 23. Background timeline ────────────────────────────────────────────────
fig("background-timeline", "A background program runs beside the main program. It shares global variables and IO, skips motion, tool and homing steps, and can be stopped or waited for.", 760, 230, [
  text(20, 64, "main", { weight: "bold" }), text(20, 144, "background", { weight: "bold" }),
  rect(110, 44, 120, 32, { fill: C.blueL, stroke: C.blue, r: 4 }), text(170, 65, "setup", { anchor: "middle", size: 12 }),
  rect(230, 44, 70, 32, { fill: C.tealL, stroke: C.teal, r: 4 }), text(265, 65, "Start", { anchor: "middle", size: 11 }),
  rect(300, 44, 300, 32, { fill: C.blueL, stroke: C.blue, r: 4 }), text(450, 65, "pick and place (motion)", { anchor: "middle", size: 12 }),
  rect(600, 44, 70, 32, { fill: C.tealL, stroke: C.teal, r: 4 }), text(635, 65, "Stop", { anchor: "middle", size: 11 }),
  arrow(265, 76, 300, 124, { stroke: C.teal }), arrow(635, 76, 600, 124, { stroke: C.teal }),
  rect(300, 124, 300, 32, { fill: C.tealL, stroke: C.teal, r: 4 }), text(450, 145, "repeat forever: watch sensor, pulse output", { anchor: "middle", size: 12 }),
  rect(240, 176, 420, 36, { fill: C.amberL, stroke: C.amber, r: 6 }), text(450, 199, "global variables · IO · properties — shared", { anchor: "middle", size: 12 }),
  line(450, 156, 450, 176, { dash: "3 3" }), line(450, 76, 450, 124, { dash: "3 3", stroke: C.mute }),
].join(""));

// ── 24. Vision coordinates ─────────────────────────────────────────────────
fig("vision-frames", "RunVision frame=: positions in pixels (origin top-left, y down), normalized 0–1, or robot millimetres on the camera's calibration plane.", 760, 260, [
  rect(16, 16, 230, 228, { fill: C.panel }), text(30, 40, "frame=pixel", { mono: true, weight: "bold" }),
  rect(40, 56, 180, 135, { fill: C.bg, stroke: C.sub, r: 2 }), arrow(40, 56, 120, 56, { stroke: C.ink }), arrow(40, 56, 40, 120, { stroke: C.ink }),
  label(124, 60, "x"), label(44, 132, "y"), circle(150, 130, 5, { fill: C.blue, stroke: C.blue }), label(110, 150, "(412, 288) px", { fill: C.blue }), label(40, 214, "640 × 480 image"),
  rect(266, 16, 230, 228, { fill: C.panel }), text(280, 40, "frame=normalized", { mono: true, weight: "bold" }),
  rect(290, 56, 180, 135, { fill: C.bg, stroke: C.sub, r: 2 }), label(286, 52, "0,0", { anchor: "end" }), label(474, 204, "1,1"),
  circle(400, 130, 5, { fill: C.blue, stroke: C.blue }), label(360, 150, "(0.64, 0.60)", { fill: C.blue }), label(290, 214, "independent of resolution"),
  rect(516, 16, 228, 228, { fill: C.panel }), text(530, 40, "frame=robot", { mono: true, weight: "bold" }),
  poly([[550, 170], [710, 170], [680, 100], [580, 100]], { fill: C.blueXL, stroke: C.blue }), circle(650, 140, 5, { fill: C.blue, stroke: C.blue }), label(600, 160, "x 312.4, y −41.0", { fill: C.blue }),
  label(530, 196, "mm on the calibration plane;"), label(530, 212, "points get z = plane Z."), label(530, 228, "needs a calibrated camera"),
].join(""));

// ── 25. Plugins ────────────────────────────────────────────────────────────
fig("plugins", "A plugin is a separate process the controller starts. It connects back over a local WebSocket and contributes steps, expression functions and live properties.", 760, 230, [
  rect(16, 16, 330, 198, { fill: C.panel }), text(30, 42, "Program", { weight: "bold" }),
  code(30, 76, "plugin scale.weigh", { fill: C.purple }), code(46, 94, "samples = 5"), code(46, 112, "grams -> $w"), code(30, 130, "end"),
  code(30, 162, "$t = scale.toOz($w)", { fill: C.purple }), code(30, 184, "Wait until $scale.stable", { fill: C.purple }),
  box(390, 80, 150, 60, "Controller", { sub: "supervises, restarts" }),
  arrow(346, 110, 390, 110, { stroke: C.blue }),
  arrow(540, 100, 600, 100, { stroke: C.purple }), arrow(600, 124, 540, 124, { stroke: C.purple }), label(570, 92, "local WS", { anchor: "middle", size: 10 }),
  box(596, 50, 150, 120, "Plugin process", { fill: C.purpleL, stroke: C.purple, sub: "Python · C# · exe\nsteps · functions\nproperties", subSize: 10 }),
  label(600, 186, "trusted code: same machine,"), label(600, 200, "controller's privileges"),
].join(""));

// ── manifest ───────────────────────────────────────────────────────────────
const manifestPath = path.join(ROOT, "manifest.json");
const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
manifest.figures = figures;
fs.writeFileSync(manifestPath, JSON.stringify(manifest, null, 2) + "\n");
console.log(`${figures.length} figures written`);
