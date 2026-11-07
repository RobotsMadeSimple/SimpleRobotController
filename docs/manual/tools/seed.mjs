// Seeds a THROWAWAY controller with the generic example data the manual's screenshots use:
// points, tools, a local frame, a grid, a stack, a vision program, a disabled camera, the example
// programs in seed-programs/, and one "Example <Type>" program per instruction (its reference
// example). Never point this at a live robot's controller.
//   CTRL=127.0.0.1:9124 node docs/manual/tools/seed.mjs
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const HOST = process.env.CTRL;
if (!HOST || HOST.endsWith(":9000")) { console.error("Set CTRL=host:port of a throwaway controller (not :9000)."); process.exit(1); }

const ws = new WebSocket(`ws://${HOST}/control`);
await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
const pending = new Map();
let seq = 0;
ws.onmessage = ev => { const m = JSON.parse(ev.data); if (m.type === "ack" && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); } };
const cmd = (command, params = {}) => new Promise(res => { const id = `s${++seq}`; pending.set(id, res); ws.send(JSON.stringify({ type: "Command", id, command, params })); });
const check = (what, r) => { if (!r.ok) console.log(`  ! ${what}: ${r.error ?? ""} ${r.message ?? ""}`); };

const points = {
  Home: [250, 0, 200, 0, 0, 0], Pick1: [300, -120, 40, 0, 0, 0], Place1: [300, 120, 40, 0, 0, 90],
  Approach: [300, 0, 120, 0, 0, 0], TrayOrigin: [180, 150, 30, 0, 0, 0], BinBase: [380, -150, 20, 0, 0, 0], Via1: [360, 0, 60, 0, 0, 0],
};
for (const [name, [x, y, z, rx, ry, rz]] of Object.entries(points)) {
  await cmd("TeachPoint", { name });
  check(`point ${name}`, await cmd("EditPoint", { name, x, y, z, rx, ry, rz }));
}
check("tool", await cmd("CreateTool", { name: "Gripper", x: 0, y: 0, z: 85, rx: 0, ry: 0, rz: 0, description: "Two-finger gripper" }));
check("tool", await cmd("CreateTool", { name: "Probe", x: 12, y: 0, z: 60, rx: 0, ry: 0, rz: 0, description: "Touch probe" }));
check("local", await cmd("CreateLocal", { name: "Fixture1", x: 220, y: -60, z: 10, rx: 0, ry: 0, rz: 15, description: "Work fixture" }));
check("grid", await cmd("SaveGrid", { id: "grid-tray", name: "Tray", basePointName: "TrayOrigin", rowOffsetX: 30, rowOffsetY: 0, rowOffsetZ: 0, colOffsetX: 0, colOffsetY: 30, colOffsetZ: 0, rowCount: 3, colCount: 4, rotation: 0 }));
check("stack", await cmd("SaveStack", { id: "stack-bins", name: "Bins", basePointName: "BinBase", offsetX: 0, offsetY: 0, offsetZ: 12, maxCount: 5 }));
const listCams = async () => { const r = (await cmd("GetCameras")).cameras ?? []; const l = typeof r === "string" ? JSON.parse(r) : r; return Array.isArray(l) ? l : []; };
const tops = (await listCams()).filter(c => c.name === "Top Camera");
for (const extra of tops.slice(1)) await cmd("RemoveCamera", { id: extra.id });
if (!tops.length) check("camera", await cmd("AddCamera", { name: "Top Camera", deviceIndex: 9, enabled: false, width: 640, height: 480, targetFps: 5 }));
const camList = await listCams();
const camId = (Array.isArray(camList) ? camList : []).find(c => c.name === "Top Camera")?.id ?? "";
check("vision", await cmd("SaveVisionProgram", {
  id: "vis-detect", name: "Detect Parts", cameraId: camId, layers: [],
  tools: [
    { id: "t-edge", name: "Top edge", type: "findLine", enabled: true, input: "camera", fixtureId: null, includeInPass: true,
      region: { shape: "rect", x: 0.2, y: 0.2, width: 0.5, height: 0.12, rotation: 0 }, params: {} },
    { id: "t-slots", name: "Slots", type: "pixelCount", enabled: true, input: "camera", fixtureId: null, includeInPass: true,
      region: { shape: "rect", x: 0.25, y: 0.4, width: 0.5, height: 0.4, rotation: 0 }, grid: { rows: 3, cols: 4, inset: 0.1 },
      params: { mode: "gray", grayMin: 0, grayMax: 80 } },
  ],
}));

async function saveText(text, meta) {
  const p = await cmd("ParseProgramText", { text, validate: false });
  if (!p.program) { console.log(`  ! parse ${meta.name}: ${JSON.stringify(p.problems).slice(0, 400)}`); return false; }
  check(`save ${meta.name}`, await cmd("SaveBuiltProgram", { ...p.program, ...meta }));
  const errs = (p.problems ?? []).filter(x => x.severity === "error");
  if (errs.length) console.log(`  ! ${meta.name}: ${JSON.stringify(errs).slice(0, 300)}`);
  return true;
}
const progDir = path.join(HERE, "seed-programs");
for (const f of fs.readdirSync(progDir).sort()) {
  const text = fs.readFileSync(path.join(progDir, f), "utf8").replace(/^﻿/, "");
  const m = text.match(/^@program (\{.*\})/m);
  await saveText(text, m ? JSON.parse(m[1]) : { name: f.replace(/\.srs$/, "") });
}
const ref = JSON.parse(fs.readFileSync(path.join(ROOT, "reference.json"), "utf8"));
let n = 0;
for (const s of ref.steps) if (s.example.srs && await saveText(s.example.srs, { name: `Example ${s.type}`, description: `${s.name} example` })) n++;
console.log(`seeded ${Object.keys(points).length} points, ${n} instruction examples`);
ws.close();
process.exit(0);
