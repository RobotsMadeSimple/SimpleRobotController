// Refresh data/step-catalog.json and data/expression-symbols.json from a running controller.
//   CTRL=127.0.0.1:9124 node docs/manual/tools/dump-catalog.mjs
// Use a throwaway controller (any port); the catalog does not depend on the robot's data, the
// symbols' aux/plugin entries are made generic by gen-reference.mjs.
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const DATA = path.resolve(HERE, "..", "data");
const HOST = process.env.CTRL || "127.0.0.1:9000";

const ws = new WebSocket(`ws://${HOST}/control`);
await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
const pending = new Map();
ws.onmessage = ev => {
  const m = JSON.parse(ev.data);
  if (m.type === "ack" && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
};
const cmd = (command, params = {}) => new Promise(res => {
  const id = `${command}-${Date.now()}`;
  pending.set(id, res);
  ws.send(JSON.stringify({ type: "Command", id, command, params }));
});
const strip = ({ type, command, id, ok, ...rest }) => rest;

fs.mkdirSync(DATA, { recursive: true });
const cat = strip(await cmd("GetStepCatalog"));
fs.writeFileSync(path.join(DATA, "step-catalog.json"), JSON.stringify(cat, null, 2) + "\n");
const sym = strip(await cmd("GetExpressionSymbols"));
delete sym.variables;
fs.writeFileSync(path.join(DATA, "expression-symbols.json"), JSON.stringify(sym, null, 2) + "\n");
console.log(`catalog v${cat.catalogVersion}: ${cat.steps.length} steps; ${sym.functions.length} functions, ${sym.properties.length} properties`);
ws.close();
process.exit(0);
