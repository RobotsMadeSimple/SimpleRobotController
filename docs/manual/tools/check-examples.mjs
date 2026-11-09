// Parses every ```srs block in the chapters and every reference example on a controller, and
// reports parse errors — so the manual never shows code the language rejects.
// Blocks containing "…" are fragments and are skipped. Chapter snippets often use variables they
// do not declare, so only parse errors count; --validate adds the validator (useful for the
// reference examples, which are complete programs).
//   CTRL=127.0.0.1:9124 node docs/manual/tools/check-examples.mjs [--validate]
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const HOST = process.env.CTRL;
if (!HOST) { console.error("Set CTRL=host:port"); process.exit(1); }
const ws = new WebSocket(`ws://${HOST}/control`);
await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
const pending = new Map();
let seq = 0;
ws.onmessage = ev => { const m = JSON.parse(ev.data); if (m.type === "ack" && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); } };
const cmd = (command, params = {}) => new Promise(res => { const id = `c${++seq}`; pending.set(id, res); ws.send(JSON.stringify({ type: "Command", id, command, params })); });

const blocks = [];
for (const f of fs.readdirSync(path.join(ROOT, "chapters")).sort()) {
  const md = fs.readFileSync(path.join(ROOT, "chapters", f), "utf8").replace(/\r\n/g, "\n");
  for (const m of md.matchAll(/```srs\n([\s\S]*?)\n```/g)) blocks.push({ where: f, text: m[1] });
}
const ref = JSON.parse(fs.readFileSync(path.join(ROOT, "reference.json"), "utf8"));
for (const s of ref.steps) blocks.push({ where: `reference ${s.type}`, text: s.example.srs });

let bad = 0, skipped = 0;
for (const b of blocks) {
  if (b.text.includes("…")) { skipped++; continue; }
  const r = await cmd("ParseProgramText", { text: b.text, validate: process.argv.includes("--validate") });
  const errs = (r.problems ?? []).filter(p => p.severity === "error" && !/^unknown(Plugin|PluginStep)$/.test(p.code));
  if (errs.length) {
    bad++;
    console.log(`✗ ${b.where}: ${b.text.split("\n")[0]}`);
    for (const e of errs) console.log(`    line ${e.line}: ${e.code} — ${e.message}`);
  }
}
console.log(`${blocks.length - skipped} examples checked, ${skipped} fragments skipped, ${bad} with errors`);
ws.close();
process.exit(bad ? 1 : 0);
