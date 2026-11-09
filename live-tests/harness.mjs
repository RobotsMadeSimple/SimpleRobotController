// Live test harness for the local SimpleRobotController: save a built program, run it, sample
// status while it runs, read back its monitor variables and final status. Node 24 (global WebSocket).
const HOST = process.env.CTRL || "127.0.0.1:9000";

export class Ctrl {
  constructor() { this.pending = new Map(); this.n = 0; }
  async open() {
    this.ws = new WebSocket(`ws://${HOST}/control`);
    await new Promise((res, rej) => { this.ws.onopen = res; this.ws.onerror = rej; });
    this.ws.onmessage = ev => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      if (m.type === "ack" && this.pending.has(m.id)) { this.pending.get(m.id)(m); this.pending.delete(m.id); }
    };
  }
  close() { this.ws.close(); }
  cmd(command, params = {}) {
    const id = `t${++this.n}${Date.now()}`;
    return new Promise((res, rej) => {
      const to = setTimeout(() => { this.pending.delete(id); rej(new Error(`timeout ${command}`)); }, 10000);
      this.pending.set(id, m => { clearTimeout(to); res(m); });
      this.ws.send(JSON.stringify({ type: "Command", id, command, params }));
    });
  }
  async status() { return this.cmd("GetStatus"); }
  async programs() { return (await this.cmd("GetPrograms")).programs ?? []; }
  async vars(name) {
    const r = await this.cmd("GetProgramVariables", { name });
    return Object.fromEntries((r.variables ?? []).map(v => [v.name, v.value]));
  }
}

export const sleep = ms => new Promise(r => setTimeout(r, ms));
let idn = 0;
export const sid = () => `live-${Date.now().toString(36)}-${++idn}`;

export const num = (name, value = 0) => ({ id: sid(), name, value, displayOnMonitor: true });
export const flag = name => ({ id: sid(), name, value: 0, isBoolean: true, displayOnMonitor: true });
export const pts = name => ({ id: sid(), name, value: 0, items: [], elementType: "Point" });
export const setv = (variableName, variableExpr) => ({ id: sid(), type: "SetVariable", variableName, variableExpr });

/**
 * Saves `program` (name, steps, variables), starts it, samples `sample(status)` every ~40 ms
 * while it runs, waits for it to leave Running, and returns { status, error, vars, samples, ms }.
 */
export async function runProgram(c, program, { sample = s => ({ x: s.x, y: s.y, z: s.z, moving: s.moving }), timeoutMs = 30000 } = {}) {
  const saved = await c.cmd("SaveBuiltProgram", { name: program.name, description: "live test", steps: program.steps, variables: program.variables });
  if (!saved.ok) throw new Error(`save failed: ${JSON.stringify(saved)}`);
  const t0 = Date.now();
  const started = await c.cmd("StartProgram", { programName: program.name });
  if (!started.ok) throw new Error(`start failed: ${JSON.stringify(started)}`);
  const samples = [];
  let summary;
  let lastVars = {};
  while (Date.now() - t0 < timeoutMs) {
    samples.push({ t: Date.now() - t0, ...sample(await c.status()) });
    summary = (await c.programs()).find(p => p.name === program.name);
    try { const v = await c.vars(program.name); if (Object.keys(v).length) lastVars = v; } catch { /* not running */ }
    if (summary && Date.now() - t0 > 300 && !["Running", "Starting", "Finishing"].includes(summary.status)) break;
    await sleep(40);
  }
  return { status: summary?.status, error: summary?.errorDescription ?? "", summary, vars: lastVars, samples, ms: Date.now() - t0 };
}
