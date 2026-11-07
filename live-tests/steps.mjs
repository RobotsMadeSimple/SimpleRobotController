// Live checks for the motion, data and operator steps (docs/program-language.md §4.3): Move Arc
// (path sampled against the circle, a full circle back to the start, collinear → arcDegenerate),
// Save Position (Points list and saved point), Set Speed Factor ($program.speedFactor and the
// move getting slower), List (every action), the expression additions (inBox, inCircle,
// .length / .indexOf / .toNumber) and Prompt + AnswerPrompt (answers, bad values, stale answers,
// timeouts, promptTimeout caught by a Try), plus the validator's checks over the wire.
//
// Moves the robot: relative moves of up to 40 mm in X and 20 mm in Y — simulated on a PC with no
// motor driver. Run against your own scratch controller, never an arm that is not clear to move:
//
//   CTRL=localhost:9112 node live-tests/steps.mjs [filter]
import { Ctrl, runProgram, num, pts, setv, sid, sleep } from "./harness.mjs";

const filter = process.argv[2] ?? "";
const V = !!process.env.V;

// ── step builders ───────────────────────────────────────────────────────────
const text = (name, stringValue = "") => ({ id: sid(), name, isString: true, stringValue, displayOnMonitor: true });
const list = (name, elementType, items = []) => ({ id: sid(), name, value: 0, elementType, items });
const listOp = (variableName, listAction, extra = {}) => ({ id: sid(), type: "ListOp", variableName, listAction, ...extra });
const moveC = (via, end, extra = {}) => ({ id: sid(), type: "MoveC", viaPointName: via, pointName: end, ...extra });
const moveL = extra => ({ id: sid(), type: "MoveL", ...extra });
const speedFactor = speedFactor => ({ id: sid(), type: "SetSpeedFactor", speedFactor });
const savePos = extra => ({ id: sid(), type: "SavePosition", ...extra });
const prompt = extra => ({ id: sid(), type: "Prompt", promptMessage: "Live test prompt", ...extra });
const tryc = (trySteps, catchSteps, extra = {}) => ({ id: sid(), type: "Try", trySteps, catchSteps, ...extra });
const strEq = (left, right) => ({ combinator: "ALL", items: [{ id: sid(), left, operator: "==", right }] });
const iff = (condition, ifSteps) => ({ id: sid(), type: "IfCondition", condition, ifSteps });

const cases = [];
const test = (name, fn) => cases.push({ name, fn });
const eq = (a, b, what) => { if (a !== b) throw new Error(`${what}: expected ${JSON.stringify(b)}, got ${JSON.stringify(a)}`); };
const near = (a, b, tol, what) => { if (!(Math.abs(a - b) <= tol)) throw new Error(`${what}: expected ${b} ± ${tol}, got ${a}`); };
const ok = r => { if (!r.ok) throw new Error(JSON.stringify(r)); return r; };

async function summaryOf(c, name) { return (await c.programs()).find(p => p.name === name); }

async function save(c, p) {
  ok(await c.cmd("SaveBuiltProgram", { name: p.name, description: "live test", steps: p.steps, variables: p.variables }));
}

/** Waits until `pred(summary)` holds (or the timeout). */
async function until(c, name, pred, timeoutMs = 10000) {
  const t0 = Date.now();
  let s;
  while (Date.now() - t0 < timeoutMs) {
    s = await summaryOf(c, name);
    if (s && pred(s)) return s;
    await sleep(30);
  }
  throw new Error(`timed out waiting on ${name}: ${JSON.stringify(s)}`);
}
const finished = s => !["Running", "Starting", "Finishing"].includes(s.status);

/** Saves a point at the robot's position + (dx, dy, dz). */
async function pointAt(c, name, dx, dy, dz = 0) {
  const st = await c.status();
  ok(await c.cmd("TeachPoint", { name }));
  ok(await c.cmd("EditPoint", { name, x: st.x + dx, y: st.y + dy, z: st.z + dz }));
}

/** Saved points as { name, x, y, z } (the repository serialises them PascalCase). */
async function points(c) {
  return JSON.parse((await c.cmd("GetPoints")).points ?? "[]")
    .map(pt => ({ name: pt.Name ?? pt.name, x: pt.X ?? pt.x, y: pt.Y ?? pt.y, z: pt.Z ?? pt.z }));
}

// ── expressions ─────────────────────────────────────────────────────────────

test("expressions: inBox, inCircle and the text helpers", async c => {
  const p = { name: "zz live steps expr", variables: [
      text("code", "A-42"), text("qty", " 17 "), text("bad", "12abc"),
      num("len"), num("dash"), num("none"), num("q"), num("b"), num("box"), num("out"), num("circle"), num("has") ],
    steps: [
      setv("len", "$code.length"), setv("dash", "$code.indexOf(\"-\")"), setv("none", "$code.indexOf(\"z\")"),
      setv("q", "$qty.toNumber() * 2"), setv("b", "$bad.toNumber()"), setv("has", "$code.contains(\"42\")"),
      setv("box", "inBox(1, 2, 3, 0, 0, 0, 5, 5, 5)"), setv("out", "inBox(1, 2, 9, 5, 5, 5, 0, 0, 0)"),
      setv("circle", "inCircle(3, 4, 0, 0, 5)"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`);
  eq(r.vars.len, 4, "length"); eq(r.vars.dash, 1, "indexOf"); eq(r.vars.none, -1, "indexOf absent");
  eq(r.vars.q, 34, "toNumber"); eq(r.vars.b, 0, "bad text is 0"); eq(r.vars.has, 1, "contains in a field");
  eq(r.vars.box, 1, "inBox"); eq(r.vars.out, 0, "outside the box"); eq(r.vars.circle, 1, "inCircle edge");
});

// ── lists ───────────────────────────────────────────────────────────────────

test("list: number list actions", async c => {
  const p = { name: "zz live steps list", variables: [
      list("v", "Number", [{ value: 3 }, { value: 1 }, { value: 2 }]), list("w", "Number"),
      num("n"), num("first"), num("last"), num("total"), num("copied") ],
    steps: [
      listOp("v", "append", { listValueExpr: "5" }),            // 3 1 2 5
      listOp("v", "sort"),                                       // 1 2 3 5
      listOp("v", "removeAt", { listIndex: 0 }),                 // 2 3 5
      listOp("v", "insert", { listIndex: 1, listValueExpr: "$v[0] * 10" }), // 2 20 3 5
      listOp("v", "set", { listIndex: 3, listValueExpr: "50" }), // 2 20 3 50
      listOp("v", "sort", { listSortDescending: true }),         // 50 20 3 2
      listOp("w", "copyFrom", { listSourceVariable: "v" }),
      listOp("w", "reverse"),                                    // 2 3 20 50
      listOp("v", "clear"),
      setv("n", "len($v)"), setv("first", "$w[0]"), setv("last", "$w[3]"), setv("total", "sum($w)"), setv("copied", "len($w)"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`);
  eq(r.vars.n, 0, "cleared"); eq(r.vars.copied, 4, "copy"); eq(r.vars.first, 2, "first"); eq(r.vars.last, 50, "last"); eq(r.vars.total, 75, "sum");
});

test("list: points and records", async c => {
  const p = { name: "zz live steps list2", variables: [
      list("p", "Point"), list("r", "Record", [{ id: 1, w: 5 }, { id: 2, w: 1 }]),
      num("hereX"), num("robotX"), num("z1"), num("firstId"), num("score") ],
    steps: [
      listOp("p", "append"),                                                  // here
      listOp("p", "append", { listFields: { x: "1", y: "2", z: "3" } }),
      listOp("p", "setField", { listIndex: 1, listFields: { z: "$p[1].z + 10" } }),
      listOp("r", "sort", { listSortField: "w" }),
      listOp("r", "setField", { listIndex: 0, listFields: { score: "0.5" } }),
      setv("hereX", "$p[0].x"), setv("robotX", "$robot.x"), setv("z1", "$p[1].z"),
      setv("firstId", "$r[0].id"), setv("score", "$r[0].score"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`);
  near(r.vars.hereX, r.vars.robotX, 1e-6, "append with no fields = here");
  eq(r.vars.z1, 13, "setField"); eq(r.vars.firstId, 2, "sorted by w"); eq(r.vars.score, 0.5, "new field");
});

test("list: out-of-range index is caught by a Try as indexOutOfRange", async c => {
  const p = { name: "zz live steps list3", variables: [list("v", "Number"), text("code"), num("caught")],
    steps: [
      tryc([listOp("v", "removeAt", { listIndex: 4 })], [iff(strEq("$code", "indexOutOfRange"), [setv("caught", "1")])],
           { errorCodeVariableName: "code" }),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`); eq(r.vars.caught, 1, "caught");
});

test("list: a misfit ends the run with badListOperation", async c => {
  const p = { name: "zz live steps list4", variables: [list("v", "Number")],
    steps: [tryc([listOp("v", "setField", { listIndex: 0, listFields: { x: "1" } })], [])] };
  const r = await runProgram(c, p);
  eq(r.status, "Error", "not catchable"); eq(r.summary.errorCode, "badListOperation", "code");
});

// ── save position ───────────────────────────────────────────────────────────

test("save position: points list and a saved point", async c => {
  await c.cmd("DeletePoint", { name: "zz live here" }).catch(() => {});
  const p = { name: "zz live steps save", variables: [pts("taught"), num("n"), num("dx"), num("rx")],
    steps: [
      savePos({ variableName: "taught", savePositionMode: "append" }),
      savePos({ variableName: "taught", savePositionMode: "insert", savePositionIndex: 0 }),
      savePos({ savePositionTo: "point", pointName: "zz live here" }),
      setv("n", "len($taught)"), setv("dx", "$taught[1].x - $robot.x"), setv("rx", "$robot.x"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`); eq(r.vars.n, 2, "two poses"); near(r.vars.dx, 0, 1e-6, "pose");
  const saved = (await points(c)).find(pt => pt.name === "zz live here");
  if (!saved) throw new Error("saved point missing");
  near(saved.x, r.vars.rx, 1e-6, "saved point x");
  ok(await c.cmd("DeletePoint", { name: "zz live here" }));
});

// ── speed factor ────────────────────────────────────────────────────────────

test("speed factor: readable and slows the move", async c => {
  const run = async (factor, name) => {
    const p = { name, variables: [num("seen")], steps: [
      speedFactor(factor), setv("seen", "$program.speedFactor"),
      moveL({ offsetX: 20, speed: 40, accel: 2000, decel: 2000 }), moveL({ offsetX: -20, speed: 40, accel: 2000, decel: 2000 }),
    ] };
    const r = await runProgram(c, p);
    eq(r.status, "Complete", `status (${r.error})`); eq(r.vars.seen, factor, "$program.speedFactor");
    return r.ms;
  };
  const full = await run(100, "zz live steps speed1");
  const quarter = await run(25, "zz live steps speed2");
  if (V) console.log(`   100 %: ${full} ms, 25 %: ${quarter} ms`);
  if (!(quarter > full * 2.5)) throw new Error(`25 % run took ${quarter} ms vs ${full} ms at 100 %`);
});

// ── move arc ────────────────────────────────────────────────────────────────

test("move arc: follows the circle, then a second arc closes it", async c => {
  const st = await c.status();
  await pointAt(c, "zz arc via", 20, 20);
  await pointAt(c, "zz arc end", 40, 0);
  await pointAt(c, "zz arc back", 20, -20);
  await pointAt(c, "zz arc start", 0, 0);
  const p = { name: "zz live steps arc", variables: [], steps: [
    moveC("zz arc via", "zz arc end", { speed: 40, arcTolerance: 0.05 }),
    moveC("zz arc back", "zz arc start", { speed: 40 }),
  ] };
  const r = await runProgram(c, p, { sample: s => ({ x: s.x, y: s.y, z: s.z }) });
  eq(r.status, "Complete", `status (${r.error})`);
  const cx = st.x + 20, cy = st.y;
  let worst = 0;
  for (const s of r.samples) worst = Math.max(worst, Math.abs(Math.hypot(s.x - cx, s.y - cy) - 20));
  if (V) console.log(`   ${r.samples.length} samples, worst radial error ${worst.toFixed(4)} mm`);
  near(worst, 0, 0.2, "samples on the circle");
  if (r.samples.length < 10) throw new Error(`only ${r.samples.length} samples`);
  const end = await c.status();
  near(end.x, st.x, 1e-3, "back at the start x"); near(end.y, st.y, 1e-3, "back at the start y");
});

test("move arc: collinear points fail arcDegenerate", async c => {
  await pointAt(c, "zz arc mid", 10, 0);
  await pointAt(c, "zz arc far", 20, 0);
  const r = await runProgram(c, { name: "zz live steps arc2", variables: [], steps: [moveC("zz arc mid", "zz arc far")] });
  eq(r.status, "Error", "status"); eq(r.summary.errorCode, "arcDegenerate", "code");
  if (!r.error.includes("in a line")) throw new Error(r.error);
});

// ── prompt ──────────────────────────────────────────────────────────────────

test("prompt: published, bad answers refused, answered, then stale", async c => {
  const p = { name: "zz live steps prompt", variables: [num("choice", -1), num("qty")], steps: [
    prompt({ promptTitle: "Setup", promptButtons: ["Go", "Skip"], promptResultVariableName: "choice",
             promptInput: "number", promptDefault: "2 + 3", promptMin: 1, promptMax: 10, promptInputVariableName: "qty" }),
  ] };
  await save(c, p);
  ok(await c.cmd("StartProgram", { programName: p.name }));
  const s = await until(c, p.name, x => x.prompt);
  const pr = s.prompt;
  eq(pr.title, "Setup", "title"); eq(pr.buttons.join(","), "Go,Skip", "buttons"); eq(pr.input, "number", "input");
  eq(pr.default, "5", "default"); eq(pr.min, 1, "min"); eq(pr.stepId, p.steps[0].id, "stepId");

  let a = await c.cmd("AnswerPrompt", { promptId: pr.id, button: 1, value: "50" });
  eq(a.ok, false, "out of range refused"); if (!String(a.error).startsWith("badValue")) throw new Error(a.error);
  a = await c.cmd("AnswerPrompt", { promptId: pr.id, button: 7, value: "3" });
  if (!String(a.error).startsWith("badButton")) throw new Error(JSON.stringify(a));
  ok(await c.cmd("AnswerPrompt", { promptId: pr.id, button: 1, value: "7" }));
  const done = await until(c, p.name, finished);
  eq(done.status, "Complete", "status"); eq(done.prompt ?? null, null, "prompt cleared");
  const v = await c.vars(p.name);
  eq(v.choice, 1, "button index"); eq(v.qty, 7, "typed number");
  a = await c.cmd("AnswerPrompt", { promptId: pr.id, button: 0 });
  if (!String(a.error).startsWith("stalePrompt")) throw new Error(JSON.stringify(a));
});

test("prompt: timeout presses the default button; no button is a catchable promptTimeout", async c => {
  const p = { name: "zz live steps prompt2", variables: [num("choice", -1), text("code"), num("caught")], steps: [
    prompt({ promptButtons: ["Retry", "Skip"], promptTimeoutMs: 300, promptTimeoutButton: 1, promptResultVariableName: "choice" }),
    tryc([prompt({ promptTimeoutMs: 200 })], [iff(strEq("$code", "promptTimeout"), [setv("caught", "1")])], { errorCodeVariableName: "code" }),
  ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`); eq(r.vars.choice, 1, "timeout button"); eq(r.vars.caught, 1, "promptTimeout caught");
});

test("prompt: refused while stopped, re-shown on continue", async c => {
  const p = { name: "zz live steps prompt3", variables: [num("choice", -1)], steps: [
    prompt({ promptButtons: ["A", "B"], promptResultVariableName: "choice" }),
  ] };
  await save(c, p);
  ok(await c.cmd("StartProgram", { programName: p.name }));
  const first = (await until(c, p.name, x => x.prompt)).prompt;
  ok(await c.cmd("StopProgram", { programName: p.name }));
  await until(c, p.name, x => x.status === "Stopped");
  await sleep(1000);
  const a = await c.cmd("AnswerPrompt", { promptId: first.id, button: 0 });
  if (!String(a.error).startsWith("programPaused")) throw new Error(JSON.stringify(a));
  ok(await c.cmd("StartProgram", { programName: p.name })); // Continue
  const again = (await until(c, p.name, x => x.prompt && x.prompt.id !== first.id)).prompt;
  ok(await c.cmd("AnswerPrompt", { promptId: again.id, button: 1 }));
  const done = await until(c, p.name, finished);
  eq(done.status, "Complete", "status"); eq((await c.vars(p.name)).choice, 1, "answer");
});

// ── validation and catalog over the wire ───────────────────────────────────

test("validator and catalog know the new steps", async c => {
  const program = { name: "zz live steps check", variables: [list("v", "Number")], steps: [
    { id: sid(), type: "MoveC", pointName: "zz arc end" },
    listOp("v", "append"),
    prompt({ promptButtons: ["1", "2", "3", "4", "5"] }),
  ] };
  const r = ok(await c.cmd("ValidateBuiltProgram", { program }));
  const has = (code, field) => r.problems.some(p => p.code === code && p.field === field);
  if (!has("missingField", "via")) throw new Error("no via problem: " + JSON.stringify(r.problems));
  if (!has("missingField", "listValueExpr")) throw new Error("no list value problem");
  if (!has("missingField", "promptButtons")) throw new Error("no buttons problem");
  const cat = ok(await c.cmd("GetStepCatalog"));
  if (!cat.errorCodes.some(e => e.code === "badListOperation" && e.catchable === false)) throw new Error("badListOperation missing");
});

// ── run ─────────────────────────────────────────────────────────────────────
const c = new Ctrl();
await c.open();
let pass = 0, fail = 0;
for (const t of cases.filter(t => t.name.includes(filter))) {
  try { await t.fn(c); pass++; console.log(`ok   ${t.name}`); }
  catch (e) { fail++; console.log(`FAIL ${t.name}\n     ${e.message}`); }
}
for (const p of await c.programs())
  if (p.name.startsWith("zz live steps")) await c.cmd("DeleteBuiltProgram", { name: p.name });
for (const pt of await points(c))
  if (pt.name.startsWith("zz arc") || pt.name === "zz live here") await c.cmd("DeletePoint", { name: pt.name });
c.close();
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
