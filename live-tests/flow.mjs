// Live checks for program flow, errors, routines and the debugger (docs/program-language.md
// §4.2): Try / catch / retry, Raise Error, Switch, Break / Next, routine parameters, Wait
// onTimeout=error, SetOutput toggle, the error status (errorCode + failing step), breakpoints,
// step mode, StepBuiltProgram, SetBreakpoints and run-from-step. No robot motion.
//
//   CTRL=localhost:9111 node live-tests/flow.mjs [filter]
import { Ctrl, runProgram, num, setv, sid, sleep } from "./harness.mjs";

const filter = process.argv[2] ?? "";
const V = !!process.env.V;

// ── step builders ───────────────────────────────────────────────────────────
const cond = expr => ({ combinator: "ALL", items: [{ id: sid(), expr }] });
const strEq = (left, right) => ({ combinator: "ALL", items: [{ id: sid(), left, operator: "==", right }] });
const iff = (condition, ifSteps, elseSteps) => ({ id: sid(), type: "IfCondition", condition, ifSteps, elseSteps });
const loop = (loopCount, loopSteps) => ({ id: sid(), type: "Loop", loopCount, loopSteps });
const brk = () => ({ id: sid(), type: "LoopControl", loopControl: "break" });
const cont = () => ({ id: sid(), type: "LoopControl", loopControl: "continue" });
const raise = (errorMessage, errorCode) => ({ id: sid(), type: "RaiseError", errorMessage, errorCode });
const tryc = (trySteps, catchSteps, extra = {}) => ({ id: sid(), type: "Try", trySteps, catchSteps, ...extra });
const text = name => ({ id: sid(), name, isString: true, stringValue: "" });
const waitUntil = (expr, waitTimeoutMs, waitTimeoutAction) =>
  ({ id: sid(), type: "Wait", waitMode: "condition", waitCondition: cond(expr), waitTimeoutMs, waitTimeoutAction });

const cases = [];
const test = (name, fn) => cases.push({ name, fn });
const eq = (a, b, what) => { if (a !== b) throw new Error(`${what}: expected ${JSON.stringify(b)}, got ${JSON.stringify(a)}`); };

async function summaryOf(c, name) { return (await c.programs()).find(p => p.name === name); }

/** Waits until the program is no longer Running (paused, complete or error). */
async function settle(c, name, timeoutMs = 10000) {
  const t0 = Date.now();
  let s;
  while (Date.now() - t0 < timeoutMs) {
    s = await summaryOf(c, name);
    if (s && Date.now() - t0 > 150 && !["Running", "Starting", "Finishing"].includes(s.status)) return s;
    await sleep(40);
  }
  return s;
}

async function save(c, p) {
  const r = await c.cmd("SaveBuiltProgram", { id: p.id, name: p.name, description: "live test", steps: p.steps,
    variables: p.variables, isRoutine: !!p.isRoutine });
  if (!r.ok) throw new Error(`save failed: ${JSON.stringify(r)}`);
}

// ── cases ───────────────────────────────────────────────────────────────────

test("try catch: raise caught, error code tested, program continues", async c => {
  const p = { name: "zz live flow try", variables: [num("x"), num("y"), num("z"), text("code")], steps: [
    tryc([setv("x", "1"), raise("no part {$x}", "noPart"), setv("x", "2")],
         [setv("y", "5"), iff(strEq("$code", "noPart"), [setv("z", "1")])],
         { errorCodeVariableName: "code" }),
    setv("z", "$z + 10"),
  ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", "status"); eq(r.vars.x, 1, "x"); eq(r.vars.y, 5, "y"); eq(r.vars.z, 11, "z");
});

test("try retry: body re-runs, then the catch", async c => {
  const p = { name: "zz live flow retry", variables: [num("n"), num("y")], steps: [
    tryc([setv("n", "$n + 1"), raise("again")], [setv("y", "1")], { tryRetries: 2, tryRetryDelayMs: 50 }),
  ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", "status"); eq(r.vars.n, 3, "attempts"); eq(r.vars.y, 1, "catch ran");
});

test("uncaught raise: Error, errorCode and the failing step", async c => {
  const w = { id: sid(), type: "Wait", waitMs: 20 };
  const bad = raise("stop here", "jam");
  const p = { name: "zz live flow raise", variables: [num("x")], steps: [w, bad, setv("x", "1")] };
  const r = await runProgram(c, p);
  eq(r.status, "Error", "status"); eq(r.error, "stop here", "message");
  eq(r.summary.errorCode, "jam", "errorCode"); eq(r.summary.currentStepId, bad.id, "failing step");
});

test("wait onTimeout=error: uncaught is waitTimeout, caught continues", async c => {
  const w = waitUntil("0", 50, "error");
  let r = await runProgram(c, { name: "zz live flow wait", variables: [num("x")], steps: [w, setv("x", "1")] });
  eq(r.status, "Error", "status"); eq(r.summary.errorCode, "waitTimeout", "errorCode"); eq(r.summary.currentStepId, w.id, "step");
  r = await runProgram(c, { name: "zz live flow wait2", variables: [num("x"), num("y")],
    steps: [tryc([waitUntil("0", 50, "error"), setv("x", "1")], [setv("y", "1")])] });
  eq(r.status, "Complete", "caught status"); eq(r.vars.x, 0, "x"); eq(r.vars.y, 1, "y");
});

test("switch picks the matching case", async c => {
  const sw = { id: sid(), type: "Switch", switchExpr: "$n", switchCases: [
    { id: sid(), values: ["1"], steps: [setv("x", "10")] },
    { id: sid(), values: ["2", "3"], steps: [setv("x", "20")] },
  ], switchDefault: [setv("x", "-1")] };
  const r = await runProgram(c, { name: "zz live flow switch", variables: [num("x"), num("n", 3)], steps: [sw] });
  eq(r.status, "Complete", "status"); eq(r.vars.x, 20, "x");
});

test("break and continue", async c => {
  const p = { name: "zz live flow loops", variables: [num("i"), num("x"), num("y")], steps: [
    loop(10, [setv("i", "$i + 1"), iff(cond("$i % 2 == 1"), [cont()]), setv("x", "$x + 1"), iff(cond("$i >= 6"), [brk()])]),
    setv("y", "1"),
  ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", "status"); eq(r.vars.i, 6, "i"); eq(r.vars.x, 3, "x"); eq(r.vars.y, 1, "after");
});

test("routine parameters in / out / inOut and Return", async c => {
  const routine = { id: "zz-live-flow-routine", name: "zz live flow routine", isRoutine: true,
    variables: [{ ...num("a"), paramMode: "in" }, { ...num("acc"), paramMode: "inOut" }, { ...num("out"), paramMode: "out" }],
    steps: [setv("acc", "$acc + $a"), setv("out", "$a * 2"), { id: sid(), type: "Return" }, setv("out", "999")] };
  await save(c, routine);
  const call = { id: sid(), type: "CallRoutine", routineId: routine.id, routineName: routine.name,
    routineArgs: { a: "$k + 1", acc: "$total" },
    routineOutputs: [{ key: "acc", variableName: "total" }, { key: "out", variableName: "twice" }] };
  const r = await runProgram(c, { name: "zz live flow call", variables: [num("k", 4), num("total", 100), num("twice")], steps: [call, call] });
  eq(r.status, "Complete", "status"); eq(r.vars.total, 110, "total"); eq(r.vars.twice, 10, "twice");
  await c.cmd("DeleteBuiltProgram", { name: routine.name });
});

test("SetOutput toggle and expression value", async c => {
  const out = extra => ({ id: sid(), type: "SetOutput", outputCard: "stb", outputNumber: 2, ...extra });
  const r = await runProgram(c, { name: "zz live flow output", variables: [num("a"), num("b"), num("n", 5)], steps: [
    out({ outputValue: false }), out({ outputAction: "toggle" }), setv("a", "$stb.out2"),
    out({ outputValue: true, expressions: { outputValue: "$n < 3" } }), setv("b", "$stb.out2"),
  ] });
  eq(r.status, "Complete", "status"); eq(r.vars.a, 1, "toggled on"); eq(r.vars.b, 0, "expression off");
});

test("debugger: breakpoint, step, SetBreakpoints, continue", async c => {
  const a = setv("x", "1"), b = setv("x", "2"), d = setv("x", "3"), e = setv("x", "4");
  const p = { name: "zz live flow debug", variables: [num("x")], steps: [a, b, d, e] };
  await save(c, p);
  let r = await c.cmd("ExecuteBuiltProgram", { name: p.name, breakpoints: [b.id] });
  if (!r.ok) throw new Error(JSON.stringify(r));
  let s = await settle(c, p.name);
  eq(s.status, "Stopped", "paused"); eq(s.pauseReason, "breakpoint", "reason"); eq(s.pausedAtStepId, b.id, "at b");
  eq((await c.vars(p.name)).x, 1, "x before b");

  r = await c.cmd("StepBuiltProgram", {});
  if (!r.ok) throw new Error(JSON.stringify(r));
  s = await settle(c, p.name);
  eq(s.pauseReason, "step", "step reason"); eq(s.pausedAtStepId, d.id, "at d"); eq((await c.vars(p.name)).x, 2, "x after b");

  r = await c.cmd("SetBreakpoints", { stepIds: [e.id] });
  eq(r.count, 1, "breakpoint count");
  await c.cmd("StartProgram", { programName: p.name }); // Continue
  s = await settle(c, p.name);
  eq(s.pauseReason, "breakpoint", "reason 2"); eq(s.pausedAtStepId, e.id, "at e");
  await c.cmd("StartProgram", { programName: p.name });
  s = await settle(c, p.name);
  eq(s.status, "Complete", "complete"); eq(s.pauseReason ?? null, null, "pause cleared");
});

test("debugger: step mode and run from a step inside a loop", async c => {
  const inner = setv("y", "$y + 1");
  const p = { name: "zz live flow from", variables: [num("x"), num("y")], steps: [
    setv("x", "100"), loop(2, [setv("x", "$x + 1"), inner]),
  ] };
  await save(c, p);
  let r = await c.cmd("ExecuteBuiltProgram", { name: p.name, startAtStepId: inner.id, stepMode: true });
  if (!r.ok) throw new Error(JSON.stringify(r));
  let s = await settle(c, p.name);
  eq(s.pauseReason, "step", "step mode"); eq(s.pausedAtStepId, inner.id, "starts at the inner step");
  await c.cmd("StartProgram", { programName: p.name }); // Continue leaves step mode
  s = await settle(c, p.name);
  eq(s.status, "Complete", "complete");
  const v = await c.vars(p.name);
  eq(v.y, 2, "y"); eq(v.x, 1, "x (first assignment skipped, one full pass)");

  r = await c.cmd("ExecuteBuiltProgram", { name: p.name, startAtStepId: "no-such-step" });
  eq(r.ok, false, "bad start step refused");
  if (V) console.log("   refused:", r.error);
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
  if (p.name.startsWith("zz live flow")) await c.cmd("DeleteBuiltProgram", { name: p.name });
c.close();
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
