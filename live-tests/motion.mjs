// Live checks for joint moves, motion profiles and reach (docs/program-blocks.md "Move to Joints",
// docs/manual chapters 06 / 10 / 17): Move to Joints from literals, from a joints value saved with
// Save Position → joints, and from jointsOf(); the S-curve profile (config and per move) against the
// trapezoid on the same move, blended and jumped; reachable() / jointsOf() / poseOf() in conditions,
// Set Variable and a while loop; outOfReach from jointsOf() and from a move, caught by a Try; the
// catalog, symbols, validator and text language over the wire; Move to Joints on CNC4Axis.
//
// Moves the robot (simulated on a PC with no motor driver) and CHANGES ITS CONFIG (joint limits,
// motion profile, robot type — all restored at the end). Run it against your own scratch
// controller only, never a real arm:
//
//   CTRL=localhost:9161 node live-tests/motion.mjs [filter]
import { Ctrl, runProgram, num, setv, sid, sleep } from "./harness.mjs";

const filter = process.argv[2] ?? "";
const V = !!process.env.V;

const text = (name, stringValue = "") => ({ id: sid(), name, isString: true, stringValue, displayOnMonitor: true });
const list = (name, elementType, items = []) => ({ id: sid(), name, value: 0, elementType, items });
const step = (type, extra = {}) => ({ id: sid(), type, ...extra });
const tryc = (trySteps, catchSteps) => step("Try", { trySteps, catchSteps, errorVariableName: "msg", errorCodeVariableName: "code" });
const cond = expr => ({ combinator: "ALL", items: [{ id: sid(), expr }] });
const iff = (expr, ifSteps, elseSteps = []) => step("IfCondition", { condition: cond(expr), ifSteps, elseSteps });

const cases = [];
const test = (name, fn) => cases.push({ name, fn });
const eq = (a, b, what) => { if (a !== b) throw new Error(`${what}: expected ${JSON.stringify(b)}, got ${JSON.stringify(a)}`); };
const near = (a, b, tol, what) => { if (!(Math.abs(a - b) <= tol)) throw new Error(`${what}: expected ${b} ± ${tol}, got ${a}`); };
const ok = r => { if (!r.ok) throw new Error(JSON.stringify(r)); return r; };
const joints = s => [s.joint1Angle, s.joint2X, s.joint2Z, s.joint4Angle];

const LIMITS = {
  jointLimitsEnabled: true,
  joint1Min: -170, joint1Max: 170, joint2Min: 100, joint2Max: 480,
  joint3Min: 0, joint3Max: 400, joint4Min: -360, joint4Max: 360,
};

/** Waits for the robot to stop moving. */
async function idle(c) {
  for (let i = 0; i < 200; i++) { if (!(await c.status()).moving) return; await sleep(25); }
  throw new Error("robot never stopped");
}

/** A homed ASTRO at its home pose (J1 107°, radial 435, vertical 378) with the test's limits. */
async function homeAstro(c, extra = {}) {
  const cfg = await c.cmd("GetRobotConfig");
  if (cfg.robotType !== "ASTRO") ok(await c.cmd("SetRobotConfig", { robotType: "ASTRO" }));
  ok(await c.cmd("SetRobotConfig", { ...LIMITS, motionProfile: "trapezoid", scurveSmoothness: 50, scurveJerk: null, ...extra }));
  await c.cmd("ClearFault");
  await c.cmd("SetHomed");
  await sleep(100);
  await idle(c);
}

async function savePointHere(c, name, dx = 0, dy = 0, dz = 0) {
  const st = await c.status();
  ok(await c.cmd("TeachPoint", { name }));
  ok(await c.cmd("EditPoint", { name, x: st.x + dx, y: st.y + dy, z: st.z + dz }));
}

/** Saves a point <dr> mm further out along the arm (the radial joint grows by dr; J1 unchanged). */
async function savePointRadial(c, name, dr, dz = 0) {
  const st = await c.status();
  const a = st.joint1Angle * Math.PI / 180;
  // ASTRO world: the arm points along (cos J1, −sin J1) in X / Y (the kinematics flip Y).
  await savePointHere(c, name, dr * Math.cos(a), -dr * Math.sin(a), dz);
}

// ── Move to Joints ────────────────────────────────────────────────────────

test("movejoints: literals, a saved joints value and jointsOf()", async c => {
  await homeAstro(c);
  const p = { name: "zz live motion joints", variables: [
      list("home", "Record"), num("h1"), num("h2"), num("h3"), num("h4"), num("back1"), num("back2"), num("viaFk") ],
    steps: [
      step("MoveJoints", { j1: 90, j2: 400, j3: 300, j4: 20, speed: 120 }),
      step("SavePosition", { variableName: "home", savePositionSource: "joints" }),
      setv("h1", "$home[0].j1"), setv("h2", "$home[0].j2"), setv("h3", "$home[0].j3"), setv("h4", "$home[0].j4"),
      step("MoveJoints", { j1: 60, j4: -30, speed: 120 }),          // J2/J3 stay
      step("MoveJoints", { jointsExpr: "$home[0]", speed: 120 }),  // back
      setv("back1", "$robot.joint1"), setv("back2", "$robot.joint2x"),
      step("MoveJoints", { jointsExpr: "jointsOf(poseOf($home[0]))", expressions: { j3: "$home[0].j3 - 50" }, speed: 120 }),
      setv("viaFk", "$robot.joint2z"),
    ] };
  const r = await runProgram(c, p, { sample: s => ({ j: joints(s), moving: s.moving }) });
  eq(r.status, "Complete", `status (${r.error})`);
  near(r.vars.h1, 90, 0.01, "saved J1"); near(r.vars.h2, 400, 0.01, "saved J2"); near(r.vars.h3, 300, 0.01, "saved J3"); near(r.vars.h4, 20, 0.01, "saved J4");
  near(r.vars.back1, 90, 0.01, "back to J1"); near(r.vars.back2, 400, 0.01, "back to J2");
  near(r.vars.viaFk, 250, 0.05, "jointsOf(poseOf(…)) round trip + J3 override");
  // The middle move kept J2 / J3: no sample may show them changing while J1 swings 90 → 60.
  const mid = r.samples.filter(s => s.j[0] < 89 && s.j[0] > 61);
  if (V) console.log("   mid-move samples:", mid.length);
  for (const s of mid) { near(s.j[1], 400, 0.05, "J2 held"); near(s.j[2], 300, 0.05, "J3 held"); }
  const st = await c.status();
  near(st.joint2Z, 250, 0.05, "final J3");
});

test("movejoints: a joint past its limit fails outOfReach before moving", async c => {
  await homeAstro(c);
  const before = await c.status();
  const p = { name: "zz live motion jlimit", variables: [text("msg"), text("code"), num("caught"), num("isReach")],
    steps: [
      tryc([step("MoveJoints", { j2: 520, speed: 100 })], [setv("caught", "1")]),
      setv("isReach", "$code.contains(\"outOfReach\") + $msg.contains(\"J2 limit (520 > 480)\")"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`);
  eq(r.vars.caught, 1, "caught"); eq(r.vars.isReach, 2, "code + reason");
  const after = await c.status();
  near(after.joint2X, before.joint2X, 1e-6, "J2 did not move");
  eq(after.faulted ?? false, false, "no joint-limit fault latched");
});

// ── Motion profiles ───────────────────────────────────────────────────────

async function timedMove(c, name, moveExtra) {
  ok(await c.cmd("SetSpeedOverride", { percent: 100 }));
  const p = { name, variables: [], steps: [
      step("MoveL", { offsetZ: -150, speed: 100, accel: 200, decel: 200, ...moveExtra }),
      step("MoveL", { offsetZ: 150, speed: 100, accel: 200, decel: 200, ...moveExtra }) ] };
  const r = await runProgram(c, p, { sample: s => ({ z: s.z, moving: s.moving }) });
  eq(r.status, "Complete", `${name} (${r.error})`);
  // Moving time from the samples: first to last sample with the robot moving.
  const m = r.samples.filter(s => s.moving);
  return { ms: m.length ? m[m.length - 1].t - m[0].t : 0, samples: r.samples };
}

test("profile: config S-curve takes longer than the trapezoid on the same move; per-move override too", async c => {
  await homeAstro(c);
  const trap = await timedMove(c, "zz live motion trap", {});
  ok(await c.cmd("SetRobotConfig", { motionProfile: "scurve", scurveSmoothness: 100 }));
  const cfg = await c.cmd("GetRobotConfig");
  eq(cfg.motionProfile, "scurve", "config echoes motionProfile"); eq(cfg.scurveSmoothness, 100, "smoothness");
  const scurve = await timedMove(c, "zz live motion scurve", {});
  ok(await c.cmd("SetRobotConfig", { motionProfile: "trapezoid" }));
  const perMove = await timedMove(c, "zz live motion permove", { motionProfile: "scurve" });
  const forcedTrap = await (async () => { ok(await c.cmd("SetRobotConfig", { motionProfile: "scurve", scurveSmoothness: 100 }));
    const t = await timedMove(c, "zz live motion forcedtrap", { motionProfile: "trapezoid" });
    ok(await c.cmd("SetRobotConfig", { motionProfile: "trapezoid" })); return t; })();
  // Two 150 mm moves at 100 mm/s, 200 mm/s²: trapezoid 2 × 2.0 s; S-curve at 100 % smoothness 2 × 2.5 s.
  if (V) console.log(`   trapezoid ${trap.ms} ms · s-curve ${scurve.ms} ms · per-move s-curve ${perMove.ms} ms · per-move trapezoid ${forcedTrap.ms} ms`);
  near(trap.ms, 4000, 350, "trapezoid duration");
  near(scurve.ms, 5000, 350, "S-curve duration");
  near(perMove.ms, 5000, 350, "per-move S-curve duration");
  near(forcedTrap.ms, 4000, 350, "per-move trapezoid under an S-curve config");
  const bad = await c.cmd("SetRobotConfig", { motionProfile: "wobbly" });
  eq(bad.ok, false, "unknown profile rejected");
});

test("profile: S-curve with blending and a jump completes at the targets", async c => {
  await homeAstro(c, { motionProfile: "scurve", scurveSmoothness: 60 });
  await savePointHere(c, "zz live motion b0");
  await savePointRadial(c, "zz live motion b1", -80, 0);       // in towards the column, then down
  await savePointRadial(c, "zz live motion b2", -80, -60);
  const p = { name: "zz live motion blend", variables: [num("x"), num("y"), num("z")], steps: [
      step("MoveL", { pointName: "zz live motion b1", blend: true, blendRadius: 15, speed: 80 }),
      step("MoveL", { pointName: "zz live motion b2", blend: true, blendRadius: 15, speed: 80 }),
      step("MoveL", { pointName: "zz live motion b0", speed: 80 }),
      step("JumpL", { pointName: "zz live motion b2", jumpZ: 370, speed: 80 }),
      step("JumpJ", { pointName: "zz live motion b0", jumpZ: 370, speed: 80 }),
      setv("x", "$robot.x"), setv("y", "$robot.y"), setv("z", "$robot.z") ] };
  const r = await runProgram(c, p, { sample: s => ({ x: s.x, y: s.y, z: s.z, moving: s.moving }) });
  ok(await c.cmd("SetRobotConfig", { motionProfile: "trapezoid" }));
  eq(r.status, "Complete", `status (${r.error})`);
  const pts = JSON.parse((await c.cmd("GetPoints")).points ?? "[]");
  const b0 = pts.find(q => (q.Name ?? q.name) === "zz live motion b0");
  near(r.vars.x, b0.X ?? b0.x, 0.01, "x"); near(r.vars.y, b0.Y ?? b0.y, 0.01, "y"); near(r.vars.z, b0.Z ?? b0.z, 0.01, "z");
  // No jumps in the sampled path: at 80 mm/s and ~40–60 ms between samples a step stays well under 15 mm.
  let maxStep = 0;
  for (let i = 1; i < r.samples.length; i++) {
    const a = r.samples[i - 1], b = r.samples[i];
    maxStep = Math.max(maxStep, Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z) / Math.max(1, (b.t - a.t) / 50));
  }
  if (V) console.log(`   largest step per 50 ms: ${maxStep.toFixed(2)} mm`);
  if (maxStep > 15) throw new Error(`path jumped ${maxStep.toFixed(1)} mm`);
  for (const n of ["b0", "b1", "b2"]) await c.cmd("DeletePoint", { name: `zz live motion ${n}` });
});

// ── Reach ─────────────────────────────────────────────────────────────────

test("reach: reachable / jointsOf / poseOf in conditions, Set Variable and a while loop", async c => {
  await homeAstro(c);
  await savePointHere(c, "zz live reach near");
  await savePointRadial(c, "zz live reach far", 120);        // radial 435 + 120 → past J2's 480
  const p = { name: "zz live motion reach", variables: [
      num("near"), num("far"), num("j2"), num("rx"), num("n"), num("both"), text("msg"), text("code"), num("caught"), num("isReach") ],
    steps: [
      iff("reachable(\"zz live reach near\")", [setv("near", "1")]),
      iff("not reachable(\"zz live reach far\")", [setv("far", "1")]),
      setv("j2", "jointsOf(\"zz live reach near\").j2"),
      setv("rx", "poseOf(jointsOf(\"zz live reach near\")).x - $robot.x"),
      // Walk outwards 20 mm at a time while the next point is still reachable (no motion).
      step("Loop", { loopMode: "while", loopWhileCondition: cond("$n < 50 and reachable(\"zz live reach near\") and $n * 20 < 400"), loopSteps: [setv("n", "$n + 1")] }),
      setv("both", "reachable(\"zz live reach near\", \"none\", \"none\") + reachable(\"zz live reach far\", \"\", \"none\")"),
      tryc([setv("j2", "jointsOf(\"zz live reach far\").j2")], [setv("caught", "1")]),
      setv("isReach", "$code.contains(\"outOfReach\")"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`);
  if (V) console.log("   reason:", r.vars.msg);
  eq(r.vars.near, 1, "near reachable"); eq(r.vars.far, 1, "far not reachable");
  near(r.vars.j2, 435, 0.05, "jointsOf().j2 = the radial joint");
  near(r.vars.rx, 0, 0.01, "poseOf(jointsOf(p)) = p");
  eq(r.vars.n, 20, "while loop ran on a reachable() condition");
  eq(r.vars.both, 1, "explicit tool / local arguments");
  eq(r.vars.caught, 1, "jointsOf() out of reach caught"); eq(r.vars.isReach, 1, "code outOfReach");
});

test("reach: a move to an unreachable point fails outOfReach, the robot stays put, the Try handles it", async c => {
  await homeAstro(c);
  await savePointRadial(c, "zz live reach far2", 200);
  const before = await c.status();
  const p = { name: "zz live motion reachmove", variables: [text("msg"), text("code"), num("caught"), num("isReach"), num("moved")],
    steps: [
      tryc([step("MoveL", { pointName: "zz live reach far2", speed: 100 })], [setv("caught", "1")]),
      setv("isReach", "$code.contains(\"outOfReach\")"),
      step("MoveL", { offsetZ: -10, speed: 100 }),                 // a reachable move still runs afterwards
      setv("moved", "$robot.z"),
    ] };
  const r = await runProgram(c, p);
  eq(r.status, "Complete", `status (${r.error})`);
  if (V) console.log("   reason:", r.vars.msg);
  eq(r.vars.caught, 1, "caught"); eq(r.vars.isReach, 1, "outOfReach");
  near(r.vars.moved, before.z - 10, 0.01, "the next move ran");
  const after = await c.status();
  near(after.x, before.x, 0.01, "x unchanged by the refused move");
  eq(after.faulted ?? false, false, "no fault");
  await c.cmd("DeletePoint", { name: "zz live reach far2" });
  for (const n of ["near", "far"]) await c.cmd("DeletePoint", { name: `zz live reach ${n}` });
});

// ── Over the wire: catalog, symbols, validator, text ──────────────────────

test("wire: catalog, symbols, validator and SRS text know the new step and functions", async c => {
  const cat = await c.cmd("GetStepCatalog");
  const mj = (cat.steps ?? []).find(s => s.type === "MoveJoints");
  if (!mj) throw new Error("MoveJoints missing from GetStepCatalog");
  eq(mj.aliases.includes("MoveAbsJ"), true, "alias");
  eq((cat.errorCodes ?? []).find(e => e.code === "outOfReach")?.catchable, true, "outOfReach catchable");
  const sym = await c.cmd("GetExpressionSymbols");
  for (const f of ["reachable", "jointsOf", "poseOf"])
    if (!(sym.functions ?? []).some(x => x.name === f)) throw new Error(`${f} missing from GetExpressionSymbols`);
  const v = await c.cmd("ValidateBuiltProgram", { program: { name: "zz v", steps: [
      { id: "a", type: "MoveJoints", jointsExpr: "$pts[0]" },
      { id: "b", type: "MoveL", pointName: "x", motionProfile: "wobbly" } ],
    variables: [list("pts", "Point")] } });
  const codes = (v.problems ?? []).map(q => `${q.stepId}:${q.code}:${q.field}`);
  if (V) console.log("  ", codes.join(" "));
  if (!codes.some(x => x.startsWith("a:") && x.endsWith(":jointsExpr"))) throw new Error(`no jointsExpr problem: ${codes}`);
  if (!codes.some(x => x.startsWith("b:") && x.endsWith(":motionProfile"))) throw new Error(`no motionProfile problem: ${codes}`);
  const src = "#!srs 1\nvar home: list record\nMoveAbsJ $home[0], j4=45, profile=scurve\nif reachable(\"P1\")\n  MoveJoints j1=10\nend\n";
  const parsed = await c.cmd("ParseProgramText", { text: src });
  if (V) console.log("   parse:", JSON.stringify(parsed).slice(0, 300));
  const prog = parsed.program;
  if (!prog) throw new Error(`ParseProgramText: ${JSON.stringify(parsed.problems)}`);
  eq(prog.steps[0].type, "MoveJoints", "alias parses to MoveJoints");
  eq(prog.steps[0].jointsExpr, "$home[0]", "joints value");
  const fmt = await c.cmd("FormatProgramText", { program: prog });
  if (V) console.log("   format:\n" + fmt.text);
  if (!/MoveJoints \$home\[0\], j4=45, profile=scurve/.test(fmt.text ?? "")) throw new Error(`format: ${fmt.text}`);
});

// ── CNC4Axis ──────────────────────────────────────────────────────────────

test("cnc: Move to Joints moves the axes directly (identity kinematics)", async c => {
  ok(await c.cmd("SetRobotConfig", { robotType: "CNC4Axis", jointLimitsEnabled: false }));
  await c.cmd("SetHomed"); await sleep(100); await idle(c);
  const p = { name: "zz live motion cnc", variables: [num("x"), num("y"), num("z"), num("rz"), num("j3")], steps: [
      step("MoveJoints", { j1: 40, j2: 25, j3: -12, j4: 90, speed: 60 }),
      setv("x", "$robot.x"), setv("y", "$robot.y"), setv("z", "$robot.z"), setv("rz", "$robot.rz"),
      setv("j3", "$robot.joint2z") ] };
  const r = await runProgram(c, p);
  ok(await c.cmd("SetRobotConfig", { robotType: "ASTRO" }));
  await homeAstro(c, { jointLimitsEnabled: false });
  eq(r.status, "Complete", `status (${r.error})`);
  near(r.vars.x, 40, 0.01, "X"); near(r.vars.y, 25, 0.01, "Y"); near(r.vars.z, -12, 0.01, "Z"); near(r.vars.rz, 90, 0.01, "RZ");
  near(r.vars.j3, -12, 0.01, "joint readout = axis");
});

// ── runner ────────────────────────────────────────────────────────────────

const c = new Ctrl();
await c.open();
let pass = 0, fail = 0;
for (const t of cases.filter(t => t.name.includes(filter))) {
  const t0 = Date.now();
  try { await t.fn(c); pass++; console.log(`✓ ${t.name} (${Date.now() - t0} ms)`); }
  catch (e) { fail++; console.log(`✗ ${t.name}: ${e.message}`); }
}
// Leave the scratch controller as it started: limits off, trapezoid.
try { await c.cmd("SetRobotConfig", { jointLimitsEnabled: false, motionProfile: "trapezoid", scurveSmoothness: 50, scurveJerk: null }); } catch { /* best effort */ }
for (const p of await c.programs()) if (p.name?.startsWith("zz live motion")) await c.cmd("DeleteBuiltProgram", { name: p.name });
c.close();
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
