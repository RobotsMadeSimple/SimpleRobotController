// Live tests: "Stop or record on condition" on move steps, against the local controller.
// Robot motion is simulated (no driver attached); aux axes simulate offline.
import { Ctrl, runProgram, num, flag, pts, setv, sid, sleep } from "./harness.mjs";

const AUX = "AUX_STEPPER_001";
const auxPos = a => `$aux.${AUX}.${a}.position`;
const near = (v, lo, hi) => typeof v === "number" && v >= lo && v <= hi;
const stop = (expr, mode, extra = {}) => ({
  moveStopCondition: { combinator: "ALL", items: [{ id: sid(), expr }] }, moveStopMode: mode, ...extra,
});

const robotVars = () => [num("sx"), num("sy"), num("sz"), num("hx"), num("hz"), num("ex"), num("ey"), num("ez"), num("ex2"), flag("fired"), pts("hit")];
const startRobot = [setv("sx", "$robot.x"), setv("sy", "$robot.y"), setv("sz", "$robot.z")];
const endRobot = [
  setv("ex", "$robot.x - $sx"), setv("ey", "$robot.y - $sy"), setv("ez", "$robot.z - $sz"),
  { id: sid(), type: "Wait", waitMs: 400 },
  setv("ex2", "$robot.x - $sx"),
  { id: sid(), type: "Wait", waitMs: 600 },
];
const readHit = [setv("hx", "$hit[0].x - $sx"), setv("hz", "$hit[0].z - $sz")];

const TESTS = [
  {
    name: "MoveL stop: ends at the condition, next step runs",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "MoveL", offsetX: 40, speed: 20, ...stop("$robot.x >= $sx + 15", "stop", { moveStopPositionVariableName: "hit", moveStopFlagVariableName: "fired" }) },
      ...readHit, ...endRobot],
    check: v => [
      ["fired = 1", v.fired === 1],
      ["recorded x ≈ +15", near(v.hx, 15, 16.5)],
      ["robot stopped ≈ +15 (not +40)", near(v.ex, 15, 17)],
      ["robot stays stopped", Math.abs(v.ex2 - v.ex) < 0.01],
    ],
  },
  {
    name: "MoveL record: captures +15, finishes at +40",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "MoveL", offsetX: 40, speed: 20, ...stop("$robot.x >= $sx + 15", "record", { moveStopPositionVariableName: "hit", moveStopFlagVariableName: "fired" }) },
      ...readHit, ...endRobot],
    check: v => [
      ["fired = 1", v.fired === 1],
      ["recorded x ≈ +15", near(v.hx, 15, 16.5)],
      ["robot finished at +40", near(v.ex, 39.9, 40.1)],
    ],
  },
  {
    name: "MoveL condition never true: flag 0, full move",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "MoveL", offsetX: 20, speed: 40, ...stop("$robot.x >= $sx + 1000", "stop", { moveStopFlagVariableName: "fired" }) },
      setv("fired", "$fired + 10"), // proves the flag was written as 0 (0 + 10), not left unset
      ...endRobot],
    check: v => [["flag written as 0", v.fired === 10], ["full move +20", near(v.ex, 19.9, 20.1)]],
  },
  {
    name: "MoveJ stop on robot.x",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "MoveJ", offsetX: 40, speed: 20, ...stop("$robot.x >= $sx + 10", "stop", { moveStopPositionVariableName: "hit", moveStopFlagVariableName: "fired" }) },
      ...readHit, ...endRobot],
    check: v => [["fired = 1", v.fired === 1], ["stopped well short of +40", near(v.ex, 9, 20)], ["recorded x ≥ +10", near(v.hx, 10, 20)]],
  },
  {
    name: "JumpL stop on the lift leg: remaining legs skipped",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "JumpL", offsetX: 40, speed: 20, expressions: { jumpZ: "$sz + 20" },
        ...stop("$robot.z >= $sz + 10", "stop", { moveStopPositionVariableName: "hit", moveStopFlagVariableName: "fired" }) },
      ...readHit, ...endRobot],
    check: v => [
      ["fired = 1", v.fired === 1],
      ["stopped ≈ z+10 on the lift", near(v.ez, 10, 11.5)],
      ["never traversed (x ≈ 0)", near(v.ex, -0.1, 0.1)],
      ["recorded z ≈ +10", near(v.hz, 10, 11.5)],
    ],
  },
  {
    name: "JumpL record: whole jump completes, lift-leg position captured",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "JumpL", offsetX: 40, speed: 40, expressions: { jumpZ: "$sz + 20" },
        ...stop("$robot.z >= $sz + 10", "record", { moveStopPositionVariableName: "hit", moveStopFlagVariableName: "fired" }) },
      ...readHit, ...endRobot],
    check: v => [
      ["fired = 1", v.fired === 1],
      ["recorded z ≈ +10, x ≈ 0", near(v.hz, 10, 11.5) && near(v.hx, -0.1, 0.1)],
      ["ended at the target (x +40, z back to start)", near(v.ex, 39.9, 40.1) && near(v.ez, -0.1, 0.1)],
    ],
  },
  {
    name: "Watched MoveL with blend on: no blend, stops, next MoveL runs",
    vars: robotVars(),
    steps: [...startRobot,
      { id: sid(), type: "MoveL", offsetX: 40, speed: 20, blend: true, ...stop("$robot.x >= $sx + 15", "stop", { moveStopFlagVariableName: "fired" }) },
      { id: sid(), type: "MoveL", offsetY: 10, speed: 40 },
      ...endRobot],
    check: v => [["fired = 1", v.fired === 1], ["x stayed ≈ +15", near(v.ex, 15, 17)], ["second move ran (y +10)", near(v.ey, 9.9, 10.1)]],
  },
  {
    name: "Unknown variable in the condition: program errors with a clear message",
    vars: robotVars(),
    steps: [...startRobot, { id: sid(), type: "MoveL", name: "Bad watch", offsetX: 20, speed: 40, ...stop("$nope > 1", "stop") }, ...endRobot],
    expectError: /Stop condition on Bad watch/,
    check: () => [],
  },
  {
    name: "AuxMove stop (raw-step axis 0)",
    vars: [num("sa"), num("ap"), num("apd"), num("ae"), flag("af")],
    steps: [setv("sa", auxPos(0)),
      { id: sid(), type: "AuxMove", auxDeviceId: AUX, auxAxisIndex: 0, auxSteps: 4000, auxVelocity: 2000, auxAccel: 8000, auxDecel: 8000, auxWaitForDone: true,
        ...stop(`${auxPos(0)} >= $sa + 1500`, "stop", { moveStopPositionVariableName: "ap", moveStopFlagVariableName: "af" }) },
      setv("apd", "$ap - $sa"), setv("ae", `${auxPos(0)} - $sa`), { id: sid(), type: "Wait", waitMs: 800 }],
    check: v => [["fired = 1", v.af === 1], ["recorded ≈ +1500 steps", near(v.apd, 1500, 1700)], ["stopped short of 4000 (smooth decel)", near(v.ae, 1500, 2200)]],
  },
  {
    name: "AuxMove record",
    vars: [num("sa"), num("ap"), num("apd"), num("ae"), flag("af")],
    steps: [setv("sa", auxPos(0)),
      { id: sid(), type: "AuxMove", auxDeviceId: AUX, auxAxisIndex: 0, auxSteps: 4000, auxVelocity: 2000, auxAccel: 8000, auxDecel: 8000, auxWaitForDone: true,
        ...stop(`${auxPos(0)} >= $sa + 1500`, "record", { moveStopPositionVariableName: "ap", moveStopFlagVariableName: "af" }) },
      setv("apd", "$ap - $sa"), setv("ae", `${auxPos(0)} - $sa`), { id: sid(), type: "Wait", waitMs: 800 }],
    check: v => [["fired = 1", v.af === 1], ["recorded ≈ +1500", near(v.apd, 1500, 1700)], ["finished at +4000", near(v.ae, 3999, 4001)]],
  },
  {
    name: "Gripper Close stop halfway",
    vars: [num("gOpen"), num("gClosed"), num("gMid"), num("gp"), num("gEnd"), flag("gf")],
    steps: [
      { id: sid(), type: "GripperAction", auxDeviceId: AUX, auxAxisIndex: 2, gripperAction: "Close" },
      setv("gClosed", auxPos(2)),
      { id: sid(), type: "GripperAction", auxDeviceId: AUX, auxAxisIndex: 2, gripperAction: "Open" },
      setv("gOpen", auxPos(2)),
      setv("gMid", "($gOpen + $gClosed) / 2"),
      { id: sid(), type: "GripperAction", auxDeviceId: AUX, auxAxisIndex: 2, gripperAction: "Close",
        ...stop(`${auxPos(2)} <= $gMid`, "stop", { moveStopPositionVariableName: "gp", moveStopFlagVariableName: "gf" }) },
      setv("gEnd", auxPos(2)), { id: sid(), type: "Wait", waitMs: 800 }],
    check: v => {
      const span = v.gOpen - v.gClosed;
      return [
        ["open/closed differ", Math.abs(span) > 0],
        ["fired = 1", v.gf === 1],
        ["stopped between mid and closed, well short of closed", span > 0 && v.gEnd <= v.gMid + 0.01 && v.gEnd > v.gClosed + span * 0.2],
      ];
    },
  },
];

const only = process.argv[2];
const c = new Ctrl(); await c.open();
let failed = 0;
const created = [];
for (const [i, t] of TESTS.entries()) {
  if (only && !t.name.toLowerCase().includes(only.toLowerCase())) continue;
  const name = `zz live stopcond ${i + 1}`;
  created.push(name);
  const r = await runProgram(c, { name, steps: t.steps, variables: t.vars });
  const err = r.summary?.errorDescription ?? "";
  const checks = t.expectError
    ? [["status Error", r.status === "Error"], [`message ${t.expectError}`, t.expectError.test(err)]]
    : [["status Complete", r.status === "Complete"], ...t.check(r.vars)];
  const ok = checks.every(([, pass]) => pass);
  if (!ok) failed++;
  console.log(`${ok ? "PASS" : "FAIL"}  ${t.name}  (${r.ms} ms)`);
  for (const [label, pass] of checks) if (!pass || process.env.V) console.log(`        ${pass ? "ok " : "BAD"} ${label}`);
  if (!ok || process.env.V) console.log(`        status=${r.status} error="${err}" vars=${JSON.stringify(r.vars)}`);
  await sleep(200);
}
for (const name of created) await c.cmd("DeleteBuiltProgram", { name });
c.close();
console.log(failed ? `\n${failed} failed` : "\nall passed");
process.exit(failed ? 1 : 0);
