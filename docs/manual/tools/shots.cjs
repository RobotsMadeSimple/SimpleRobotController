// Captures the manual's app screenshots into docs/manual/screens/ and lists them in manifest.json.
//
// Needs: a THROWAWAY controller seeded by seed.mjs, and the Expo web app on :8081.
//   copy docs/manual/tools/shots.cjs into SimpleRobotApp/, then from SimpleRobotApp/:
//   CTRL_PORT=9124 node shots.cjs [ref|main|all]      (default all), and delete the copy.
// Uses the app's own Playwright (SimpleRobotApp/node_modules). Existing ref-*.png are kept unless
// FORCE=1, so an interrupted run can be resumed.
const { chromium } = require("playwright");
const fs = require("fs");
const path = require("path");

const MANUAL = process.env.MANUAL || path.resolve(__dirname, "../SimpleRobotController/docs/manual");
const PORT = process.env.CTRL_PORT || "9124";
const APP = process.env.APP_URL || "http://localhost:8081";
const OUT = path.join(MANUAL, "screens");
const MODE = process.argv[2] || "all";
if (PORT === "9000") throw new Error("Use a throwaway controller, not :9000");
fs.mkdirSync(OUT, { recursive: true });
const ref = JSON.parse(fs.readFileSync(path.join(MANUAL, "reference.json"), "utf8"));
const manifestPath = path.join(MANUAL, "manifest.json");
const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
const screens = new Map((manifest.screens || []).map(s => [s.id, s]));
const note = (id, caption) => screens.set(id, { id, file: `screens/${id}.png`, caption });

// ── controller commands (for starting programs) ──
let ws, seq = 0;
const pending = new Map();
async function ctrl() {
  ws = new WebSocket(`ws://127.0.0.1:${PORT}/control`);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.onmessage = ev => { const m = JSON.parse(ev.data); if (m.type === "ack" && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); } };
}
const cmd = (command, params = {}) => new Promise(res => { const id = `x${++seq}`; pending.set(id, res); ws.send(JSON.stringify({ type: "Command", id, command, params })); });
const sleep = ms => new Promise(r => setTimeout(r, ms));
const vis = loc => loc.locator("visible=true");

async function openBuilder(page, name) {
  await page.goto(`${APP}/program/builder?name=${encodeURIComponent(name)}`);
  await vis(page.getByText("Add Step", { exact: true })).first().waitFor({ timeout: 20000 });
  await sleep(900);
}

async function refShots(page) {
  for (const s of ref.steps) {
    const file = path.join(OUT, `ref-${s.type}.png`);
    if (fs.existsSync(file) && !process.env.FORCE) { note(`ref-${s.type}`, `${s.name} in the Blocks editor`); continue; }
    try { await refShot(page, s); } catch (e) { console.log(`! ${s.type}: ${String(e.message).split(/\r?\n/)[0]}`); }
  }
  console.log(" reference shots done");
}

async function refShot(page, s) {
  {
    await openBuilder(page, `Example ${s.type}`);
    const add = await vis(page.getByText("Add Step", { exact: true })).first().boundingBox();
    const hdr = await vis(page.getByText("STEPS", { exact: true })).first().boundingBox();
    // the Add Step button is as wide as the step list: crop from under the STEPS header to above it
    const clip = { x: 0, y: Math.round(hdr.y + hdr.height + 18), width: 0, height: 0 };
    const btnBox = await page.evaluate(({ ax, ay }) => {
      let el = document.elementFromPoint(ax, ay);
      while (el && el.getBoundingClientRect().width < 400) el = el.parentElement;
      const r = el.getBoundingClientRect(); return { x: r.x, y: r.y, w: r.width, h: r.height };
    }, { ax: add.x + add.width / 2, ay: add.y + add.height / 2 });
    clip.x = Math.round(btnBox.x - 6); clip.width = Math.round(btnBox.w + 12);
    clip.height = Math.round(btnBox.y - 10 - clip.y);
    await page.screenshot({ path: path.join(OUT, `ref-${s.type}.png`), clip });
    note(`ref-${s.type}`, `${s.name} in the Blocks editor`);
    process.stdout.write(".");
  }
}

async function modal(page, programName, cardNo, label, desc, id, caption) {
  await openBuilder(page, programName);
  const re = new RegExp(`^${cardNo} · ${label.toUpperCase().replace(/[()]/g, ".")}$`);
  await vis(page.getByText(re)).first().click({ force: true });
  const d = vis(page.getByText(desc, { exact: true })).first();
  await d.waitFor({ timeout: 8000 });
  await sleep(1200);
  const db = await d.boundingBox();
  const save = await vis(page.getByText("Save", { exact: true })).last().boundingBox();
  const cancel = await vis(page.getByText("Cancel", { exact: true })).last().boundingBox();
  const panel = await page.evaluate(({ x, y }) => {
    let el = document.elementFromPoint(x, y);
    // climb to the modal card: wide, tall, and starting above the description (so it has the title)
    const small = e => { const r = e.getBoundingClientRect(); return r.width < 380 || r.height < 300 || r.top > y - 40; };
    while (el && el.parentElement && small(el)) el = el.parentElement;
    const r = el.getBoundingClientRect(); return { x: r.x, y: r.y, w: r.width, h: r.height };
  }, { x: db.x + 4, y: db.y + 4 });
  const bottom = Math.max(save.y + save.height, cancel.y + cancel.height) + 24;
  const clip = { x: Math.round(panel.x - 4), y: Math.round(Math.max(0, panel.y - 4)), width: Math.round(panel.w + 8), height: 0 };
  clip.height = Math.round(Math.min(bottom, panel.y + panel.h + 4) - clip.y);
  await page.screenshot({ path: path.join(OUT, `${id}.png`), clip });
  note(id, caption);
  await page.keyboard.press("Escape");
}

async function modalShots(page) {
  await modal(page, "Example MoveL", 2, "Move Linear", "Move to a target in a straight line.", "editor-movel", "The Move Linear step editor: target, then optional speed, offsets, blending and a stop condition.");
  await modal(page, "Example Wait", 2, "Wait", "Wait for a time, or until a condition holds.", "editor-wait", "The Wait step editor in condition mode, with a timeout and a timed-out flag.");
  await modal(page, "Example SetOutput", 2, "Set Output", "Turn a digital output on or off, toggle it or pulse it.", "editor-setoutput", "The Set Output step editor with a 250 ms blocking pulse.");
  await modal(page, "Example Try", 1, "Try", "Run steps; on a runtime error retry them or run the catch steps.", "editor-try", "The Try step editor: retries, delay, which codes to catch and where the message goes.");
  await modal(page, "Example Prompt", 1, "Prompt", "Ask the operator: show a message with buttons and an optional input.", "editor-prompt", "The Prompt step editor: message, title, buttons and the result variable.");
}

async function mainShots(page, ctx) {
  await openBuilder(page, "Pick and Place");
  await page.screenshot({ path: path.join(OUT, "builder.png") });
  note("builder", "The Program Builder in Blocks view: program settings and variables on the left, the steps on the right.");

  await vis(page.getByText("Text", { exact: true })).first().click({ force: true });
  await sleep(3500);
  await page.screenshot({ path: path.join(OUT, "text-editor.png") });
  note("text-editor", "The same program in Text view (Simple Robot Script), with syntax colouring.");

  await openBuilder(page, "Pick and Place");
  await vis(page.getByText("Add Step", { exact: true })).first().click({ force: true });
  await sleep(1800);
  await page.screenshot({ path: path.join(OUT, "step-picker.png") });
  note("step-picker", "Add Step opens the step picker, grouped by category.");
  await page.keyboard.press("Escape");

  await modalShots(page);

  // validation problems
  await openBuilder(page, "Validation Demo");
  const chip = vis(page.getByText(/error/)).first();
  await chip.click({ force: true });
  await sleep(1500);
  await page.screenshot({ path: path.join(OUT, "validation.png") });
  note("validation", "Problems found before a run: an unknown point (warning) and a misspelt variable (error). Tap a problem to jump to its step.");
  await page.keyboard.press("Escape");
  await openBuilder(page, "Validation Demo");
  await vis(page.getByText("Text", { exact: true })).first().click({ force: true });
  await sleep(4000);
  await page.screenshot({ path: path.join(OUT, "text-problems.png") });
  note("text-problems", "In Text view the same problems are underlined on their lines.");

  // monitor while running
  const progs = JSON.parse((await cmd("GetBuiltPrograms")).programs);
  const counter = progs.find(p => p.name === "Counter Demo");
  await cmd("StartProgram", { programName: "Counter Demo" });
  await page.goto(`${APP}/program/monitor-program?name=${encodeURIComponent("Counter Demo")}`);
  await sleep(7000);
  await page.screenshot({ path: path.join(OUT, "monitor.png") });
  note("monitor", "The monitor while a program runs: progress, the current step, the robot position and the run log.");
  // paused at a breakpoint
  await cmd("AbortProgram", { programName: "Counter Demo" });
  await sleep(800);
  const loop = counter.steps.find(s => s.type === "Loop");
  const waitStep = loop.loopSteps.find(s => s.type === "Wait");
  await cmd("ExecuteBuiltProgram", { name: "Counter Demo", breakpoints: [waitStep.id] });
  await sleep(5000);
  await page.screenshot({ path: path.join(OUT, "monitor-breakpoint.png") });
  note("monitor-breakpoint", "Paused at a breakpoint: the pause reason is shown, with Continue, Step and Stop.");
  await cmd("AbortProgram", { programName: "Counter Demo" });

  // operator prompt
  await cmd("StartProgram", { programName: "Prompt Demo" });
  await page.goto(`${APP}/program/monitor-program?name=${encodeURIComponent("Prompt Demo")}`);
  await sleep(6000);
  await page.screenshot({ path: path.join(OUT, "prompt-dialog.png") });
  note("prompt-dialog", "A Prompt step shows its dialog on whatever screen the operator is on.");
  await cmd("AbortProgram", { programName: "Prompt Demo" });

  // phone layout
  const phone = await ctx.browser().newContext({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 2 });
  phone.setDefaultNavigationTimeout(180000);
  const pp = await phone.newPage();
  await pp.goto(`${APP}/station?host=localhost&port=${PORT}`);
  await sleep(15000);
  await openBuilder(pp, "Inspect and Sort");
  await pp.screenshot({ path: path.join(OUT, "builder-phone.png") });
  note("builder-phone", "The Blocks editor on a phone.");
  await phone.close();
}

(async () => {
  await ctrl();
  const browser = await chromium.launch();
  const ctx = await browser.newContext({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1.25 });
  ctx.setDefaultNavigationTimeout(180000);
  const page = await ctx.newPage();
  await page.goto(`${APP}/station?host=localhost&port=${PORT}`);
  await sleep(15000);
  try {
    if (MODE === "all" || MODE === "main") await mainShots(page, ctx);
    if (MODE === "modals") await modalShots(page);
    if (MODE === "all" || MODE === "ref") {
      await page.setViewportSize({ width: 1500, height: 1500 });
      await refShots(page);
    }
  } finally {
    manifest.screens = [...screens.values()].sort((a, b) => a.id.localeCompare(b.id));
    fs.writeFileSync(manifestPath, JSON.stringify(manifest, null, 2) + "\n");
    await browser.close();
    ws.close();
  }
})().catch(e => { console.error(e); process.exit(1); });
