// Builds docs/manual/SimpleRobot-Programming-Manual.pdf from the manual source.
//   node docs/manual/tools/build-pdf.mjs [--html]     (--html also keeps the intermediate HTML)
//
// Markdown chapters (manifest.json) + reference.json + figures/*.svg + screens/*.png → one HTML
// document → Chromium page.pdf() with print CSS. Two passes: the first PDF is read back to find the
// page of every chapter, section and instruction (Chromium writes named destinations for link
// targets), the second fills those page numbers into the table of contents and indexes.
// Uses the Playwright Chromium in SimpleRobotApp/node_modules (override with PLAYWRIGHT_DIR).
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { createRequire } from "node:module";
import { fileURLToPath, pathToFileURL } from "node:url";
import { render, inline, esc, slug, highlightSrs } from "./md.mjs";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const OUT_PDF = path.join(ROOT, "SimpleRobot-Programming-Manual.pdf");
const appDir = process.env.PLAYWRIGHT_DIR || path.resolve(ROOT, "../../../SimpleRobotApp");
const { chromium } = createRequire(path.join(appDir, "package.json"))("playwright");

const manifest = JSON.parse(fs.readFileSync(path.join(ROOT, "manifest.json"), "utf8"));
const ref = JSON.parse(fs.readFileSync(path.join(ROOT, manifest.reference), "utf8"));
const css = fs.readFileSync(path.join(HERE, "print.css"), "utf8");
const chapters = new Map(manifest.chapters.map(c => [c.id, c]));
const stepByType = new Map(ref.steps.map(s => [s.type, s]));
const keywords = new Set(ref.keywords.statements.map(k => k.word));
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), "srmanual-"));

// ── assets ──────────────────────────────────────────────────────────────────
const svgCache = new Map();
function inlineSvg(rel) {
  const file = path.join(ROOT, rel);
  if (!svgCache.has(file)) svgCache.set(file, fs.readFileSync(file, "utf8").replace(/^<svg /, '<svg class="fig-svg" preserveAspectRatio="xMidYMid meet" '));
  return svgCache.get(file);
}
const jpegOf = new Map(); // png path → jpeg file url (filled by toJpeg before the build)
const imgUrl = rel => {
  const file = path.join(ROOT, rel);
  return jpegOf.get(file) ?? pathToFileURL(file).href;
};
const assetRel = src => src.replace(/^\.\.\//, "");

// ── links and ids ───────────────────────────────────────────────────────────
const link = href => {
  if (!href.startsWith("#")) return href;
  const id = href.slice(1);
  if (chapters.has(id)) return `#ch-${id}`;
  return href;
};
const used = new Set();
const uniq = id => { let u = id, n = 2; while (used.has(u)) u = `${id}-${n++}`; used.add(u); return u; };

const code = (lang, text) => lang === "srs"
  ? `<pre class="code srs">${highlightSrs(text, keywords)}</pre>`
  : `<pre class="code ${esc(lang)}">${esc(text)}</pre>`;
let figNo = 0;
const figureHtml = (src, caption) => {
  const rel = assetRel(src);
  const isSvg = rel.endsWith(".svg");
  const kind = isSvg ? "figure" : "screen";
  figNo++;
  const body = isSvg ? inlineSvg(rel) : `<img src="${imgUrl(rel)}" alt="">`;
  return `<figure class="${kind}">${body}<figcaption><b>Figure ${figNo}.</b> ${inline(caption, { link })}</figcaption></figure>`;
};

// ── chapter rendering ───────────────────────────────────────────────────────
const toc = []; // { level, id, title, number }
function renderChapter(ch) {
  const md = fs.readFileSync(path.join(ROOT, ch.file), "utf8");
  const chId = `ch-${ch.id}`;
  used.add(chId);
  toc.push({ level: 1, id: chId, title: ch.title, number: ch.number });
  let skippedH1 = false;
  const html = render(md, {
    link, code, figure: figureHtml,
    asset: src => imgUrl(assetRel(src)),
    heading: (level, text, id) => {
      if (level === 1 && !skippedH1) { skippedH1 = true; return ""; }
      const hid = uniq(`${ch.id}--${id}`);
      if (level === 2) toc.push({ level: 2, id: hid, title: text });
      return `<h${level} id="${hid}">${inline(text, { link })}</h${level}>`;
    },
  });
  const label = /^[A-Z]$/.test(ch.number) ? `Appendix ${ch.number}` : `Chapter ${ch.number}`;
  return `<section class="chapter" id="${chId}" data-title="${esc(ch.title)}">
<div class="ch-head"><div class="ch-num">${label}</div><h1>${esc(ch.title)}</h1></div>
${html}
${ch.generated ? GENERATED[ch.generated]() : ""}
</section>`;
}

const pg = id => `<span class="pg" data-target="${id}"></span>`;
const yesNo = b => b ? `<span class="yes">Yes</span>` : `<span class="no">No</span>`;

// ── generated sections ──────────────────────────────────────────────────────
const GENERATED = {
  steps() {
    const parts = [];
    for (const cat of ref.categories) {
      const cid = `refcat-${cat.key}`;
      used.add(cid);
      toc.push({ level: 2, id: cid, title: cat.label });
      const steps = cat.steps.map(t => stepByType.get(t));
      parts.push(`<section class="ref-cat"><h2 id="${cid}">${esc(cat.label)}</h2><p>${esc(cat.description)}</p>
<table class="ref-index"><thead><tr><th>Step</th><th>Keyword</th><th>Description</th><th class="pgcol">Page</th></tr></thead><tbody>
${steps.map(s => `<tr><td><a href="#ref-${s.type}">${esc(s.name)}</a></td><td><code>${esc(s.keyword)}</code></td><td>${esc(s.description)}</td><td class="pgcol">${pg(`ref-${s.type}`)}</td></tr>`).join("")}
</tbody></table></section>`);
      for (const s of steps) parts.push(refEntry(s));
    }
    return parts.join("\n");
  },
  functions() {
    const groups = {};
    for (const p of ref.properties) (groups[p.group] ??= []).push(p);
    const gTitle = { robot: "Robot", program: "Program", time: "Time", aux: "Aux axes", time_ms: "Built-in variable", camera: "Cameras", plugin: "Plugins" };
    const id1 = uniq("functions--function-list"), id2 = uniq("functions--properties"), id3 = uniq("functions--io-names");
    toc.push({ level: 2, id: id1, title: "Function list" }, { level: 2, id: id2, title: "Properties" }, { level: 2, id: id3, title: "IO names" });
    return `<h2 id="${id1}">Function list</h2>
<table class="fn"><thead><tr><th>Function</th><th>Result</th></tr></thead><tbody>
${ref.functions.map(f => `<tr><td><code>${esc(f.signature)}</code></td><td>${esc(f.description)}</td></tr>`).join("")}
</tbody></table>
<h2 id="${id2}">Properties</h2><p>Read-only; resolved live whenever an expression reads them. Never assignable.</p>
${Object.entries(groups).map(([g, ps]) => `<h3>${esc(gTitle[g] ?? g)}</h3><table class="fn"><thead><tr><th>Property</th><th>Type</th><th>Value</th></tr></thead><tbody>
${ps.map(p => `<tr><td><code>${esc(p.name)}</code></td><td>${esc(p.type)}</td><td>${esc(p.description)}</td></tr>`).join("")}</tbody></table>`).join("\n")}
<h2 id="${id3}">IO names</h2>
<table class="fn"><thead><tr><th>Name</th><th>Value</th></tr></thead><tbody>
${ref.io.map(p => `<tr><td><code>${esc(p.name)}</code></td><td>${inline(p.description)}</td></tr>`).join("")}</tbody></table>`;
  },
  errorCodes() {
    const id1 = uniq("error-codes--runtime-error-codes"), id2 = uniq("error-codes--validation-problems"), id3 = uniq("error-codes--parse-problems");
    toc.push({ level: 2, id: id1, title: "Runtime error codes" }, { level: 2, id: id2, title: "Validation problems" }, { level: 2, id: id3, title: "Parse problems" });
    const row = e => `<tr><td><code>${esc(e.code)}</code></td><td>${esc(e.label)}</td><td>${e.catchable ? '<span class="yes">catchable</span>' : '<span class="no">fatal</span>'}</td><td>${inline(e.description)}${e.raisedBy.length ? `<div class="small">Raised by: ${e.raisedBy.map(t => `<a href="#ref-${t}">${esc(stepByType.get(t).name)}</a>`).join(", ")}</div>` : ""}</td></tr>`;
    return `<h2 id="${id1}">Runtime error codes</h2>
<table class="codes"><thead><tr><th>Code</th><th>Label</th><th>Kind</th><th>When</th></tr></thead><tbody>${ref.errorCodes.map(row).join("")}</tbody></table>
<h2 id="${id2}">Validation problems</h2><p>Reported by the validator (<code>ValidateBuiltProgram</code>) for both views, on the step and field concerned.</p>
<table class="codes"><thead><tr><th>Code</th><th>Severity</th><th>Meaning</th></tr></thead><tbody>
${ref.problemCodes.validation.map(p => `<tr><td><code>${esc(p.code)}</code></td><td>${p.severity === "error" ? '<span class="no">error</span>' : '<span class="warn">warning</span>'}</td><td>${inline(p.meaning)}</td></tr>`).join("")}</tbody></table>
<h2 id="${id3}">Parse problems</h2><p>Reported when text is read back (<code>ParseProgramText</code>), with a line and column. A line with a problem is skipped and parsing carries on, so every problem is reported at once.</p>
<table class="codes"><thead><tr><th>Code</th><th>Meaning</th></tr></thead><tbody>
${ref.problemCodes.parse.map(p => `<tr><td><code>${esc(p.code)}</code></td><td>${inline(p.meaning)}</td></tr>`).join("")}</tbody></table>`;
  },
  keywords() {
    const id1 = uniq("keywords--statement-keywords"), id2 = uniq("keywords--other-reserved-words");
    toc.push({ level: 2, id: id1, title: "Statement keywords" }, { level: 2, id: id2, title: "Other reserved words" });
    const all = ref.steps.map(s => ({ word: s.keyword === "$" ? "$var = …  /  $list.op(…)" : s.keyword === "#" ? "#  /  ##" : s.keyword, s })).sort((a, b) => a.word.localeCompare(b.word, "en", { sensitivity: "base" }));
    const chip = w => `<code>${esc(w)}</code>`;
    return `<h2 id="${id1}">Statement keywords</h2><p>Every step, by its text keyword, with the page of its reference entry.</p>
<table class="kw-table"><thead><tr><th>Keyword</th><th>Step</th><th>Category</th><th class="pgcol">Page</th></tr></thead><tbody>
${all.map(({ word, s }) => `<tr><td><code>${esc(word)}</code></td><td><a href="#ref-${s.type}">${esc(s.name)}</a></td><td>${esc(s.categoryLabel)}</td><td class="pgcol">${pg(`ref-${s.type}`)}</td></tr>`).join("")}
</tbody></table>
<h2 id="${id2}">Other reserved words</h2>
<table><tbody>
<tr><th>Flow</th><td class="chips">${ref.keywords.flow.map(chip).join(" ")}</td></tr>
<tr><th>Declarations</th><td class="chips">${ref.keywords.declarations.map(chip).join(" ")}</td></tr>
<tr><th>Values and operators</th><td class="chips">${ref.keywords.values.map(chip).join(" ")}</td></tr>
<tr><th>Attributes</th><td class="chips">${ref.keywords.attributes.map(chip).join(" ")}</td></tr>
<tr><th>Step type names</th><td class="chips">${ref.keywords.stepTypeNames.map(chip).join(" ")}</td></tr>
</tbody></table>`;
  },
};

function refEntry(s) {
  const id = `ref-${s.type}`;
  used.add(id);
  const kw = s.keyword === "$" ? (s.type === "SetVariable" ? "$var = …" : "$list.…(…)") : s.keyword === "#" ? "# / ##" : s.keyword;
  const params = s.params.length ? `<table class="params"><thead><tr><th>Parameter</th><th>In text</th><th>Type / values</th><th>Unit</th><th>Default</th></tr></thead><tbody>
${s.params.map(p => {
    const opts = p.options ? `<div class="small">${p.options.map(o => `<code>${esc(o.value)}</code> ${esc(o.label)}`).join(" · ")}</div>` : "";
    const range = p.max != null ? `<div class="small">range ${p.min ?? "…"} – ${p.max}</div>` : p.min ? `<div class="small">at least ${p.min}</div>` : "";
    const when = p.showIf ? `<div class="small">when ${esc(p.showIf.field)} = ${esc(p.showIf.in.join(" / "))}</div>` : "";
    return `<tr><td><b>${esc(p.label)}</b>${p.required ? '<span class="req" title="required">*</span>' : ""}${p.expression ? ' <span class="fx">fx</span>' : ""}${p.help ? `<div class="small">${esc(p.help)}</div>` : ""}${when}</td>
<td><code>${esc(p.inText)}</code></td><td>${esc(p.value)}${opts}${range}</td><td>${esc(p.unit)}</td><td>${p.default !== "" ? `<code>${esc(p.default)}</code>` : ""}</td></tr>`;
  }).join("")}</tbody></table>` : `<p class="small">No parameters.</p>`;
  const bodies = s.bodies.length ? `<p class="small">Contains steps: ${s.bodies.map(esc).join(", ")}.</p>` : "";
  const errors = s.errors.length ? `<table class="errs"><thead><tr><th>Code</th><th>Kind</th><th>When</th></tr></thead><tbody>${s.errors.map(e => `<tr><td><code>${esc(e.code)}</code></td><td>${e.catchable ? '<span class="yes">catchable</span>' : '<span class="no">fatal</span>'}</td><td>${inline(e.when)}</td></tr>`).join("")}</tbody></table>` : `<p class="small">This step raises no runtime error codes of its own.</p>`;
  const shot = path.join(ROOT, s.example.screen);
  const blocks = fs.existsSync(shot) ? `<div class="ex-blocks"><div class="ex-label">Blocks</div><img src="${imgUrl(s.example.screen)}" alt=""></div>` : "";
  return `<article class="ref-entry" id="${id}">
<header class="ref-head"><h3>${esc(s.name)}</h3><span class="ref-kw">${esc(kw)}</span></header>
<div class="ref-meta"><span>${esc(s.categoryLabel)}</span><span>Background programs: ${yesNo(s.background)}</span>${s.aliases.length ? `<span>Also: ${s.aliases.map(esc).join(", ")}</span>` : ""}</div>
<p class="ref-desc">${esc(s.description)}</p>
<div class="ref-label">Syntax</div><pre class="syntax">${esc(s.syntax)}</pre>
<div class="ref-label">Parameters</div>${params}${bodies}
${s.notes.length ? `<div class="ref-label">Description and notes</div><ul class="notes">${s.notes.map(n => `<li>${inline(n, { link })}</li>`).join("")}</ul>` : ""}
<div class="ref-label">Errors</div>${errors}
<div class="ref-label">Example</div><div class="example">${blocks}<div class="ex-text"><div class="ex-label">Text</div>${code("srs", s.example.srs)}</div></div>
${s.seeAlso.length ? `<div class="see-also"><b>See also:</b> ${s.seeAlso.map(a => `<a href="#ref-${a.type}">${esc(a.name)}</a> <span class="small">(${esc(a.keyword)}, p.&#8201;${pg(`ref-${a.type}`)})</span>`).join(" · ")}</div>` : ""}
</article>`;
}

// ── document ────────────────────────────────────────────────────────────────
function buildHtml() {
  used.clear(); toc.length = 0; figNo = 0;
  const body = [];
  for (const part of manifest.parts) {
    const pid = part.id;
    used.add(pid);
    toc.push({ level: 0, id: pid, title: part.title });
    body.push(`<section class="part" id="${pid}"><div class="part-title">${esc(part.title)}</div>
<ol class="part-list">${part.chapters.map(c => `<li><span>${esc(chapters.get(c).number)}</span> ${esc(chapters.get(c).title)}</li>`).join("")}</ol></section>`);
    for (const c of part.chapters) body.push(renderChapter(chapters.get(c)));
  }
  const tocHtml = `<section class="toc" id="toc"><h1>Contents</h1>${toc.map(t => t.level === 0
    ? `<div class="toc-part"><a href="#${t.id}">${esc(t.title)}</a></div>`
    : `<div class="toc-l${t.level}"><a href="#${t.id}">${t.number ? `<span class="toc-num">${esc(t.number)}</span>` : ""}${esc(t.title)}</a><span class="toc-dots"></span>${pg(t.id)}</div>`).join("\n")}</section>`;
  const date = new Date().toISOString().slice(0, 10);
  const cover = `<section class="cover">
<div class="cover-band"></div>
<div class="cover-body">
<div class="cover-brand">Robots Made Simple</div>
<h1>SimpleRobot<br>Programming Manual</h1>
<p class="cover-sub">${esc(manifest.subtitle)}</p>
<div class="cover-meta">
<div><span>Language</span>Simple Robot Script v${ref.generatedFrom.languageVersion}</div>
<div><span>Step catalog</span>v${ref.generatedFrom.catalogVersion} · ${ref.steps.length} instructions</div>
<div><span>Built</span>${date}</div>
</div>
</div>
<div class="cover-art">${inlineSvg("figures/jump.svg")}</div>
</section>`;
  return `<!doctype html><html><head><meta charset="utf-8"><title>${esc(manifest.title)}</title><style>${css}</style></head>
<body>${cover}${tocHtml}${body.join("\n")}</body></html>`;
}

// ── PDF: named destinations → page numbers ──────────────────────────────────
function destPages(buf) {
  const s = buf.toString("latin1");
  const offsets = new Map();
  // classic xref tables (possibly several sections)
  let xi = s.lastIndexOf("startxref");
  const start = Number(s.slice(xi + 9).trim().split(/\s/)[0]);
  const xs = s.slice(start, start + 40);
  if (!xs.startsWith("xref")) throw new Error("PDF uses an xref stream; page-number extraction not supported");
  let p = start + 4;
  const xref = s.slice(p).split("trailer")[0].trim().split(/\r?\n/);
  for (let i = 0; i < xref.length;) {
    const [first, count] = xref[i].trim().split(/\s+/).map(Number);
    for (let k = 0; k < count; k++) {
      const [off, , type] = xref[i + 1 + k].trim().split(/\s+/);
      if (type === "n") offsets.set(first + k, Number(off));
    }
    i += count + 1;
  }
  const obj = n => { const o = offsets.get(n); const e = s.indexOf("endobj", o); const t = s.slice(o, e); const st = t.indexOf("stream"); return st >= 0 ? t.slice(0, st) : t; };
  const trailer = s.slice(s.lastIndexOf("trailer"));
  const rootN = Number(trailer.match(/\/Root\s+(\d+)\s+0\s+R/)[1]);
  const root = obj(rootN);
  const pagesN = Number(root.match(/\/Pages\s+(\d+)\s+0\s+R/)[1]);
  const pageIndex = new Map();
  const walk = n => {
    const o = obj(n);
    if (/\/Type\s*\/Pages\b/.test(o)) {
      const kids = o.match(/\/Kids\s*\[([^\]]*)\]/)[1].match(/(\d+)\s+0\s+R/g).map(r => Number(r.split(/\s/)[0]));
      kids.forEach(walk);
    } else pageIndex.set(n, pageIndex.size + 1);
  };
  walk(pagesN);
  const dests = new Map();
  const decode = name => name.replace(/#([0-9a-fA-F]{2})/g, (_, h) => String.fromCharCode(parseInt(h, 16)));
  const readDict = text => { for (const m of text.matchAll(/\/([^\s/\[<>()]+)\s*\[\s*(\d+)\s+0\s+R/g)) dests.set(decode(m[1]), pageIndex.get(Number(m[2]))); };
  const dm = root.match(/\/Dests\s+(\d+)\s+0\s+R/);
  if (dm) readDict(obj(Number(dm[1])));
  const nm = root.match(/\/Names\s+(\d+)\s+0\s+R/) || root.match(/\/Names\s*<<([\s\S]*?)>>/);
  if (nm) {
    const namesDict = /^\d+$/.test(nm[1]) ? obj(Number(nm[1])) : nm[1];
    const dt = namesDict.match(/\/Dests\s+(\d+)\s+0\s+R/);
    const walkNames = n => {
      const o = obj(n);
      const kids = o.match(/\/Kids\s*\[([^\]]*)\]/);
      if (kids) kids[1].match(/(\d+)\s+0\s+R/g).forEach(r => walkNames(Number(r.split(/\s/)[0])));
      const names = o.match(/\/Names\s*\[([\s\S]*)\]/);
      if (names) for (const m of names[1].matchAll(/\(((?:\\.|[^\\)])*)\)\s*(?:\[\s*(\d+)\s+0\s+R|(\d+)\s+0\s+R)/g)) {
        let page = m[2] ? pageIndex.get(Number(m[2])) : null;
        if (!page && m[3]) { const d = obj(Number(m[3])); const pm = d.match(/\[\s*(\d+)\s+0\s+R/); if (pm) page = pageIndex.get(Number(pm[1])); }
        dests.set(m[1].replace(/\\(.)/g, "$1"), page);
      }
    };
    if (dt) walkNames(Number(dt[1]));
  }
  return { dests, pages: pageIndex.size };
}

// ── images: PNG → JPEG for a smaller PDF (screens/ keeps the PNGs) ──────────
async function toJpeg(page) {
  const dir = path.join(ROOT, "screens");
  if (!fs.existsSync(dir)) return;
  for (const f of fs.readdirSync(dir).filter(f => f.endsWith(".png"))) {
    const file = path.join(dir, f);
    const data = fs.readFileSync(file).toString("base64");
    const jpg = await page.evaluate(async ({ data, ref }) => {
      const img = new Image();
      img.src = `data:image/png;base64,${data}`;
      await img.decode();
      const scale = Math.min(1, (ref ? 1100 : 1500) / img.width);
      const c = document.createElement("canvas");
      c.width = Math.round(img.width * scale); c.height = Math.round(img.height * scale);
      const g = c.getContext("2d");
      g.fillStyle = "#fff"; g.fillRect(0, 0, c.width, c.height);
      g.imageSmoothingQuality = "high";
      g.drawImage(img, 0, 0, c.width, c.height);
      return c.toDataURL("image/jpeg", 0.84).split(",")[1];
    }, { data, ref: f.startsWith("ref-") });
    const out = path.join(TMP, f.replace(/\.png$/, ".jpg"));
    fs.writeFileSync(out, Buffer.from(jpg, "base64"));
    jpegOf.set(file, pathToFileURL(out).href);
  }
}

const footer = `<div style="width:100%;font-family:'Segoe UI',Helvetica,Arial,sans-serif;font-size:7.5px;color:#64748b;padding:0 15mm;display:flex;justify-content:space-between;">
<span>SimpleRobot Programming Manual</span><span class="pageNumber"></span></div>`;
const header = `<div style="width:100%;font-family:'Segoe UI',Helvetica,Arial,sans-serif;font-size:7.5px;color:#94a3b8;padding:0 15mm;display:flex;justify-content:space-between;">
<span>Robots Made Simple</span><span>Blocks · Simple Robot Script v${ref.generatedFrom.languageVersion}</span></div>`;

async function pdfOnce(page, html) {
  const file = path.join(TMP, "manual.html");
  fs.writeFileSync(file, html);
  await page.goto(pathToFileURL(file).href, { waitUntil: "load" });
  await page.evaluate(() => document.fonts.ready);
  return page.pdf({ format: "A4", printBackground: true, displayHeaderFooter: true, headerTemplate: header, footerTemplate: footer,
    margin: { top: "16mm", bottom: "15mm", left: "15mm", right: "15mm" }, preferCSSPageSize: false, outline: true, tagged: false });
}

const browser = await chromium.launch();
try {
  const page = await browser.newPage();
  await page.goto("about:blank");
  await toJpeg(page);
  let html = buildHtml();
  const first = await pdfOnce(page, html);
  const { dests, pages } = destPages(first);
  const fill = h => h.replace(/<span class="pg" data-target="([^"]+)"><\/span>/g, (_, id) => `<span class="pg">${dests.get(id) ?? "?"}</span>`);
  const missing = [...html.matchAll(/data-target="([^"]+)"/g)].map(m => m[1]).filter(id => !dests.has(id));
  if (missing.length) console.warn(`no page found for ${new Set(missing).size} targets, e.g. ${[...new Set(missing)].slice(0, 5).join(", ")}`);
  html = fill(html);
  const pdf = await pdfOnce(page, html);
  const check = destPages(pdf);
  fs.writeFileSync(OUT_PDF, pdf);
  if (process.argv.includes("--html")) fs.copyFileSync(path.join(TMP, "manual.html"), path.join(TMP, "manual-final.html"));
  const shifted = [...dests].filter(([id, p]) => check.dests.get(id) !== p).length;
  console.log(`${path.relative(process.cwd(), OUT_PDF)}: ${check.pages} pages, ${(pdf.length / 1048576).toFixed(1)} MB${shifted ? `, ${shifted} page refs shifted between passes` : ""}`);
  console.log(`intermediate files: ${TMP}`);
} finally {
  await browser.close();
}
