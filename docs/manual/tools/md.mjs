// A small Markdown renderer for the manual's chapters (no dependencies). It covers what the
// chapters use: ATX headings, paragraphs, fenced code, GFM tables, block quotes (callouts),
// bullet and numbered lists, images as figures, and inline code / bold / italic / links.

export const esc = s => String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

export const slug = s => String(s).toLowerCase().replace(/<[^>]+>/g, "").replace(/[`*_]/g, "")
  .replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "");

/** Inline Markdown → HTML. `opts.link(href)` may rewrite link targets. */
export function inline(text, opts = {}) {
  const codes = [];
  let s = text.replace(/`([^`]+)`/g, (_, c) => { codes.push(c); return `\u0000${codes.length - 1}\u0000`; });
  s = esc(s);
  s = s.replace(/!\[([^\]]*)\]\(([^)]+)\)/g, (_, alt, src) => `<img src="${opts.asset ? opts.asset(src) : src}" alt="${alt}">`);
  s = s.replace(/\[([^\]]+)\]\(([^)]+)\)/g, (_, t, href) => `<a href="${opts.link ? opts.link(href) : href}">${t}</a>`);
  s = s.replace(/\*\*([^*]+)\*\*/g, "<strong>$1</strong>");
  s = s.replace(/(^|[^\w*])\*([^*\s][^*]*?)\*(?=[^\w*]|$)/g, "$1<em>$2</em>");
  s = s.replace(/\\\|/g, "|");
  return s.replace(/\u0000(\d+)\u0000/g, (_, i) => `<code>${esc(codes[Number(i)].replace(/\\\|/g, "|"))}</code>`);
}

const splitRow = line => line.trim().replace(/^\|/, "").replace(/\|$/, "").split(/(?<!\\)\|/).map(c => c.trim());

/**
 * Block Markdown → HTML.
 *   opts.code(lang, text) → HTML for a fenced block
 *   opts.figure(src, caption) → HTML for an image on its own line
 *   opts.heading(level, text, id) → HTML (default <hN id>)
 */
export function render(md, opts = {}) {
  const lines = md.replace(/\r\n/g, "\n").split("\n");
  const out = [];
  let i = 0;
  const inl = t => inline(t, opts);
  while (i < lines.length) {
    const line = lines[i];
    if (!line.trim()) { i++; continue; }
    let m;
    if ((m = line.match(/^```(\w*)\s*$/))) {
      const buf = [];
      i++;
      while (i < lines.length && !/^```\s*$/.test(lines[i])) buf.push(lines[i++]);
      i++;
      out.push(opts.code ? opts.code(m[1], buf.join("\n")) : `<pre><code>${esc(buf.join("\n"))}</code></pre>`);
      continue;
    }
    if ((m = line.match(/^(#{1,4})\s+(.*)$/))) {
      const level = m[1].length, text = m[2].trim(), id = slug(text);
      out.push(opts.heading ? opts.heading(level, text, id) : `<h${level} id="${id}">${inl(text)}</h${level}>`);
      i++;
      continue;
    }
    if ((m = line.match(/^!\[([^\]]*)\]\(([^)]+)\)\s*$/))) {
      out.push(opts.figure ? opts.figure(m[2], m[1]) : `<figure><img src="${m[2]}"><figcaption>${inl(m[1])}</figcaption></figure>`);
      i++;
      continue;
    }
    if (line.trim().startsWith("|") && i + 1 < lines.length && /^\s*\|?\s*:?-{3,}/.test(lines[i + 1])) {
      const head = splitRow(line);
      i += 2;
      const rows = [];
      while (i < lines.length && lines[i].trim().startsWith("|")) rows.push(splitRow(lines[i++]));
      out.push(`<table><thead><tr>${head.map(h => `<th>${inl(h)}</th>`).join("")}</tr></thead><tbody>` +
        rows.map(r => `<tr>${r.map(c => `<td>${inl(c)}</td>`).join("")}</tr>`).join("") + "</tbody></table>");
      continue;
    }
    if (line.startsWith(">")) {
      const buf = [];
      while (i < lines.length && lines[i].startsWith(">")) buf.push(lines[i++].replace(/^>\s?/, ""));
      const text = buf.join(" ");
      const kind = (text.match(/^\*\*(Note|Tip|Caution|Warning)\*\*/) || [])[1];
      const body = kind ? text.replace(/^\*\*\w+\*\*\s*/, "") : text;
      out.push(`<div class="callout ${kind ? kind.toLowerCase() : "note"}">${kind ? `<span class="callout-label">${kind}</span>` : ""}<p>${inl(body)}</p></div>`);
      continue;
    }
    if (/^(\s*)([-*]|\d+\.)\s+/.test(line)) {
      const ordered = /^\s*\d+\./.test(line);
      const items = [];
      while (i < lines.length && lines[i].trim()) {
        const l = lines[i];
        const im = l.match(/^\s*(?:[-*]|\d+\.)\s+(.*)$/);
        if (im && !/^\s{2,}/.test(l)) items.push(im[1]);
        else if (im && items.length && /^\s{2,}/.test(l)) items.push(im[1]); // shallow nesting flattened
        else if (items.length) items[items.length - 1] += " " + l.trim();
        i++;
      }
      const tag = ordered ? "ol" : "ul";
      out.push(`<${tag}>${items.map(t => `<li>${inl(t)}</li>`).join("")}</${tag}>`);
      continue;
    }
    const buf = [lines[i++].trim()]; // always consume the first line, so nothing can stall here
    while (i < lines.length && lines[i].trim() && !/^(```|#{1,4}\s|>|\|)/.test(lines[i]) && !/^\s*([-*]|\d+\.)\s+/.test(lines[i]) && !/^!\[/.test(lines[i]))
      buf.push(lines[i++].trim());
    out.push(`<p>${inl(buf.join(" "))}</p>`);
  }
  return out.join("\n");
}

// ── Simple Robot Script highlighting ───────────────────────────────────────
const FLOW = new Set(["if", "elif", "else", "end", "repeat", "forever", "as", "while", "for", "in", "switch", "case", "default",
  "try", "catch", "break", "continue", "return", "exit", "raise", "label", "goto", "call", "out", "pause", "status",
  "stopwatch", "prompt", "plugin", "unknown", "var", "computed", "param", "global", "persistent", "input", "monitor",
  "inout", "list", "and", "or", "not"]);
const VALUES = new Set(["on", "off", "true", "false", "here", "grid", "stack", "toggle", "send", "receive", "_"]);

export function highlightSrs(src, keywords = new Set()) {
  return src.split("\n").map(line => {
    let out = "", i = 0;
    const tok = (cls, t) => `<span class="t-${cls}">${esc(t)}</span>`;
    if (/^\s*@\w+/.test(line)) {
      const m = line.match(/^(\s*)(@\w+)(.*)$/);
      return esc(m[1]) + tok("attr", m[2]) + highlightRest(m[3]);
    }
    return highlightRest(line);
    function highlightRest(l) {
      let o = "", j = 0;
      while (j < l.length) {
        const c = l[j], rest = l.slice(j);
        let m;
        if (c === "#") { o += tok(rest.startsWith("##") ? "section" : "comment", rest); break; }
        if (c === '"') {
          let k = j + 1;
          while (k < l.length && l[k] !== '"') k += l[k] === "\\" ? 2 : 1;
          const str = l.slice(j, k + 1);
          o += `<span class="t-str">${esc(str).replace(/\{[^}]*\}/g, b => `<span class="t-tpl">${b}</span>`)}</span>`;
          j = k + 1;
          continue;
        }
        if ((m = rest.match(/^\$[A-Za-z_]\w*/))) { o += tok("var", m[0]); j += m[0].length; continue; }
        if ((m = rest.match(/^\d+(\.\d+)?(e-?\d+)?(ms|s)?\b/)) && !/\w/.test(l[j - 1] ?? "")) { o += tok("num", m[0]); j += m[0].length; continue; }
        if ((m = rest.match(/^[A-Za-z_]\w*/))) {
          const w = m[0], lw = w.toLowerCase();
          const next = l.slice(j + w.length);
          if (/^\s*=(?!=)/.test(next) && !FLOW.has(lw)) o += tok("arg", w);
          else if (keywords.has(w) && /^\s*(\S|$)/.test(next) && /^\s*$/.test(l.slice(0, j))) o += tok("kw", w);
          else if (FLOW.has(lw) && (/^\s*$/.test(l.slice(0, j)) || ["as", "in", "and", "or", "not", "out", "list", "if", "end"].includes(lw) || /^(global|persistent|input|monitor|var|computed|param)$/.test(lw))) o += tok("flow", w);
          else if (VALUES.has(lw)) o += tok("val", w);
          else o += esc(w);
          j += w.length;
          continue;
        }
        if (rest.startsWith("->")) { o += tok("op", "->"); j += 2; continue; }
        o += esc(c);
        j++;
      }
      return o;
    }
  }).join("\n");
}
