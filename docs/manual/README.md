# SimpleRobot Programming Manual

The source of `SimpleRobot-Programming-Manual.pdf`: the programming manual for Blocks and Simple
Robot Script, organised like the Epson SPEL+ and ABB RAPID manuals (concepts, one entry per
instruction, functions, error codes, appendices).

**Rebuild the PDF:** `node docs/manual/tools/build-pdf.mjs` (from `SimpleRobotController/`; uses
the Playwright Chromium in `../SimpleRobotApp/node_modules`).

## Layout

| Path | What | Made by |
|---|---|---|
| `manifest.json` | parts, chapters in order (id, number, title, file, `generated`), figures, screens | hand + `figures.mjs` / `shots.cjs` fill the lists |
| `chapters/NN-slug.md` | one chapter per file — plain Markdown (headings, tables, ```` ```srs ```` code, `![caption](../figures/x.svg)`) | hand-written |
| `reference.json` | the instruction reference (one entry per step), functions, properties, IO names, keywords, error and problem codes | `tools/gen-reference.mjs` |
| `figures/*.svg` | diagrams — plain SVG shapes and text (no `foreignObject`, no CSS, no external fonts) so react-native-svg can draw them | `tools/figures.mjs` |
| `screens/*.png` | app screenshots (Blocks editor, step editors, text editor, monitor, one Blocks image per instruction example) | `tools/shots.cjs` |
| `data/step-catalog.json`, `data/expression-symbols.json` | snapshots of `GetStepCatalog` / `GetExpressionSymbols` | `tools/dump-catalog.mjs` |
| `tools/reference-notes.mjs` | the hand-written part of each instruction entry: notes, errors, example, see also | hand-written |
| `tools/build-pdf.mjs`, `tools/md.mjs`, `tools/print.css` | Markdown → HTML → PDF (Chromium `page.pdf()`), table of contents and indexes with page numbers; screenshots are re-encoded as JPEG for the PDF only | — |
| `tools/seed.mjs`, `tools/seed-programs/*.srs` | seeds a throwaway controller with the generic example points, tools, frames, grid, stack and programs the screenshots show | — |
| `tools/check-examples.mjs` | parses every ```` ```srs ```` block and reference example on a controller | — |

Links inside chapters: `#<chapter id>` (e.g. `#motion`) points to a chapter, `#ref-<StepType>` (e.g.
`#ref-MoveL`) to an instruction entry in `reference.json`.

A chapter with `"generated"` in the manifest is its Markdown file followed by a section rendered from
`reference.json` (`steps`, `functions`, `errorCodes`, `keywords`). Other readers (the app's docs
viewer) can render the same JSON natively.

## Updating

1. **Catalog changed** (new step, field, unit, default): start a throwaway controller and run
   `CTRL=127.0.0.1:<port> node docs/manual/tools/dump-catalog.mjs`, then
   `node docs/manual/tools/gen-reference.mjs`. A new step also needs an entry in
   `tools/reference-notes.mjs` (the generator warns).
2. **Diagrams:** edit `tools/figures.mjs`, run it (rewrites `figures/` and the manifest's figure list).
3. **Screenshots:** with a throwaway controller seeded by `tools/seed.mjs` and the Expo web app on
   `:8081`, copy `tools/shots.cjs` into `SimpleRobotApp/` and run it from there (see its header).
4. **Examples:** `CTRL=127.0.0.1:<port> node docs/manual/tools/check-examples.mjs` — every script example must parse.
5. **PDF:** `node docs/manual/tools/build-pdf.mjs`.
