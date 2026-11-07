// Generates docs/manual/reference.json — the instruction, function, property, keyword and
// error-code reference — from the controller's own data:
//   data/step-catalog.json        GetStepCatalog (dump-catalog.mjs refreshes it)
//   data/expression-symbols.json  GetExpressionSymbols (functions + properties)
//   ../program-text.md            the catalog-generated argument table (text-form value words)
//   tools/reference-notes.mjs     hand-written notes, errors, examples, see-also
//
//   node docs/manual/tools/gen-reference.mjs
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, "..");
const read = p => JSON.parse(fs.readFileSync(path.join(ROOT, p), "utf8"));
const catalog = read("data/step-catalog.json");
const symbols = read("data/expression-symbols.json");
const notes = (await import(pathToFileURL(path.join(HERE, "reference-notes.mjs")).href)).default;

// ── Text-form value words from program-text.md's generated table ──────────
const textDoc = fs.readFileSync(path.resolve(ROOT, "..", "program-text.md"), "utf8").replace(/\r\n/g, "\n");
const gen = textDoc.slice(textDoc.indexOf("<!-- BEGIN GENERATED ARGUMENTS"), textDoc.indexOf("<!-- END GENERATED ARGUMENTS"));
const valueWords = {}; // label -> fieldLabel -> value text
let cur = null;
for (const line of gen.split("\n")) {
  const h = line.match(/^\*\*(.+?)\*\* — /);
  if (h) { cur = valueWords[h[1]] = {}; continue; }
  if (cur && line.startsWith("| ") && !line.startsWith("| In text") && !line.startsWith("|---")) {
    const cells = line.slice(1, -1).split(/(?<!\\)\|/).map(c => c.trim().replace(/\\\|/g, "|"));
    cur[cells[1]] = cells[2].replace(/`/g, "");
  }
}

const KIND_TEXT = {
  number: "number", integer: "number", bool: "on / off", enum: "option", text: "text",
  template: "\"text {$var}\"", expr: "expression", condition: "condition", varWrite: "$variable",
  listVar: "$list", pointRef: "point name", toolRef: "tool name", localRef: "local name",
  routineRef: "routine name", programRef: "program name", visionRef: "vision program name",
  deviceRef: "device name", cameraRef: "camera name", labelRef: "label name", target: "target",
  axes6: "(x=…, y=…, z=…, rx=…, ry=…, rz=…)", textList: "(a, b, …)", exprMap: "rows: name = expression",
  mappingOut: "rows: key -> $variable", rows: "rows: \"key\" = value", pluginRef: "plugin.step",
};
const KIND_TYPE = { // a short "type" word for the parameter table
  number: "Number", integer: "Integer", bool: "Boolean", enum: "Option", text: "Text", template: "Template",
  expr: "Expression", condition: "Condition", varWrite: "Variable", listVar: "List variable",
  pointRef: "Point", toolRef: "Tool", localRef: "Local frame", routineRef: "Routine", programRef: "Program",
  visionRef: "Vision program", deviceRef: "Device", cameraRef: "Camera", labelRef: "Label", target: "Target",
  axes6: "6 axes", textList: "Text list", exprMap: "Rows", mappingOut: "Output rows", rows: "Rows",
  pluginRef: "Plugin step", steps: "Steps", branches: "Branches", cases: "Cases", json: "JSON",
};
const BODY_KINDS = new Set(["steps", "branches", "cases", "json"]);

const fmtDefault = d => d == null ? "" : typeof d === "boolean" ? (d ? "on" : "off") : Array.isArray(d) ? d.join(", ") : String(d);

const catLabel = Object.fromEntries(catalog.categories.map(c => [c.key, c.label]));
const byType = Object.fromEntries(catalog.steps.map(s => [s.type, s]));
const errorLabel = Object.fromEntries(catalog.errorCodes.map(e => [e.code, e.label]));

const steps = catalog.steps.map(s => {
  const n = notes[s.type] ?? {};
  if (!notes[s.type]) console.warn(`no notes for ${s.type}`);
  const words = valueWords[s.label] ?? {};
  const params = s.fields.filter(f => !f.hidden && !BODY_KINDS.has(f.kind)).map(f => ({
    key: f.key,
    label: f.label,
    inText: f.position != null ? `position ${f.position + 1}` : f.arg ? `${f.arg}=` : "rows",
    type: KIND_TYPE[f.kind] ?? f.kind,
    kind: f.kind,
    value: words[f.label] ?? KIND_TEXT[f.kind] ?? f.kind,
    unit: f.unit ?? "",
    default: fmtDefault(f.default),
    min: f.min ?? null,
    max: f.max ?? null,
    required: !!f.required,
    expression: !!(f.exprKey || f.exprKeys?.length) || ["expr", "condition"].includes(f.kind),
    options: f.options?.map(o => ({ value: o.value, label: o.label })) ?? null,
    showIf: f.showIf ?? null,
    help: f.help ?? "",
    group: f.group ?? "",
  }));
  const bodies = s.fields.filter(f => BODY_KINDS.has(f.kind) && !f.hidden).map(f => f.label);
  return {
    type: s.type,
    keyword: s.keyword,
    aliases: s.aliases,
    name: s.label,
    category: s.category,
    categoryLabel: catLabel[s.category],
    description: s.description.replace(/\.?$/, "."),
    syntax: s.syntax,
    background: s.background,
    body: s.body,
    bodies,
    params,
    notes: n.notes ?? [],
    errors: (n.errors ?? []).map(([code, when]) => ({ code, label: errorLabel[code] ?? "Raised error", catchable: catalog.errorCodes.find(e => e.code === code)?.catchable ?? true, when })),
    example: { srs: n.example ?? "", screen: `screens/ref-${s.type}.png` },
    seeAlso: (n.seeAlso ?? []).filter(t => byType[t]).map(t => ({ type: t, name: byType[t].label, keyword: byType[t].keyword })),
    chapter: n.chapter ?? null,
  };
});

// ── Error codes ───────────────────────────────────────────────────────────
const ERROR_TEXT = {
  waitTimeout: "A condition Wait with `onTimeout=error` reached its timeout.",
  visionTimeout: "Run Vision got no fresh result in time.",
  visionFailed: "The vision processor was lost, the robot output frame is not available, or a camera had no frame (Capture / Save Image).",
  httpFailed: "HTTP Request: network error or a non-2xx status.",
  httpTimeout: "HTTP Request: no reply in time.",
  receiveTimeout: "HTTP Receive: nothing was posted in time.",
  pluginFailed: "A plugin step or function answered with an error.",
  pluginTimeout: "A plugin step did not answer in time.",
  pluginNotRunning: "The plugin is installed but not running.",
  promptTimeout: "A Prompt timed out and has no timeout button.",
  pointNotFound: "A template or list target names no saved point, a Points list is empty, or a via point is unknown.",
  arcDegenerate: "Move Arc: start, via and end are on one line, or two are the same.",
  indexOutOfRange: "List / Save Position: the index is outside the list.",
  userError: "The default code of Raise Error. Any identifier you raise yourself is also catchable.",
  motionFault: "The motion system reported a fault.",
  jointLimit: "A target is outside a joint's soft limits.",
  driverDisconnected: "The motor driver disconnected.",
  emergencyStop: "The emergency stop was pressed.",
  userStop: "The operator stopped the program.",
  unknownVariable: "An expression names a variable, property or IO point that does not exist.",
  expressionSyntax: "An expression does not parse (also `unknownFunction` / `badArity`).",
  computedVariable: "A step tried to write a computed variable.",
  badListOperation: "A List or Save Position step does not fit its list (an authoring error).",
};
const raisedBy = {};
for (const s of steps) for (const e of s.errors) (raisedBy[e.code] ??= []).push(s.type);
const errorCodes = catalog.errorCodes.map(e => ({ ...e, description: ERROR_TEXT[e.code] ?? "", raisedBy: [...new Set(raisedBy[e.code] ?? [])] }));

// Problem codes reported before a run (docs/program-text.md "Problems", expressions-and-variables.md §4)
const parseCodes = [
  ["syntax", "Not valid here (with what was expected)."],
  ["unknownKeyword", "A statement word that does not exist (with \"did you mean …\")."],
  ["unknownArgument", "A named value the step does not have."],
  ["duplicateArgument", "A named value given twice."],
  ["missingArgument", "A required value is missing."],
  ["badValue", "The wrong kind of value: text where a number belongs, an option that does not exist, an expression without parentheses in a positional slot."],
  ["expressionSyntax", "From the expression checker, pointing inside the expression (also `unknownFunction`, `badArity`)."],
  ["missingEnd", "A block without `end`."],
  ["unexpectedEnd", "An `end` with no block to close."],
  ["misplacedClause", "`else` / `case` / `catch` in the wrong place."],
  ["declarationKind", "An impossible declaration, e.g. `persistent computed`."],
  ["stringRequired", "A text variable assigned an expression, or a number variable assigned text."],
  ["duplicateId", "Two steps with the same `@id`."],
  ["ambiguousName", "An underscore-form name fits more than one item — write the quoted name."],
  ["unknownPoint / unknownTool / unknownRoutine / unknownGrid / unknownDevice / unknownLabel …", "Warning: the name does not resolve (yet); it is kept."],
  ["unknownField", "Warning: an `@with` key that is not a field (dropped)."],
];
const validationCodes = [
  ["unknownPoint, unknownTool, unknownLocal, unknownRoutine, unknownVisionProgram, unknownGrid, unknownStack, unknownProgram", "error", "A referenced item does not exist."],
  ["unknownLabel / duplicateLabel", "error", "Go To Label names no label / two labels share a name."],
  ["unknownVariable / unknownProperty", "error", "An expression names something that does not exist."],
  ["expressionSyntax / unknownFunction / badArity", "error", "An expression does not parse."],
  ["readOnlyProperty", "error", "A property or IO name used as a write target or declared as a variable."],
  ["missingField", "error", "A required field is empty."],
  ["emptyLoop / emptyBranch", "error", "A loop or branch with no steps."],
  ["routineRecursion", "error", "A routine calls itself (directly or indirectly)."],
  ["loopControlOutsideLoop", "error", "Break / Next outside a loop (or crossing a routine boundary)."],
  ["computedCycle / computedVariable / computedGlobalScope / computedKindConflict", "error", "Computed-variable rules (see Data and variables)."],
  ["unknownParam / paramOutputType", "error", "A routine argument or output that does not fit the routine's parameters."],
  ["unsupportedOnStep", "error", "A field the step does not support (e.g. a stop condition on Move Arc)."],
  ["cameraNotCalibrated / badOutputFrame", "error", "Run Vision robot frame without a calibrated camera / an unknown frame."],
  ["unknownPlugin / unknownPluginStep / pluginParamMissing / pluginParamEnum / pluginOutputType / unknownPluginFunction", "error", "Plugin step and function checks."],
  ["paramMissing", "warning", "A routine parameter with no argument keeps its declared value."],
  ["duplicateCaseValue", "warning", "Two Switch cases share a value."],
  ["pluginNotRunning", "warning", "The plugin is installed but not running."],
  ["untaughtGrid", "warning", "A grid or stack whose steps are all zero."],
  ["pointWriteInBackground", "warning", "A background program overwrites a saved point."],
  ["disabledStep / unreachableStep / unusedVariable / unknownStepType", "warning", "Housekeeping hints."],
];

// ── Functions and properties ──────────────────────────────────────────────
const functions = symbols.functions.filter(f => !f.pluginId).map(f => ({ signature: f.signature, name: f.name, description: f.description }));
const props = [];
const seen = new Set();
for (const p of symbols.properties.filter(p => !p.pluginId)) {
  let name = p.name, description = p.description;
  const aux = name.match(/^aux\.[^.]+\.(\d+\.)?(position|moving)$/);
  if (aux) {
    name = aux[1] ? "aux.<deviceId>.<axis>.position" : "aux.<deviceId>.moving";
    description = aux[1] ? "Aux axis position (steps)" : "1 while the aux device is moving";
  }
  if (seen.has(name)) continue;
  seen.add(name);
  props.push({ name: "$" + name, type: p.type, description, group: name.split(".")[0] });
}
props.push({ name: "$camera.<cameraId>.calibrated", type: "boolean", description: "1 when the camera has a camera-to-robot calibration", group: "camera" });
props.push({ name: "$<pluginId>.<name>", type: "number", description: "A running plugin's live property, e.g. $scale.weight", group: "plugin" });
const io = [
  { name: "$stb.in1 … $stb.in4", description: "Driver board inputs (1/0)" },
  { name: "$stb.out1 … $stb.out4", description: "Driver board outputs (1/0)" },
  { name: "$relay.1 … $relay.4", description: "USB relay channels (1/0)" },
  { name: "$io.<device>.<pin>", description: "An IO device pin: digital 1/0, a load cell in grams. Names with spaces use their underscore form: $io.Arduino_Nano.Load_Cell" },
];

// ── Keywords ──────────────────────────────────────────────────────────────
const flowWords = ["if", "elif", "else", "end", "repeat", "forever", "as", "while", "for", "in", "switch", "case", "default",
  "try", "catch", "break", "continue", "return", "exit", "raise", "label", "goto", "call", "out", "pause", "status",
  "stopwatch", "prompt", "plugin", "unknown"];
const declWords = ["var", "computed", "param", "global", "persistent", "input", "monitor", "number", "bool", "string", "image", "stopwatch", "list", "point", "record"];
const valueWordsList = ["on", "off", "true", "false", "here", "grid", "stack", "toggle", "send", "receive", "and", "or", "not", "_"];
const keywords = {
  statements: steps.filter(s => /^[A-Z]/.test(s.keyword)).map(s => ({ word: s.keyword, type: s.type, name: s.name })),
  flow: flowWords, declarations: declWords, values: valueWordsList,
  attributes: ["@name", "@off", "@id", "@with", "@program"],
  stepTypeNames: catalog.steps.map(s => s.type),
};

const reference = {
  generatedFrom: { catalogVersion: catalog.catalogVersion, languageVersion: catalog.languageVersion, generatedAt: new Date().toISOString().slice(0, 10) },
  categories: catalog.categories.map(c => ({ ...c, steps: steps.filter(s => s.category === c.key).map(s => s.type) })).filter(c => c.steps.length),
  steps,
  errorCodes,
  problemCodes: { parse: parseCodes.map(([code, meaning]) => ({ code, meaning })), validation: validationCodes.map(([code, severity, meaning]) => ({ code, severity, meaning })) },
  functions,
  properties: props,
  io,
  keywords,
  commonFields: catalog.commonFields,
};
fs.writeFileSync(path.join(ROOT, "reference.json"), JSON.stringify(reference, null, 2) + "\n");
console.log(`reference.json: ${steps.length} steps, ${errorCodes.length} error codes, ${functions.length} functions, ${props.length} properties`);
