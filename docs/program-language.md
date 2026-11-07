# Program editor v2 and the text language — design spec

Status: **design spec** for Kitchen order b10ec549 (2026-11-06). Package A (contracts and
scaffolding) has landed — see [Package A — as built](#package-a--as-built-the-frozen-contracts-read-this-before-bg)
for the frozen contracts; the rest is not built yet. It is
written for the implementation agents and is meant to be split along the work packages in
[§8](#8-work-packages). When a package lands, move the user-facing parts into
[program-blocks.md](program-blocks.md) and [expressions-and-variables.md](expressions-and-variables.md),
and leave this file as the design record.

Lucas's brief: *"flesh out the regular program editor as much as you can, and maybe also create a
matching textual programming language users can choose as well … use Epson's SPEL+ manual and ABB's
manual for guidance."* The vision-inspection rework is the model to copy. It used:

- generic building blocks;
- clear categories;
- typed parameters whose metadata drives the UI;
- a unit test for every item;
- full-page config editors;
- drag-bar reordering.

Contents

1. [Summary](#1-summary)
2. [Inventory: what exists today](#2-inventory-what-exists-today)
3. [SPEL+ and RAPID: what they offer](#3-spel-and-rapid-what-they-offer)
4. [Gap analysis and new building blocks](#4-gap-analysis-and-new-building-blocks)
5. [The step catalog](#5-the-step-catalog)
6. [The text language (SRS)](#6-the-text-language-srs)
7. [Editor UX](#7-editor-ux)
8. [Work packages](#8-work-packages)
9. [Open questions for Lucas](#9-open-questions-for-lucas)

---

## 1. Summary

- **Keep one program model.** The flat `ProgramStep` JSON stays the only stored form. The text
  language is a second *view* of the same `BuiltProgram`. It is never stored. Blocks and Text
  round-trip 1:1, ids included: `Normalize(Parse(Format(p), base: p)) == Normalize(p)` (§6.9).
- **Add a step catalog.** One C# table, `StepCatalog`, describes every step type: its category,
  label, fields (kind, unit, default, range, whether an expression is allowed, conditional
  visibility) and its text syntax. `GetStepCatalog` publishes it. It drives:
  - generic field rendering in the app for simple steps;
  - completion and hover help in the text editor;
  - the generated block reference.

  This is the vision editor's `TOOL_META` / `ParamFieldSpec` pattern, applied to program steps.
- **Fill the gaps that SPEL+ and RAPID show up.** Most of the new blocks are generic. In priority
  order:
  - **P0, finish half-built features:** step enable/disable is wired in the validator and progress
    reporter but missing from the model in both repos. Add Note/Section blocks.
  - **P1, program structure and runtime:**
    - flow: Break/Continue, Return, Exit, Switch/Case;
    - errors: Try/Catch with retry, Raise error, an error mode for wait timeouts;
    - routines with in/out parameters;
    - operator prompts;
    - List operations;
    - Save Position ("Here"), a program speed factor, and an Arc move (MoveC);
    - run-from-step, single-step and breakpoints.
  - **P2:** move-distance IO triggers, program guards (interrupt-lite), joint-target moves, a
    grid-cell loop, edge waits and Jump arch rounding.
- **Parse and format on the controller.** The parser and printer are written once, in C#, beside
  `ExpressionEvaluator`, the validator and the repositories that resolve names to ids. The app calls
  `FormatProgramText` / `ParseProgramText`. The app's only language code is a small tokenizer for
  colouring.
  - This means no second parser to drift from the first.
  - It gets xUnit round-trip tests.
  - The MCP server and live tests can read and write programs as text, and so can Claude.
- **Seven work packages.** Package **A** (contracts and scaffolding) lands first and alone. Then
  **B–G** run in parallel with almost no file overlap (§8).

---

## 2. Inventory: what exists today

### 2.1 Program model (controller `RobotControl/Models/`)

`BuiltProgram { id, name, description, steps[], variables[], displayPage*, isRoutine, isBackground,
killBackgroundOnStop, lastUpdatedUnixMs }`. A program can be one of three kinds:

- a **main** program;
- a **routine**: hidden from the list, called with `CallRoutine`, and sharing the caller's variable
  scope; it has **no parameters**;
- a **background** program: runs in parallel, and skips motion, tool and homing steps.

`ProgramStep` is one flat class keyed by `type`. There are 47 `StepType` values, including
`Unknown`. Every step has `id`, `name`, `comment` and `expressions{key → expr}`.

- The `expressions` map overrides a numeric literal at run time.
- Its keys are usually the JSON field name, but some are synthetic: `gridRowIndex`, `gridColIndex`,
  `gridGridIndex`, `stackIndex`.

`ProgramVariable` comes in these kinds:

- number, boolean (`isBoolean`), string (`isString` + `stringValue`) and image (`isImage`) scalars;
- lists (`items` + `elementType`: Number | Boolean | Point | Record);
- stopwatches (`isStopwatch`);
- computed formulas (`isComputed`).

The flags `isGlobal`, `isPersistent`, `isInput` and `displayOnMonitor` apply on top of any kind, and
`valueExpression` gives an initial value.

Steps reference these repositories by id; the name is kept as a display fallback:

- points;
- tools;
- locals (user frames);
- grids (the pallet equivalent: base point + row and column offsets + rotation);
- stacks (1-D: base + index × offset);
- routines and background programs;
- vision programs;
- labels;
- plugins (`pluginId` + `pluginStepId`);
- IO devices (`outputDeviceId`, `loadCellDeviceId`, `auxDeviceId`);
- cameras.

Note: `Grid.basePointName` and `RobotStack.basePointName` are still linked **by name**, which breaks
the id-over-name rule. Fixing that is out of scope here, but it is listed in §4.6.

### 2.2 Step types (controller handler and app category)

| App category | Step types | Key fields | Gaps |
|---|---|---|---|
| Motion | `MoveL` `MoveJ` | target (below), `speed accel decel`, `blend blendRadius`, `offset*`, `override*`, `toolOffset*`, `localId`, `moveStop*` | Only consecutive `MoveL` steps blend. |
| | `JumpL` `JumpJ` | the above, plus `jumpZ jumpZStart jumpZEnd` | This is the SPEL `Jump … LimZ` gate move. No arch rounding (SPEL `Arch`). |
| | `ThreadMove` | `threadDistance threadPitch threadPeck threadPeckDepth threadPeckRetract threadExitHeight threadReverseOut` | |
| | `RunHoming`; `SetJointPosition` (declares a joint value, no motion); `CncProgram` (`cncSpec`, edited in cnc-builder) | | No joint-target move (RAPID `MoveAbsJ`) and no arc (`MoveC` / `Arc3`). |
| Motion settings | `SetSpeedL` `SetSpeedJ` `SetBlendRadius` `SetTool` `SetLocal` `ClearLocal` | | No program speed factor (`VelSet`, SPEL `Power Low`). |
| Logic & flow | `IfCondition` (if / elseIf[] / else); `Loop` (count with 0 = infinite, forEach, while); `CallRoutine`; `Label`; `GoToLabel`; `PauseProgram` | `ConditionGroup{combinator, items[{expr, join}]}` | No break, continue, return, exit, switch, try or raise. Routines have no parameters. |
| Variables & timing | `SetVariable` (a numeric expression, or a template for a string variable); `StopwatchControl`; `Wait` (duration, or condition + `waitTimeoutMs` + timed-out flag) | | A timeout can only continue. There are no list operations. |
| I/O & aux | `SetOutput` (card stb / relay / io, pulse, blocking pulse); `Tare` `ReadLoadCell` `StartRecording` `StopRecording` `ExportData`; `AuxMove` `AuxContinuous` `AuxStop` `AuxEnable` `GripperAction` `ConveyorAction` `RotaryIndex` | | `outputValue` is literal only. No toggle. |
| Vision | `RunVision` (`toolOutputs[]`, `outputFrame`); `CaptureImage`; `SaveImage` | | |
| Background | `StartBackground` `StopBackground` `WaitForBackground` | | |
| Communication | `HttpRequest` (`jsonOutbound` / `jsonInbound`); `HttpReceive`; `StatusUpdate` | | No operator prompt (RAPID `TPReadFK` / `UIMsgBox`, SPEL `MsgBox` / `Input`). |
| Plugins | `Plugin` (typed params and outputs from the manifest) | | This is the existing metadata-driven step, and the model for §5. |

**Move target.** A move resolves its target from the first of these that is set:

1. `gridPoint`
2. `stackPoint`
3. `varPointName` (deprecated)
4. `pointNameExpr`: either `$pts[$i]` coordinates, or a template that names a saved point
5. `pointName` / `pointId`
6. nothing: the current position, so offsets act as a relative move

Then the modifiers apply:

- `offset*` — SPEL `P1 +Z(20)`
- `override*` — SPEL `P1 :Z(100)`
- `toolOffset*` — RAPID `RelTool`
- `localId` — SPEL `Local`, RAPID `wobj`

**Stop or record on condition** is the SPEL `Till`/`Find` and the RAPID `SearchL`. It uses
`moveStopCondition`, `moveStopMode` (`stop` | `record`), `moveStopPositionVariableName` and
`moveStopFlagVariableName`. It is already a strong feature.

### 2.3 Expressions and variables

- **Values.** The language is numeric: doubles, with 1/0 for booleans. It reads:
  - variables (`$vars`);
  - IO (`$stb.in1`, `$relay.3`, `$io.<nano>.<pin>`);
  - properties (`$robot.*`, `$program.*`, `$time.*`, `$aux.*`, `$camera.*`, `$<plugin>.*`);
  - lists (`$l[i]`, `$pts[i].x`, `.length`).
- **Operators.** Arithmetic, comparison (equality within 1e-9), `and or not`, and `?:`.
- **Functions.** About 35 built in, plus plugin functions (`id.fn()`).
- **Strings.** Strings are not values. They live in templates (`"Bin {$i}"`), and the only string
  methods are `.contains` / `.startsWith` / `.endsWith`.
- **Checking.**
  - `ExpressionEvaluator.TryParse` gives syntax errors with a position.
  - `ValidateBuiltProgram` returns problems keyed by `stepId` + `stepPath`, with about 40 codes.
  - `GetExpressionSymbols` feeds the app's expression assist.

The app has **no expression parser of its own**. It asks the controller for evaluation, live values
and validation. That is the precedent for putting the text parser on the controller too.

### 2.4 Execution (controller `ProgramExecutor` + `Execution/`)

- **Frame stack.** The executor walks a stack of `StepListFrame` once per control tick. Frame kinds
  are `Plain`, `CountLoop`, `WhileLoop`, `ForEach` and `Cnc`.
- **Handlers.** Each step has an `IStepHandler` that returns `Advance`, `Yield` or `Finished`. The
  handlers are registered in `ExecutionContext.StepHandlers` (`ExecutionContext.cs:397`).
- **Errors end the program.** Every error goes through `Finish(Error, message)`. There is no
  catch or retry.
- **Stop pauses.** A user Stop *pauses* the program, keeps the stack and resumes the interrupted
  motion.
- **No routine boundary.** `CallRoutine` pushes a `Plain` frame. There is no routine boundary on
  the stack, so `GoToLabel` can unwind out of a routine. `Return` and `break` will need a marker
  frame kind.
- **Dead feature.** `ProgressReporter.SkippedDisabled` and
  `ProgramValidator.ValidationCodes.DisabledStep` exist, but **`ProgramStep` has no `enabled` field
  in either repo**. The `enabled` flag described in expressions-and-variables.md §1 is dropped on
  the wire, and nothing skips the step. The app has the vestiges too:
  - an unused `builderStyles.ts:287 stepCardDisabled`;
  - InfoTip copy that says "copy, cut, **disable**" (`builder.tsx:1631`).

### 2.5 App builder (`app/(tabs)/program/builder.tsx`, `src/components/ui/builder/`)

**What works well, and should be kept**

- **Document history.** One `EditorDoc` with `useDocumentHistory`: 100 entries, merge within the
  same tick, and typing coalesced within 1 s. Undo/redo buttons and shortcuts. A dirty check against
  a snapshot.
- **Clipboard and selection.** A module-level step clipboard works across programs and across
  pushed routine screens; paste clones with fresh ids. Long-press multi-select offers copy, cut,
  wrap in Loop, make Routine and delete.
- **Live validation.** Debounced 500 ms. Problems roll up to parent steps. There is a problems pill
  and a `ValidationPanel`, and tapping a problem opens that step. Run is blocked on errors.
- **Revisions.** List and restore, as an undoable edit.
- **Expression editing.** `ExpressionEditorModal` has operator chips, caret autocomplete
  (`completions.ts`), a live value (`useLiveValue`) and a helper panel on wide screens.
- **Variables.**
  - `VariableEditModal` covers every kind.
  - Renaming rewrites every `$name` reference and bare-name field (`renameVariable.ts`).
  - Any field can create a new variable through `VarPicker` ("+ New … variable").
- **Plugin steps.** These are already descriptor-driven: `PluginStepFields.tsx` renders params
  from the manifest.

**Gaps and smells**

1. **`StepConfigModal.tsx` is 3,667 lines of hand-written JSX.**
   - `renderMainBody` switches on about 45 types, and `renderSubPage` switches on 8 sub-pages.
   - It has about 25 picker-open `useState`s.
   - The 6-axis offset, override and tool-offset blocks are copy-pasted three times (`:871-956`).
   - The point-label logic is duplicated (`:969` and `:1002`).
   - There is a dead ternary at `:3559`.
   - Nothing maps fields to UI except the JSX itself.
2. **Step metadata is spread over five parallel structures in `stepUtils.tsx`:**
   - the `StepIcon` switch;
   - `STEP_THEME`, which holds labels;
   - `STEP_TYPES`, which holds labels *again*, plus descriptions;
   - `STEP_CATEGORIES`;
   - the `stepLabel` and `stepDetail` switches.

   Defaults live separately again, in `builder.tsx:661 defaultStep`, a 45-field literal. A new step
   type touches 7 places (listed in §8 package A).
3. **Modals stack on modals.** The step modal opens sub-pages, then `VarPickerModal`, then
   `ExpressionEditorModal`. Edits are draft-then-Save, unlike the vision editor's live edits with
   autosave.
4. **Only one scope is visible at a time.** Loop and If bodies are entered through a breadcrumb.
   You cannot see a body in context, drag a step into or out of a body, or collapse anything.
5. **No field-level validation.** The modal receives no problems. Only plugin params check
   themselves.
6. **Missing tools:**
   - enable/disable;
   - Notes or sections;
   - search or filter of steps;
   - teach-here (create or update a point from the live pose);
   - run-from-step, single step, breakpoints;
   - a current-step highlight in the builder.
7. **The builder hand-rolls drag-reorder** (`builder.tsx:570-657`), while the vision editor uses
   the generic `src/components/ui/useDragReorder.ts` + `DragHandle`.
8. **The golden drift guard only works one way, and only for fields someone adds to it.** Several
   TS fields are missing from the golden, so they are unguarded:
   - `jsonUrl` and the other `json*` fields;
   - `captureImage*`;
   - `httpReceive*`;
   - `unknownStepType`.

   The `StepType` union itself is not guarded at all.

**Vision editor patterns to copy** (from `src/models/visionTools.ts`, `vision-editor.tsx` and
`src/components/ui/vision-editor/`):

- **A metadata table** (`TOOL_META`, with `ParamFieldSpec {kind int|number|slider|segment|bool|select,
  min, max, step, desc, showIf}`), plus factories (`newTool`) instead of default literals.
- **One generic renderer**, `ParamFields.tsx`. Bespoke code is kept only for genuinely special
  sections.
- **Categories as data**, with tints and icons.
- **A full-page config editor** (`ToolConfigPage`): a Done button, Android back closes it, two panes
  on wide screens, `GroupCard` sections, and a generic `ChoiceSheet` for picks.
- **Live edits with a 400 ms debounced autosave.** No draft/Save step.
- **A per-item test run with a LIVE RESULT card.** Programs need the same thing: test one step or
  one body.
- **An enable switch on every row,** with disabled rows dimmed.
- **Drag bars that refuse a drop which would break dependency order,** with an explanation. Steps
  need the same for a `goto` that crosses a routine boundary.

---

## 3. SPEL+ and RAPID: what they offer

Sources:

- Epson SPEL+ Language Reference: RC+ 7.0 v7.5 and v4.2.
- ABB *RAPID Instructions, Functions and Data types* (3HAC16581-1), *RAPID overview* (3HAC16580-1)
  and *RAPID kernel* (3HAC16585-1).

The full PDFs were not machine-readable from here: the Epson PDF did not parse, and the ABB library
returned 403. So this section was cross-checked against the manual indexes and listings only. **Check
the exact keyword spelling against the manuals before quoting it in user docs.** The concepts are
standard and stable.

| Area | Epson SPEL+ | ABB RAPID | What we have |
|---|---|---|---|
| PTP / linear | `Go`, `Move` | `MoveJ`, `MoveL` | MoveJ, MoveL |
| Gate move | `Jump P1 C2 LimZ -10 Sense Sw(5)=On`: arch table, LimZ, Sense | — (write it with Offs) | JumpL/J with jumpZ; no arch, no sense |
| Circular | `Arc mid, end` (XY), `Arc3` (3D) | `MoveC CirPoint, ToPoint` | none (CNC tessellates DXF arcs) |
| Joint target | `Go JA(…)`, `Agl()` | `MoveAbsJ jpos` | none |
| Blending | `CP On/Off`, the `CP` modifier, `Fine` (in-position) | zonedata `fine` / `z1`…`z200`, `\Inpos` | `blend` / `blendRadius` on MoveL only |
| Dynamics | `Speed % [,depart, appro]`, `Accel`, `SpeedS`, `AccelS`, `Power Low`, `Weight` | speeddata `v100`, `\V`, `\T`; `VelSet %`; `AccSet`; `GripLoad` | per-step speed/accel/decel, SetSpeedL/J; no program % |
| Stop on condition | `Till cond`, `TillOn`; `Find cond` + `FindPos` / `PosFound` | `SearchL \Stop sig, pHit, pTo` | moveStopCondition stop/record + position/flag — already better than both |
| IO during motion | `Move P1 !D50; On 1; D90; Off 2!` | `TriggIO` / `TriggEquip` + `TriggL`; `MoveLDO` | none |
| Point arithmetic | `P1 +Z(20)`, `P1 :Z(100)`, `XY()`, `CX(P1)`, `P1 = Here`, `Dist` | `Offs()`, `RelTool()`, `CRobT()`, `CJointT()`, `Distance()` | offsets / overrides / toolOffset per step, `$robot.*`, `dist()`; no "save Here" |
| Frames / tools | `Local n`, `Tool n` / `TLSet`, `Base`, `Arm`, `Hand Lefty/Righty` | wobjdata, tooldata, loaddata | Locals, Tools |
| Pallet | `Pallet n, P1, P2, P3, cols, rows`, `Pallet(n, i)` | — | Grids + Stacks (index, or row/col) |
| Zones | `Box`, `Plane`, `InsideBox()`, `XYLim` | `WZBoxDef`, `WZLimSup`, `WZDOSet` | none |
| Output | `On n [,time [,parallel]]`, `Off`, `Out port, v`, `MemOn` | `SetDO`, `PulseDO \PLength`, `InvertDO`, `SetGO`, `SetAO` | SetOutput + pulse (+ blocking) |
| Wait | `Wait t`, `Wait Sw(1)=On, 5` then `TW` | `WaitTime`, `WaitDI di1,1 \MaxTime \TimeFlag` (otherwise `ERR_WAIT_MAXTIME`), `WaitUntil` | Wait duration / condition + timeout + flag; no "error on timeout" |
| Branches | `If/ElseIf/Else`, `Select…Case…Send` | `IF/ELSEIF`, `TEST…CASE…DEFAULT` | If/ElseIf/Else; no switch |
| Loops | `For…Step…Next`, `Do While/Until…Loop`, `Exit For/Do` | `FOR…FROM…TO…STEP`, `WHILE`, no break (via GOTO) | count / forEach / while; no break / continue / step |
| Jumps | `GoTo`, `GoSub`/`Return` | `GOTO` (within a routine) | Label / GoToLabel (can unwind out of a routine) |
| Subroutines | `Function f(a, ByRef b) … Fend`, `Call`, `Exit Function` | `PROC p(num a, INOUT num b, \switch s)`, `FUNC`, `RETURN` | CallRoutine (no params, shared scope) |
| Errors | `OnErr GoTo h`, `Err`/`ErrMsg$`, `EResume [Next]`, `Error n` | `ERROR` handler, `ERRNO`, `RETRY`, `TRYNEXT`, `RAISE`, `BookErrNo` | none — everything is fatal |
| Interrupts | `Trap n, Sw(1)=On Call f`; `Trap Emergency/Error/Abort/Pause` | `CONNECT ir WITH trap`; `ISignalDI`, `ITimer`, `IEnable/IDisable` | none (background programs cannot move) |
| Tasks | `Xqt f`, `Halt`/`Resume`/`Quit`, `SyncLock`, `Signal`/`WaitSig`, NoEmgAbort tasks | Multitasking, `PERS` shared, `WaitSyncTask`, `TestAndSet` | Start/Stop/WaitForBackground, global vars; no params or signals |
| Data | Integer/Real/Double/String, arrays to 3D, `Global [Preserve]`, `Module` | num/dnum/bool/string, `RECORD`, arrays `{3,4}`, `CONST/VAR/PERS` | number/bool/string/image/list(Number, Boolean, Point, Record), global, persistent, computed |
| Strings | `Left$ Mid$ Right$ Len InStr Str$ Val Fmt$ ParseStr` | `StrLen StrPart StrFind StrToVal NumToStr` | templates + contains/startsWith/endsWith |
| Timers | `TmReset`/`Tmr`, `Time$`, `Date$` | `ClkStart/Stop/Read`, `CDate`, `CTime` | Stopwatch, `$time.*` |
| Comms | `OpenCom`, `OpenNet`, `Print #`, `Input #`, `ChkNet`, `AOpen`/`ROpen`/`WOpen` | `Open`/`Write`/`ReadStr`, `Socket*` | HttpRequest / HttpReceive, ExportData (CSV/JSON); serial/TCP is plugin territory |
| Operator | `Print`, `Input`, `MsgBox`, GUI Builder | `TPWrite`, `TPReadNum`, `TPReadFK`, `UIMsgBox`, `UINumEntry` | StatusUpdate (one-way), display pages |
| Vision / force / conveyor | `VRun`/`VGet`, Force Guide, `Cnv_*` | Integrated Vision, FC, conveyor wobj | RunVision (robot frame), load-cell stop conditions, ConveyorAction (no tracking) |
| Comments | `'` | `!` | step `comment` |

**Takeaways for a generic hobby/maker 4-axis arm.** ASTRO is J1 rotary, J2/J3 CoreXY, J4 wrist, with
Arduino IO and a phone/web editor.

- **Already ahead.** The gate (Jump) move and stop/record-on-condition are covered, and are better
  than both languages: any expression can be the condition, and the position is recorded. The pallet
  is covered by Grids and Stacks.
- **What both vendors have that we lack, and that maps cleanly onto blocks:**
  1. structured errors (RETRY / TRYNEXT / RAISE, OnErr / EResume);
  2. select / switch;
  3. break / continue / return;
  4. routine parameters (`PROC` IN/INOUT, `ByRef`);
  5. operator prompts (TPReadFK / UIMsgBox);
  6. save the current position ("Here", `CRobT`);
  7. a program speed percentage (VelSet, Power Low);
  8. circular moves;
  9. IO triggered along a move (TriggIO, `!D50!`);
  10. interrupts (Trap / ISignalDI), done as guards;
  11. joint-target moves (MoveAbsJ);
  12. zones (Box / WorldZones).
- **Deliberately not copied** (too niche, or the wrong mental model for makers):
  - typed records and PERS-in-source;
  - group and BCD IO, analog speed outputs;
  - path recorder;
  - conveyor encoder tracking and multimove;
  - preprocessor includes;
  - raw socket and serial in the core language. Plugins are the extension point for those.

---

## 4. Gap analysis and new building blocks

Priorities:

- **P0** — fix or finish what exists. Small, and first.
- **P1** — the core of this order.
- **P2** — after P1, if there is time.
- **P3** — noted only.

Every block below is generic: no project-specific wording, defaults or labels. Model shape follows
the existing convention: one flat `ProgramStep`, `*Expr` / `expressions` for computed values, ids for
links and names as display only.

Common notation: `T?` is an optional field. *Expr* means the catalog marks it expression-capable:
the literal sits in the field and an expression goes in `expressions[exprKey]`.

### 4.1 P0 — foundation

| # | Change | Model | Controller | App |
|---|---|---|---|---|
| 1 | **Enable/disable a step** | `ProgramStep.enabled?: bool` (null = true) | `ExecuteStepInner`: if `enabled == false`, call `Progress.SkippedDisabled(step)` and `CompleteNow`. A disabled block step skips its whole body. The validator emits `disabledStep` (warning) and does not check problems inside a disabled body. | A row switch, as in the vision editor (dim + strike the label); a select-bar "Disable/Enable" action. Text: `@off`. |
| 2 | **Note** (section header / free note), SPEL `'`, RAPID `!` | new `StepType.Note`; text in `comment`; `noteStyle?: "note" \| "section"` | no-op handler (`Advance`), not counted as a step in progress (CountSteps skips it) | A divider-style row; a section row can collapse the steps up to the next section (view state only). Text: a comment paragraph / `## Title`. |
| 3 | **Complete the golden guard** | add the missing `json*`, `captureImage*`, `httpReceive*` and `unknownStepType` keys and all new fields to both goldens; add `steptypes.golden.json`, the list of every `StepType` | `GoldenStepTypeTests`: the enum equals the list | `programStepGolden.check.ts`: `satisfies` covering every field; `StepType` union checked against the list (`const` array + type-equality assertion) |
| 4 | **`StepTree` helper** | — | `ProgramStep.ChildLists()` gives every nested step list with its role (`loopSteps`, `ifSteps`, `elseIfBranches[i].steps`, `elseSteps`, `switchCases[i].steps`, `switchDefault`, `trySteps`, `catchSteps`, `cncProgramSteps`). Used by CountSteps, the validator walk, the reconciler and the printer. | `src/models/stepTree.ts`: `childLists(step)` / `walkSteps()`, used by paste (fresh ids), rename, the clipboard, validation indexing and scope navigation |

### 4.2 P1 — flow and program structure (controller package B)

| Block | Category | Fields (type, default, unit) | Semantics | App UI |
|---|---|---|---|---|
| **LoopControl** (break / continue) | Logic & flow | `loopControl: "break" \| "continue"` (default break) | Unwinds frames up to the nearest loop frame (`CountLoop`, `WhileLoop`, `ForEach`). Break pops the loop without re-pushing it. Continue pops the body and runs the normal next-pass logic (`PopExhaustedFrame`). It never crosses a `Routine` frame. Validator: `loopControlOutsideLoop` (error). | Two tiles, "Break loop" and "Next iteration". Text: `break` / `continue`. |
| **Return** | Logic & flow | — | In a routine: unwind to its `Routine` frame and copy the out-parameters (below). In a main or background program: finish `Complete`. | Tile "Return". Text: `return`. |
| **EndProgram** (SPEL `Quit`, RAPID `EXIT`) | Logic & flow | `statusMessage?` (template) | Finish `Complete` with the message, from any depth. A background program ends only itself. | Text: `exit` / `exit "Done {$n}"` |
| **RaiseError** (SPEL `Error n`, RAPID `RAISE`) | Logic & flow | `errorMessage` (template, required); `errorCode?` (identifier, default `userError`) | `ctx.Fail(code, message)` (below). Caught by an enclosing Try, otherwise the program ends `Error` with the message. | Text: `raise "No part at {$i}", code=noPart` |
| **Switch** (SPEL `Select`, RAPID `TEST`) | Logic & flow | `switchExpr` (*expression*, required); `switchCases: SwitchCase[]` where `SwitchCase {id, values: string[], steps: ProgramStep[]}` (each value is an expression, compared with `==` within 1e-9); `switchDefault?: ProgramStep[]` | Evaluates `switchExpr` once. The first case with any equal value runs; otherwise the default. No fall-through. Numeric only: test strings with `if $s.contains(…)`. Validator: `duplicateCaseValue` (warning, for literal duplicates), `emptyBranch` (as for If). | A row like `IfConditionBody`: a case chip per value, an Enter pill per case, add Case / Default. Text: `switch … case … default … end`. |
| **Try** (RAPID `ERROR` + `RETRY`; SPEL `OnErr` + `EResume`) | Logic & flow | `trySteps`; `catchSteps?`; `tryRetries?: int` (default 0); `tryRetryDelayMs?: int` (default 0, ms); `catchCodes?: string[]` (empty = every catchable code); `errorVariableName?` (string variable, receives the message); `errorCodeVariableName?` (string variable, receives the code) | Enter: push a `Try` marker frame, then the body. When `ctx.Fail(code, msg)` happens inside, at any depth (routines too):<br>• Pop frames down to the Try marker.<br>• If attempts ≤ `tryRetries`, wait `tryRetryDelayMs`, then push the body again.<br>• Otherwise write the error variables and push `catchSteps`.<br>• An empty catch means ignore the error and continue after the block.<br>• A code not in `catchCodes` propagates outward.<br>Catchable vs fatal: see below. | A row with Try / Catch bodies like If. Settings: retries, delay, codes (a multi-select from the catalog's code list), variables. "Wrap in Try" in the select bar. Text: `try … catch … end`. |
| **Wait on-timeout** | Variables & timing | `Wait.waitTimeoutAction?: "continue" \| "error"` (default continue) | `error`: on timeout, `ctx.Fail("waitTimeout", "Timed out after N ms waiting for <cond>")`. | A segment under Timeout. Text: `onTimeout=error`. |
| **Routine parameters** (RAPID `PROC (num a, INOUT num b)`, SPEL `ByRef`) | — | `ProgramVariable.paramMode?: "in" \| "out" \| "inOut"` (on a routine's variables); `CallRoutine.routineArgs?: {param: expr\|template}`; `CallRoutine.routineOutputs?: [{key: param, variableName}]` (the `PluginOutputMapping` shape) | `CallRoutine` pushes a new `Routine` frame (instead of `Plain`) that holds the outputs. After `Vars.Initialize(routine)`, each `in`/`inOut` param is set from its argument, evaluated in the *caller's* scope before the push. A template argument is for a string param. On Return, or when the frame is exhausted, each mapped `out`/`inOut` param is copied to the caller's variable. Params live in today's shared scope, so name them distinctly (true locals are P3). Validator: `unknownParam`, `paramMissing` (an `in` param with no arg and no default — a warning, because the declared value is used), `paramOutputType`. | VariableEditModal: a "Routine parameter" segment (Not a param / In / Out / In-Out), shown only on routines. CallRoutine: an argument row per `in`/`inOut` param (an ExpressionInput), plus an output VarPicker per `out`/`inOut`. Text: `param in n = 1` / `call Pick(n=$count), out found=$ok`. |
| **SetOutput** toggle + expression value | I/O | `outputAction?: "set" \| "toggle"` (default set); the value may come from `expressions.outputValue` (non-zero = on) | Toggle reads the current output state from the same IO snapshot the expressions use. | A "Toggle" segment; the value can be an fx. Text: `SetOutput stb 2 toggle` / `SetOutput stb 2 ($n > 3)`. |
| **Debugging** | — | `ExecuteBuiltProgram` gains `startAtStepId?`, `breakpoints?: string[]` and `stepMode?: bool`. New commands: `StepBuiltProgram {}` (run one step then pause) and `SetBreakpoints {stepIds}`. The status gets `pauseReason` and `pausedAtStepId` (package A adds these to the status model). | A **breakpoint** pauses *before* that step (`pauseReason: "breakpoint"`); Continue resumes. In **step mode** every step pauses before it runs. **Run from step** builds the frame stack down that step's path: enclosing loops start at the iteration where the step sits (count loop: iteration 0; forEach: index 0); an If / Switch / Try ancestor enters the branch that contains the step without evaluating it. Background programs ignore breakpoints. | The builder row context menu gets "Run from here" and "Toggle breakpoint" (a red dot in the rail). Monitor: Continue / Step / Stop buttons; the paused step is highlighted. |

**Catchable vs fatal errors.** `ExecutionContext` gains `Fail(string code, string message)`, which
unwinds to an enclosing Try or else calls `Finish(Error, message)`. Package B converts the existing
handlers that end in a *runtime* error to call `Fail` with a code.

- **Catchable codes** (the catalog lists them):
  - `waitTimeout`, `visionTimeout`, `visionFailed`;
  - `httpFailed`, `httpTimeout`, `receiveTimeout`;
  - `pluginFailed`, `pluginTimeout`, `pluginNotRunning`;
  - `promptTimeout`, `pointNotFound` (a template target that names no point), `arcDegenerate`;
  - `userError`, and any `RaiseError` code.
- **Fatal codes, never catchable:**
  - motion faults, joint limits, driver disconnects and e-stop;
  - a user Stop;
  - `unknownVariable`, `expressionSyntax` and computed-variable writes. These are authoring bugs:
    catching them would hide a typo that could move the robot somewhere wrong.

`$program.lastError` / `$program.lastErrorCode` are **not** added: a string is not an expression value.
Use the Try variables instead.

### 4.3 P1 — motion, data and operator blocks (controller package C)

| Block | Category | Fields | Semantics | App UI |
|---|---|---|---|---|
| **SavePosition** (SPEL `P1 = Here`, RAPID `CRobT`) | Motion | `savePositionTo: "variable" \| "point"` (default variable); `variableName` (a Points list); `savePositionMode?: "replace" \| "append" \| "insert" \| "set"` (default replace = a one-element list, as `moveStopPositionVariableName` does); `savePositionIndex?` (*expression*, key `savePositionIndex`); `pointId` / `pointName` (when `to = point`); `savePositionSource?: "actual" \| "target"` (default actual) | Writes the TCP pose. It is expressed in the **active local frame**, so a later move to it under the same local returns to the same place. `to = point` overwrites or creates a saved point through `PointRepository`, so points history records it. Validator: `pointWriteInBackground` (warning). | Point/variable picker + mode. Text: `SavePosition $pts, mode=append` / `SavePosition point Pick1`. |
| **SetSpeedFactor** (RAPID `VelSet`, SPEL `Power Low`) | Motion settings | `speedFactor` (*expression*, %, 1–100, default 100) | effective = operator override × factor / 100, for this program's moves (blended runs too). Resets to 100 when the program finishes or is reset. Readable as `$program.speedFactor`. A factor cannot exceed the operator's override (safety). | A slider + fx. Text: `SetSpeedFactor 50`. |
| **MoveC** (RAPID `MoveC`, SPEL `Arc3`) | Motion | end target = the standard target fields; `viaPointId` / `viaPointName` / `viaPointNameExpr` (the same resolution as the target; grid/stack via is P3); `speed accel decel`; `arcTolerance?` (mm, default 0.1); `localId`, `offset*` (applied to both via and end) | Circle through start, via and end, in 3D. Start is the current pose, or the previous waypoint inside a blended run. It is tessellated so the chord error is ≤ `arcTolerance` (maximum 2° per segment), then dispatched with `Motion.StartContinuous(waypoints, radii)` (the existing blended path). RZ is interpolated linearly by arc angle. Collinear points → `Fail("arcDegenerate")`. No stop condition in v1 (validator: `unsupportedOnStep`). | Two target pickers (Via, End). The modal preview card shows the arc in XY. Text: `MoveC P3, via=P2, speed=50`. |
| **ListOp** | Variables & timing | `variableName` (list); `listAction: "append" \| "insert" \| "removeAt" \| "clear" \| "set" \| "setField" \| "sort" \| "reverse" \| "copyFrom"`; `listIndex?` (*expression*, key `listIndex`); `listValueExpr?` (expression, for a Number / Boolean element); `listFields?: {field: expr}` (Point / Record element, or the one field for `setField`); `listSourceVariable?` (copyFrom); `listSortField?`; `listSortDescending?: bool` | Mutates the list in the variable scope. An out-of-range index → `Fail("indexOutOfRange")`, catchable. `append` with no fields on a Point list appends the current TCP pose ("append Here"). Globals: the global list is updated. | A segment for the action, then fields shown with `showIf` (catalog-driven, no bespoke JSX). Text: method-call statements (§6.6). |
| **Prompt** (RAPID `TPReadFK` / `UIMsgBox` / `TPReadNum`, SPEL `MsgBox` / `Input`) | Communication | `promptTitle?` (template); `promptMessage` (template, required); `promptButtons?: string[]` (1–4, default `["OK"]`); `promptInput?: "none" \| "number" \| "text"`; `promptDefault?` (expression or template); `promptMin?` / `promptMax?` (number); `promptResultVariableName?` (number = button index from 0, or a string variable = button label); `promptInputVariableName?`; `promptTimeoutMs?` (0 = wait forever); `promptTimeoutButton?` (index chosen on timeout; null → `Fail("promptTimeout")`) | The step publishes `prompt {id, stepId, programName, title, message, buttons, input, default, min, max, timeoutMs}` in the program status (package A adds the DTO) and yields. **`AnswerPrompt {promptId, button, value?}`** completes it; the first answer wins and later ones get `stalePrompt`. Stop cancels the prompt, and Continue re-shows it. Background programs can prompt too: prompts are queued per program. | `PromptDialog` mounted app-wide (package G). Text: `prompt "Load the tray", buttons=("Go", "Skip"), result=$choice`. |
| **Expression functions** | — | `inBox(x, y, z, x1, y1, z1, x2, y2, z2)` → 1/0; `inCircle(x, y, cx, cy, r)` → 1/0. String helpers that return **numbers**: `$s.length`, `$s.indexOf("x")` (−1 if absent), `$s.toNumber()` (NaN → evaluates to 0 and sets nothing; see open question 6). | Added to `ExpressionEvaluator.Functions` / `StringMethodNode`; `GetExpressionSymbols` lists them. | appear in the helper panel automatically |

### 4.4 P2 — after P1

These are specified here so the model can take them without churn. **Package A does not add their
fields.** A follow-up package does, with goldens.

| Block / field | Fields | Notes |
|---|---|---|
| **Move triggers** (TriggIO, SPEL `!D50; On 1!`) | `moveTriggers?: [{id, at: "distance" \| "percent" \| "beforeEnd", value, actionType: "output" \| "variable", outputCard, outputNumber, outputDeviceId, outputValue, variableName, valueExpr}]` on MoveL, MoveJ, MoveC and Jump | Checked every executor tick against the distance travelled along the commanded path. Accuracy is about tick period × speed; the docs must say so. Blended runs: per segment. |
| **Program guards** (interrupt-lite; Trap / ISignalDI) | `BuiltProgram.guards?: [{id, name, condition: ConditionGroup, edge: "level" \| "rising", action: "stop" \| "pause" \| "routine", routineId?, message?}]` | Evaluated every tick while the program runs (moves included). `stop`/`pause` act like a user Stop. `routine` stops motion like a user Stop, runs the routine as a nested frame, then resumes the interrupted motion (the existing resume snapshot). A guard cannot fire inside its own routine. Text: top-level `on rising $stb.in4 stop "Door opened"`. |
| **MoveJoints** (MoveAbsJ) | `joint1..joint4?` (*expression*; deg / mm per joint) | Needs forward kinematics from arbitrary joints. `IRobotKinematics.ForwardKinematics(toolOffset)` reads the *current* joints, so add an `FK(joints, tool)` overload. Then MoveJ. |
| **Grid loop** | `loopMode: "grid"`, `loopGridId`, `loopRowVariableName`, `loopColVariableName`, `forEachIndexVariableName`, `loopGridOrder?: "rows" \| "cols" \| "snake"` | The pallet loop. Text: `for $r, $c in grid Tray`. |
| **Wait edge** | `waitEdge?: "level" \| "rising" \| "falling"` | A rising edge waits for the condition to be false, then true. |
| **Jump arch** (SPEL `Arch`) | `jumpArch?` (mm) | The corner blend at the top of the lift and lower legs, through the blended-path dispatcher. |
| **Step test-run** (the vision LIVE RESULT pattern) | `TestRunSteps {program, stepIds}` | Runs a selection in a scratch executor with the program's variables. Builder "Test" button. |

### 4.5 Editor (blocks) improvements (app package E)

Highest value first:

1. **Catalog-driven fields** (`CatalogStepFields.tsx`), using the `ParamFields` pattern. Every new
   P1 block renders from the catalog with *no bespoke JSX*.
   - Fields carry their unit suffix, a default shown as placeholder, range clamping, an fx toggle
     where `exprKey` is set, and `showIf`.
   - Existing bespoke sections (point picker, RunVision outputs, HTTP rows, Plugin) stay as named
     "special" renderers.
   - Migrate simple existing steps opportunistically: SetSpeed*, SetBlendRadius, Wait, Stopwatch,
     AuxStop, AuxEnable, Tare, CaptureImage and the like. Deleting the hand-written JSX for them
     shrinks `StepConfigModal`.
2. **Factories from the catalog.** `newStep(type)` replaces the 45-field `defaultStep` literal.
3. **Enable switch and breakpoint dot** on every row. Note and section rows; collapse a section.
4. **Nested bodies shown inline.** An expand chevron on Loop / If / Switch / Try rows renders the
   body indented, up to 2 levels deep (deeper still uses Enter + breadcrumb). This is the vision
   editor's `fixtureDepth` indentation idea.
5. **Use `useDragReorder` + `DragHandle`.** Delete the hand-rolled PanResponder. Allow dropping into
   an expanded body. Refuse drops that break structure (a `break` outside a loop, a label moved into
   another routine), with an explanation, like `orderProblems`.
6. **Full-page step editor on phones** (`StepConfigPage`, the `ToolConfigPage` pattern): GroupCard
   sections, Done, Android back closes it, live edits into the document (undoable, coalesced), no
   Save. Keep the centred modal on desktop if Lucas prefers it (open question 3).
   The feedback rules still hold:
   - the field being typed stays visible above the keyboard;
   - a modal's Cancel/Save footer stays fixed.
7. **Field-level problems.** Pass `problems` for the step into the editor and show each one under
   its field: `ValidationProblem.field` → the catalog `key`.
8. **Teach-here** in the point picker: "Save current position as new point…" and "Update this point
   to here" (with a confirm and a points-history entry). It links by the new point's id.
9. **Step search / filter** in the toolbar: by label, keyword, `$var` used or point used. Matches are
   highlighted, and "next" enters the scope.
10. **Rich row summaries from the catalog** (`summary` templates): `MoveL → Pick1 · 200 mm/s ·
    blend 5 · until…`. This replaces most of the `stepDetail` switch.
11. **Select-bar actions:**
    - "Wrap in If / Try / Loop";
    - "Extract to routine", which also turns used variables into params (P2);
    - "Disable / Enable";
    - "Copy as text", which puts the SRS text on the clipboard and is good for forum and Discord
      sharing;
    - "Paste text", which parses SRS from the clipboard.
12. **Run controls in the builder:** Run from here, Step, breakpoints, and the current step
    highlighted while running. The `stepId` is already in the progress reports.

### 4.6 Noted, not planned (P3)

- True local variable scope for routines.
- Records with string fields.
- A string expression type (`Mid$`, `Fmt$` equivalents beyond templates).
- Zone definitions as a controller config (Box / WorldZones) — `inBox()` covers ad-hoc checks.
- Inter-program signals / mutex (global variables + Wait cover it today).
- Background program parameters.
- Grid/stack `basePointName` → `basePointId` (id-over-name).
- Serial / TCP / file blocks — plugins.

---

## 5. The step catalog

### 5.1 Why

There are five parallel metadata tables in `stepUtils.tsx`, a defaults literal in `builder.tsx`,
3,600 lines of per-type JSX, the validator's per-type switch and, soon, a text printer and parser.
All of them need the same facts about each step. The vision rework solved this with one metadata
table and one generic renderer. Do the same for steps, but **on the controller**, because the text
language (on the controller) and the app both need it.

### 5.2 Shape — `GetStepCatalog {}` → `{ catalogVersion, languageVersion, categories, steps, errorCodes }`

```jsonc
{
  "catalogVersion": 1,
  "languageVersion": 1,
  "categories": [ { "key": "motion", "label": "Motion", "description": "…" }, … ],
  "errorCodes": [ { "code": "waitTimeout", "label": "Wait timed out", "catchable": true }, … ],
  "steps": [ {
    "type": "MoveL",
    "category": "motion",
    "label": "Move Linear",
    "description": "Move to a target in a straight line",
    "keyword": "MoveL",            // canonical text keyword; matched case-insensitively
    "aliases": [],
    "background": false,           // allowed in background programs (= !IsRestrictedInBackground)
    "body": null,                  // null | "steps" | "branches" (if) | "cases" (switch) | "tryCatch" | "mapping"
    "syntax": "MoveL <target>[, speed=…][, blend=…][, until=…]",   // human-readable, for hover help
    "summary": "→ {target} · {speed} · blend {blendRadius}",        // row summary template, empty parts dropped
    "fields": [ FieldSpec, … ]
  }, … ]
}
```

```jsonc
FieldSpec = {
  "key": "speed",                  // ProgramStep JSON name, or a composite: "target", "via", "offset", "override", "toolOffset", "until"
  "arg": "speed",                  // text named-argument name (defaults to key); "" = positional only
  "position": null,                // 0-based positional slot in text, or null
  "label": "Speed",
  "group": "Dynamics",             // UI section (GroupCard)
  "kind": "number",                // see the table below
  "unit": "mm/s",                  // display unit (mm, deg, ms, mm/s, mm/s², deg/s, %, g, steps)
  "default": null,                 // shown as placeholder; null = "program default"
  "min": 0, "max": null, "step": 1,
  "options": null,                 // enum: [{ "value": "stop", "label": "Stop" }, …]
  "exprKey": "speed",              // key in step.expressions when an expression is allowed; null = literal only
  "required": false,
  "showIf": { "field": "loopMode", "in": ["count"] },   // field-equality predicate (serialisable, unlike the vision showIf fn)
  "help": "Overrides the program's linear speed for this move only."
}
```

| `kind` | Stored as | UI control | Text value |
|---|---|---|---|
| `number` `integer` | number (+ `expressions[exprKey]`) | NumberStepper + fx | literal or expression |
| `bool` | bool | Switch | `on` / `off` / `true` / `false` |
| `enum` | string | SegmentedControl / ChoiceSheet | bare word |
| `text` | string | TextInput | `"…"` |
| `template` | string with `{$…}` | TemplateInput | `"…"` |
| `expr` | string expression | ExpressionInput | expression |
| `condition` | ConditionGroup | ConditionEditor | expression (top-level `and`/`or` split into items, §6.9) |
| `varWrite` / `varRead` / `listVar` | variable name | VarPicker (filtered by the catalog's `varKinds`) | `$name` |
| `pointRef` `toolRef` `localRef` `gridRef` `stackRef` `routineRef` `programRef` `visionRef` `deviceRef` `cameraRef` `labelRef` | `<x>Id` + `<x>Name` | repository pickers | identifier or `"quoted name"` |
| `target` (composite) | the move-target fields | point picker sub-page | §6.5 |
| `axes6` (composite) | `<prefix>X…RZ` | the 6-axis block (rx/ry hidden on 4-axis robots) | `(x=…, z=…)` |
| `mappingIn` / `mappingOut` / `rows` | lists of `{key, …}` | row editors | body lines (§6.6) |
| `steps` | ProgramStep[] | nested body | indented block |
| `json` | anything | none (edited elsewhere: CNC builder) | `@with {…}` |

**Coverage tests** (controller, package A) — the "test every tool" pattern:

1. Every `StepType` except `Unknown` has exactly one catalog entry.
2. Every catalog `key` resolves: a `ProgramStep` property, or a composite that lists its properties.
3. Every `ProgramStep` JSON property is claimed by at least one step's fields, or by the common set
   (`id name comment type enabled expressions`), or by an explicit `legacyFields` list
   (`varPointName`, `varPointIndex`, `jsonImageOutbound`, `cncProgramSteps`).
4. Every `exprKey` the executor reads with `EvalField` / `OptionalField` appears in the catalog. This
   is a grep-based test over `Execution/**/*.cs` for the string literal keys.
5. Every `errorCodes` code used by `ctx.Fail` exists in the catalog.

The app's `stepUtils` keeps **presentation only**: icon and colour per type, and the category tint.
Labels, descriptions, defaults, field metadata and summaries come from the catalog. A controller
without `GetStepCatalog` (older build) falls back to the app's built-in tables, so do not delete
those until the controller has shipped.

---

## 6. The text language (SRS)

> **Built (package D).** The user reference is [program-text.md](program-text.md); differences
> from this section are listed under [Package D — as built](#package-d--as-built). This section
> stays as the design record.

### 6.1 Name

Proposed: **Simple Robot Script (SRS)**, with file extension **`.srs`**.

- It ties to the brand (SimpleRobotController / SimpleRobotApp).
- It is short to say in videos.
- It avoids `.step` (a CAD format) and `.prg` (SPEL's own).

Alternatives are listed in §9.

### 6.2 Design principles

1. **1:1 with blocks.** Every statement is exactly one step; every block body is an indented block
   ending in `end`. Switching views never loses information. Anything without sugar falls back to
   `@with {json}`, so even future or unknown fields round-trip.
2. **Same expressions.** Every expression is the existing expression language, unchanged, with
   `$vars`. The text parser only finds where an expression *ends* and hands the substring to
   `ExpressionEvaluator.TryParse`.
3. **The keyword is the block name.** Steps use their `StepType` name (`MoveL`, `SetOutput`,
   `RunVision`), so what users learn in Blocks carries over. Flow control and assignment use
   friendlier lowercase forms (`if`, `repeat`, `$x = …`), the SPEL/RAPID-style constructs everyone
   expects. Keywords are case-insensitive; the formatter prints the canonical casing.
4. **Line-oriented and phone-friendly.** One statement per line. Blocks close with `end`, not
   significant indentation: indentation is fragile when typing on a phone. The formatter re-indents
   everything canonically (gofmt-style).
5. **Names in text, ids in JSON.** Text refers to points, tools, routines, devices and so on by name.
   The controller resolves names to ids against its repositories and keeps existing ids (§6.10).

### 6.3 Lexical rules

| Element | Rule |
|---|---|
| Line | One statement. A line continues while a `(` `[` or `{` is open, or after a trailing `,`. |
| Comment | `#` to end of line, outside strings. (`#` is not valid in expressions, so there is no clash.) |
| Pragma | First line `#!srs 1` (language version). The formatter emits it; the parser accepts it missing. |
| Strings | `"…"`, with escapes `\"` `\\` `\n` `\t`. Template braces `{…}` are kept verbatim (interpolated at run time as today). |
| Numbers | as in expressions (`3`, `-2.5`, `1e3`). Literal **duration** suffixes are allowed where the field is in ms: `500ms`, `1.5s`. |
| Booleans | `on` `off` `true` `false`. |
| Identifiers | `[A-Za-z_][A-Za-z0-9_]*`. A repository name that is not an identifier is written as a string: `"Pick 1"`. |
| Variables | `$name` — in expressions, and in *write* slots (`result=$v`, `for $v in …`). |
| Positional args | single tokens only: an identifier, string, number, `on`/`off`, `$var` with optional `[…]` / `.field`, or a `( … )` parenthesised expression. Anything else goes in named args. This keeps the grammar unambiguous without commas between positionals. |
| Named args | `name=value`, comma-separated after the positionals. A value is an expression (ends at a top-level `,`, end of line or `#`), a string, `on`/`off`, an enum word, a tuple `( k=v, … )` or a list `( a, b )`. |

### 6.4 Program structure

```srs
#!srs 1
@program {"name": "Pick and place", "isRoutine": false, "isBackground": false}   # only in exported files

# ── declarations (any order, before or between statements; formatter puts them first) ──
var count = 0                 # Parts placed          ← a trailing comment becomes the description
var ok: bool = off
var label: string = "Bin {$i}"
var pts: list point
var vals: list number = (1, 2.5, 3)
var cells: list record
var photo: image
var lap: stopwatch
var startZ = $robot.z         # non-literal → valueExpression
computed area = $w * $h
computed ready: bool = $area > 100 and $stb.in1
global persistent monitor var runs = 0       # modifiers in canonical order: global persistent input monitor
param in height = 20          # routine parameter (paramMode in | out | inout)
param out found: bool = off

# ── statements ──
## Pick                       ← "##" line = Note(section)
@name "Approach"
MoveL Pick1, offset=(z=20), speed=200
...
```

- `@program` is read only when importing a file with no `base` program (§6.11). Otherwise program
  metadata (name, kind, display page) never appears in the text. It is edited in the Blocks view's
  header as today.
- Point-list literals print as `((x=1, y=2, z=3), (x=…))` and record lists as `((row=0, col=1), …)`.
  Element fields that are 0 are omitted for points only.

### 6.5 Statements — complete mapping

Canonical forms are shown. `[…]` means optional. Every named arg corresponds to one catalog
`FieldSpec.arg`, and the full per-field table comes from `GetStepCatalog`.

**Attributes and comments** (they attach to the next statement):

| Text | Field |
|---|---|
| `# text` lines directly above a statement (no blank line between) | `comment` (lines joined with `\n`) |
| a `# text` paragraph followed by a blank line, `end`, or the end of the file | a **Note** step (`noteStyle` note) |
| `## Title` | a **Note** step (`noteStyle` section) |
| `@name "Approach"` | `name` |
| `@off` | `enabled: false` |
| `@id "3f2c…"` | `id` (optional; the formatter prints it only with `includeIds`) |
| `@with {"field": value, …}` | fields without sugar, merged last (deep merge for `expressions`) |

**Move target** (MoveL, MoveJ, JumpL, JumpJ, MoveC end and `via=`; plugin `point` params use the
same rules):

| Text | Fields |
|---|---|
| `Pick1` / `"Pick 1"` (no `{` or `$`) | `pointId` + `pointName` |
| `"bin{$i}"`, `$target` | `pointNameExpr` (template naming a saved point) |
| `$pts[$i]` | `pointNameExpr` (coordinates of a Point-list element) |
| `grid Tray[$r, $c]` / `grid Tray[$n]` | `gridPoint {gridId, rowIndex, colIndex}` (+ `expressions.gridRowIndex/ColIndex`) / `{gridIndex, useGridIndex: true}` |
| `stack Bins[$i]` | `stackPoint` (+ `expressions.stackIndex`) |
| `here` | no target — current position (use with `offset=`) |

**Motion and motion settings**

```srs
MoveL Pick1
MoveJ "Pick 1", speed=50, accel=100, decel=100
MoveL here, offset=(z=20)
MoveL $pts[$i], blend=5                    # blend=on (radius from SetBlendRadius) | blend=5 | blend=$r | blend=off
MoveL Pick1, offset=(x=1, z=$h), set=(z=30), toolOffset=(z=-5), local=Fixture1
MoveL Pick1, until=$io.Nano.Load >= 2000, untilMode=record, untilPos=$hitPts, untilFlag=$hit
JumpL Place1, z=50                         # jumpZ;  zStart=40, zEnd=60 for jumpZStart / jumpZEnd
JumpJ grid Tray[$r, $c], z=$safeZ
MoveC P3, via=P2, speed=50, tolerance=0.1
ThreadMove distance=10, pitch=0.5, peck=on, peckDepth=2, peckRetract=1, exitHeight=5, reverseOut=on
RunHoming
SetJointPosition J3, 90                    # jointIndex 2; value may be an expression: SetJointPosition J1, ($enc * 0.1)
CncProgram "part.dxf", safeZ=5             # cncSpec travels in @with {"cncSpec": {…}}
SetSpeedL 200, accel=500, decel=500
SetSpeedJ 50
SetBlendRadius 5
SetSpeedFactor 50
SetTool Gripper
SetLocal Fixture1
ClearLocal
SavePosition $pts, mode=append            # mode=replace|append|insert|set, index=…, source=actual|target
SavePosition point Pick1
```

| Arg | Field(s) |
|---|---|
| `speed accel decel` | same names |
| `blend` | `blend` + `blendRadius` |
| `offset=(…)` | `offsetX…offsetRZ` |
| `set=(…)` | `overrideX…overrideRZ` |
| `toolOffset=(…)` | `toolOffsetX…toolOffsetRZ` |
| `local` | `localId` + `localName` |
| `until` / `untilMode` / `untilPos` / `untilFlag` | `moveStopCondition` / `moveStopMode` / `moveStopPositionVariableName` / `moveStopFlagVariableName` |

**I/O, load cell, aux**

```srs
SetOutput stb 1 on                         # card [device] number value
SetOutput relay 3 off, pulse=250ms, blocking=on
SetOutput io "Arduino Nano" 4 on
SetOutput stb 2 toggle
SetOutput stb 2 ($count > 3)
Wait 500ms
Wait $delay                                # waitMs expression
Wait until $stb.in1 and $ok, timeout=5s, timedOut=$late, onTimeout=error
Tare device="Arduino Nano", pin=2
ReadLoadCell $grams, device="Arduino Nano", pin=2, unit=g
StartRecording $rec, device="Arduino Nano", pin=2
StopRecording device="Arduino Nano", pin=2
ExportData csv "logs/run-{$time_ms}.csv"
  "weight" = $grams
  "series" = list $rec
  "photo"  = image $photo
end
AuxMove device="Aux", axis=0, distance=10, unit=mm, velocity=20, accel=50, absolute=on, wait=on
AuxMove device="Aux", axis=0, steps=-200, until=$stb.in2
AuxContinuous device="Aux", axis=0, velocity=-10, accel=50
AuxStop device="Aux", axis=0, decel=1000, immediate=on
AuxEnable device="Aux", axis=0, on
GripperAction close, device="Aux", axis=1, until=$io.Nano.Grip >= 300     # open | close | to 25
ConveyorAction forward, device="Aux", axis=2                              # forward | reverse | stop
RotaryIndex +1, count=3, device="Aux", axis=3
```

**Flow**

```srs
if $count > 5 and $ok
  …
elif $mode == 2
  …
else
  …
end

repeat 3 as $i            # count loop; "as $i" = forEachIndexVariableName; "repeat forever" = loopCount 0
  …
end
while $stb.in1 == 0
  …
end
for $v, $i in $vals       # forEach; "for $v in $vals", "for _, $i in $pts"
  …
end

switch $bin
case 1, 2
  …
case 3
  …
default
  …
end

try retry=2, delay=500ms
  …
catch $msg, code=$code, only=(waitTimeout, httpFailed)
  …
end

break
continue
return
exit "Done after {$count} parts"
raise "No part at slot {$i}", code=noPart
label retry
goto retry
call Pick
call Pick(height=20, count=$n), out found=$ok
pause
status "Placed {$count}"
status warning "Parts low"               # severity Warning → statusSeverity + the app's per-severity field
```

**Variables and lists**

```srs
$count = $count + 1                       # SetVariable
$label = "Bin {$i}"                       # string variable: RHS must be a string (template)
$vals.append($grams)                      # ListOp — statement-level method calls, not expressions
$vals.insert(0, $grams)
$vals.removeAt($i)
$vals.clear()
$vals[$i] = $grams                        # listAction set
$pts[$i].z = 10                           # listAction setField
$pts.append(x=$robot.x, y=$robot.y, z=5)
$pts.append()                             # append the current TCP pose
$cells.sort(by=value, desc)
$vals.reverse()
$copy.copyFrom($vals)
stopwatch start $lap                      # StopwatchControl: start | stop | reset
```

**Vision, communication, background, plugins, operator**

```srs
RunVision "Detect Parts", frame=robot, pass=$visionOk, timeout=30s
  "Top edge".angle -> $edgeAngle          # tool by unique name, else by id: t5.cells
  "Board".cells   -> $whiteCells
end
CaptureImage $photo, camera="Top"
SaveImage "shots/{$time_ms}.jpg", camera="Top"
StartBackground Conveyor
StopBackground Conveyor
WaitForBackground Conveyor
HttpRequest "http://192.168.4.10:8080/move", wait=on, timeout=5s
  send "board" = list $cells
  send "photo" = image $photo
  send "n"     = $count
  receive "move" -> $move
end
HttpReceive "checkers", timeout=60s
  receive "x" -> $x
end
prompt "Load the tray, then continue", title="Setup", buttons=("Continue", "Skip"), result=$choice
prompt "How many parts?", input=number, default=10, min=1, max=50, value=$qty, timeout=60s, onTimeout=0
plugin scale.weigh, timeout=5s
  samples = $n * 2                        # param text, unquoted (number/boolean params)
  unit = "g"                              # quoted = exact text (string/enum/point templates)
  grams -> $w
end
unknown "MoveWithForce"                   # Unknown step: the whole raw step in @with
```

### 6.6 Grammar (EBNF)

```ebnf
file        = [ pragma NL ] { line } ;
pragma      = "#!srs" ws INT ;
line        = ws ( decl | attr | stmt | comment | (* blank *) ) [ comment ] NL ;
comment     = "#" { ANY } ;

attr        = "@" ( "name" ws STRING | "off" | "id" ws STRING | "with" ws JSON | "program" ws JSON ) ;

decl        = { modifier ws } ( "var" | "computed" | "param" ws pmode ) ws ident-decl [ ":" ws type ] [ ws "=" ws init ] ;
modifier    = "global" | "persistent" | "input" | "monitor" ;
pmode       = "in" | "out" | "inout" ;
ident-decl  = IDENT ;                                   (* declared without $ *)
type        = "number" | "bool" | "string" | "image" | "stopwatch" | "list" ws elem ;
elem        = "number" | "bool" | "point" | "record" ;
init        = EXPR | STRING | tuple | list ;

stmt        = block | simple ;
block       = if | repeat | while | for | switch | try | mapping ;
if          = "if" ws COND NL body { "elif" ws COND NL body } [ "else" NL body ] "end" ;
repeat      = "repeat" ws ( "forever" | EXPR-TOKEN ) [ ws "as" ws VAR ] NL body "end" ;
while       = "while" ws COND NL body "end" ;
for         = "for" ws ( VAR | "_" ) [ "," ws VAR ] ws "in" ws VAR NL body "end" ;
switch      = "switch" ws EXPR NL { "case" ws EXPR { "," ws EXPR } NL body } [ "default" NL body ] "end" ;
try         = "try" [ ws args ] NL body [ "catch" [ ws VAR ] [ "," ws args ] NL body ] "end" ;
mapping     = keyword-stmt NL { map-line NL } "end" ;    (* only for catalog steps with body = "mapping" *)
map-line    = ( "send" ws STRING ws "=" ws ( [ "list" | "image" ] ws VAR | EXPR )
              | "receive" ws STRING ws "->" ws VAR
              | ( STRING | IDENT ) [ "." IDENT ] ws "->" ws VAR        (* RunVision / plugin outputs *)
              | ( STRING | IDENT ) ws "=" ws ( EXPR | STRING | "list" ws VAR | "image" ws VAR ) ) ;
body        = { line } ;

simple      = assign | listop | flowword | keyword-stmt ;
assign      = VAR [ "[" EXPR "]" [ "." IDENT ] ] ws "=" ws ( EXPR | STRING ) ;
listop      = VAR "." IDENT "(" [ arglist ] ")" ;
flowword    = "break" | "continue" | "return" | "pause"
            | "exit" [ ws STRING ] | "raise" ws STRING [ "," ws args ]
            | "label" ws NAME | "goto" ws NAME
            | "call" ws NAME [ "(" [ named { "," named } ] ")" ] [ "," ws "out" ws named { "," ws "out" ws named } ]
            | "status" [ ws SEVERITY ] ws STRING [ "," ws args ]
            | "stopwatch" ws ( "start" | "stop" | "reset" ) ws VAR
            | "prompt" ws STRING [ "," ws args ]
            | "plugin" ws IDENT "." IDENT [ "," ws args ]           (* followed by a mapping body *)
            | "unknown" ws STRING ;
keyword-stmt= KEYWORD { ws positional } [ [ "," ] ws args ] ;
positional  = IDENT | STRING | NUMBER [ UNIT ] | BOOL | VAR-REF | "(" EXPR ")" | target ;
target      = ( "grid" | "stack" ) ws NAME "[" EXPR [ "," EXPR ] "]" | "here" | NAME | STRING | VAR-REF ;
args        = named { "," ws named } ;
named       = IDENT "=" value ;
value       = EXPR | STRING | BOOL | NUMBER UNIT | tuple | list | target ;
tuple       = "(" IDENT "=" EXPR { "," IDENT "=" EXPR } ")" ;
list        = "(" value { "," value } ")" ;
NAME        = IDENT | STRING ;
COND        = EXPR ;                    (* split at top-level and/or into ConditionItems *)
EXPR        = (* the existing expression language; the text parser scans to the end delimiter —
                 top-level ",", NL, "#" outside strings, or a closing bracket — then calls
                 ExpressionEvaluator.TryParse on the substring *) ;
```

`KEYWORD` is any catalog `keyword` or alias. A statement whose keyword is not in the catalog →
`unknownKeyword`, with a "did you mean" from edit distance.

### 6.7 A complete example

```srs
#!srs 1
var count = 0              # Parts placed this run
var ok: bool = off
var grams = 0
var target = 25            # Fill weight (g)
var parts: list point

## Setup
SetTool Gripper
SetSpeedL 200, accel=800, decel=800
prompt "Load the tray and press Start", buttons=("Start", "Cancel"), result=$ok
if $ok == 1
  exit "Cancelled by operator"
end

## Find parts
try retry=2, delay=1s
  RunVision "Find parts", frame=robot, pass=$ok
    "Parts".points -> $parts
  end
  if not $ok
    raise "No parts found", code=noParts
  end
catch $msg
  status error "Vision failed: {$msg}"
  exit
end

## Pick and place
for _, $i in $parts
  JumpL $parts[$i], z=40
  GripperAction close, until=$io.Nano.Grip >= 300
  JumpL grid Tray[$count], z=40
  GripperAction open
  ReadLoadCell $grams, device="Scale Nano", pin=2
  if $grams < $target
    status warning "Slot {$count} light: {round($grams, 1)} g"
    continue
  end
  $count = $count + 1
end
status "Placed {$count} parts"
```

### 6.8 Where parsing lives: on the controller

**Decision: one C# implementation in `RobotControl/Language/`**, exposed as WebSocket commands. The
app ships only a tokenizer for colouring.

Why:

1. **Resolution needs the controller.** Turning `Pick1` into a `pointId`, `"Top edge"` into a vision
   tool id, `"Arduino Nano"` into a device id, or a routine name into a `routineId` needs the
   repositories. Only the controller has all of them; the app fetches some of them.
2. **Expressions are parsed by `ExpressionEvaluator`, and the app has no expression parser.** A TS
   parser would either skip checking expressions or need a second expression parser. That is the
   drift problem the golden files already exist to fight.
3. **Testing.** xUnit is already in the controller, with fixtures. The app has no unit-test runner,
   only ad-hoc `scripts/*.mts` checks.
4. **Other clients come for free:** `mcp/robot_mcp.py` (Claude can then write programs as text), the
   live-test harness, a future website "copy this program" feature, and Discord snippets.
5. **The builder already needs a live controller.** Programs live there, and validation, symbols and
   live values are controller round-trips today. Text mode adds a debounced parse (about 300 ms),
   the same as validation.

Cost: text editing needs a connection. Offline, the Text view shows the last formatted text read-only
with a banner, the same as other controller-backed editors.

Rejected:

- **A TS parser in the app plus C# validation.** It needs two grammars, and resolution in the app.
- **A WASM build of the C# parser.** It is heavy for React Native and adds build complexity for no
  gain while the editor is online-only anyway.

### 6.9 Round-trip contract

Commands (package A registers stubs, package D implements):

```jsonc
FormatProgramText { program: BuiltProgram, includeIds?: false }
→ { text, languageVersion, map: [ { stepId, stepPath, line, endLine } ] }

ParseProgramText  { text, base?: BuiltProgram, validate?: true }
→ { ok,                                   // false when any error-severity *parse* problem exists
    program?: BuiltProgram,               // base's metadata + parsed variables/steps (ids reconciled)
    problems: [ TextProblem ],            // parse problems + (validate) ValidateBuiltProgram problems mapped to lines
    map: [ { stepId, stepPath, line, endLine } ] }

TextProblem = { line, col, endLine, endCol, severity: "error" | "warning", code, message,
                stepId?, field?, source: "parse" | "validate" }
```

- Lines and columns are **1-based**. Columns count UTF-16 code units, to match JS string indexing.
- An expression error's position is the `ExpressionParseException.Position` inside the expression,
  plus the column where that substring starts.
- Validator problems map through `map` (`stepId` → lines), and through `field` → the catalog `arg` →
  that argument's span, when it is present.

**The guarantee, tested by package D:**

```
Normalize(Parse(Format(P), base: P).program) == Normalize(P)       // JSON deep-equal, for every fixture P
Format(Parse(Format(P)).program) == Format(P)                      // idempotent formatter
```

`Normalize` holds the only allowed differences. Each one is a canonicalisation that does not change
behaviour:

1. Legacy `varPointName` / `varPointIndex` → `pointNameExpr: "$name[index]"`.
2. Condition items:
   - A legacy `left`/`operator`/`right` row → `expr`. String operators become
     `$l.contains("r")` and similar.
   - An `expr` with a top-level `and`/`or` → split into items.
   - Each item after the first gets an explicit `join`, and `combinator` becomes `"ALL"`.
3. Empty lists and empty dictionaries → absent (null).
4. A literal shadowed by an `expressions[key]` entry is kept from the base step when the ids
   reconcile (below); otherwise it is null.
5. `pointName` / `toolName` / `localName` / … are refreshed from the repository for the id. Renamed
   things print their new name.
6. A `valueExpression` that is a plain number literal → `value`.
7. `statusWarning` / `statusError` / `statusMessage` are canonicalised per severity, as the app writes
   them.
8. `jsonImageOutbound` rows → `jsonOutbound` rows with `imageVar`. **Verify first that the
   controller treats them identically** (`HttpSteps.cs`). If it does not, print them via `@with`
   instead.

### 6.10 Id reconciliation (`Language/IdReconciler.cs`)

Ids matter because:

- revisions diff on JSON;
- the monitor and validation key on `stepId`;
- labels link by `labelId`;
- breakpoints are step ids;
- UI state (selection, expansion) is keyed by id.

Text has no ids unless `@id` is present. So `ParseProgramText` reconciles the parsed tree against
`base`:

1. Work per step list pair: the old list and the new list, at the same role and path.
2. The key of a step is `type | primaryRef`, where `primaryRef` is the first of:
   - `pointId`
   - `pointNameExpr`
   - `gridId` / `stackId` + index text
   - `routineId`
   - `variableName`
   - `labelName`
   - `visionProgramId`
   - `pluginId.pluginStepId`
   - `backgroundProgramId`
   - otherwise `""`
3. Take the longest common subsequence on keys. Each match is an anchor.
4. Between anchors, pair the leftover old and new steps of the same `type` in order.
5. A matched new step takes the old `id`. It also inherits the shadowed literals (Normalize rule 4)
   and the ids of its condition items, else-if branches and switch cases, by position. Then recurse
   into child lists by role, using `StepTree.ChildLists`.
6. `@id "…"` in the text always wins. A duplicate id → `duplicateId` (error).
7. Unmatched steps get `Guid.NewGuid().ToString()`. The app uses `crypto.randomUUID()`, so the
   format is the same.
8. **Variables** match by name. **Labels:** a Label keeps the `labelId` of its matched step. A
   `goto NAME` resolves to the `labelId` of the Label named NAME in the same program. An unknown name
   → `unknownLabel`. Duplicate label names are already a validator error.

### 6.11 Problem codes (parse)

| Code | Meaning |
|---|---|
| `syntax` | anything not matched by the grammar (with an "expected …" message) |
| `unknownKeyword` | a statement keyword not in the catalog (did you mean …) |
| `unknownArgument` | a named arg the step does not have |
| `duplicateArgument` | the same arg given twice |
| `missingArgument` | a required positional or arg is missing |
| `badValue` | wrong value kind: a non-literal in a literal-only field, or an enum word not in the options |
| `expressionSyntax`, `unknownFunction`, `badArity` | from `TryParse`, mapped to line and column |
| `missingEnd`, `unexpectedEnd`, `misplacedClause` | block structure: `elif`/`else`/`case`/`catch` in the wrong place |
| `declarationKind` | e.g. `computed` with `persistent` (this mirrors `computedKindConflict`) |
| `stringRequired` | assigning an expression to a string variable, or a string to a number variable |
| `duplicateId` | two steps carry the same `@id` |
| `unknownPoint`, `unknownTool`, `unknownRoutine`, … | **warnings** at parse time. The name is kept, the id is null, and the validator reports them as it does for blocks today. This lets a user type a program before teaching its points. |

Error recovery is line-based: report the problem, skip to the next line, and keep the block stack. A
stray `end` pops one level; a missing `end` is reported at EOF against the opener's line.

### 6.12 Importing and exporting files

- **Export.** The program list gets "Export as text (.srs)": `FormatProgramText` plus the
  `@program` header, shared through the existing share and download path.
- **Import.** "Import text": `ParseProgramText` with no `base`, then a name and kind dialog
  pre-filled from `@program`, then `SaveBuiltProgram`. Names that do not resolve stay as names (they
  show as warnings), so a program shared between robots imports and then needs its points taught.
- **MCP.** `robot_mcp.py` gets the tools `robot_program_text_get(name)` and
  `robot_program_text_set(name, text, validate=true)`.

---

## 7. Editor UX

### 7.1 Switching views

- `BuiltProgram.editorMode?: "blocks" | "text"` is owned by the app, like `displayPageMode`. It is
  the program's preferred view and is remembered per program. There is also an app setting, "Default
  editor for new programs".
- The `EditorToolbar` gets a **Blocks | Text** segmented control.
  - **Blocks → Text:** call `FormatProgramText` on the current *unsaved* document.
  - **Text → Blocks:** call `ParseProgramText` with `base` = the document. If `ok`, commit the parsed
    program as **one undoable edit** to the `EditorDoc`, so undo works across the switch. If not, a
    sheet says "Fix N errors to switch, or discard text changes."
- **Saving in Text mode** parses, then commits, then saves, with the normal revision. Parse errors
  block saving. Validation errors do not; they block Run, as today.
- Text is regenerated from the document whenever the user enters Text view. While in Text view, the
  text buffer is the source, and the document is updated on each successful debounced parse. So
  validation, live symbols and Run all see the edit.

### 7.2 The code editor component (`src/components/ui/code-editor/`)

**Engine.** CodeMirror 6, bundled locally because robots can be offline. React Native has no code
editor.

- **Native:** CodeMirror runs inside a `react-native-webview`, the same split already used by
  `PluginUiView.tsx` / `.web.tsx` and `DisplayPageView`.
- **Web and Electron:** `CodeEditor.web.tsx` mounts CodeMirror directly in a div.

The bridge carries:

| Direction | Messages |
|---|---|
| app → editor | `setText`, `setDiagnostics`, `setHighlightLines`, `setCompletions`, `focus`, `insertSnippet` |
| editor → app | `change {text}`, `cursor {line, col}`, `requestCompletions {line, col, prefix}` |

Features:

- **Syntax colouring** by a small TS tokenizer, `srsTokens.ts`. It colours keywords (from the
  catalog), `$vars`, strings and templates, numbers with units, comments, `@attrs`, and repository
  names (unresolved names underlined once the parse result comes back). The same token list feeds a
  CodeMirror `StreamLanguage`.
- **Diagnostics** come from `ParseProgramText`: squiggles, gutter markers and a tap-to-read tooltip.
  The existing `ValidationPanel` lists them with line numbers, and tapping one jumps to it.
- **Completion.**
  - After `$`: reuse `completions.ts` with `GetExpressionSymbols`.
  - At the start of a line: catalog keywords, as snippets with placeholders. For example `MoveL`
    expands to `MoveL ${target}, speed=${…}`.
  - After `MoveL `: point names.
  - After `,`: the step's remaining named args, with units in the detail.
  - After `call `: routines.
- **Hover / long-press help:** the catalog's `syntax` + `help` for the keyword or arg under the
  cursor.
- **Mobile key row** above the keyboard: `$ ( ) [ ] = , " # ⇥`, plus "Insert step…". The latter
  opens the existing `StepTypePicker` and inserts that step's canonical text, from
  `FormatProgramText` on `newStep(type)`.
- **Keyboard visibility.** Per the feedback rule, the line being typed must stay visible above the
  keyboard. The WebView height follows `src/platform/keyboard.ts`, and CodeMirror scrolls the cursor
  into view. Add a `keyboard-audit` scenario for the text editor.
- **Format** button. Comment / uncomment a selection: `#` toggles; for steps prefer `@off`.
- **Run highlight.** While a program runs, the monitor status `stepId` is mapped to a line through
  `map`, and the lines are tinted. A paused-at-breakpoint step gets a stronger tint. Breakpoints can
  be toggled in the gutter; they are the same step-id breakpoints as in Blocks.

### 7.3 Monitor and runtime (app package G)

- **`PromptDialog`** is mounted app-wide, in the root layout, so a prompt is answered from any
  screen. It shows the program name, title, message, buttons and input, and a countdown when there is
  a timeout. It sends `AnswerPrompt`. It closes when the status no longer carries the prompt, for
  example because another device answered it.
- **Debug controls** on the monitor: Continue, Step and Stop. A pause reason chip shows breakpoint,
  step, pause step or user. Run from step is reachable from the builder.
- **Breakpoints** are held per program in `useBreakpoints(programId)`, created in package A: memory
  plus local storage. They are passed with `ExecuteBuiltProgram` and pushed live with
  `SetBreakpoints`.

---

## 8. Work packages

Rules for every package:

- **Commits.** Edit files directly in the main repo folders: no worktrees. Branch `feat/plugins` in
  both repos. Do **not** commit or open PRs unless asked. Follow the emoji conventional-commit style.
- **Never use the running controller on :9000.** For live tests, start a private instance:

  ```
  dotnet run --project RobotControl -- --port 9100 --data <scratch dir>
  CTRL=localhost:9100 node live-tests/<suite>.mjs
  ```

  Stop it by PID afterwards. Stop any Expo/Metro dev servers by PID too.
- **Model changes.** After package A, only package A's owner (or the coordinator) changes
  `ProgramSteps.cs`, `robotModels.tsx` or the goldens. A package that finds it needs a field asks
  for it; it does not add it.
- **Shared Markdown.** Each package edits only its own section of `program-blocks.md`. Package A
  creates the headings. Use small, targeted edits, re-reading first, because several agents work in
  the same working copy.

### Package A — contracts and scaffolding (first, alone, about ½ day)

**Controller**

- `RobotControl/Models/ProgramSteps.cs`:
  - new `StepType` values: `Note`, `LoopControl`, `Return`, `EndProgram`, `RaiseError`, `Switch`,
    `Try`, `SavePosition`, `SetSpeedFactor`, `ListOp`, `Prompt`, `MoveC`;
  - the §4.1–4.3 fields: `enabled`, `noteStyle`, `loopControl`, `errorMessage`, `errorCode`,
    `switchExpr`, `switchCases`, `switchDefault`, `trySteps`, `catchSteps`, `tryRetries`,
    `tryRetryDelayMs`, `catchCodes`, `errorVariableName`, `errorCodeVariableName`,
    `waitTimeoutAction`, `routineArgs`, `routineOutputs`, `outputAction`, `savePosition*`,
    `speedFactor`, `list*`, `prompt*`, `viaPoint*`, `arcTolerance`;
  - the `SwitchCase` class;
  - `ChildLists()` in a new `Models/StepTree.cs`.
- `Models/ProgramVariables.cs`: `paramMode`. `Models/BuiltProgram.cs` (and `SaveBuiltProgramParams`):
  `editorMode`.
- `Models/ProgramCycle.cs` (status DTO): `pauseReason`, `pausedAtStepId`, and `prompt`
  (`ProgramPromptInfo`).
- `RobotControl/Programs/StepCatalog.cs` (new), with entries for **all** step types, old and new, and
  the error code list. `Commands/StepCatalogCommands.cs` (new) implements `GetStepCatalog`.
- **Stubs**, registered in `Commands/CommandDispatcher.cs`:
  - `Commands/LanguageCommands.cs` (`FormatProgramText`, `ParseProgramText` → `notImplemented`);
  - `Commands/OperatorCommands.cs` (`AnswerPrompt`);
  - `Commands/DebugCommands.cs` (`StepBuiltProgram`, `SetBreakpoints`).
- `Execution/ExecutionContext.cs`: `Fail(string code, string message)`, which for now just calls
  `Finish(Error, message)`. Package B makes it unwind to a Try.
- **Handler stubs** registered in `ExecutionContext.StepHandlers`: new empty files
  `Execution/Steps/FlowExtSteps.cs` (package B) and `MotionExtSteps.cs`, `DataSteps.cs`,
  `OperatorSteps.cs` (package C). Each new type's handler errors with "not implemented yet".
- `Validation/ProgramValidator.cs`:
  - make `ProgramValidator` and `Walker` `partial`;
  - walk child lists through `StepTree`;
  - declare `partial void CheckFlowExt(ProgramStep s, At at, …)` and `partial void
    CheckStepsExt(…)`;
  - create empty `ProgramValidator.Flow.cs` (package B) and `ProgramValidator.Steps.cs` (package C).
- `Execution/ProgressReporter.cs`: `CountSteps` goes through `StepTree` (Notes not counted), and
  `StepDescription` gets cases for the new types.
- **Goldens:** add every new field and the missing old ones to
  `RobotControl.Tests/Golden/programstep.golden.json`; add `steptypes.golden.json`.
- **Tests:** `GoldenProgramStepTests` asserts; new `StepCatalogTests` (the 5 coverage tests in
  §5.2) and `StepTreeTests`.
- **Docs:** headings in `program-blocks.md` for each new block.

**App**

- `src/models/robotModels.tsx`: the types (StepType union, ProgramStep fields, SwitchCase,
  `paramMode`, `editorMode`, status `prompt` / `pauseReason`, `StepCatalog` / `FieldSpec` /
  `TextProblem` types).
- `src/models/programStepGolden.check.ts`: mirror the golden; add the `STEP_TYPES_ALL` const and the
  type-equality check.
- `src/models/stepTree.ts` (new).
- `src/services/RobotConnectService.tsx`: client methods `getStepCatalog`, `formatProgramText`,
  `parseProgramText`, `answerPrompt`, `stepBuiltProgram`, `setBreakpoints`, and
  `executeBuiltProgram(name, {startAtStepId, breakpoints, stepMode})`.
- `src/components/ui/builder/useStepCatalog.ts` (new): fetch once per connection, with a fallback.
- `src/components/ui/builder/useBreakpoints.ts` (new, complete).
- `src/components/ui/builder/stepUtils.tsx`: minimal entries for the new types (icon, theme, label,
  category) so the picker lists them.
- `app/(tabs)/program/builder.tsx`: `defaultStep` entries for the new types, `editorMode` state, and
  the toolbar toggle wired to mount `<TextProgramView>`.
- `src/components/ui/builder/TextProgramView.tsx` (new **stub**: "Text editor coming").
- `EditorToolbar.tsx`: the Blocks | Text control.

**Done when:**

- the controller builds and `dotnet test` is green;
- `npx tsc --noEmit` is clean;
- a program with each new step type saves and loads unchanged through `SaveBuiltProgram` on a private
  :9100 instance.

#### Package A — as built (the frozen contracts; read this before B–G)

Where the build differs from or adds to the text above, **this list wins**.

**Model** (`Models/ProgramSteps.cs`, mirrored in the app's `robotModels.tsx`; both goldens list every
field):

- New `StepType` values are appended **after `Unknown`** (existing ordinals unchanged): `Note`,
  `LoopControl`, `Return`, `EndProgram`, `RaiseError`, `Switch`, `Try`, `SavePosition`,
  `SetSpeedFactor`, `ListOp`, `Prompt`, `MoveC`.
- Fields exactly as §4.1–4.3: `enabled`, `noteStyle`, `loopControl`, `errorMessage`, `errorCode`,
  `switchExpr`, `switchCases` (`SwitchCase {id, values: string[], steps}`), `switchDefault`,
  `trySteps`, `catchSteps`, `tryRetries`, `tryRetryDelayMs`, `catchCodes`, `errorVariableName`,
  `errorCodeVariableName`, `waitTimeoutAction`, `routineArgs` (`{param: expr}`), `routineOutputs`
  (`[{key, variableName}]` — C# reuses `PluginOutputMapping`, TS `RoutineOutputMapping`),
  `outputAction`, `savePositionTo`, `savePositionMode`, `savePositionIndex` (int literal; expression
  key `savePositionIndex`), `savePositionSource`, `speedFactor`, `viaPointId`, `viaPointName`,
  `viaPointNameExpr`, `arcTolerance`, `listAction`, `listIndex` (int; key `listIndex`),
  `listValueExpr`, `listFields`, `listSourceVariable`, `listSortField`, `listSortDescending`,
  `promptTitle`, `promptMessage`, `promptButtons`, `promptInput`, `promptDefault` (string),
  `promptMin`, `promptMax`, `promptResultVariableName`, `promptInputVariableName`, `promptTimeoutMs`,
  `promptTimeoutButton`. EndProgram's message is `statusMessage`; SavePosition reuses
  `variableName` / `pointId` / `pointName`; ListOp reuses `variableName`.
- `ProgramStep.IsEnabled` (C#, `[JsonIgnore]`) = `enabled != false`.
- `ProgramVariable.paramMode`: `"in" | "out" | "inOut"` (`ParamModes` constants). The text form
  `param inout` maps to `"inOut"`.
- `BuiltProgram.editorMode`: `"blocks" | "text"`. `SaveBuiltProgram` takes it; a save that omits it
  keeps the stored value.
- Goldens: `RobotControl.Tests/Golden/programstep.golden.json` (now every field, both directions:
  `EveryControllerFieldIsInTheGolden`), `steptypes.golden.json` (the enum, in order) and
  **`builtprogram.golden.json`** (program + variable fields, incl. `editorMode` / `paramMode`).
  App mirror: `src/models/programStepGolden.check.ts` + `npx tsx scripts/golden-check.mts`.

**`StepTree`** (`Models/StepTree.cs`): `step.ChildLists(executableOnly = false)` →
`StepChildList(Role, Index?, Steps)` with `Path` (`loopSteps`, `elseIfBranches[1].steps`,
`switchCases[0].steps`, …) and `IsLoopBody`; roles in `StepTree.Roles` (order frozen: loopSteps,
ifSteps, elseIfBranches, elseSteps, switchCases, switchDefault, trySteps, catchSteps,
cncProgramSteps). `executableOnly` drops baked `cncProgramSteps` when `cncSpec` is set.
`StepTree.Walk(steps, enabledOnly, executableOnly)` (pre-order) and `StepTree.WalkWithPaths(steps)`.
Used by CountSteps (both), the validator and the step-array list of `ValidateBuiltProgram`'s
unknown-type patcher.

**Catalog** (`RobotControl/Programs/`, namespace `Controller.RobotControl.Programs`):
`StepCatalog.cs` (the table), `StepCatalogModels.cs` (wire classes), `ErrorCodes.cs`.
`GetStepCatalog {}` → `{ catalogVersion: 1, languageVersion: 1, categories, errorCodes, steps,
commonFields, legacyFields }`. Differences from §5.2:

- `commonFields` = `id name comment type enabled expressions unknownStepType`; `legacyFields` =
  `varPointName varPointIndex jsonImageOutbound cncProgramSteps`. `Unknown` has no entry.
- Category keys are the app's: `motion motionSettings flow variables io vision background comms
  plugins`. New blocks: Note/LoopControl/Return/EndProgram/RaiseError/Switch/Try → `flow`;
  SavePosition/MoveC → `motion`; SetSpeedFactor → `motionSettings`; ListOp → `variables`;
  Prompt → `comms`.
- `FieldSpec` gains `props` (the ProgramStep properties a composite or reference field stores
  into, e.g. `target` → `pointId pointName pointNameExpr gridPoint stackPoint`; `localId` →
  `localId localName`), `exprKeys` (a composite's expression keys: target →
  `gridRowIndex gridColIndex gridGridIndex stackIndex`; axes6 → its six fields), `varKinds`
  (`number boolean string image stopwatch list` or `list:<Number|Boolean|Point|Record>`) and
  `hidden`. Optional metadata that is null is omitted from the JSON.
- Composite keys: `target`, `via` (MoveC, props `viaPoint*`), `offset`, `override` (arg `set`),
  `toolOffset`. **No `until` composite:** the stop condition is four real fields with args `until`,
  `untilMode`, `untilPos`, `untilFlag`. `blend` (bool) and `blendRadius` are two fields; the
  `blend=5` text sugar is package D's.
- Kinds (`FieldKinds.All`): §5.2's list **plus** `pluginRef` (pluginId + pluginStepId), `exprMap`
  (`{name: expr}` — routineArgs, listFields, pluginParams), `textList` (promptButtons, catchCodes),
  `branches` (elseIfBranches), `cases` (switchCases). **`mappingIn` is not used**: rows that send
  values out are `rows` (jsonOutbound, exportColumns); rows that read a value into a variable are
  `mappingOut` (jsonInbound, httpReceiveInbound, toolOutputs, pluginOutputs, routineOutputs).
- `showIf {field, in}` compares the other field's value **as text**, an absent field reading as its
  catalog `default` (booleans `"true"`/`"false"`).
- `keyword`: step types keep their name; flow forms use the text words (`if switch repeat break
  try raise call return exit label goto pause stopwatch status prompt plugin`, Loop aliases
  `for while`, LoopControl alias `continue`). SetVariable and ListOp have keyword `$`, Note `#` —
  statement forms, not keywords.
- `background` = `!ProgramExecutor.IsRestrictedInBackground(type)` (now `internal`; MoveC and
  SetSpeedFactor added to it).
- Error codes (`ErrorCodes`, `ErrorCodes.IsCatchable(code)` — a code not in the list, i.e. a
  RaiseError code of the user's own, is catchable): catchable `waitTimeout visionTimeout
  visionFailed httpFailed httpTimeout receiveTimeout pluginFailed pluginTimeout pluginNotRunning
  promptTimeout pointNotFound arcDegenerate indexOutOfRange userError`; fatal `motionFault
  jointLimit driverDisconnected emergencyStop userStop unknownVariable expressionSyntax
  computedVariable`. A new code goes in `ErrorCodes` (test 5 greps `ctx.Fail("…"` and
  `ctx.Fail(ErrorCodes.X`).
- Tests: `StepCatalogTests` (the five coverage tests + shape, keyword, wire-name checks).

**Execution:** `ExecutionContext.Fail(code, message)` → `Finish(Error, message)` for now. New
handler files `FlowExtSteps.cs` (B; **Note is implemented**: it advances without reporting a
completion, so it is never counted), `MotionExtSteps.cs`, `DataSteps.cs`, `OperatorSteps.cs` (C);
every other new type is `NotImplementedStep.Instance` (ends the run: "… not implemented on this
controller yet"). **Enable/disable is done end to end**: `ProgramExecutor.ExecuteStepInner` skips a
disabled step (`Progress.SkippedDisabled` + `CompleteNow`; a disabled Note just advances), a blended
MoveL run stops at a disabled step, CountSteps counts a disabled step once without its body and
skips Notes; tests in `DisabledStepTests.cs` (B may extend it).

**Validator:** `ProgramValidator` and `Walker` are `partial`; `ValidationCodes` has the new names
(`loopControlOutsideLoop unknownParam paramMissing paramOutputType duplicateCaseValue
unsupportedOnStep pointWriteInBackground`). Hooks, called after the per-type switch and before the
nested bodies are walked (bodies of every new block are already walked via StepTree):

```csharp
partial void CheckFlowExt(ProgramStep s, string sp, At at, List<List<ProgramStep>> chain, Body body); // B: ProgramValidator.Flow.cs
partial void CheckStepsExt(ProgramStep s, At at);                                                      // C: ProgramValidator.Steps.cs
```

A disabled step gets one `disabledStep` warning (field `enabled`) and nothing inside it is checked;
disabled steps declare no implicit variables; a disabled GoToLabel does not make the rest
unreachable.

**Status DTO** (`Models/ProgramCycle.cs`): the programs summary gains `currentStepId`,
`pauseReason` (`PauseReasons`: `breakpoint | step | pauseStep | user`), `pausedAtStepId` and
`prompt` (`ProgramPromptInfo {id, stepId, programName, title, message, buttons, input, default,
min, max, timeoutMs, shownUnixMs}`); the live `activePrograms` block gains `currentStepId` and
`prompt`. `currentStepId` is new (for the run highlight in Blocks and Text). The PauseProgram step
already reports `pauseStep` and a user Stop `user`. **`ProgramCycleManager` now has
`SetPauseState(name, reason, pausedAtStepId)`, `SetPrompt(name, prompt|null)`, `GetPrompt(name)` and
`GetPrompts()`** — B and C call these instead of editing ProgramCycleManager. A terminal status
clears pause + prompt; Resume clears the pause but keeps the prompt.

**Commands** (registered in `CommandDispatcher`): `GetStepCatalog` (real); `FormatProgramText`,
`ParseProgramText` (`Commands/LanguageCommands.cs`), `AnswerPrompt` (`OperatorCommands.cs`),
`StepBuiltProgram`, `SetBreakpoints` (`DebugCommands.cs`) are stubs that fail with
`{ok: false, error: "notImplemented: …"}` (`CommandNotImplementedException`). Param classes in
`Models/BuiltProgram.cs`: `ExecuteBuiltProgramParams {name, startAtStepId?, breakpoints?, stepMode?}`,
`SetBreakpointsParams {stepIds}`, `AnswerPromptParams {promptId, button, value?}` (value as text),
`FormatProgramTextParams`, `ParseProgramTextParams`. **`ExecuteBuiltProgram` refuses any debug
option** with a `notImplemented` error until package B wires them (running from the top instead
of mid-program would move the robot unexpectedly).

**Text contract** (`Language/TextProblem.cs`, now package D's): `TextProblem {line, col, endLine,
endCol, severity, code, message, stepId?, field?, source}` — 1-based, UTF-16 columns, `endCol`
exclusive; `ProgramTextMapEntry {stepId, stepPath, line, endLine}` (inclusive lines);
`FormatProgramTextResult {text, languageVersion, map}`; `ParseProgramTextResult {ok, program?,
problems, map}`; `TextProblemCodes` (§6.11).

**App** (SimpleRobotApp):

- `src/models/robotModels.tsx`: the StepType union (new members appended, same order as the
  controller), every new ProgramStep field, `SwitchCase`, `RoutineOutputMapping`, enum types
  (`NoteStyle`, `ListAction`, `PromptInput`, …), `ParamMode`, `EditorMode`, `isStepEnabled(step)`,
  `ProgramSummary.currentStepId / pauseReason / pausedAtStepId / prompt` (`ProgramPrompt`,
  `PauseReason`), `ExecuteOptions`, `StepCatalog` / `StepSpec` / `FieldSpec` / `FieldKind` /
  `StepCategoryKey`, `TextProblem`, `ProgramTextMapEntry`, `FormatProgramTextResult`,
  `ParseProgramTextResult`, and the new `KnownValidationCode` names.
- `src/models/programStepGolden.check.ts`: `goldenProgramStep` (all fields), `STEP_TYPES_ALL`
  (with a both-ways type-equality check against `StepType`) and `goldenBuiltProgram`.
  **`npx tsx scripts/golden-check.mts`** compares them with the controller's golden files key by
  key, both directions, and checks `stepTree` against the golden step's bodies and paths.
- `src/models/stepTree.ts`: `STEP_LIST_ROLES`, `childLists(step, executableOnly?)` (with
  `role`, `index`, `branchId`, `path`), `walkSteps(steps, {enabledOnly?, root?})` (with `path`
  and `depth`), `flattenSteps`, `findStep`, `mapStepTree(step, fn)` (immutable deep map). Already
  used by paste (fresh ids incl. switch cases), `rehydrateIds` and `backfillPointIds`;
  `stepClipboard.ts`, `renameVariable.ts`, validation indexing and scope navigation (`ScopeFrame`
  has no switch/try kinds yet) are package E's to move over.
- `RobotConnectService`: `getStepCatalog()`, `formatProgramText(program, {includeIds})`,
  `parseProgramText(text, {base, validate})`, `answerPrompt(promptId, button, value?)`,
  `stepBuiltProgram()`, `setBreakpoints(stepIds)`, `executeBuiltProgram(name, options?)` and
  `executeBuiltProgramDebug(name, options)` (rejects with the typed error). A `notImplemented: …`
  answer becomes **`NotImplementedCommandError`** (`isNotImplementedCommand(e)`; a subclass of
  `CommandFailedError`); an older controller still gives `UnsupportedCommandError`.
  `SaveBuiltProgram` / `ValidateBuiltProgram` now carry `editorMode`.
- `src/components/ui/builder/useStepCatalog.ts`: module-level store, `loadStepCatalog()`,
  `stepSpecFor(type)`, `fieldSpecFor(type, key)`, `useStepCatalog(connected?)` (fetch on mount
  and on reconnect; `status: "unsupported"` on an older controller → keep using stepUtils).
  The builder already calls `useStepCatalog(connected)`.
- `src/components/ui/builder/useBreakpoints.ts` (complete): `useBreakpoints(programId)` →
  `{ids, list, has, toggle, set, clear, retain}`, persisted per program id in AsyncStorage
  (`rms.breakpoints.<id>`); plain `getBreakpoints(id)` / `setBreakpointsFor(id, ids)` for non-React
  callers (the monitor).
- `stepUtils.tsx`: icon, theme, label and picker entry for each new type; Switch, Try, Break/Next,
  Raise Error, Return, End Program and Note are in Logic & Flow; Move Arc and Save Position in
  Motion; Set Speed Factor in Motion Settings; List in Variables & Timing; Prompt in
  Communication. `BACKGROUND_RESTRICTED` gains MoveC and SetSpeedFactor.
- `builder.tsx`: `newBlockDefaults(type)` (catalog defaults: Switch starts with one case `["1"]`,
  Try with empty bodies and 0 retries, Prompt with `["OK"]`, …); `editorMode` is builder state
  (not in the undo document — a view switch is not an edit), loaded from and saved with the
  program; `changeEditorMode(mode)` leaves block scope and selection; in Text mode the steps
  section is `<TextProgramView>` with the props below; `applyTextProgram(p)` commits steps +
  variables as one undoable edit. Select bar has **Disable / Enable**; `StepRow` dims a disabled
  step and labels it "· DISABLED". StepConfigModal has no fields for the new types yet (E).
- `EditorToolbar.tsx`: optional `editorMode` / `onEditorModeChange` props render the Blocks | Text
  `EditorModeToggle` before undo/redo.
- `TextProgramView.tsx` (stub, package F fills it): props `{program, onApply, onSwitchToBlocks,
  connected, problems?, breakpoints?, runningStepId?}` — `program` is the current unsaved document.

### Package B — controller flow, errors, routines and debugging (after A)

| Owns | Files |
|---|---|
| Executor core | `RobotControl/ProgramExecutor.cs`, `Execution/StepListFrame.cs` (FrameKind `Routine`, `Try`, `Switch`), `Execution/ExecutionContext.cs` (`Fail`, Try unwinding, debug state) |
| Handlers | `Execution/Steps/ControlFlowSteps.cs` (CallRoutine → Routine frame + params; GoToLabel stops at the routine boundary), `Execution/Steps/FlowExtSteps.cs` (LoopControl, Return, EndProgram, RaiseError, Switch, Try, Note) |
| Fail conversion | **all existing** `Execution/Steps/*.cs` that end in runtime errors (`IoSteps` Wait timeout + SetOutput toggle/expr, `VisionSteps`, `HttpSteps`, `PluginSteps`, `MotionSteps` pointNotFound, `AuxSteps`) |
| Scope | `Execution/VariableScope.cs` (param in/out copy) |
| Validation | `Validation/ProgramValidator.Flow.cs` (loopControlOutsideLoop, unknownParam, paramMissing, paramOutputType, duplicateCaseValue, emptyBranch for cases/try, disabledStep) |
| Commands | `Commands/BuiltProgramCommands.cs` (ExecuteBuiltProgram options), `Commands/DebugCommands.cs` |
| Tests | `RobotControl.Tests/FlowControlTests.cs`, `TryCatchTests.cs`, `RoutineParamTests.cs`, `DebuggerTests.cs`, `DisabledStepTests.cs`; `live-tests/flow.mjs` |
| Docs | the matching `program-blocks.md` sections; §1 of `expressions-and-variables.md` (enabled is now real) |

Contracts used: package A's fields and status DTO, and `StepTree`. Provides `ctx.Fail(code, msg)`,
which package C's new handlers call.

> **Hand-off to package C:** package A adds `ctx.Fail(code, message)` as a one-line method that just
> calls `Finish(Error, message)`. Package C calls it from day one, and package B later replaces its
> body with Try unwinding. Neither package then needs the other's code to land first.

### Package C — controller motion, data and operator steps (after A, parallel with B)

| Owns | Files |
|---|---|
| Handlers (new files only) | `Execution/Steps/MotionExtSteps.cs` (MoveC tessellation via `MotionDispatcher.StartContinuous`, SavePosition, SetSpeedFactor), `Execution/Steps/DataSteps.cs` (ListOp), `Execution/Steps/OperatorSteps.cs` (Prompt state machine) |
| Prompt plumbing | `Commands/OperatorCommands.cs` (AnswerPrompt), `ProgramCycleManager.cs` (publish/clear prompt in status) |
| Speed factor | wherever the override is applied for program moves (the `ApplySpeedOverride` path in `RobotController.cs` / `Motion/`) — **read, then make the smallest hook**; `RobotPropertySource.cs` for `$program.speedFactor` |
| Expressions | `RobotControl/ExpressionEvaluator.cs` (`inBox`, `inCircle`, string `.length` / `.indexOf` / `.toNumber`), `ComputedPropertySource.cs` if needed |
| Validation | `Validation/ProgramValidator.Steps.cs` (arc via missing, unsupportedOnStep, pointWriteInBackground, listAction field requirements, prompt buttons 1–4) |
| Tests | `MoveCTests.cs` (tessellation geometry: chord tolerance, 3D plane, collinear), `ListOpTests.cs` (every action × element type), `PromptTests.cs`, `SavePositionTests.cs`, `SpeedFactorTests.cs`, `ExpressionFunctionTests` additions; `live-tests/steps.mjs` |
| Docs | the matching `program-blocks.md` sections; the new functions in `expressions-and-variables.md` |

### Package D — the text language on the controller (after A, parallel)

| Owns | Files |
|---|---|
| Language | `RobotControl/Language/SrsLexer.cs`, `SrsParser.cs` (recursive descent, line-based recovery), `SrsPrinter.cs` (catalog-driven + sugar), `SrsAst.cs` (internal), `IdReconciler.cs`, `ProgramNormalizer.cs`, `NameResolver.cs` (repositories → ids, warnings), `TextProblem.cs` |
| Commands | `Commands/LanguageCommands.cs` (fill the stub; `validate` maps `ValidateBuiltProgram` problems to lines) |
| Tests | `RobotControl.Tests/Language/RoundTripTests.cs` (every fixture program + the golden step + a catalog-driven random-step generator, 500 seeds), `ParserErrorTests.cs` (line and column for each code), `PrinterGoldenTests.cs` (`Fixtures/srs/*.srs` expected text), `ReconcilerTests.cs` |
| MCP | `mcp/robot_mcp.py`: `robot_program_text_get` / `robot_program_text_set` |
| Docs | move §6 of this file into `docs/program-text.md` (the user reference) once stable |

It needs only package A, because new-step syntax is catalog-driven and it does not need B or C to
execute anything. It must cover the new steps' fields from day one.

#### Package D — as built

The user reference is **[program-text.md](program-text.md)** (with the final grammar and a
catalog-generated argument table that `DocTableTests` keeps current). Where the build differs from
§6, this list wins:

- **How "every field round-trips" is guaranteed.** The parser builds each step as JSON (ProgramStep
  names). The printer writes each step's sugar, then *parses that statement back on its own* and
  diffs the result against the step; the difference is written as `@with {…}` (deep merge, `null`
  removes). A step whose sugar cannot be read back falls back to its bare keyword (every StepType
  name is a keyword: `IfCondition`, `Loop`, `SetVariable`, `ListOp`, `Note` …) plus `@with`. Bodies
  and else-if/case lines stay structural unless a branch cannot be written, in which case the whole
  `elseIfBranches` / `switchCases` array (with ids) travels in `@with`.
- **Normalize** (`ProgramNormalizer`) has the §6.9 rules plus: implied defaults written out (Loop
  `loopMode` "count", Wait `waitMode` "duration", LoopControl "break", ListOp "append", SetOutput
  `outputCard` "stb"); `noteStyle` "note" and `enabled: true` → absent; false variable flags →
  absent; a list variable's missing `elementType` → Record; legacy `values/points/objects` →
  `items`; Point elements carry all six axes; missing ids filled from names that resolve;
  StatusUpdate's missing `statusSeverity` inferred as the app does (rule 7 only infers the severity
  — moving text between `statusMessage`/`statusWarning` would change the run, so it does not); old
  editor leftovers dropped (`outputNumber`/`outputValue` off SetOutput, `waitMs` off Wait,
  `loopCount` off Loop). Legacy left/operator/right condition rows become `expr` only where exact
  (numeric operators; string `contains`/`startsWith`/`endsWith` on a declared string variable with
  literal text); others stay rows and ride in `@with`.
- **Mapping bodies are optional:** `RunVision` / `HttpRequest` / `HttpReceive` / `ExportData` /
  `plugin` take rows and `end` only when they have rows (a row line is recognisable: it starts with
  a string, `send`/`receive`, or `name =` / `name ->` / `name.`).
- **Positionals** may also be separated by commas (`SetJointPosition J3, 90`) and may follow named
  args (`AuxEnable device=Aux, on`). `Wait until …`, `SavePosition point X`, `GripperAction to 25`,
  `SetOutput … toggle`, `J<n>`, `else if` are sugar. Enum words: stopwatch `start|stop|reset`,
  gripper `open|close|to`, conveyor `forward|reverse|stop`; other enums use the option value.
- **Values:** `(…)` always means an expression (`speed=(200)` is the expression `200`); a positional
  expression must be a `$var` token or parenthesised. Expression-or-template slots (`call` args,
  plugin params, prompt `default`) print quoted unless the text looks like an expression.
- **`@with` keys that are not fields** → `unknownField` warning (dropped). `missingArgument` is not
  reported for a step that has an `@with`. New codes: `unknownDevice`, `unknownCamera`,
  `unknownField` (warnings). Unknown names of id-only references (devices, cameras, grids, stacks,
  vision tools) keep the typed text as the id.
- **Ids:** besides LCS + type pairing, a step moved elsewhere in the same list keeps its id (same
  key, unmatched on both sides). Steps that arrived as JSON in `@with` are never re-id'd. Condition
  items split from one `expr` get `<id>.<k>` ids in Normalize. Variables match by name and inherit
  `value` when they have a `valueExpression`.
- **Commands:** `FormatProgramText` also takes `includeHeader` (the `@program {…}` line, for
  export; `FormatProgramTextParams.IncludeHeader`). Note the WebSocket ack merges result fields, so
  `ParseProgramText`'s `ok` *is* the ack's `ok` (the app's client already handles this; the MCP
  `send` has `keep_ok=True` for it).
- **Cross-package edits (minimal):** `BuiltProgramCommands.BuildValidationContext` is `internal`
  and `CommandDispatcher` passes it to `LanguageCommands(robot, …)`; `StepCatalogTests` no longer
  expects the language commands to be stubs.
- **Tests:** `RoundTripTests` (golden step, golden step as every type, every type empty and with
  catalog defaults, the samples, 500 random seeds, ids mode), `ParserErrorTests` (every code with
  its line/col/endCol, UTF-16 columns, recovery), `ReconcilerTests`, `PrinterGoldenTests`
  (`Fixtures/srs/*.srs`, `SRS_UPDATE_GOLDEN=1` rewrites), `DocTableTests`, and `RealProgramTests`
  (round-trips a copied data dir given in `SRS_REAL_DIR`; all 20 of Lucas's programs pass).
  MCP: `mcp/program_text_check.py` (live, moves nothing).

### Package E — app Blocks editor (after A, parallel)

| Owns | Files |
|---|---|
| Generic fields | `src/components/ui/builder/CatalogStepFields.tsx` (new, the ParamFields pattern), `catalogFieldControls.tsx` (new) |
| Bodies | `SwitchBody.tsx`, `TryBody.tsx` (new, modelled on `IfConditionBody.tsx`); inline expand in `StepRow.tsx` |
| Editor | `StepConfigModal.tsx` (route new types to CatalogStepFields; migrate simple steps; field-level problems; dedupe the axes blocks), optional `StepConfigPage.tsx` (full page on phones) |
| Builder | `app/(tabs)/program/builder.tsx` (after A's edit: enable toggle, Note rows and section collapse, `useDragReorder` swap, select-bar actions including Copy/Paste as text, search, run from here, breakpoint toggle, `newStep()` from the catalog), `editorDocument.ts`, `stepUtils.tsx` (summaries from the catalog), `renameVariable.ts` (new bare-name keys: `errorVariableName`, `errorCodeVariableName`, `promptResultVariableName`, `promptInputVariableName`, `listSourceVariable`, `routineOutputs[].variableName`, …), `stepClipboard.ts` (deep ids via `stepTree`) |
| Routine params | `VariableEditModal.tsx` (paramMode), the CallRoutine args/outputs section |
| Teach-here | the point sub-page in `StepConfigModal.tsx` (save/update a point from the live pose) |
| Checks | `npx tsc --noEmit`; `npm run keyboard-audit`, with scenarios for each new modal field; a Playwright sweep of the builder at phone, tablet and desktop widths (memory: `reference_app_ui_testing`) |

### Package F — app Text editor (after A; integrates with D)

| Owns | Files |
|---|---|
| Component | `src/components/ui/code-editor/CodeEditor.tsx` (WebView), `CodeEditor.web.tsx`, `codeEditorHtml.ts` (inline HTML + bundled CodeMirror 6 build), `bridge.ts` |
| Language | `src/components/ui/code-editor/srsTokens.ts` (tokenizer + CodeMirror StreamLanguage), `srsCompletions.ts` (keywords and args from the catalog; `$` via `expressions/completions.ts`) |
| View | `src/components/ui/builder/TextProgramView.tsx` (fill A's stub: debounced parse, diagnostics, apply-to-document, run highlight, key row, insert step) |
| Import/export | `app/(tabs)/program/robot-programs.tsx` (Export as .srs / Import text) |
| Checks | `scripts/srs-tokens-check.mts` (tokenizer on the fixture `.srs` files), a keyboard-audit scenario, a web Playwright check. Until D lands, build against a mock `parseProgramText` that echoes. |

### Package G — app monitor, prompts and debugging (after A; integrates with B and C)

| Owns | Files |
|---|---|
| Prompt | `src/components/ui/program/PromptDialog.tsx` (new), mounted in `app/_layout.tsx` (root) |
| Monitor | `app/(tabs)/program/monitor-program.tsx` (pause-reason chip, Continue / Step, highlighted step, breakpoints passed through) |
| Checks | tsc, keyboard-audit scenarios (prompt number/text input), live run against :9100 once B and C land |

### Order and dependencies

```
            ┌─ B (controller flow/errors/debug) ─┐
A ──────────┼─ C (controller motion/data/prompt) ┼─► G integration (needs B + C live)
(contracts) ├─ D (controller language) ──────────┼─► F integration (needs D live)
            ├─ E (app blocks)                    │
            └─ F (app text, against mocks)       ┘
```

**File overlap after A, by design:**

- B and C never touch the same file.
  - B owns every *existing* step handler file.
  - C creates only new handler files, plus `ExpressionEvaluator.cs`, `ProgramCycleManager.cs` and the
    speed-override hook.
- E owns `builder.tsx`. F only fills `TextProgramView.tsx` and new files. G only touches the monitor,
  the root layout and the new dialog.
- Shared docs: section-scoped edits only.

### Shared contracts to agree in A (the frozen list)

1. The field names and enum strings in §4.1–4.3. Once they are in the goldens, they are frozen.
2. `StepTree.ChildLists()` roles, and their order (it is used for paths:
   `steps[2].switchCases[0].steps[1]`, `steps[3].trySteps[0]`).
3. The `GetStepCatalog` JSON shape (§5.2), and the `FieldSpec.kind` list.
4. The error codes and their catchability (§4.2).
5. The status DTO: `pauseReason`, `pausedAtStepId`, `prompt`.
6. The command names and params: `FormatProgramText`, `ParseProgramText`, `AnswerPrompt`,
   `StepBuiltProgram`, `SetBreakpoints`, and the `ExecuteBuiltProgram` options.
7. `TextProblem` (1-based line and column, UTF-16 columns) and the `map` entries.

---

## 9. Open questions for Lucas

The defaults chosen in this spec are in brackets. Building can start on them, but each is cheap to
change before package A freezes the names.

1. **Language name.** [**Simple Robot Script, `.srs`**.] Alternatives: *RoboText* (`.rtx`), *Steps*
   (`.steps`), *RMS Script* (`.rms`).
2. **Keywords.** [**Step types keep their PascalCase names** (`MoveL`, `SetOutput`), and flow and
   assignment are lowercase (`if`, `repeat`, `$x = …`).] The alternative is all-lowercase friendly
   words (`move linear to Pick1`), which is more readable but further from the blocks and from
   SPEL/RAPID.
3. **Step editor on phones.** [**Full page**, like the vision `ToolConfigPage`, with the centred modal
   kept on desktop.] Or keep the modal everywhere.
4. **Try / Catch scope.** [**Runtime failures only:** timeouts, vision, HTTP, plugin, prompt, raise.
   Expression typos and motion faults stay fatal.]
5. **SavePosition to a saved point.** [**Allowed,** with a validator warning in background programs
   and a points-history entry.] Or variables only.
6. **`$s.toNumber()` on bad text.** [**Evaluates to 0.**] Or make it a catchable `badNumber` error.
7. **The P2 items:** move triggers, guards, MoveJoints, grid loop, edge wait, jump arch. Should any
   of them move up into P1?
