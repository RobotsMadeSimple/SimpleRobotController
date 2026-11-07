using Controller.RobotControl.Execution;

namespace Controller.RobotControl.Validation
{
    // Package C (docs/program-language.md §8): checks for MoveC (via point missing or unknown,
    // unsupportedOnStep), SavePosition (destination, mode, pointWriteInBackground), SetSpeedFactor
    // (range), ListOp (the fields each listAction needs for the list's element type) and Prompt
    // (1–4 buttons, input, timeout button).
    internal static partial class ProgramValidator
    {
        private sealed partial class Walker
        {
            private static readonly string[] PointAxisNames = ["x", "y", "z", "rx", "ry", "rz"];

            partial void CheckStepsExt(ProgramStep s, At at)
            {
                switch (s.Type)
                {
                    case StepType.MoveC:          ValidateMoveC(s, at); break;
                    case StepType.SavePosition:   ValidateSavePosition(s, at); break;
                    case StepType.SetSpeedFactor: ValidateSpeedFactor(s, at); break;
                    case StepType.ListOp:         ValidateListOp(s, at); break;
                    case StepType.Prompt:         ValidatePrompt(s, at); break;
                }
            }

            private static bool HasExpr(ProgramStep s, string key) => s.Expressions?.ContainsKey(key) == true;

            // ── MoveC ─────────────────────────────────────────────────────────

            private void ValidateMoveC(ProgramStep s, At at)
            {
                CheckMoveTarget(s, at);
                if ((!string.IsNullOrEmpty(s.LocalId) || !string.IsNullOrEmpty(s.LocalName)) &&
                    _ctx.LocalExists != null && !_ctx.LocalExists(s.LocalId, s.LocalName))
                    Add(at, ValidationCodes.UnknownLocal, $"Local '{s.LocalName ?? s.LocalId}' does not exist", "localId");

                // The via point: the same rules as a target, on the via fields.
                if (!string.IsNullOrEmpty(s.ViaPointNameExpr))
                {
                    var expr = s.ViaPointNameExpr;
                    if (MoveTargetResolver.TryParsePointsRef(expr, out var refName, out var idxExpr) && IsList(refName))
                    {
                        CheckPointList(refName, at, "via");
                        if (!string.IsNullOrWhiteSpace(idxExpr)) CheckExpr(idxExpr, at, "via");
                    }
                    else if (expr.Contains('$') || expr.Contains('{'))
                        CheckTemplate(expr, at, "via", ValidationSeverity.Error);
                    else if (_ctx.PointExists != null && !_ctx.PointExists(null, expr.Trim()))
                        Add(at, ValidationCodes.UnknownPoint, $"Via point '{expr.Trim()}' does not exist", "via");
                }
                else if (!string.IsNullOrEmpty(s.ViaPointId) || !string.IsNullOrEmpty(s.ViaPointName))
                {
                    if (_ctx.PointExists != null && !_ctx.PointExists(s.ViaPointId, s.ViaPointName))
                        Add(at, ValidationCodes.UnknownPoint, $"Via point '{s.ViaPointName ?? s.ViaPointId}' does not exist", "via");
                    else if (!string.IsNullOrEmpty(s.ViaPointId) && s.ViaPointId == s.PointId ||
                             string.IsNullOrEmpty(s.ViaPointId) && string.IsNullOrEmpty(s.PointId) &&
                             !string.IsNullOrEmpty(s.ViaPointName) &&
                             string.Equals(s.ViaPointName, s.PointName, StringComparison.OrdinalIgnoreCase))
                        Add(at, ValidationCodes.MissingField,
                            "The via point is the end point — an arc needs three different points", "via");
                }
                else
                    Add(at, ValidationCodes.MissingField, "Move Arc has no via point", "via");

                // Not part of a Move Arc: say so rather than ignore it silently.
                if (MoveStopConditions.HasCondition(s))
                    Add(at, ValidationCodes.UnsupportedOnStep,
                        "Move Arc does not support a stop condition — the arc would run to its end", "moveStopCondition");
                if (MoveTargetResolver.HasToolOffset(s))
                    Add(at, ValidationCodes.UnsupportedOnStep, "Move Arc ignores a tool offset", "toolOffset",
                        ValidationSeverity.Warning);
                if (s.Blend == true)
                    Add(at, ValidationCodes.UnsupportedOnStep, "Move Arc always stops at its end point; blending is ignored", "blend",
                        ValidationSeverity.Warning);
                if (s.ArcTolerance is { } tol && !(tol >= ArcPath.MinTolerance))
                    Add(at, ValidationCodes.MissingField,
                        tol > 0 ? $"Arc tolerance below {ArcPath.MinTolerance} mm is raised to it"
                                : $"Arc tolerance must be above 0 — the default {ArcPath.DefaultTolerance} mm is used",
                        "arcTolerance", ValidationSeverity.Warning);
            }

            // ── SavePosition ──────────────────────────────────────────────────

            private void ValidateSavePosition(ProgramStep s, At at)
            {
                if (s.SavePositionSource is not (null or "" or "actual" or "target"))
                    Add(at, ValidationCodes.MissingField, $"Position source '{s.SavePositionSource}' must be actual or target", "savePositionSource");

                if (s.SavePositionTo == "point")
                {
                    if (string.IsNullOrEmpty(s.PointId) && string.IsNullOrWhiteSpace(s.PointName))
                        Add(at, ValidationCodes.MissingField, "No saved point is selected", "pointId");
                    else if (string.IsNullOrWhiteSpace(s.PointName) && _ctx.PointExists != null && !_ctx.PointExists(s.PointId, null))
                        Add(at, ValidationCodes.UnknownPoint, $"Point '{s.PointId}' does not exist", "pointId");
                    if (_main.IsBackground)
                        Add(at, ValidationCodes.PointWriteInBackground,
                            "A background program overwrites a saved point here — the main program may be moving to it",
                            "pointId", ValidationSeverity.Warning);
                    return;
                }
                if (s.SavePositionTo is not (null or "" or "variable"))
                {
                    Add(at, ValidationCodes.MissingField, $"Save to '{s.SavePositionTo}' must be variable or point", "savePositionTo");
                    return;
                }

                if (string.IsNullOrWhiteSpace(s.VariableName))
                    Add(at, ValidationCodes.MissingField, "No Points variable is selected", "variableName");
                else
                {
                    var name = s.VariableName.Trim().TrimStart('$');
                    if (CheckTarget(name, at, "variableName", requireDeclared: false))
                    {
                        var sym = Lookup(name);
                        if (sym is { Kind: SymKind.List } && sym.ElementType != ListElementType.Point)
                            Add(at, ValidationCodes.UnknownVariable, $"'${name}' is a {sym.ElementType} list, not a Points list", "variableName");
                        else if (sym is { Kind: not SymKind.List })
                            Add(at, ValidationCodes.UnknownVariable, $"'${name}' is not a list — Save Position needs a Points list", "variableName");
                    }
                }

                var mode = string.IsNullOrEmpty(s.SavePositionMode) ? "replace" : s.SavePositionMode;
                if (mode is not ("replace" or "append" or "insert" or "set"))
                    Add(at, ValidationCodes.MissingField, $"Mode '{mode}' must be replace, append, insert or set", "savePositionMode");
                else if (mode is "insert" or "set" && s.SavePositionIndex is < 0 && !HasExpr(s, "savePositionIndex"))
                    Add(at, ValidationCodes.MissingField, "The index cannot be negative", "savePositionIndex");
            }

            // ── SetSpeedFactor ────────────────────────────────────────────────

            private void ValidateSpeedFactor(ProgramStep s, At at)
            {
                if (HasExpr(s, "speedFactor")) return; // checked with every expression; clamped at run time
                if (s.SpeedFactor is not { } f)
                    Add(at, ValidationCodes.MissingField, "No speed factor is set — 100 % is used", "speedFactor", ValidationSeverity.Warning);
                else if (f < 1 || f > 100)
                    Add(at, ValidationCodes.MissingField,
                        $"Speed factor {f} % is outside 1–100 % and is limited to {Math.Clamp(f, 1, 100)} %", "speedFactor",
                        ValidationSeverity.Warning);
            }

            // ── ListOp ────────────────────────────────────────────────────────

            private void ValidateListOp(ProgramStep s, At at)
            {
                Sym? sym = null;
                string name = "";
                if (string.IsNullOrWhiteSpace(s.VariableName))
                    Add(at, ValidationCodes.MissingField, "No list variable is selected", "variableName");
                else
                {
                    name = s.VariableName.Trim().TrimStart('$');
                    Use(name);
                    sym = Lookup(name);
                    if (sym == null)
                        Add(at, ValidationCodes.UnknownVariable, $"Unknown list variable '${name}'", "variableName");
                    else if (sym.Kind != SymKind.List)
                    {
                        Add(at, ValidationCodes.UnknownVariable, $"'${name}' is not a list variable", "variableName");
                        sym = null;
                    }
                }

                var action = string.IsNullOrEmpty(s.ListAction) ? "append" : s.ListAction;
                if (action is not ("append" or "insert" or "removeAt" or "clear" or "set" or "setField" or "sort" or "reverse" or "copyFrom"))
                {
                    Add(at, ValidationCodes.MissingField, $"Unknown list action '{action}'", "listAction");
                    return;
                }

                // The element type decides what an action needs; unknown (no list) = check only what is typed.
                ListElementType? et = sym?.ElementType;
                bool scalar = et is ListElementType.Number or ListElementType.Boolean;

                if (action is "insert" or "removeAt" or "set" or "setField")
                {
                    if (s.ListIndex is null && !HasExpr(s, "listIndex"))
                        Add(at, ValidationCodes.MissingField, "No index is set — index 0 is used", "listIndex", ValidationSeverity.Warning);
                    else if (s.ListIndex is < 0 && !HasExpr(s, "listIndex"))
                        Add(at, ValidationCodes.MissingField, "The index cannot be negative", "listIndex");
                }

                if (action is "append" or "insert" or "set")
                {
                    bool hasValue  = !string.IsNullOrWhiteSpace(s.ListValueExpr);
                    bool hasFields = s.ListFields is { Count: > 0 };
                    if (hasValue) CheckExpr(s.ListValueExpr, at, "listValueExpr");
                    if (scalar)
                    {
                        bool valueField = hasFields && s.ListFields!.Keys.Any(k => string.Equals(k?.Trim(), "value", StringComparison.OrdinalIgnoreCase));
                        if (!hasValue && !valueField)
                            Add(at, ValidationCodes.MissingField, $"No value is set for the {et} list", "listValueExpr");
                        if (hasFields && s.ListFields!.Keys.Any(k => !string.Equals(k?.Trim(), "value", StringComparison.OrdinalIgnoreCase)))
                            Add(at, ValidationCodes.UnsupportedOnStep, $"A {et} list has no fields — use the value", "listFields");
                    }
                    else if (et is ListElementType.Point or ListElementType.Record)
                    {
                        if (hasValue)
                            Add(at, ValidationCodes.UnsupportedOnStep,
                                $"A {et} list element takes fields, not a single value", "listValueExpr");
                        if (!hasFields && !(et == ListElementType.Point && action == "append"))
                            Add(at, ValidationCodes.MissingField,
                                et == ListElementType.Point ? "No axes are set (x, y, z, rx, ry, rz)" : "No fields are set", "listFields");
                    }
                    CheckListFields(s, at, et);
                }
                else if (action == "setField")
                {
                    if (scalar)
                        Add(at, ValidationCodes.UnsupportedOnStep, $"A {et} list has no fields — use Set element", "listAction");
                    else if (s.ListFields is not { Count: > 0 })
                        Add(at, ValidationCodes.MissingField, "No field is set", "listFields");
                    CheckListFields(s, at, et);
                }
                else if (action == "sort")
                {
                    var field = s.ListSortField?.Trim();
                    if (scalar && !string.IsNullOrEmpty(field) && !field.Equals("value", StringComparison.OrdinalIgnoreCase))
                        Add(at, ValidationCodes.UnsupportedOnStep, $"A {et} list sorts by value, not by '{field}'", "listSortField");
                    else if (et is ListElementType.Point or ListElementType.Record && string.IsNullOrEmpty(field))
                        Add(at, ValidationCodes.MissingField, $"Choose the field to sort the {et} list by", "listSortField");
                    else if (et == ListElementType.Point && !string.IsNullOrEmpty(field) &&
                             !PointAxisNames.Contains(field, StringComparer.OrdinalIgnoreCase))
                        Add(at, ValidationCodes.MissingField, $"'{field}' is not a point axis (x, y, z, rx, ry, rz)", "listSortField");
                }
                else if (action == "copyFrom")
                {
                    if (string.IsNullOrWhiteSpace(s.ListSourceVariable))
                        Add(at, ValidationCodes.MissingField, "No list to copy from is selected", "listSourceVariable");
                    else
                    {
                        var src = s.ListSourceVariable.Trim().TrimStart('$');
                        Use(src);
                        var srcSym = Lookup(src);
                        if (srcSym is not { Kind: SymKind.List })
                            Add(at, ValidationCodes.UnknownVariable,
                                srcSym == null ? $"Unknown list variable '${src}'" : $"'${src}' is not a list variable", "listSourceVariable");
                        else if (et is { } to && !CanCopyList(srcSym.ElementType, to))
                            Add(at, ValidationCodes.UnsupportedOnStep,
                                $"Cannot copy a {srcSym.ElementType} list into '${name}', a {to} list", "listSourceVariable");
                    }
                }
            }

            private void CheckListFields(ProgramStep s, At at, ListElementType? et)
            {
                foreach (var (rawKey, expr) in s.ListFields ?? [])
                {
                    var key = rawKey?.Trim() ?? "";
                    if (key.Length == 0)
                    {
                        Add(at, ValidationCodes.MissingField, "A field has no name", "listFields");
                        continue;
                    }
                    if (et == ListElementType.Point && !PointAxisNames.Contains(key, StringComparer.OrdinalIgnoreCase))
                        Add(at, ValidationCodes.MissingField, $"'{key}' is not a point axis (x, y, z, rx, ry, rz)", $"listFields.{key}");
                    if (string.IsNullOrWhiteSpace(expr))
                        Add(at, ValidationCodes.MissingField, $"Field '{key}' has no value — it evaluates to 0", $"listFields.{key}",
                            ValidationSeverity.Warning);
                    else
                        CheckExpr(expr, at, $"listFields.{key}");
                }
            }

            private static bool CanCopyList(ListElementType from, ListElementType to) =>
                from == to || to == ListElementType.Record ||
                (from is ListElementType.Number or ListElementType.Boolean && to is ListElementType.Number or ListElementType.Boolean);

            // ── Prompt ────────────────────────────────────────────────────────

            private void ValidatePrompt(ProgramStep s, At at)
            {
                if (string.IsNullOrWhiteSpace(s.PromptMessage))
                    Add(at, ValidationCodes.MissingField, "No message is set", "promptMessage");
                else CheckTemplate(s.PromptMessage, at, "promptMessage", ValidationSeverity.Warning);
                CheckTemplate(s.PromptTitle, at, "promptTitle", ValidationSeverity.Warning);

                var buttons = s.PromptButtons ?? [];
                int labelled = buttons.Count(b => !string.IsNullOrWhiteSpace(b));
                if (buttons.Count > 4)
                    Add(at, ValidationCodes.MissingField, $"A prompt has 1–4 buttons; only the first 4 of {buttons.Count} are shown", "promptButtons");
                if (labelled < buttons.Count)
                    Add(at, ValidationCodes.MissingField, "A button has no label and is left out", "promptButtons", ValidationSeverity.Warning);
                int shown = Math.Max(1, Math.Min(4, labelled));

                var input = string.IsNullOrEmpty(s.PromptInput) ? "none" : s.PromptInput;
                if (input is not ("none" or "number" or "text"))
                {
                    Add(at, ValidationCodes.MissingField, $"Input '{input}' must be none, number or text", "promptInput");
                    input = "none";
                }

                if (input == "number")
                {
                    if (!string.IsNullOrWhiteSpace(s.PromptDefault)) CheckExpr(s.PromptDefault, at, "promptDefault");
                    if (s.PromptMin is { } min && s.PromptMax is { } max && min > max)
                        Add(at, ValidationCodes.MissingField, $"Minimum {min} is above maximum {max}", "promptMin");
                }
                else if (input == "text")
                    CheckTemplate(s.PromptDefault, at, "promptDefault", ValidationSeverity.Warning);

                CheckPromptVariable(s.PromptResultVariableName, at, "promptResultVariableName");
                if (input == "none")
                {
                    if (!string.IsNullOrWhiteSpace(s.PromptInputVariableName))
                        Add(at, ValidationCodes.UnsupportedOnStep, "The prompt has no input, so its input variable is never set",
                            "promptInputVariableName", ValidationSeverity.Warning);
                }
                else CheckPromptVariable(s.PromptInputVariableName, at, "promptInputVariableName");

                if (s.PromptTimeoutMs is < 0)
                    Add(at, ValidationCodes.MissingField, "Timeout cannot be negative", "promptTimeoutMs");
                if (s.PromptTimeoutButton is { } tb)
                {
                    if (tb < 0 || tb >= shown)
                        Add(at, ValidationCodes.MissingField, $"On timeout, button {tb} does not exist (0–{shown - 1})", "promptTimeoutButton");
                    else if (s.PromptTimeoutMs is null or <= 0)
                        Add(at, ValidationCodes.UnsupportedOnStep, "The prompt waits forever, so the timeout button is never pressed",
                            "promptTimeoutButton", ValidationSeverity.Warning);
                }
            }

            private void CheckPromptVariable(string? raw, At at, string field)
            {
                if (string.IsNullOrWhiteSpace(raw)) return;
                var name = raw.Trim().TrimStart('$');
                if (!CheckTarget(name, at, field, requireDeclared: false)) return;
                if (Lookup(name) is { Kind: SymKind.List or SymKind.Image or SymKind.Stopwatch } sym)
                    Add(at, ValidationCodes.UnknownVariable,
                        $"'${name}' is {(sym.Kind == SymKind.List ? "a list" : sym.Kind == SymKind.Image ? "an image" : "a stopwatch")} — a prompt writes a number or text",
                        field);
            }
        }
    }
}
