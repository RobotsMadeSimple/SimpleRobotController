using System.Text.Json;
using System.Text.Json.Nodes;

namespace Controller.RobotControl.Language;

/// <summary>
/// The canonical form of a program (docs/program-text.md, "Round trip"). Every rule here is a
/// rewrite that does not change what the program does; the printer prints the normalized
/// program, and the round-trip guarantee is stated modulo this function:
/// <c>Normalize(Parse(Format(P), base: P)) == Normalize(P)</c>.
/// <list type="number">
/// <item>Legacy <c>varPointName</c>/<c>varPointIndex</c> → <c>pointNameExpr "$name[index]"</c>.</item>
/// <item>Conditions: legacy left/operator/right rows → <c>expr</c> where that is exact; an
/// <c>expr</c> with a top-level and/or is split into items; every item after the first has an
/// explicit join (AND/OR), the first none; <c>combinator</c> "ALL".</item>
/// <item>Empty lists and objects → absent (not a list variable's <c>items</c>, nor inside the
/// opaque <c>cncSpec</c> / <c>displayPageLayout</c>).</item>
/// <item>(Reconciler) a literal shadowed by <c>expressions[key]</c> is inherited from the base step.</item>
/// <item>Reference names (point, tool, local, routine, background program, vision program, via
/// point, goto label) are refreshed from the repository for their id; a missing id is filled in
/// from the name when the name resolves.</item>
/// <item>A <c>valueExpression</c> that is a plain number → <c>value</c>.</item>
/// <item>StatusUpdate: a missing <c>statusSeverity</c> is inferred the way the app does.</item>
/// <item>Legacy <c>jsonImageOutbound</c> rows → <c>jsonOutbound</c> rows with <c>imageVar</c>
/// (HttpSteps sends them identically: after the outbound rows, same key wins last).</item>
/// <item>Defaults that the text always states: Loop mode "count", Wait mode "duration", SetOutput card "stb",
/// LoopControl "break", ListOp action "append"; Note style "note" → absent; <c>enabled: true</c>
/// → absent; false variable flags → absent; a list variable's missing element type → Record;
/// legacy list fields (values/points/objects) → <c>items</c>; Point elements carry all six axes.</item>
/// <item>Old-editor leftovers: <c>outputNumber</c>/<c>outputValue</c> on a step that is not a
/// SetOutput, <c>waitMs</c> on one that is not a Wait, <c>loopCount</c> on one that is not a Loop
/// are dropped (the old builder stamped them on every step; only their own type reads them).</item>
/// </list>
/// </summary>
internal static class ProgramNormalizer
{
    /// <summary>A normalized deep copy.</summary>
    public static BuiltProgram Normalize(BuiltProgram program, NameResolver names)
    {
        var p = SrsJson.Clone(program);
        var stringVars = new HashSet<string>(
            (p.Variables ?? []).Where(v => v.IsString == true).Select(v => v.Name), StringComparer.Ordinal);
        foreach (var v in p.Variables ?? []) NormalizeVariable(v);
        var labels = LabelMap(p.Steps);
        foreach (var s in StepTree.Walk(p.Steps)) NormalizeStep(s, names, labels, stringVars);
        var json = SrsJson.ToJson(p);
        StripEmpty(json, isProgram: true);
        return json.Deserialize<BuiltProgram>(SrsJson.Options)!;
    }

    /// <summary>The JSON used for comparisons: <see cref="Normalize"/>, then empty lists and
    /// objects removed (which deserializing would otherwise re-create on non-nullable lists).</summary>
    public static JsonObject NormalizeJson(BuiltProgram program, NameResolver names)
    {
        var json = SrsJson.ToJson(Normalize(program, names));
        StripEmpty(json, isProgram: true);
        return json;
    }

    // ── Steps ──────────────────────────────────────────────────────────────

    public static void NormalizeStep(ProgramStep s, NameResolver names, LabelTable labels, ISet<string> stringVars)
    {
        if (s.Enabled == true) s.Enabled = null;

        // 1. legacy variable-point target
        if (s.VarPointName != null)
        {
            if (s.VarPointName.Length > 0)
                s.PointNameExpr = $"${s.VarPointName}[{(string.IsNullOrEmpty(s.VarPointIndex) ? "0" : s.VarPointIndex)}]";
            s.VarPointName = null;
            s.VarPointIndex = null;
        }
        else if (s.VarPointIndex != null && string.IsNullOrEmpty(s.VarPointName)) s.VarPointIndex = null;

        // 2. conditions
        s.Condition          = NormalizeCondition(s.Condition, stringVars);
        s.LoopWhileCondition = NormalizeCondition(s.LoopWhileCondition, stringVars);
        s.WaitCondition      = NormalizeCondition(s.WaitCondition, stringVars);
        s.MoveStopCondition  = NormalizeCondition(s.MoveStopCondition, stringVars);
        foreach (var b in s.ElseIfBranches ?? []) if (b != null) b.Condition = NormalizeCondition(b.Condition, stringVars) ?? new();

        // 5. reference names
        (s.PointId, s.PointName)       = Ref(names, SrsRefKind.Point, s.PointId, s.PointName);
        (s.ViaPointId, s.ViaPointName) = Ref(names, SrsRefKind.Point, s.ViaPointId, s.ViaPointName);
        (s.ToolId, s.ToolName)         = Ref(names, SrsRefKind.Tool, s.ToolId, s.ToolName);
        (s.LocalId, s.LocalName)       = Ref(names, SrsRefKind.Local, s.LocalId, s.LocalName);
        (s.RoutineId, s.RoutineName)   = Ref(names, SrsRefKind.Routine, s.RoutineId, s.RoutineName);
        (s.BackgroundProgramId, s.BackgroundProgramName) = Ref(names, SrsRefKind.Program, s.BackgroundProgramId, s.BackgroundProgramName);
        (s.VisionProgramId, s.VisionProgramName)         = Ref(names, SrsRefKind.Vision, s.VisionProgramId, s.VisionProgramName);
        if (s.Type == StepType.GoToLabel)
        {
            if (!string.IsNullOrEmpty(s.LabelId) && labels.NameOf(s.LabelId) is { } ln) s.LabelName = ln;
            else if (string.IsNullOrEmpty(s.LabelId) && !string.IsNullOrEmpty(s.LabelName) && labels.IdOf(s.LabelName) is { } lid)
                s.LabelId = lid;
        }

        // 7. status severity, as the app infers it
        if (s.Type == StepType.StatusUpdate && s.StatusSeverity == null)
            s.StatusSeverity = !string.IsNullOrEmpty(s.StatusWarning) ? "Warning"
                             : !string.IsNullOrEmpty(s.StatusError) ? "Error" : "Info";

        // 8. legacy image rows
        if (s.JsonImageOutbound != null)
        {
            var rows = s.JsonOutbound ?? [];
            foreach (var m in s.JsonImageOutbound)
                if (m != null && !string.IsNullOrWhiteSpace(m.Key) && !string.IsNullOrWhiteSpace(m.VariableName))
                    rows.Add(new JsonKeyValue { Key = m.Key, Expr = "", ImageVar = m.VariableName });
            s.JsonOutbound = rows.Count > 0 ? rows : null;
            s.JsonImageOutbound = null;
        }

        // 10. leftovers of the old editor, which stamped these defaults on every step: only their
        // own step type reads them (SetOutput, Wait, Loop — handlers, descriptions and validator).
        if (s.Type != StepType.SetOutput) { s.OutputNumber = null; s.OutputValue = null; }
        if (s.Type != StepType.Wait) s.WaitMs = null;
        if (s.Type != StepType.Loop) s.LoopCount = null;

        // 9. defaults the text always states
        switch (s.Type)
        {
            case StepType.Loop:        s.LoopMode ??= "count"; break;
            case StepType.Wait:        s.WaitMode ??= "duration"; break;
            case StepType.LoopControl: s.LoopControl ??= "break"; break;
            case StepType.ListOp:      s.ListAction ??= "append"; break;
            case StepType.SetOutput:   s.OutputCard ??= "stb"; break;
            case StepType.Note:        if (s.NoteStyle == "note") s.NoteStyle = null; break;
        }
    }

    private static (string? Id, string? Name) Ref(NameResolver names, SrsRefKind kind, string? id, string? name)
    {
        if (!string.IsNullOrEmpty(id))
            return names.NameOf(kind, id) is { } n ? (id, n) : (id, name);
        if (!string.IsNullOrEmpty(name) && names.IdOf(kind, name) is { } found)
            return (found, names.NameOf(kind, found));
        return (id, name);
    }

    // ── Conditions ─────────────────────────────────────────────────────────

    public static ConditionGroup? NormalizeCondition(ConditionGroup? g, ISet<string> stringVars)
    {
        if (g == null) return null;
        string defaultJoin = g.Combinator == "ANY" ? "OR" : "AND";
        var items = new List<ConditionItem>();
        for (int i = 0; i < (g.Items?.Count ?? 0); i++)
        {
            var it = g.Items![i];
            if (it == null) continue;
            string join = string.IsNullOrEmpty(it.Join) ? defaultJoin
                        : it.Join.ToUpperInvariant() == "OR" ? "OR" : "AND";
            var expr = it.Expr;
            if (string.IsNullOrWhiteSpace(expr)) expr = LegacyToExpr(it, stringVars);
            if (string.IsNullOrWhiteSpace(expr))
            {
                // A legacy row with no exact expression form: kept as it is.
                items.Add(new ConditionItem { Id = it.Id, Expr = it.Expr, Join = items.Count == 0 ? null : join,
                                              Left = it.Left, Operator = it.Operator, Right = it.Right });
                continue;
            }
            var parts = SplitCondition(expr) ?? [("", expr)];
            for (int k = 0; k < parts.Count; k++)
            {
                string j = k == 0 ? join : parts[k].Join;
                items.Add(new ConditionItem
                {
                    Id   = k == 0 ? it.Id : $"{it.Id}.{k}",
                    Expr = parts[k].Text,
                    Join = items.Count == 0 ? null : j,
                });
            }
        }
        return new ConditionGroup { Combinator = "ALL", Items = items };
    }

    private static readonly HashSet<string> NumericOps = ["==", "!=", "<", "<=", ">", ">="];

    /// <summary>The exact expression for a legacy left/operator/right row, or null.</summary>
    public static string? LegacyToExpr(ConditionItem it, ISet<string> stringVars)
    {
        string left = (it.Left ?? "").Trim(), right = (it.Right ?? "").Trim(), op = it.Operator ?? "==";
        bool leftIsStringVar = left.StartsWith('$') && stringVars.Contains(left[1..]);
        string? expr = null;
        if (NumericOps.Contains(op) && !leftIsStringVar)
            expr = $"{Side(left)} {op} {Side(right)}";
        else if (op is "contains" or "startsWith" or "endsWith" && leftIsStringVar && SrsCursor.IsIdent(left[1..]) &&
                 !right.Contains('{') && !right.Contains('$') && !right.Contains('"') && !right.Contains('\\'))
            expr = $"{left}.{op}(\"{it.Right}\")";
        if (expr == null) return null;
        try { ExpressionEvaluator.Validate(expr); return expr; }
        catch { return null; }
    }

    private static string Side(string x)
    {
        if (x.Length == 0) return "0";
        if (SrsText.TryLiteral(x, false, out _) || SrsText.IsVarRefToken(x)) return x;
        return "(" + x + ")";
    }

    /// <summary>
    /// Splits an expression at its top-level <c>and</c> / <c>or</c> (and <c>&amp;&amp;</c> / <c>||</c>)
    /// into (join, text) parts — the first part's join is "". Null when there is nothing to split,
    /// when a top-level <c>?</c> makes splitting change the meaning, or when a part does not parse.
    /// </summary>
    public static List<(string Join, string Text)>? SplitCondition(string expr)
    {
        var parts = new List<(string Join, string Text)>();
        int depth = 0, segStart = 0;
        string join = "";
        int i = 0;
        char prevSig = '\0';
        while (i < expr.Length)
        {
            char c = expr[i];
            if (c == '"')
            {
                int j = expr.IndexOf('"', i + 1);
                if (j < 0) return null;
                i = j + 1; prevSig = '"'; continue;
            }
            if (c == '$')
            {
                i++;
                while (i < expr.Length && (SrsCursor.IsIdentChar(expr[i]) ||
                       (expr[i] == '.' && i + 1 < expr.Length && SrsCursor.IsIdentChar(expr[i + 1])))) i++;
                prevSig = 'v'; continue;
            }
            if (c is '(' or '[' or '{') { depth++; i++; prevSig = c; continue; }
            if (c is ')' or ']' or '}') { depth--; i++; prevSig = c; continue; }
            if (depth == 0 && c == '?') return null;
            if (depth == 0 && i + 1 < expr.Length && ((c == '&' && expr[i + 1] == '&') || (c == '|' && expr[i + 1] == '|')))
            {
                parts.Add((join, expr[segStart..i].Trim()));
                join = c == '&' ? "AND" : "OR";
                i += 2; segStart = i; prevSig = c; continue;
            }
            if (SrsCursor.IsIdentStart(c))
            {
                int s = i;
                while (i < expr.Length && SrsCursor.IsIdentChar(expr[i])) i++;
                string w = expr[s..i];
                if (depth == 0 && prevSig != '.' && (w.Equals("and", StringComparison.OrdinalIgnoreCase) ||
                                                     w.Equals("or", StringComparison.OrdinalIgnoreCase)))
                {
                    parts.Add((join, expr[segStart..s].Trim()));
                    join = w.Equals("and", StringComparison.OrdinalIgnoreCase) ? "AND" : "OR";
                    segStart = i;
                }
                prevSig = 'w';
                continue;
            }
            if (char.IsAsciiDigit(c))
            {
                while (i < expr.Length && (char.IsAsciiLetterOrDigit(expr[i]) || expr[i] == '.')) i++;
                prevSig = '0'; continue;
            }
            if (!char.IsWhiteSpace(c)) prevSig = c;
            i++;
        }
        if (parts.Count == 0) return null;
        parts.Add((join, expr[segStart..].Trim()));
        foreach (var (_, t) in parts)
        {
            if (t.Length == 0) return null;
            try { ExpressionEvaluator.Validate(t); } catch { return null; }
        }
        return parts;
    }

    // ── Variables ──────────────────────────────────────────────────────────

    private static readonly string[] Axes = ["x", "y", "z", "rx", "ry", "rz"];

    public static void NormalizeVariable(ProgramVariable v)
    {
        if (v.IsBoolean == false) v.IsBoolean = null;
        if (v.IsGlobal == false) v.IsGlobal = null;
        if (v.DisplayOnMonitor == false) v.DisplayOnMonitor = null;
        if (v.IsInput == false) v.IsInput = null;
        if (v.IsStopwatch == false) v.IsStopwatch = null;
        if (v.IsPersistent == false) v.IsPersistent = null;
        if (v.IsString == false) v.IsString = null;
        if (v.IsImage == false) v.IsImage = null;
        if (v.IsComputed == false) v.IsComputed = null;

        if (v.Items == null)
        {
            if (v.Points != null) { v.Items = v.Points.Select(ObjectRecord.FromPoint).ToList(); v.ElementType = ListElementType.Point; }
            else if (v.Objects != null) { v.Items = v.Objects; v.ElementType = ListElementType.Record; }
            else if (v.Values is { Count: > 0 }) { v.Items = v.Values.Select(ObjectRecord.FromScalar).ToList(); v.ElementType = ListElementType.Number; }
        }
        v.Points = null; v.Objects = null; v.Values = null;
        if (v.Items != null)
        {
            v.ElementType ??= ListElementType.Record;
            if (v.ElementType == ListElementType.Point)
                for (int i = 0; i < v.Items.Count; i++)
                {
                    var r = v.Items[i] ?? new ObjectRecord();
                    var n = new ObjectRecord();
                    foreach (var a in Axes) n[a] = r.TryGetValue(a, out var d) ? d : 0;
                    foreach (var (k, d) in r) if (!Axes.Contains(k, StringComparer.OrdinalIgnoreCase)) n[k] = d;
                    v.Items[i] = n;
                }
        }

        if (v.IsComputed != true && v.IsString != true && v.Items == null &&
            v.ValueExpression is { } ve && SrsText.TryLiteral(ve.Trim(), false, out var lit) && ve.Trim() == ve)
        {
            v.Value = lit;
            v.ValueExpression = null;
        }
    }

    // ── Empty lists / objects ─────────────────────────────────────────────

    private static readonly HashSet<string> Opaque = ["cncSpec", "displayPageLayout"];

    public static void StripEmpty(JsonObject o, bool isProgram = false, bool isVariable = false)
    {
        foreach (var (k, v) in o.ToList())
        {
            if (Opaque.Contains(k)) continue;
            if (isVariable && k == "items") { if (v is JsonArray ia) foreach (var e in ia) if (e is JsonObject eo) StripEmpty(eo); continue; }
            switch (v)
            {
                case JsonObject co:
                    StripEmpty(co);
                    if (co.Count == 0) o.Remove(k);
                    break;
                case JsonArray ca:
                    foreach (var e in ca)
                        if (e is JsonObject eo) StripEmpty(eo, isVariable: isProgram && k == "variables");
                    if (ca.Count == 0) o.Remove(k);
                    break;
            }
        }
    }

    // ── Labels ─────────────────────────────────────────────────────────────

    public static LabelTable LabelMap(IEnumerable<ProgramStep>? steps) =>
        new(StepTree.Walk(steps).Where(s => s.Type == StepType.Label).ToList());
}

/// <summary>The program's Label steps: name ↔ labelId (first label of a name wins).</summary>
internal sealed class LabelTable(List<ProgramStep> labels)
{
    public string? NameOf(string labelId)
    {
        foreach (var l in labels) if (l.LabelId == labelId && l.LabelName != null) return l.LabelName;
        return null;
    }

    public string? IdOf(string name)
    {
        foreach (var l in labels) if (l.LabelName == name) return l.LabelId;
        foreach (var l in labels) if (string.Equals(l.LabelName, name, StringComparison.OrdinalIgnoreCase)) return l.LabelId;
        return null;
    }

    public bool Has(string name) =>
        labels.Any(l => string.Equals(l.LabelName, name, StringComparison.OrdinalIgnoreCase));
}
