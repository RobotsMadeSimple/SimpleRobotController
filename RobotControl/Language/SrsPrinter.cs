using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using Controller.RobotControl.Programs;
using K = Controller.RobotControl.Programs.FieldKinds;

namespace Controller.RobotControl.Language;

/// <summary>
/// Prints a (normalized) program as Simple Robot Script. Each step is printed in its sugar
/// form — catalog driven, with hand-written forms for flow, assignment, list operations and
/// rows — and then <b>checked</b>: the statement is parsed back on its own and compared with
/// the step's JSON. Whatever the sugar did not reproduce goes into <c>@with {…}</c> (a deep
/// merge, with null removing a value), so every field round-trips by construction. When a
/// sugar form cannot be read back at all, the step falls back to its bare keyword plus
/// <c>@with</c>.
/// </summary>
internal sealed class SrsPrinter
{
    private readonly NameResolver _names;
    private readonly LabelTable _labels;
    private readonly Dictionary<string, ProgramVariable> _vars;
    private readonly bool _includeIds;
    private readonly StringBuilder _sb = new();
    private readonly List<ProgramTextMapEntry> _map = [];
    private int _line = 1;

    private const string Indent = "  ";

    private SrsPrinter(BuiltProgram p, NameResolver names, bool includeIds)
    {
        _names = names;
        _labels = ProgramNormalizer.LabelMap(p.Steps);
        _vars = new Dictionary<string, ProgramVariable>(StringComparer.Ordinal);
        foreach (var v in p.Variables ?? []) _vars.TryAdd(v.Name, v);
        _includeIds = includeIds;
    }

    public static FormatProgramTextResult Print(BuiltProgram normalized, NameResolver names, bool includeIds, bool includeHeader)
    {
        var pr = new SrsPrinter(normalized, names, includeIds);
        pr.Emit($"#!srs {StepCatalog.LanguageVersion}", 0);
        if (includeHeader)
        {
            var h = SrsJson.ToJson(normalized);
            foreach (var k in new[] { "id", "steps", "variables", "lastUpdatedUnixMs" }) h.Remove(k);
            pr.Emit("@program " + SrsJson.Write(h), 0);
        }
        var vars = normalized.Variables ?? [];
        foreach (var v in vars) pr.PrintVariable(v);
        if (vars.Count > 0 && normalized.Steps.Count > 0) pr.Emit("", 0);
        pr.PrintSteps(normalized.Steps, 0, "steps");
        return new FormatProgramTextResult
        {
            Text = pr._sb.ToString(),
            LanguageVersion = StepCatalog.LanguageVersion,
            Map = pr._map,
        };
    }

    private void Emit(string text, int indent)
    {
        if (text.Length > 0) for (int i = 0; i < indent; i++) _sb.Append(Indent);
        _sb.Append(text).Append('\n');
        _line++;
        _lastBlank = text.Length == 0;
    }

    private bool _lastBlank;

    // ── Values ─────────────────────────────────────────────────────────────

    private static bool Parses(string e)
    {
        try { ExpressionEvaluator.Validate(e); return !string.IsNullOrWhiteSpace(e); }
        catch { return false; }
    }

    /// <summary>An expression for a number slot after <c>name=</c>: literal-looking or already
    /// parenthesised expressions are wrapped so they read back as expressions.</summary>
    private static string? NamedExpr(string? e, bool ms = false, bool boolSlot = false)
    {
        if (e == null || !RawOk(e, true)) return null;
        bool looksLiteral = SrsText.TryLiteral(e, ms, out _) || (boolSlot && SrsText.IsBoolWord(e, out _));
        if (looksLiteral || SrsText.IsWrapped(e)) return SrsText.RawSafe("(" + e + ")") ? "(" + e + ")" : null;
        return e;
    }

    /// <summary>An expression for a positional slot: a <c>$var</c> token bare, anything else in parentheses.</summary>
    private static string? PositionalExpr(string? e)
    {
        if (e == null || !RawOk(e, false)) return null;
        if (SrsText.IsVarRefToken(e)) return e;
        var w = "(" + e + ")";
        return SrsText.RawSafe(w, false) ? w : null;
    }

    private static bool RawOk(string e, bool stopComma) => SrsText.RawSafe(e, stopComma) && Parses(e);

    /// <summary>A value that is an expression or a template: raw when it reads back as the
    /// same expression, otherwise quoted.</summary>
    private static string? ExprOrTemplate(string? v, bool stopComma)
    {
        if (v == null) return null;
        if (RawOk(v, stopComma) && !IsLoneString(v) && LooksLikeExpression(v)) return v;
        return SrsText.Quotable(v) ? SrsText.Quote(v) : null;
    }

    /// <summary>Raw is kept for what reads as an expression — a number, on/off, or anything with a
    /// <c>$</c>, an operator or a call; a bare word (<c>g</c>, <c>P1</c>) is written as text.</summary>
    private static bool LooksLikeExpression(string v) =>
        SrsText.TryLiteral(v, false, out _) || SrsText.IsBoolWord(v, out _) ||
        v.Any(ch => ch is '$' or '(' or '+' or '-' or '*' or '/' or '<' or '>' or '=' or '!' or '%' or '^' or '?' or '&' or '|');

    private static bool IsLoneString(string v)
    {
        if (v.Length < 2 || v[0] != '"') return false;
        var c = new SrsCursor(new SrsLine { Text = v, Map = new int[v.Length + 1] });
        return c.TryString(out _) != null && c.AtEnd;
    }

    private static string? Var(string? name) => name != null && SrsCursor.IsIdent(name) ? "$" + name : null;

    private static string? Str(string? s) => s != null && SrsText.Quotable(s) ? SrsText.Quote(s) : null;

    private static string? NameText(string? s) => string.IsNullOrEmpty(s) || !SrsText.Quotable(s) ? null : SrsText.Name(s);

    private static string Expr(ProgramStep s, string key) =>
        s.Expressions != null && s.Expressions.TryGetValue(key, out var e) ? e : null!;

    private static bool HasExpr(ProgramStep s, string? key) => key != null && s.Expressions != null && s.Expressions.ContainsKey(key);

    private static JsonNode? Prop(JsonObject j, string key) => j.TryGetPropertyValue(key, out var v) ? v : null;

    private static string? NumberText(JsonNode? v, bool ms)
    {
        if (v is not JsonValue jv || !jv.TryGetValue<double>(out var d) || !double.IsFinite(d)) return null;
        return ms ? SrsText.Ms(d) : SrsText.Num(d);
    }

    private string? CondText(ConditionGroup? g, bool stopComma)
    {
        if (g?.Items == null || g.Items.Count == 0) return null;
        var sb = new StringBuilder();
        for (int i = 0; i < g.Items.Count; i++)
        {
            var it = g.Items[i];
            if (string.IsNullOrWhiteSpace(it.Expr)) return null;
            if (i > 0) sb.Append(string.Equals(it.Join, "OR", StringComparison.OrdinalIgnoreCase) ? " or " : " and ");
            sb.Append(it.Expr);
        }
        var t = sb.ToString();
        return RawOk(t, stopComma) ? t : null;
    }

    private string? RefText(SrsRefKind kind, string? id, string? name)
    {
        var n = _names.NameOf(kind, id) ?? name;
        if (string.IsNullOrEmpty(n) && kind is SrsRefKind.Device or SrsRefKind.Camera) n = id;
        return NameText(n);
    }

    private string? TargetText(ProgramStep s, bool via)
    {
        if (!via)
        {
            if (s.GridPoint is { } g && !string.IsNullOrEmpty(g.GridId))
            {
                var name = NameText(_names.NameOf(SrsRefKind.Grid, g.GridId) ?? g.GridId);
                if (name == null) return null;
                string? Idx(double? lit, string key)
                {
                    if (HasExpr(s, key)) return NamedExpr(Expr(s, key));
                    return lit is { } d && double.IsFinite(d) ? SrsText.Num(d) : null;
                }
                if (g.UseGridIndex)
                {
                    var i = Idx(g.GridIndex, "gridGridIndex");
                    return i == null ? null : $"grid {name}[{i}]";
                }
                var r = Idx(g.RowIndex, "gridRowIndex");
                var c = Idx(g.ColIndex, "gridColIndex");
                return r == null || c == null ? null : $"grid {name}[{r}, {c}]";
            }
            if (s.StackPoint is { } st && !string.IsNullOrEmpty(st.StackId))
            {
                var name = NameText(_names.NameOf(SrsRefKind.Stack, st.StackId) ?? st.StackId);
                var i = HasExpr(s, "stackIndex") ? NamedExpr(Expr(s, "stackIndex"))
                      : st.Index is { } d && double.IsFinite(d) ? SrsText.Num(d) : null;
                return name == null || i == null ? null : $"stack {name}[{i}]";
            }
        }
        var expr = via ? s.ViaPointNameExpr : s.PointNameExpr;
        if (!string.IsNullOrEmpty(expr))
        {
            if (SrsText.IsVarRefToken(expr) && Parses(expr)) return expr;
            if ((expr.Contains('{') || expr.Contains('$')) && SrsText.Quotable(expr)) return SrsText.Quote(expr);
            return null;
        }
        var pn = via ? (_names.NameOf(SrsRefKind.Point, s.ViaPointId) ?? s.ViaPointName)
                     : (_names.NameOf(SrsRefKind.Point, s.PointId) ?? s.PointName);
        if (string.IsNullOrEmpty(pn) || pn.Contains('{') || pn.Contains('$')) return null;
        return NameText(pn);
    }

    private static string? AxesText(ProgramStep s, JsonObject j, FieldSpec f)
    {
        var parts = new List<string>();
        foreach (var prop in f.StoredProps)
        {
            string axis = prop[^2..] is "RX" or "RY" or "RZ" ? prop[^2..].ToLowerInvariant() : prop[^1..].ToLowerInvariant();
            if (HasExpr(s, prop))
            {
                var e = NamedExpr(Expr(s, prop));
                if (e != null) parts.Add($"{axis}={e}");
            }
            else if (NumberText(Prop(j, prop), false) is { } n) parts.Add($"{axis}={n}");
        }
        return parts.Count == 0 ? null : "(" + string.Join(", ", parts) + ")";
    }

    // ── Generic catalog fields ─────────────────────────────────────────────

    private static string? ShowIfText(JsonObject j, StepSpec spec, string field)
    {
        var v = Prop(j, field);
        if (v is JsonValue jv)
        {
            if (jv.TryGetValue<bool>(out var b)) return b ? "true" : "false";
            return jv.ToString();
        }
        var f = spec.Fields.FirstOrDefault(x => x.Key == field);
        return f?.Default switch { null => null, bool b => b ? "true" : "false", var d => Convert.ToString(d, SrsText.Inv) };
    }

    private static bool Visible(JsonObject j, StepSpec spec, FieldSpec f) =>
        f.ShowIf == null || f.ShowIf.In.Contains(ShowIfText(j, spec, f.ShowIf.Field) ?? "");

    private static bool Settable(FieldSpec f) => f.Kind is not (K.Steps or K.Branches or K.Cases or K.Json or K.Rows or K.MappingOut or K.ExprMap or K.PluginRef);

    private string? EnumText(ProgramStep s, FieldSpec f, JsonObject j)
    {
        if (Prop(j, f.Key) is not JsonValue v || !v.TryGetValue<string>(out var val)) return null;
        if (SrsParser.EnumWords.TryGetValue((s.Type, f.Key), out var words))
        {
            foreach (var (value, word) in words) if (value == val) return word;
            return null;
        }
        if (f.Options != null && !f.Options.Any(o => Convert.ToString(o.Value, SrsText.Inv) == val)) return null;
        return SrsCursor.IsIdent(val) ? val : SrsText.Quotable(val) ? SrsText.Quote(val) : null;
    }

    /// <summary>The text for one field's value, or null when it has none or cannot be written.</summary>
    private string? FieldText(ProgramStep s, JsonObject j, StepSpec spec, FieldSpec f, bool positional)
    {
        switch (f.Kind)
        {
            case K.Number:
            case K.Integer:
            {
                bool ms = f.Unit == "ms";
                if (HasExpr(s, f.ExprKey)) return positional ? PositionalExpr(Expr(s, f.ExprKey!)) : NamedExpr(Expr(s, f.ExprKey!), ms);
                var v = Prop(j, f.Key);
                if (v == null) return null;
                if (s.Type == StepType.SetJointPosition && f.Key == "jointIndex")
                    return v is JsonValue ji && ji.TryGetValue<int>(out var idx) && idx >= 0 ? $"J{idx + 1}" : null;
                var t = NumberText(v, ms);
                if (t != null && s.Type == StepType.RotaryIndex && f.Key == "rotaryDirection" && !t.StartsWith('-')) t = "+" + t;
                return t;
            }
            case K.Bool:
            {
                if (HasExpr(s, f.ExprKey)) return positional ? PositionalExpr(Expr(s, f.ExprKey!)) : NamedExpr(Expr(s, f.ExprKey!), boolSlot: true);
                return Prop(j, f.Key) is JsonValue v && v.TryGetValue<bool>(out var b) ? (b ? "on" : "off") : null;
            }
            case K.Enum: return EnumText(s, f, j);
            case K.Text:
            {
                if (Prop(j, f.Key) is not JsonValue v || !v.TryGetValue<string>(out var t)) return null;
                bool bare = f.Key is "labelName" or "errorCode" && SrsCursor.IsIdent(t);
                return bare ? NameText(t) : Str(t);
            }
            case K.Template:
                return Prop(j, f.Key) is JsonValue tv && tv.TryGetValue<string>(out var tt) ? Str(tt) : null;
            case K.Expr:
                return Prop(j, f.Key) is JsonValue ev && ev.TryGetValue<string>(out var et) ? (positional ? PositionalExpr(et) : ExprOrTemplate(et, true)) : null;
            case K.Condition:
            {
                var g = f.Key switch
                {
                    "moveStopCondition" => s.MoveStopCondition, "waitCondition" => s.WaitCondition,
                    "loopWhileCondition" => s.LoopWhileCondition, "condition" => s.Condition, _ => null,
                };
                return CondText(g, stopComma: true);
            }
            case K.VarWrite:
            case K.VarRead:
            case K.ListVar:
                return Prop(j, f.Key) is JsonValue vv && vv.TryGetValue<string>(out var vn) ? Var(vn) : null;
            case K.PointRef:   return RefText(SrsRefKind.Point, s.PointId, s.PointName);
            case K.ToolRef:    return RefText(SrsRefKind.Tool, s.ToolId, s.ToolName);
            case K.LocalRef:   return RefText(SrsRefKind.Local, s.LocalId, s.LocalName);
            case K.RoutineRef: return RefText(SrsRefKind.Routine, s.RoutineId, s.RoutineName);
            case K.ProgramRef: return RefText(SrsRefKind.Program, s.BackgroundProgramId, s.BackgroundProgramName);
            case K.VisionRef:  return RefText(SrsRefKind.Vision, s.VisionProgramId, s.VisionProgramName);
            case K.DeviceRef:
            case K.CameraRef:
            {
                if (Prop(j, f.Key) is not JsonValue dv || !dv.TryGetValue<string>(out var id) || id.Length == 0) return null;
                return RefText(f.Kind == K.DeviceRef ? SrsRefKind.Device : SrsRefKind.Camera, id, null);
            }
            case K.Target: return TargetText(s, via: f.Key == "via") ?? (f.Key == "via" ? null : "here");
            case K.Axes6: return AxesText(s, j, f);
            case K.TextList:
            {
                var list = f.Key == "catchCodes" ? s.CatchCodes : f.Key == "promptButtons" ? s.PromptButtons : null;
                if (list == null) return null;
                var parts = new List<string>();
                foreach (var x in list)
                {
                    var t = x == null ? null : f.Key == "catchCodes" ? NameText(x) : Str(x);
                    if (t == null) return null;
                    parts.Add(t);
                }
                return "(" + string.Join(", ", parts) + ")";
            }
            default: return null;
        }
    }

    /// <summary>
    /// <c>Keyword pos pos, name=value, …</c> from the catalog. <paramref name="skip"/> holds args
    /// the caller already wrote as sugar; <paramref name="prefix"/> replaces the positionals.
    /// </summary>
    private string Generic(ProgramStep s, JsonObject j, StepSpec spec, string keyword,
                           ISet<string>? skip = null, string? prefix = null, string posSep = " ", string bareSep = " ")
    {
        var head = new StringBuilder(keyword);
        if (prefix != null) head.Append(' ').Append(prefix);
        else
        {
            var pos = new List<string>();
            foreach (var f in spec.Fields.Where(f => f.Position != null && Settable(f)).OrderBy(f => f.Position))
            {
                if (!Visible(j, spec, f)) continue;
                var t = FieldText(s, j, spec, f, positional: true);
                if (t == null) break;
                pos.Add(t);
            }
            if (s.Type == StepType.SetOutput && s.OutputAction == "toggle" && !HasExpr(s, "outputValue") && s.OutputValue == null &&
                pos.Count == (s.OutputCard == "io" ? 3 : 2))
            {
                pos.Add("toggle");
                skip = new HashSet<string>(skip ?? new HashSet<string>()) { "action" };
            }
            if (pos.Count > 0) head.Append(' ').Append(string.Join(posSep, pos));
        }
        var named = new List<string>();
        bool blendDone = false;
        foreach (var f in spec.Fields.Where(f => f.Arg.Length > 0 && Settable(f)))
        {
            if (skip != null && skip.Contains(f.Arg)) continue;
            if (f.Key == "blend" && s.Type is StepType.MoveL or StepType.JumpL && s.Blend == true &&
                (s.BlendRadius != null || HasExpr(s, "blendRadius")))
            {
                var r = HasExpr(s, "blendRadius") ? BlendExpr(Expr(s, "blendRadius")) : NumberText(JsonValue.Create(s.BlendRadius), false);
                if (r != null) { named.Add("blend=" + r); blendDone = true; continue; }
            }
            if (f.Key == "blendRadius" && blendDone) continue;
            var t = FieldText(s, j, spec, f, positional: false);
            if (t != null) named.Add($"{f.Arg}={t}");
        }
        if (named.Count > 0) head.Append(prefix == null && head.ToString() == keyword ? bareSep : ", ").Append(string.Join(", ", named));
        return head.ToString();
    }

    /// <summary><c>blend=</c> with an expression: bool-looking texts are wrapped too.</summary>
    private static string? BlendExpr(string e) => NamedExpr(e, boolSlot: true);

    // ── Steps ──────────────────────────────────────────────────────────────

    private void PrintSteps(List<ProgramStep>? steps, int indent, string path)
    {
        if (steps == null) return;
        for (int i = 0; i < steps.Count; i++)
        {
            var s = steps[i];
            if (s == null) continue;
            bool last = i == steps.Count - 1 || steps.Skip(i + 1).All(x => x == null);
            bool isNote = s.Type == StepType.Note && s.Comment != null && (s.NoteStyle == "section" || s.NoteStyle == null);
            if (isNote && i > 0 && !_lastBlank) Emit("", 0);
            bool paragraph = PrintStep(s, indent, $"{path}[{i}]");
            if (paragraph && !last) Emit("", 0);
        }
    }

    /// <summary>One printed form of a step: the statement lines, with markers where bodies go.</summary>
    private sealed class Form
    {
        public readonly List<(string? Text, int Depth, List<ProgramStep>? Body, string? BodyPath)> Lines = [];
        public bool Paragraph;
        public Form Line(string text, int depth = 0) { Lines.Add((text, depth, null, null)); return this; }
        public Form Body(List<ProgramStep>? body, string path) { Lines.Add((null, 1, body ?? [], path)); return this; }
        /// <summary>The body roles (and the elseIfBranches / switchCases arrays) this form writes as text.</summary>
        public HashSet<string> PrintedRoles()
        {
            var set = new HashSet<string>();
            foreach (var l in Lines)
            {
                if (l.BodyPath == null) continue;
                int dot = l.BodyPath.LastIndexOf('.');
                var tail = l.BodyPath[(dot + 1)..];
                if (tail == "steps")
                {
                    var before = l.BodyPath[..dot];
                    tail = before[(before.LastIndexOf('.') + 1)..].Split('[')[0];
                }
                set.Add(tail);
            }
            // The If / Switch sugar states every branch (possibly none) as elif / case lines.
            if (BranchRole != null) set.Add(BranchRole);
            return set;
        }
        public string? BranchRole;
        public string Snippet() => string.Join("\n", Lines.Where(l => l.Text != null).Select(l => new string(' ', l.Depth * 2) + l.Text));
    }

    /// <returns>True when the step printed as a comment paragraph (a blank line must follow).</returns>
    private bool PrintStep(ProgramStep s, int indent, string path)
    {
        var want = SrsJson.ToJson(s);
        var attrs = new List<string>();
        var comments = new List<string>();
        if (s.Type != StepType.Note && s.Comment != null && !s.Comment.Contains('\r'))
        {
            comments.AddRange(s.Comment.Split('\n').Select(l => l.Length == 0 ? "#" : "# " + l));
            want.Remove("comment");
        }
        if (_includeIds && !string.IsNullOrEmpty(s.Id) && SrsText.Quotable(s.Id)) attrs.Add("@id " + SrsText.Quote(s.Id));
        if (s.Name != null && SrsText.Quotable(s.Name)) { attrs.Add("@name " + SrsText.Quote(s.Name)); want.Remove("name"); }
        if (s.Enabled == false) { attrs.Add("@off"); want.Remove("enabled"); }

        bool branchesInWith = false;
        Form? form = null;
        JsonObject? with = null;
        foreach (var candidate in new[] { 0, 1, 2 })
        {
            // 0: full sugar, 1: sugar without elif/case lines (they go in @with), 2: the bare keyword.
            if (candidate == 1 && s.ElseIfBranches is not { Count: > 0 } && s.SwitchCases is not { Count: > 0 }) continue;
            var f = candidate == 2 ? Fallback(s, path) : Sugar(s, want, path, noBranches: candidate == 1);
            if (f == null) continue;
            var (ok, diff, missingArg) = Check(s, want, f);
            if (!ok) continue;
            bool branchMismatch = diff.ContainsKey("elseIfBranches") || diff.ContainsKey("switchCases");
            if (branchMismatch && candidate == 0) continue;
            if (branchMismatch)
            {
                // the branches travel whole (with their bodies and ids)
                var full = SrsJson.ToJson(s);
                if (diff.ContainsKey("elseIfBranches")) diff["elseIfBranches"] = full["elseIfBranches"]?.DeepClone();
                if (diff.ContainsKey("switchCases")) diff["switchCases"] = full["switchCases"]?.DeepClone();
                branchesInWith = true;
            }
            form = f;
            with = diff.Count > 0 || missingArg ? diff : null;
            break;
        }
        if (form == null) throw new InvalidOperationException($"SRS printer: no form reads back for step {s.Id} ({s.Type})");

        foreach (var c in comments) Emit(c, indent);
        foreach (var a in attrs) Emit(a, indent);
        if (with != null) Emit("@with " + SrsJson.Write(with), indent);

        int start = _line;
        foreach (var (text, depth, body, bodyPath) in form.Lines)
        {
            if (text != null) { Emit(text, indent + depth); continue; }
            if (branchesInWith && bodyPath != null && (bodyPath.Contains(".elseIfBranches[") || bodyPath.Contains(".switchCases["))) continue;
            PrintSteps(body, indent + 1, bodyPath!);
        }
        _map.Add(new ProgramTextMapEntry { StepId = s.Id, StepPath = path, Line = start, EndLine = _line - 1 });
        return form.Paragraph;
    }

    /// <summary>Parses the form's statement lines back on their own and diffs the result
    /// against the step. Not ok when the form does not read back as exactly one clean step.</summary>
    private (bool Ok, JsonObject Diff, bool MissingArg) Check(ProgramStep s, JsonObject want, Form f)
    {
        var doc = SrsParser.Parse(f.Snippet(), _names);
        if (doc.Steps.Count != 1 || doc.Vars.Count > 0) return (false, new(), false);
        bool missingArg = false;
        foreach (var p in doc.Problems.Where(p => p.Severity == "error"))
        {
            if (p.Code == TextProblemCodes.MissingArgument) { missingArg = true; continue; }
            return (false, new(), false);
        }
        var node = doc.Steps[0];
        var problems = new List<TextProblem>();
        if (SrsLanguage.CheckStringAssignment(node, _vars.ToDictionary(kv => kv.Key, kv => kv.Value.IsString == true), problems) > 0)
            return (false, new(), false);
        var got = SrsLanguage.BuildStep(node, null, problems, null);
        if (got.Type == StepType.GoToLabel) SrsLanguage.ResolveGoto(got, node, _labels, problems, null);
        var have = SrsJson.ToJson(got);
        var w = (JsonObject)want.DeepClone();
        var roles = f.PrintedRoles();
        Strip(w, s, roles);
        Strip(have, s, roles);
        return (true, SrsJson.Diff(w, have), missingArg);
    }

    private static readonly string[] BodyRoles =
        [StepTree.Roles.LoopSteps, StepTree.Roles.IfSteps, StepTree.Roles.ElseSteps, StepTree.Roles.SwitchDefault,
         StepTree.Roles.TrySteps, StepTree.Roles.CatchSteps];

    private static readonly string[] Conditions = ["condition", "loopWhileCondition", "waitCondition", "moveStopCondition"];

    /// <summary>Removes what the text does not carry: ids (reconciled), nested bodies (printed
    /// as text), a Label's labelId (reconciled) and literals shadowed by an expression.</summary>
    private static void Strip(JsonObject j, ProgramStep s, ISet<string> printedRoles)
    {
        j.Remove("id");
        foreach (var r in BodyRoles) if (printedRoles.Contains(r)) j.Remove(r);
        foreach (var arr in new[] { "elseIfBranches", "switchCases" })
            if (printedRoles.Contains(arr) && j[arr] is JsonArray a)
                foreach (var b in a)
                    if (b is JsonObject bo)
                    {
                        bo.Remove("id"); bo.Remove("steps");
                        if (bo["condition"] is JsonObject bc) StripItems(bc);
                    }
        foreach (var c in Conditions) if (j[c] is JsonObject co) StripItems(co);
        if (s.Type == StepType.Label) j.Remove("labelId");
        if (s.Expressions != null)
            foreach (var k in s.Expressions.Keys)
            {
                switch (k)
                {
                    case "gridRowIndex": (j["gridPoint"] as JsonObject)?.Remove("rowIndex"); break;
                    case "gridColIndex": (j["gridPoint"] as JsonObject)?.Remove("colIndex"); break;
                    case "gridGridIndex": (j["gridPoint"] as JsonObject)?.Remove("gridIndex"); break;
                    case "stackIndex": (j["stackPoint"] as JsonObject)?.Remove("index"); break;
                    default:
                        if (SrsJson.StepProps.Contains(k) && k is not ("expressions" or "id" or "type")) j.Remove(k);
                        break;
                }
            }
        if (j["expressions"] is JsonObject ex && ex.Count == 0) j.Remove("expressions");
    }

    private static void StripItems(JsonObject cond)
    {
        if (cond["items"] is JsonArray items)
            foreach (var it in items) (it as JsonObject)?.Remove("id");
    }

    // ── Sugar forms ───────────────────────────────────────────────────────

    private Form? Sugar(ProgramStep s, JsonObject j, string path, bool noBranches)
    {
        var spec = StepCatalog.Get(s.Type);
        var f = new Form();
        switch (s.Type)
        {
            case StepType.Note:
            {
                if (s.Comment == null || s.Comment.Contains('\r')) return null;
                if (s.NoteStyle == "section")
                {
                    if (s.Comment.Contains('\n')) return null;
                    return f.Line(s.Comment.Length == 0 ? "##" : "## " + s.Comment);
                }
                if (s.NoteStyle != null) return null;
                foreach (var l in s.Comment.Split('\n')) f.Line(l.Length == 0 ? "#" : "# " + l);
                f.Paragraph = true;
                return f;
            }
            case StepType.IfCondition:
            {
                var c = CondText(s.Condition, false);
                if (!noBranches) f.BranchRole = "elseIfBranches";
                f.Line(c == null ? "if" : "if " + c);
                f.Body(s.IfSteps, path + ".ifSteps");
                if (!noBranches)
                    for (int i = 0; i < (s.ElseIfBranches?.Count ?? 0); i++)
                    {
                        var b = s.ElseIfBranches![i];
                        var bc = CondText(b.Condition, false);
                        if (bc == null) return null;
                        f.Line("elif " + bc);
                        f.Body(b.Steps, $"{path}.elseIfBranches[{i}].steps");
                    }
                if (s.ElseSteps is { Count: > 0 }) { f.Line("else"); f.Body(s.ElseSteps, path + ".elseSteps"); }
                return f.Line("end");
            }
            case StepType.Loop:
            {
                string head;
                switch (s.LoopMode)
                {
                    case "count":
                    {
                        string count = HasExpr(s, "loopCount") ? PositionalExpr(Expr(s, "loopCount")) ?? ""
                                     : s.LoopCount == 0 ? "forever" : s.LoopCount is { } n ? SrsText.Num(n) : "";
                        head = "repeat" + (count.Length > 0 ? " " + count : "");
                        if (Var(s.ForEachIndexVariableName) is { } iv) head += " as " + iv;
                        break;
                    }
                    case "while":
                    {
                        var c = CondText(s.LoopWhileCondition, false);
                        if (c == null) return null;
                        head = "while " + c;
                        break;
                    }
                    case "forEach":
                    {
                        var list = Var(s.ForEachVariableName);
                        if (list == null) return null;
                        var v = s.ForEachValueVariableName == null ? "_" : Var(s.ForEachValueVariableName);
                        if (v == null) return null;
                        var iv = s.ForEachIndexVariableName == null ? null : Var(s.ForEachIndexVariableName);
                        if (v == "_" && iv == null) v = "_";
                        head = $"for {v}" + (iv != null ? $", {iv}" : "") + $" in {list}";
                        break;
                    }
                    default: return null;
                }
                f.Line(head);
                f.Body(s.LoopSteps, path + ".loopSteps");
                return f.Line("end");
            }
            case StepType.Switch:
            {
                var e = s.SwitchExpr != null && RawOk(s.SwitchExpr, false) ? s.SwitchExpr : null;
                if (!noBranches) f.BranchRole = "switchCases";
                f.Line(e == null ? "switch" : "switch " + e);
                if (!noBranches)
                    for (int i = 0; i < (s.SwitchCases?.Count ?? 0); i++)
                    {
                        var c = s.SwitchCases![i];
                        if (c.Values == null || c.Values.Count == 0 || c.Values.Any(v => v == null || !RawOk(v, true))) return null;
                        f.Line("case " + string.Join(", ", c.Values));
                        f.Body(c.Steps, $"{path}.switchCases[{i}].steps");
                    }
                if (s.SwitchDefault is { Count: > 0 }) { f.Line("default"); f.Body(s.SwitchDefault, path + ".switchDefault"); }
                return f.Line("end");
            }
            case StepType.Try:
            {
                var head = new List<string>();
                if (s.TryRetries is { } r) head.Add("retry=" + SrsText.Num(r));
                if (s.TryRetryDelayMs is { } d) head.Add("delay=" + SrsText.Ms(d));
                f.Line("try" + (head.Count > 0 ? " " + string.Join(", ", head) : ""));
                f.Body(s.TrySteps, path + ".trySteps");
                var catchParts = new List<string>();
                var msg = Var(s.ErrorVariableName);
                if (Var(s.ErrorCodeVariableName) is { } cv) catchParts.Add("code=" + cv);
                if (s.CatchCodes != null && FieldText(s, j, spec!, spec!.Fields.First(x => x.Key == "catchCodes"), false) is { } only)
                    catchParts.Add("only=" + only);
                if (s.CatchSteps is { Count: > 0 } || msg != null || catchParts.Count > 0)
                {
                    var line = "catch" + (msg != null ? " " + msg : "");
                    if (catchParts.Count > 0) line += (msg != null ? ", " : ", ") + string.Join(", ", catchParts);
                    f.Line(line);
                    f.Body(s.CatchSteps, path + ".catchSteps");
                }
                return f.Line("end");
            }
            case StepType.LoopControl:
                return s.LoopControl is "break" or "continue" ? f.Line(s.LoopControl) : null;
            case StepType.GoToLabel:
            {
                var name = (s.LabelId != null ? _labels.NameOf(s.LabelId) : null) ?? s.LabelName;
                var t = NameText(name);
                return f.Line(t == null ? "goto" : "goto " + t);
            }
            case StepType.CallRoutine:
            {
                var name = RefText(SrsRefKind.Routine, s.RoutineId, s.RoutineName);
                if (name == null) return f.Line("call");
                var sb = new StringBuilder("call ").Append(name);
                if (s.RoutineArgs is { Count: > 0 })
                {
                    var args = new List<string>();
                    foreach (var (k, v) in s.RoutineArgs)
                    {
                        if (!SrsCursor.IsIdent(k)) { args = null; break; }
                        var t = ExprOrTemplate(v, true);
                        if (t == null) { args = null; break; }
                        args.Add($"{k}={t}");
                    }
                    if (args != null) sb.Append('(').Append(string.Join(", ", args)).Append(')');
                }
                if (s.RoutineOutputs is { Count: > 0 })
                {
                    var outs = new List<string>();
                    foreach (var o in s.RoutineOutputs)
                    {
                        if (o == null || !SrsCursor.IsIdent(o.Key) || Var(o.VariableName) is not { } ov) { outs = null; break; }
                        outs.Add($"out {o.Key}={ov}");
                    }
                    if (outs != null) sb.Append(", ").Append(string.Join(", ", outs));
                }
                return f.Line(sb.ToString());
            }
            case StepType.StatusUpdate:
            {
                string? word = s.StatusSeverity switch { "Warning" => "warning", "Error" => "error", "Info" => null, _ => null };
                string field = s.StatusSeverity switch { "Warning" => "statusWarning", "Error" => "statusError", _ => "statusMessage" };
                var msgText = Str(Prop(j, field)?.GetValue<string>());
                var skip = new HashSet<string>();
                if (field == "statusWarning") skip.Add("warning");
                if (field == "statusError") skip.Add("error");
                var prefix = string.Join(" ", new[] { word, msgText }.Where(x => x != null));
                return f.Line(Generic(s, j, spec!, "status", skip, prefix: prefix.Length > 0 ? prefix : null));
            }
            case StepType.Plugin:
            {
                string? id = NameText(s.PluginId) is { } pid && NameText(s.PluginStepId) is { } sid ? $"{pid}.{sid}" : null;
                f.Line(Generic(s, j, spec!, "plugin", prefix: id, bareSep: ", "));
                var rows = new List<string>();
                bool okRows = true;
                foreach (var (k, v) in s.PluginParams ?? [])
                {
                    var kt = NameText(k); var vt = ExprOrTemplate(v, false);
                    if (kt == null || vt == null) { okRows = false; break; }
                    rows.Add($"{kt} = {vt}");
                }
                var outs = new List<string>();
                foreach (var o in s.PluginOutputs ?? [])
                {
                    var kt = NameText(o?.Key); var vt = Var(o?.VariableName);
                    if (kt == null || vt == null) { okRows = false; break; }
                    outs.Add($"{kt} -> {vt}");
                }
                if (!okRows) { rows.Clear(); outs.Clear(); }
                return Rows(f, rows.Concat(outs).ToList());
            }
            case StepType.Unknown:
                return f.Line(s.UnknownStepType != null && Str(s.UnknownStepType) is { } u ? "unknown " + u : "unknown");
            case StepType.SetVariable:
            {
                var name = Var(s.VariableName);
                if (name == null || s.VariableExpr == null) return null;
                bool isString = s.VariableName != null && _vars.TryGetValue(s.VariableName, out var vd) && vd.IsString == true;
                var rhs = isString ? Str(s.VariableExpr) : ExprOrTemplate(s.VariableExpr, false);
                return rhs == null ? null : f.Line($"{name} = {rhs}");
            }
            case StepType.ListOp: return ListOp(s, f);
            case StepType.Wait:
            {
                var skip = new HashSet<string> { "mode" };
                if (s.WaitMode == "condition")
                {
                    var c = CondText(s.WaitCondition, true);
                    if (c == null) return f.Line(Generic(s, j, spec!, "Wait", new HashSet<string> { "until" }, prefix: null));
                    skip.Add("until");
                    return f.Line(Generic(s, j, spec!, "Wait", skip, prefix: "until " + c));
                }
                if (s.WaitMode != "duration") return null;
                return f.Line(Generic(s, j, spec!, "Wait", skip));
            }
            case StepType.SavePosition:
                if (s.SavePositionTo == "point" && TargetLike(s) is { } pt)
                    return f.Line(Generic(s, j, spec!, "SavePosition", new HashSet<string> { "to", "point" }, prefix: "point " + pt));
                return f.Line(Generic(s, j, spec!, "SavePosition"));
            case StepType.GripperAction:
                if (s.GripperAction == "GripTo")
                {
                    var g = HasExpr(s, "gripTargetMm") ? PositionalExpr(Expr(s, "gripTargetMm")) : NumberText(Prop(j, "gripTargetMm"), false);
                    var skip = new HashSet<string>();
                    if (g != null) skip.Add("to");
                    return f.Line(Generic(s, j, spec!, "GripperAction", skip, prefix: g == null ? "to" : "to " + g));
                }
                return f.Line(Generic(s, j, spec!, "GripperAction"));
            case StepType.SetJointPosition:
                return f.Line(Generic(s, j, spec!, "SetJointPosition", posSep: ", "));
        }
        if (spec == null) return null;
        f.Line(Generic(s, j, spec, spec.Keyword));
        return spec.Body == "mapping" ? Rows(f, MappingRows(s)) : f;
    }

    private string? TargetLike(ProgramStep s) => RefText(SrsRefKind.Point, s.PointId, s.PointName);

    private static Form Rows(Form f, List<string>? rows)
    {
        if (rows == null || rows.Count == 0) return f;
        foreach (var r in rows) f.Line(r, 1);
        return f.Line("end");
    }

    private List<string>? MappingRows(ProgramStep s)
    {
        var rows = new List<string>();
        string? Send(JsonKeyValue? r)
        {
            if (r == null || Str(r.Key) is not { } key) return null;
            if (r.ListVar != null) return r.ImageVar == null && r.Expr == "" && Var(r.ListVar) is { } lv ? $"{key} = list {lv}" : null;
            if (r.ImageVar != null) return r.Expr == "" && Var(r.ImageVar) is { } iv ? $"{key} = image {iv}" : null;
            return RawOk(r.Expr, false) ? $"{key} = {r.Expr}" : null;
        }
        string? Recv(string? key, string? v, string lead) =>
            Str(key) is { } k && Var(v) is { } vv ? $"{lead}{k} -> {vv}" : null;
        switch (s.Type)
        {
            case StepType.RunVision:
                foreach (var o in s.ToolOutputs ?? [])
                {
                    var tool = o == null ? null : NameText(_names.ToolText(s.VisionProgramId, o.ToolId));
                    var val = o == null ? null : NameText(o.Value);
                    var v = Var(o?.Variable);
                    if (tool == null || val == null || v == null) return null;
                    rows.Add($"{tool}.{val} -> {v}");
                }
                return rows;
            case StepType.HttpRequest:
                foreach (var r in s.JsonOutbound ?? []) { var t = Send(r); if (t == null) return null; rows.Add("send " + t); }
                foreach (var r in s.JsonInbound ?? []) { var t = Recv(r?.Key, r?.VariableName, "receive "); if (t == null) return null; rows.Add(t); }
                return rows;
            case StepType.HttpReceive:
                foreach (var r in s.HttpReceiveInbound ?? []) { var t = Recv(r?.Key, r?.VariableName, "receive "); if (t == null) return null; rows.Add(t); }
                return rows;
            case StepType.ExportData:
                foreach (var r in s.ExportColumns ?? []) { var t = Send(r); if (t == null) return null; rows.Add(t); }
                return rows;
        }
        return rows;
    }

    private Form? ListOp(ProgramStep s, Form f)
    {
        var name = Var(s.VariableName);
        if (name == null) return null;
        string? Index()
        {
            if (HasExpr(s, "listIndex")) return NamedExpr(Expr(s, "listIndex"));
            return s.ListIndex is { } i ? SrsText.Num(i) : null;
        }
        string? Value() => s.ListValueExpr != null && RawOk(s.ListValueExpr, true) ? s.ListValueExpr : null;
        string? Fields()
        {
            if (s.ListFields == null || s.ListFields.Count == 0) return null;
            var parts = new List<string>();
            foreach (var (k, v) in s.ListFields)
            {
                if (!SrsCursor.IsIdent(k) || v == null || !RawOk(v, true)) return null;
                parts.Add($"{k}={v}");
            }
            return string.Join(", ", parts);
        }
        switch (s.ListAction)
        {
            case "append":
                return f.Line($"{name}.append({Value() ?? Fields() ?? ""})");
            case "insert":
            {
                var i = Index();
                if (i == null) return null;
                var rest = Value() ?? Fields();
                return f.Line($"{name}.insert({i}{(rest != null ? ", " + rest : "")})");
            }
            case "removeAt":
            {
                var i = Index();
                return i == null ? null : f.Line($"{name}.removeAt({i})");
            }
            case "clear": return f.Line($"{name}.clear()");
            case "reverse": return f.Line($"{name}.reverse()");
            case "copyFrom":
                return Var(s.ListSourceVariable) is { } src ? f.Line($"{name}.copyFrom({src})") : null;
            case "sort":
            {
                var parts = new List<string>();
                if (s.ListSortField != null && NameText(s.ListSortField) is { } by) parts.Add("by=" + by);
                if (s.ListSortDescending == true) parts.Add("desc");
                else if (s.ListSortDescending == false) parts.Add("desc=off");
                return f.Line($"{name}.sort({string.Join(", ", parts)})");
            }
            case "set":
            {
                var i = Index();
                if (i == null) return null;
                var v = Value();
                if (v != null) return f.Line($"{name}[{i}] = {v}");
                var fl = Fields();
                return fl != null ? f.Line($"{name}[{i}] = ({fl})") : null;
            }
            case "setField":
            {
                var i = Index();
                if (i == null || s.ListFields is not { Count: 1 }) return null;
                var (k, v) = s.ListFields.First();
                if (!SrsCursor.IsIdent(k) || v == null || !RawOk(v, false)) return null;
                return f.Line($"{name}[{i}].{k} = {v}");
            }
        }
        return null;
    }

    /// <summary>The bare keyword form, which always reads back (everything else rides in @with).</summary>
    private Form Fallback(ProgramStep s, string path)
    {
        var f = new Form();
        switch (s.Type)
        {
            case StepType.IfCondition:
                f.Line("if").Body(s.IfSteps, path + ".ifSteps");
                if (s.ElseSteps is { Count: > 0 }) f.Line("else").Body(s.ElseSteps, path + ".elseSteps");
                return f.Line("end");
            case StepType.Loop: return f.Line("repeat").Body(s.LoopSteps, path + ".loopSteps").Line("end");
            case StepType.Switch:
                f.Line("switch");
                if (s.SwitchDefault is { Count: > 0 }) f.Line("default").Body(s.SwitchDefault, path + ".switchDefault");
                return f.Line("end");
            case StepType.Try:
                f.Line("try").Body(s.TrySteps, path + ".trySteps");
                if (s.CatchSteps is { Count: > 0 }) f.Line("catch").Body(s.CatchSteps, path + ".catchSteps");
                return f.Line("end");
            case StepType.SetVariable: return f.Line("SetVariable");
            case StepType.ListOp: return f.Line("ListOp");
            case StepType.Note: return f.Line("Note");
            case StepType.Unknown: return f.Line("unknown");
        }
        var spec = StepCatalog.Get(s.Type);
        return f.Line(spec?.Keyword ?? s.Type.ToString());
    }

    // ── Variables ──────────────────────────────────────────────────────────

    private void PrintVariable(ProgramVariable v)
    {
        var want = SrsJson.ToJson(v);
        string? line = VarSugar(v);
        JsonObject? diff = null;
        if (line != null)
        {
            var r = CheckVar(v, want, line);
            if (r != null) diff = r; else line = null;
        }
        if (line == null)
        {
            line = "var " + (SrsCursor.IsIdent(v.Name ?? "") ? v.Name : SrsText.Quote(v.Name ?? ""));
            diff = CheckVar(v, want, line) ?? throw new InvalidOperationException($"SRS printer: variable {v.Name} does not read back");
        }
        if (_includeIds && !string.IsNullOrEmpty(v.Id) && SrsText.Quotable(v.Id)) Emit("@id " + SrsText.Quote(v.Id), 0);
        if (diff.Count > 0) Emit("@with " + SrsJson.Write(diff), 0);
        Emit(line, 0);
    }

    private JsonObject? CheckVar(ProgramVariable v, JsonObject want, string line)
    {
        var doc = SrsParser.Parse(line, _names);
        if (doc.Vars.Count != 1 || doc.Steps.Count > 0 || doc.Problems.Any(p => p.Severity == "error")) return null;
        var have = SrsJson.ToJson(SrsJson.Var(doc.Vars[0].Json));
        var w = (JsonObject)want.DeepClone();
        foreach (var o in new[] { w, have })
        {
            o.Remove("id");
            if (v.ValueExpression != null) o.Remove("value");
        }
        return SrsJson.Diff(w, have);
    }

    private static string? VarSugar(ProgramVariable v)
    {
        if (string.IsNullOrEmpty(v.Name) || !SrsText.Quotable(v.Name)) return null;
        var sb = new StringBuilder();
        if (v.IsGlobal == true) sb.Append("global ");
        if (v.IsPersistent == true) sb.Append("persistent ");
        if (v.IsInput == true) sb.Append("input ");
        if (v.DisplayOnMonitor == true) sb.Append("monitor ");
        bool computed = v.IsComputed == true;
        if (v.ParamMode is ParamModes.In or ParamModes.Out or ParamModes.InOut && !computed)
            sb.Append("param ").Append(v.ParamMode == ParamModes.InOut ? "inout" : v.ParamMode).Append(' ');
        else sb.Append(computed ? "computed " : "var ");
        sb.Append(SrsCursor.IsIdent(v.Name) && !DeclReserved.Contains(v.Name) ? v.Name : SrsText.Quote(v.Name));

        string? init = null;
        if (v.Items != null)
        {
            var elem = v.ElementType ?? ListElementType.Record;
            sb.Append(": list ").Append(elem switch { ListElementType.Number => "number", ListElementType.Boolean => "bool", ListElementType.Point => "point", _ => "record" });
            if (v.Items.Count > 0) init = ListInit(v.Items, elem);
        }
        else if (v.IsImage == true) sb.Append(": image");
        else if (v.IsStopwatch == true) sb.Append(": stopwatch");
        else if (v.IsString == true)
        {
            sb.Append(": string");
            if (v.StringValue != null) init = Str(v.StringValue);
        }
        else
        {
            bool isBool = v.IsBoolean == true;
            if (isBool) sb.Append(": bool");
            if (v.ValueExpression != null) init = RawOk(v.ValueExpression, false) && !(SrsText.TryLiteral(v.ValueExpression, false, out _) || (isBool && SrsText.IsBoolWord(v.ValueExpression, out _)) || IsLoneString(v.ValueExpression)) ? v.ValueExpression : null;
            else if (!computed)
                init = isBool && v.Value == 1 ? "on" : isBool && v.Value == 0 ? "off" : double.IsFinite(v.Value) ? SrsText.Num(v.Value) : null;
        }
        if (init != null) sb.Append(" = ").Append(init);
        if (computed && init == null) return null;
        if (!string.IsNullOrEmpty(v.Description) && !v.Description.Contains('\n') && !v.Description.Contains('\r'))
            sb.Append("  # ").Append(v.Description);
        return sb.ToString();
    }

    private static readonly HashSet<string> DeclReserved = new(StringComparer.OrdinalIgnoreCase)
        { "var", "computed", "param", "global", "persistent", "input", "monitor", "in", "out", "inout" };

    private static readonly string[] Axes = ["x", "y", "z", "rx", "ry", "rz"];

    private static string? ListInit(List<ObjectRecord> items, ListElementType elem)
    {
        var parts = new List<string>();
        foreach (var it in items)
        {
            if (it == null) return null;
            switch (elem)
            {
                case ListElementType.Number:
                case ListElementType.Boolean:
                {
                    if (it.Count != 1 || !it.TryGetValue(ObjectRecord.ScalarKey, out var d) || !double.IsFinite(d) || it.Keys.First() != ObjectRecord.ScalarKey) return null;
                    parts.Add(elem == ListElementType.Boolean && d is 0 or 1 ? (d == 1 ? "on" : "off") : SrsText.Num(d));
                    break;
                }
                case ListElementType.Point:
                {
                    var f = new List<string>();
                    foreach (var (k, d) in it)
                    {
                        if (!double.IsFinite(d) || !SrsCursor.IsIdent(k)) return null;
                        bool axis = Axes.Contains(k);
                        if (axis && d == 0) continue;
                        if (!axis && Axes.Contains(k, StringComparer.OrdinalIgnoreCase)) return null;
                        f.Add($"{k}={SrsText.Num(d)}");
                    }
                    parts.Add("(" + string.Join(", ", f) + ")");
                    break;
                }
                default:
                {
                    var f = new List<string>();
                    foreach (var (k, d) in it)
                    {
                        if (!double.IsFinite(d) || k == null || !SrsText.Quotable(k)) return null;
                        f.Add($"{(SrsCursor.IsIdent(k) ? k : SrsText.Quote(k))}={SrsText.Num(d)}");
                    }
                    parts.Add("(" + string.Join(", ", f) + ")");
                    break;
                }
            }
        }
        return "(" + string.Join(", ", parts) + ")";
    }
}
