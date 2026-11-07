using System.Text.Json;
using System.Text.Json.Nodes;
using Controller.RobotControl.Programs;
using K = Controller.RobotControl.Programs.FieldKinds;

namespace Controller.RobotControl.Language;

/// <summary>
/// Recursive-descent parser for Simple Robot Script (docs/program-text.md). Line based: each
/// logical line is one statement, attribute, declaration, comment or clause; block bodies end
/// with <c>end</c>. Error recovery is per line — a problem is reported, the line is skipped and
/// the block stack is kept. The output is an <see cref="SrsDocument"/> of JSON-carrying nodes;
/// <see cref="SrsLanguage"/> turns it into a <see cref="BuiltProgram"/>.
/// </summary>
internal sealed class SrsParser
{
    private readonly SrsSource _src;
    private readonly List<SrsLine> _lines;
    private readonly NameResolver _names;
    private readonly SrsDocument _doc = new();
    private int _i;

    private SrsParser(string text, NameResolver names)
    {
        _src = new SrsSource(text);
        _lines = _src.Lines();
        _names = names;
    }

    public static SrsDocument Parse(string text, NameResolver names)
    {
        var p = new SrsParser(text, names);
        p.ParseFile();
        return p._doc;
    }

    public SrsSource Source => _src;

    // ── Keywords ───────────────────────────────────────────────────────────

    private static readonly HashSet<string> Clauses = new(StringComparer.OrdinalIgnoreCase)
        { "end", "elif", "else", "case", "default", "catch" };

    private static readonly HashSet<string> DeclWords = new(StringComparer.OrdinalIgnoreCase)
        { "var", "computed", "param", "global", "persistent", "input", "monitor" };

    /// <summary>Statement words → the step type their form builds.</summary>
    private static readonly Dictionary<string, StepType> FlowWords = new(StringComparer.OrdinalIgnoreCase)
    {
        ["if"] = StepType.IfCondition, ["while"] = StepType.Loop, ["repeat"] = StepType.Loop, ["for"] = StepType.Loop,
        ["switch"] = StepType.Switch, ["try"] = StepType.Try, ["break"] = StepType.LoopControl,
        ["continue"] = StepType.LoopControl, ["return"] = StepType.Return, ["exit"] = StepType.EndProgram,
        ["raise"] = StepType.RaiseError, ["label"] = StepType.Label, ["goto"] = StepType.GoToLabel,
        ["call"] = StepType.CallRoutine, ["pause"] = StepType.PauseProgram, ["status"] = StepType.StatusUpdate,
        ["stopwatch"] = StepType.StopwatchControl, ["prompt"] = StepType.Prompt, ["plugin"] = StepType.Plugin,
        ["unknown"] = StepType.Unknown,
    };

    /// <summary>Every word a statement may start with (for "did you mean").</summary>
    public static IEnumerable<string> AllKeywords =>
        FlowWords.Keys.Concat(StepCatalog.Steps.Select(s => s.Type)).Concat(["SetVariable", "ListOp", "Note", "var", "computed", "param"]);

    /// <summary>Printed words of enum options that read better lower-case (both directions).</summary>
    public static readonly Dictionary<(StepType, string), (string Value, string Word)[]> EnumWords = new()
    {
        [(StepType.StopwatchControl, "stopwatchAction")] = [("Start", "start"), ("Stop", "stop"), ("Reset", "reset")],
        [(StepType.GripperAction, "gripperAction")]       = [("Open", "open"), ("Close", "close"), ("GripTo", "to")],
        [(StepType.ConveyorAction, "conveyorAction")]     = [("RunForward", "forward"), ("RunReverse", "reverse"), ("Stop", "stop")],
    };

    // ── Problems ───────────────────────────────────────────────────────────

    private void Problem(string severity, string code, string msg, SrsLine line, int start, int end)
    {
        var span = _src.Span(line.Abs(start), line.Abs(Math.Max(end, start)));
        _doc.Problems.Add(new TextProblem
        {
            Line = span.Line, Col = span.Col, EndLine = span.EndLine, EndCol = span.EndCol,
            Severity = severity, Code = code, Message = msg, Source = "parse",
        });
    }

    private void Error(string code, string msg, SrsLine line, int start, int end) => Problem("error", code, msg, line, start, end);
    private void Warn(string code, string msg, SrsLine line, int start, int end) => Problem("warning", code, msg, line, start, end);
    private void Error(string code, string msg, SrsCursor c, int start) => Error(code, msg, c.Line, start, Math.Max(c.P, start + 1));

    private SrsSpan SpanOf(SrsLine line, int start, int end) => _src.Span(line.Abs(start), line.Abs(Math.Max(end, start)));

    private static string Excerpt(string s) => s.Length > 24 ? s[..24] + "…" : s;

    // ── File and bodies ────────────────────────────────────────────────────

    private void ParseFile()
    {
        if (_lines.Count > 0 && _lines[0].Kind == SrsLineKind.Pragma)
        {
            var t = _lines[0].Text.Trim(); // "srs 1"
            var parts = t.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 2 || parts[1] != StepCatalog.LanguageVersion.ToString())
                Warn(TextProblemCodes.Syntax, $"This text is for language version {(parts.Length > 1 ? parts[1] : "?")}; this controller reads version {StepCatalog.LanguageVersion}.", _lines[0], 0, _lines[0].Text.Length);
            _i = 1;
        }
        while (true)
        {
            var (steps, term) = ParseBody(0);
            _doc.Steps.AddRange(steps);
            if (term == null) break;
            var w = new SrsCursor(term).PeekIdent() ?? "";
            if (w.Equals("end", StringComparison.OrdinalIgnoreCase))
                Error(TextProblemCodes.UnexpectedEnd, "'end' without a block to close", term, 0, term.Text.Length);
            else
                Error(TextProblemCodes.MisplacedClause, $"'{w}' outside the block it belongs to", term, 0, w.Length);
            _i++;
        }
    }

    private sealed class Attrs
    {
        public string? Name; public bool Off; public string? Id; public SrsSpan? IdSpan; public JsonObject? With;
        public SrsLine? First;
        public bool Any => Name != null || Off || Id != null || With != null;
        public void Clear() { Name = null; Off = false; Id = null; IdSpan = null; With = null; First = null; }
    }

    /// <summary>Parses statements until a clause line (end/elif/else/case/default/catch), which
    /// is returned unconsumed, or the end of the file (null).</summary>
    private (List<StepNode> Steps, SrsLine? Terminator) ParseBody(int depth)
    {
        var steps = new List<StepNode>();
        var comments = new List<SrsLine>();
        var attrs = new Attrs();

        void FlushNote()
        {
            if (comments.Count == 0) return;
            var n = new StepNode { Type = StepType.Note, Line = comments[0].Line, EndLine = comments[^1].EndLine };
            n.Json["comment"] = string.Join("\n", comments.Select(c => c.Text));
            n.Span = SpanOf(comments[0], 0, comments[0].Text.Length);
            Attach(n, attrs, null);
            steps.Add(n);
            comments.Clear();
        }

        while (_i < _lines.Count)
        {
            var line = _lines[_i];
            switch (line.Kind)
            {
                case SrsLineKind.Blank:
                    FlushNote(); _i++; continue;
                case SrsLineKind.Pragma:
                    _i++; continue;
                case SrsLineKind.Comment:
                    comments.Add(line); _i++; continue;
                case SrsLineKind.Section:
                {
                    FlushNote();
                    var n = new StepNode { Type = StepType.Note, Line = line.Line, EndLine = line.EndLine };
                    n.Json["comment"] = line.Text;
                    n.Json["noteStyle"] = "section";
                    n.Span = SpanOf(line, 0, line.Text.Length);
                    Attach(n, attrs, null);
                    steps.Add(n);
                    _i++; continue;
                }
            }

            if (line.Error != null)
            {
                Error(TextProblemCodes.Syntax, line.Error, line, IndexOfAbs(line, line.ErrorOffset), IndexOfAbs(line, line.ErrorOffset) + 1);
                _i++; continue;
            }

            var c = new SrsCursor(line);
            if (c.Peek() == '@') { ParseAttr(line, attrs); _i++; continue; }
            var word = c.PeekIdent();
            if (word != null && Clauses.Contains(word))
            {
                FlushNote();
                if (attrs.Any) Warn(TextProblemCodes.Syntax, "This attribute is not followed by a statement, so it is ignored", attrs.First!, 0, attrs.First!.Text.Length);
                return (steps, line);
            }
            if (word != null && DeclWords.Contains(word) && IsDeclaration(c))
            {
                FlushNote();
                ParseDeclaration(line, attrs);
                attrs.Clear();
                _i++; continue;
            }

            _i++;
            var node = ParseStatement(line, depth);
            if (node == null) { comments.Clear(); attrs.Clear(); continue; }
            Attach(node, attrs, comments, line.TrailingComment);
            comments.Clear();
            steps.Add(node);
        }
        FlushNote();
        return (steps, null);
    }

    private static int IndexOfAbs(SrsLine line, int abs)
    {
        for (int i = 0; i < line.Map.Length; i++) if (line.Map[i] >= abs) return i;
        return Math.Max(0, line.Map.Length - 1);
    }

    private void Attach(StepNode n, Attrs a, List<SrsLine>? comments, string? trailing = null)
    {
        var lines = comments?.Select(c => c.Text).ToList() ?? [];
        if (trailing != null) lines.Add(trailing);
        if (lines.Count > 0 && n.Type != StepType.Note) n.Json["comment"] = string.Join("\n", lines);
        if (a.Name != null) n.Json["name"] = a.Name;
        if (a.Off) n.Json["enabled"] = false;
        if (a.Id != null) { n.ExplicitId = a.Id; n.IdSpan = a.IdSpan; }
        if (a.With != null) n.With = a.With;
        a.Clear();
        CheckRequired(n);
    }

    // ── Attributes ─────────────────────────────────────────────────────────

    private void ParseAttr(SrsLine line, Attrs a)
    {
        var c = new SrsCursor(line);
        c.P++; // '@'
        int ws = c.P;
        var w = c.TryIdent()?.ToLowerInvariant();
        a.First ??= line;
        switch (w)
        {
            case "name":
            {
                var s = c.TryString(out _) ?? c.TryIdent();
                if (s == null) { Error(TextProblemCodes.Syntax, "Expected a name in quotes: @name \"…\"", c, c.P); return; }
                a.Name = s;
                break;
            }
            case "off": a.Off = true; break;
            case "on": break;
            case "id":
            {
                int st = c.P;
                var s = c.TryString(out _) ?? c.TryIdent();
                if (s == null) { Error(TextProblemCodes.Syntax, "Expected an id in quotes: @id \"…\"", c, c.P); return; }
                a.Id = s;
                a.IdSpan = SpanOf(line, ws - 1, c.P);
                break;
            }
            case "with":
            case "program":
            {
                c.Ws();
                int st = c.P;
                var json = c.Rest;
                c.P = c.S.Length;
                JsonNode? node;
                try { node = JsonNode.Parse(json); }
                catch (JsonException ex)
                {
                    Error(TextProblemCodes.Syntax, $"@{w}: {ex.Message.Split('.')[0]}", line, st, line.Text.Length);
                    return;
                }
                if (node is not JsonObject o) { Error(TextProblemCodes.Syntax, $"@{w} needs a JSON object: {{…}}", line, st, line.Text.Length); return; }
                if (w == "program") { _doc.ProgramHeader = o; return; }
                if (a.With == null) a.With = o;
                else SrsJson.Merge(a.With, o);
                break;
            }
            default:
                Error(TextProblemCodes.Syntax, $"Unknown attribute '@{w}' — use @name, @off, @id or @with", line, 0, c.P);
                return;
        }
        if (!c.AtEnd) Error(TextProblemCodes.Syntax, $"Unexpected '{Excerpt(c.Rest)}'", line, c.P, line.Text.Length);
    }

    // ── Declarations ───────────────────────────────────────────────────────

    private static bool IsDeclaration(SrsCursor c)
    {
        int save = c.P;
        try
        {
            while (c.PeekIdent() is { } w && DeclWords.Contains(w))
            {
                c.TryIdent();
                if (w.Equals("var", StringComparison.OrdinalIgnoreCase) || w.Equals("computed", StringComparison.OrdinalIgnoreCase) ||
                    w.Equals("param", StringComparison.OrdinalIgnoreCase)) return true;
            }
            return false;
        }
        finally { c.P = save; }
    }

    private void ParseDeclaration(SrsLine line, Attrs a)
    {
        var c = new SrsCursor(line);
        var v = new VarNode { Line = line.Line };
        var j = v.Json;
        bool global = false, persistent = false, input = false, monitor = false;
        string? kw = null, mode = null;
        while (c.PeekIdent() is { } w && DeclWords.Contains(w))
        {
            int ws = c.P;
            c.TryIdent();
            switch (w.ToLowerInvariant())
            {
                case "global":     global = true; break;
                case "persistent": persistent = true; break;
                case "input":      input = true; break;
                case "monitor":    monitor = true; break;
                default: kw = w.ToLowerInvariant(); break;
            }
            if (kw != null) break;
        }
        if (kw == "param")
        {
            int ms = c.P;
            var m = c.TryIdent()?.ToLowerInvariant();
            mode = m switch { "in" => ParamModes.In, "out" => ParamModes.Out, "inout" => ParamModes.InOut, _ => null };
            if (mode == null) { Error(TextProblemCodes.Syntax, "Expected in, out or inout after param", c, ms); return; }
        }
        c.Ws();
        int ns = c.P;
        var name = c.TryString(out _) ?? c.TryIdent();
        if (c.Peek() == '$' && name == null) { c.P++; name = c.TryIdent(); Warn(TextProblemCodes.Syntax, "Declare a variable without the $", line, ns, c.P); }
        if (name == null) { Error(TextProblemCodes.Syntax, "Expected the variable name", c, ns); return; }
        v.Name = name;
        v.Span = SpanOf(line, ns, c.P);
        j["name"] = name;

        string type = "number"; string? elem = null; bool typed = false;
        if (c.TryChar(':'))
        {
            int ts = c.P;
            var t = c.TryIdent()?.ToLowerInvariant();
            typed = true;
            switch (t)
            {
                case "number": case "bool": case "boolean": case "string": case "image": case "stopwatch":
                    type = t == "boolean" ? "bool" : t; break;
                case "list":
                {
                    type = "list";
                    int es = c.P;
                    var e = c.TryIdent()?.ToLowerInvariant();
                    elem = e switch { "number" => "Number", "bool" or "boolean" => "Boolean", "point" => "Point", "record" => "Record", _ => null };
                    if (elem == null) { Error(TextProblemCodes.Syntax, "Expected the element type: list number | bool | point | record", c, es); return; }
                    break;
                }
                default:
                    Error(TextProblemCodes.Syntax, "Expected a type: number, bool, string, image, stopwatch or list …", c, ts); return;
            }
        }

        if (kw == "computed" && (persistent || type is "string" or "image" or "stopwatch" or "list"))
            Error(TextProblemCodes.DeclarationKind, "A computed variable is a formula: it cannot be persistent, a string, an image, a stopwatch or a list", line, 0, line.Text.Length);

        bool hasInit = false;
        if (c.TryChar('='))
        {
            hasInit = true;
            c.Ws();
            int vs = c.P;
            if (!typed && kw != "computed" && c.Peek() == '"') type = "string";
            switch (type)
            {
                case "string":
                {
                    var s = c.TryString(out _);
                    if (s == null) { Error(TextProblemCodes.StringRequired, "A string variable's value is text in quotes", c, vs); c.P = c.S.Length; }
                    else j["stringValue"] = s;
                    break;
                }
                case "list":
                    ParseListInit(c, j, elem!);
                    break;
                case "image": case "stopwatch":
                    Error(TextProblemCodes.Syntax, $"A {type} variable has no initial value", c, vs); c.P = c.S.Length;
                    break;
                default:
                {
                    var (text, st) = c.ScanExpr(stopComma: false);
                    if (text.Length == 0) { Error(TextProblemCodes.Syntax, "Expected a value", c, st); break; }
                    if (kw == "computed") { if (CheckExpr(line, text, st)) j["valueExpression"] = text; }
                    else if (type == "bool" && SrsText.IsBoolWord(text, out bool b)) j["value"] = b ? 1 : 0;
                    else if (SrsText.TryLiteral(text, false, out double d)) j["value"] = d;
                    else if (CheckExpr(line, text, st)) j["valueExpression"] = text;
                    break;
                }
            }
        }
        if (kw == "computed" && !hasInit) Error(TextProblemCodes.MissingArgument, "A computed variable needs its formula: computed name = …", line, 0, line.Text.Length);
        if (!c.AtEnd) Error(TextProblemCodes.Syntax, $"Unexpected '{Excerpt(c.Rest)}'", line, c.P, line.Text.Length);

        switch (type)
        {
            case "bool": j["isBoolean"] = true; break;
            case "string": j["isString"] = true; break;
            case "image": j["isImage"] = true; break;
            case "stopwatch": j["isStopwatch"] = true; break;
            case "list":
                if (j["items"] == null) j["items"] = new JsonArray();
                j["elementType"] = elem;
                break;
        }
        if (kw == "computed") j["isComputed"] = true;
        if (global) j["isGlobal"] = true;
        if (persistent) j["isPersistent"] = true;
        if (input) j["isInput"] = true;
        if (monitor) j["displayOnMonitor"] = true;
        if (mode != null) j["paramMode"] = mode;
        if (line.TrailingComment != null) j["description"] = line.TrailingComment;
        if (a.Id != null) v.ExplicitId = a.Id;
        if (a.With != null) v.With = a.With;
        if (a.Name != null || a.Off) Warn(TextProblemCodes.Syntax, "@name and @off apply to steps, not variables", a.First!, 0, a.First!.Text.Length);
        _doc.Vars.Add(v);
    }

    private void ParseListInit(SrsCursor c, JsonObject j, string elem)
    {
        int st = c.P;
        if (!c.TryChar('(')) { Error(TextProblemCodes.Syntax, "A list value is written in parentheses: (1, 2, 3)", c, st); c.P = c.S.Length; return; }
        var items = new JsonArray();
        if (!c.TryChar(')'))
        {
            while (true)
            {
                c.Ws();
                int es = c.P;
                if (elem is "Point" or "Record")
                {
                    var rec = new JsonObject();
                    if (!c.TryChar('(')) { Error(TextProblemCodes.Syntax, "Each element is a tuple: (x=1, y=2)", c, es); c.P = c.S.Length; return; }
                    if (!c.TryChar(')'))
                    {
                        while (true)
                        {
                            int fs = c.P;
                            var f = c.TryString(out _) ?? c.TryIdent();
                            if (f == null || !c.TryChar('=')) { Error(TextProblemCodes.Syntax, "Expected field=value", c, fs); c.P = c.S.Length; return; }
                            var (t, ts) = c.ScanExpr();
                            if (!SrsText.TryLiteral(t, false, out double d)) { Error(TextProblemCodes.BadValue, "List elements are numbers", c, ts); c.P = c.S.Length; return; }
                            rec[elem == "Point" ? f.ToLowerInvariant() : f] = d;
                            if (c.TryChar(',')) continue;
                            if (c.TryChar(')')) break;
                            Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P); c.P = c.S.Length; return;
                        }
                    }
                    if (elem == "Point")
                    {
                        var p = new JsonObject();
                        foreach (var a in new[] { "x", "y", "z", "rx", "ry", "rz" }) p[a] = rec[a]?.DeepClone() ?? 0.0;
                        foreach (var (k, v) in rec) if (p[k] == null) p[k] = v?.DeepClone();
                        rec = p;
                    }
                    items.Add(rec);
                }
                else
                {
                    var (t, ts) = c.ScanExpr();
                    double d;
                    if (elem == "Boolean" && SrsText.IsBoolWord(t, out bool b)) d = b ? 1 : 0;
                    else if (!SrsText.TryLiteral(t, false, out d)) { Error(TextProblemCodes.BadValue, "List elements are literal values", c, ts); c.P = c.S.Length; return; }
                    items.Add(new JsonObject { ["value"] = d });
                }
                if (c.TryChar(',')) continue;
                if (c.TryChar(')')) break;
                Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P); c.P = c.S.Length; return;
            }
        }
        j["items"] = items;
    }

    // ── Statements ─────────────────────────────────────────────────────────

    private StepNode? ParseStatement(SrsLine line, int depth)
    {
        var c = new SrsCursor(line);
        if (c.Peek() == '$') return ParseDollar(line, c);

        int ws = c.P;
        var word = c.TryIdent();
        if (word == null)
        {
            Error(TextProblemCodes.Syntax, $"Expected a statement, found '{Excerpt(c.Rest)}'", line, 0, line.Text.Length);
            return null;
        }
        var node = new StepNode { Line = line.Line, EndLine = line.EndLine, Span = SpanOf(line, ws, c.P) };

        // The type-name forms (every StepType name is a keyword) — the fallback the printer uses.
        if (Enum.TryParse<StepType>(word, ignoreCase: true, out var asType) && Enum.IsDefined(asType) &&
            !int.TryParse(word, out _) && !FlowWords.ContainsKey(word))
        {
            node.Type = asType;
            switch (asType)
            {
                case StepType.SetVariable: case StepType.ListOp: case StepType.Note: case StepType.Unknown:
                    ExpectEnd(c); return node;
                case StepType.IfCondition: return ParseIf(line, c, node, depth);
                case StepType.Loop: node.Json["loopMode"] = "count"; return ParseLoopTail(line, c, node, depth);
                case StepType.Switch: return ParseSwitch(line, c, node, depth);
                case StepType.Try: return ParseTry(line, c, node, depth);
                case StepType.LoopControl: node.Json["loopControl"] = "break"; ExpectEnd(c); return node;
                case StepType.CallRoutine: return ParseCall(line, c, node);
                case StepType.StatusUpdate: return ParseStatus(line, c, node);
                case StepType.Plugin: return ParsePlugin(line, c, node);
                case StepType.GoToLabel: return ParseGoto(line, c, node);
            }
            var spec = StepCatalog.Get(asType)!;
            ParseKeywordArgs(line, c, node, spec);
            if (spec.Body == "mapping") ParseMappingBody(line, node);
            return node;
        }

        if (FlowWords.TryGetValue(word, out var type))
        {
            node.Type = type;
            switch (word.ToLowerInvariant())
            {
                case "if": return ParseIf(line, c, node, depth);
                case "while":
                {
                    node.Json["loopMode"] = "while";
                    var (t, st) = c.ScanExpr(stopComma: false);
                    if (t.Length > 0) SetCondition(node, line, "loopWhileCondition", t, st);
                    else Error(TextProblemCodes.MissingArgument, "while needs a condition", line, 0, line.Text.Length);
                    return ParseBlockBody(line, node, StepTree.Roles.LoopSteps, depth);
                }
                case "repeat":
                    node.Json["loopMode"] = "count";
                    return ParseLoopTail(line, c, node, depth);
                case "for": return ParseFor(line, c, node, depth);
                case "switch": return ParseSwitch(line, c, node, depth);
                case "try": return ParseTry(line, c, node, depth);
                case "break": node.Json["loopControl"] = "break"; ExpectEnd(c); return node;
                case "continue": node.Json["loopControl"] = "continue"; ExpectEnd(c); return node;
                case "call": return ParseCall(line, c, node);
                case "status": return ParseStatus(line, c, node);
                case "plugin": return ParsePlugin(line, c, node);
                case "goto": return ParseGoto(line, c, node);
                case "unknown":
                {
                    c.Ws();
                    var t = c.TryString(out _) ?? c.TryIdent();
                    if (t != null) node.Json["unknownStepType"] = t;
                    ExpectEnd(c);
                    return node;
                }
                default:
                    ParseKeywordArgs(line, c, node, StepCatalog.Get(type)!);
                    return node;
            }
        }

        var guess = SrsText.DidYouMean(word, AllKeywords);
        Error(TextProblemCodes.UnknownKeyword, $"Unknown statement '{word}'" + (guess != null ? $" — did you mean {guess}?" : ""), line, ws, c.P);
        return null;
    }

    private void ExpectEnd(SrsCursor c)
    {
        if (!c.AtEnd) Error(TextProblemCodes.Syntax, $"Unexpected '{Excerpt(c.Rest)}'", c.Line, c.P, c.S.Length);
    }

    // ── Generic keyword statements (catalog driven) ────────────────────────

    private static FieldSpec? FieldByArg(StepSpec spec, string arg) =>
        spec.Fields.FirstOrDefault(f => f.Arg.Length > 0 && f.Arg.Equals(arg, StringComparison.OrdinalIgnoreCase) && Settable(f));

    private static bool Settable(FieldSpec f) => f.Kind is not (K.Steps or K.Branches or K.Cases or K.Json or K.Rows or K.MappingOut or K.ExprMap or K.PluginRef);

    /// <summary>The catalog value (as text) a showIf compares against.</summary>
    private static string? ShowIfValue(StepNode n, StepSpec spec, string field)
    {
        var v = n.Json[field];
        if (v is JsonValue jv)
        {
            if (jv.TryGetValue<bool>(out var b)) return b ? "true" : "false";
            return jv.ToString();
        }
        var f = spec.Fields.FirstOrDefault(x => x.Key == field);
        return f?.Default switch { null => null, bool b => b ? "true" : "false", var d => Convert.ToString(d, SrsText.Inv) };
    }

    private static bool Visible(StepNode n, StepSpec spec, FieldSpec f) =>
        f.ShowIf == null || f.ShowIf.In.Contains(ShowIfValue(n, spec, f.ShowIf.Field) ?? "");

    private void ParseKeywordArgs(SrsLine line, SrsCursor c, StepNode n, StepSpec spec)
    {
        var positional = spec.Fields.Where(f => f.Position != null && Settable(f)).OrderBy(f => f.Position).ToList();
        int slot = 0;
        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        // Statement-specific prefixes.
        switch (n.Type)
        {
            case StepType.Wait:
                if (c.TryWord("until"))
                {
                    n.Json["waitMode"] = "condition";
                    var (t, st) = c.ScanExpr();
                    if (t.Length == 0) Error(TextProblemCodes.MissingArgument, "Wait until needs a condition", c, st);
                    else SetCondition(n, line, "waitCondition", t, st);
                    slot = positional.Count; // no positionals after a condition
                    seen.Add("until");
                }
                else n.Json["waitMode"] = "duration";
                break;
            case StepType.SavePosition:
            {
                int st = c.P;
                if (c.TryWord("point"))
                {
                    c.Ws();
                    int ns = c.P;
                    var name = c.TryString(out _) ?? c.TryIdent();
                    if (name == null) Error(TextProblemCodes.Syntax, "Expected the point name after 'point'", c, ns);
                    else { SetRef(n, line, SrsRefKind.Point, "pointId", "pointName", name, ns, c.P, "pointId"); n.Json["savePositionTo"] = "point"; }
                    slot = positional.Count;
                    seen.Add("point");
                }
                break;
            }
        }

        while (!c.AtEnd)
        {
            if (c.TryChar(',') && c.AtEnd) break;
            if (c.AtNamedArg())
            {
                int ns = c.P;
                c.Ws(); ns = c.P;
                var arg = c.TryIdent()!;
                c.TryChar('=');
                var f = FieldByArg(spec, arg);
                if (f == null)
                {
                    var guess = SrsText.DidYouMean(arg, spec.Fields.Where(x => x.Arg.Length > 0 && Settable(x)).Select(x => x.Arg));
                    Error(TextProblemCodes.UnknownArgument, $"{spec.Keyword} has no argument '{arg}'" + (guess != null ? $" — did you mean {guess}?" : ""), line, ns, ns + arg.Length);
                    c.ScanExpr();
                    continue;
                }
                if (!seen.Add(f.Arg)) Error(TextProblemCodes.DuplicateArgument, $"'{arg}' is given twice", line, ns, ns + arg.Length);
                ReadNamedValue(line, c, n, spec, f);
                continue;
            }
            // A positional value.
            while (slot < positional.Count && !Visible(n, spec, positional[slot])) slot++;
            bool op = "+-*/%^<>=!&|?".Contains(c.Peek()) && c.Peek() != '\0' &&
                      !((c.Peek() == '+' || c.Peek() == '-') && (char.IsAsciiDigit(c.Peek(1)) || c.Peek(1) == '.'));
            if (slot >= positional.Count || op)
            {
                Error(op ? TextProblemCodes.BadValue : TextProblemCodes.Syntax,
                      op ? $"Unexpected '{Excerpt(c.Rest)}' — a calculated value goes in parentheses, e.g. ($a * 2)"
                         : $"Unexpected '{Excerpt(c.Rest)}'", line, c.P, c.S.Length);
                break;
            }
            var pf = positional[slot++];
            if (!ReadPositional(line, c, n, spec, pf)) break;
        }
    }

    /// <summary>Reads one positional value into its field; false on an unrecoverable problem.</summary>
    private bool ReadPositional(SrsLine line, SrsCursor c, StepNode n, StepSpec spec, FieldSpec f)
    {
        c.Ws();
        int st = c.P;
        if (f.Kind == K.Target) { ParseTarget(line, c, n, f, isVia: false); n.FieldSpans[f.Key] = SpanOf(line, st, c.P); return true; }
        var tok = c.TryToken(out st, out bool isStr);
        if (tok == null)
        {
            Error(TextProblemCodes.Syntax, $"Expected {f.Label.ToLowerInvariant()}, found '{Excerpt(c.Rest)}'", line, c.P, c.S.Length);
            return false;
        }
        n.FieldSpans[f.Key] = SpanOf(line, st, c.P);
        // Positional sugar.
        if (n.Type == StepType.SetOutput && f.Key == "outputValue" && !isStr && tok.Equals("toggle", StringComparison.OrdinalIgnoreCase))
        {
            n.Json["outputAction"] = "toggle";
            return true;
        }
        if (n.Type == StepType.SetJointPosition && f.Key == "jointIndex" && !isStr && tok.Length > 1 &&
            (tok[0] == 'J' || tok[0] == 'j') && int.TryParse(tok[1..], out int jn) && jn >= 1)
        {
            n.Json["jointIndex"] = jn - 1;
            return true;
        }
        ApplyValue(line, n, spec, f, tok, st, isStr, positional: true);
        if (n.Type == StepType.GripperAction && f.Key == "gripperAction" && n.Json["gripperAction"]?.ToString() == "GripTo" &&
            !c.AtEnd && c.Peek() != ',' && !c.AtNamedArg())
        {
            var gf = spec.Fields.First(x => x.Key == "gripTargetMm");
            var t2 = c.TryToken(out int s2, out bool str2);
            if (t2 != null) { n.FieldSpans[gf.Key] = SpanOf(line, s2, c.P); ApplyValue(line, n, spec, gf, t2, s2, str2, positional: true); }
        }
        return true;
    }

    private void ReadNamedValue(SrsLine line, SrsCursor c, StepNode n, StepSpec spec, FieldSpec f)
    {
        c.Ws();
        int st = c.P;
        switch (f.Kind)
        {
            case K.Target:
                ParseTarget(line, c, n, f, isVia: f.Key == "via");
                n.FieldSpans[f.Key] = SpanOf(line, st, c.P);
                return;
            case K.Axes6:
                ParseAxes(line, c, n, f);
                n.FieldSpans[f.Key] = SpanOf(line, st, c.P);
                return;
            case K.TextList:
                ParseTextList(line, c, n, f);
                n.FieldSpans[f.Key] = SpanOf(line, st, c.P);
                return;
        }
        if (n.Type is StepType.MoveL or StepType.JumpL && f.Key == "blend")
        {
            var (bt, bs) = c.ScanExpr();
            n.FieldSpans[f.Key] = SpanOf(line, bs, bs + bt.Length);
            if (SrsText.IsBoolWord(bt, out bool on)) { n.Json["blend"] = on; return; }
            n.Json["blend"] = true;
            var rf = spec.Fields.First(x => x.Key == "blendRadius");
            ApplyValue(line, n, spec, rf, bt, bs, false, positional: false);
            return;
        }
        var (text, start) = c.ScanExpr();
        bool isStr = false;
        if (text.Length > 1 && text[0] == '"')
        {
            var sc = new SrsCursor(new SrsLine { Text = text, Map = new int[text.Length + 1] });
            var s = sc.TryString(out _);
            if (s != null && sc.AtEnd) { text = s; isStr = true; }
        }
        n.FieldSpans[f.Key] = SpanOf(line, start, c.P);
        if (text.Length == 0 && !isStr) { Error(TextProblemCodes.Syntax, $"Expected a value for {f.Arg}", line, start, start + 1); return; }
        ApplyValue(line, n, spec, f, text, start, isStr, positional: false);
    }

    /// <summary>Stores one scalar-ish value by its field's kind.</summary>
    private void ApplyValue(SrsLine line, StepNode n, StepSpec spec, FieldSpec f, string text, int st, bool isStr, bool positional)
    {
        int en = st + Math.Max(1, text.Length);
        switch (f.Kind)
        {
            case K.Number:
            case K.Integer:
                if (isStr) { Error(TextProblemCodes.BadValue, $"{f.Label} is a number, not text", line, st, en); return; }
                ApplyNumber(line, n, f.Key, f.ExprKey, f.Kind == K.Integer, f.Unit == "ms", text, st, positional, f.Label);
                return;
            case K.Bool:
                if (!isStr && SrsText.IsBoolWord(text, out bool b)) { n.Json[f.Key] = b; return; }
                if (f.ExprKey != null && !isStr)
                {
                    if (SrsText.IsWrapped(text)) { SetExpr(n, line, f.ExprKey, text[1..^1].Trim(), st + 1); return; }
                    if (!positional || SrsText.IsVarRefToken(text)) { SetExpr(n, line, f.ExprKey, text, st); return; }
                }
                Error(TextProblemCodes.BadValue, $"{f.Label} is on or off" + (f.ExprKey != null ? ", or an expression in parentheses" : ""), line, st, en);
                return;
            case K.Enum:
            {
                if (EnumWords.TryGetValue((n.Type, f.Key), out var words))
                    foreach (var (val, w) in words)
                        if (w.Equals(text, StringComparison.OrdinalIgnoreCase)) { n.Json[f.Key] = val; return; }
                var opt = f.Options?.FirstOrDefault(o => string.Equals(Convert.ToString(o.Value, SrsText.Inv), text, StringComparison.OrdinalIgnoreCase));
                if (opt != null) { n.Json[f.Key] = Convert.ToString(opt.Value, SrsText.Inv); return; }
                var choices = words != null ? words.Select(w => w.Word) : f.Options?.Select(o => Convert.ToString(o.Value, SrsText.Inv)!) ?? [];
                Error(TextProblemCodes.BadValue, $"{f.Label} is one of: {string.Join(", ", choices)}", line, st, en);
                return;
            }
            case K.Text:
                if (!isStr && !SrsCursor.IsIdent(text)) { Error(TextProblemCodes.BadValue, $"{f.Label} is text in quotes", line, st, en); return; }
                n.Json[f.Key] = text;
                return;
            case K.Template:
                if (!isStr) { Error(TextProblemCodes.BadValue, $"{f.Label} is text in quotes", line, st, en); return; }
                n.Json[f.Key] = text;
                return;
            case K.Expr:
                if (isStr) { n.Json[f.Key] = text; return; }   // a template (e.g. a text prompt's default)
                if (CheckExpr(line, text, st)) n.Json[f.Key] = text;
                return;
            case K.Condition:
                if (isStr) { Error(TextProblemCodes.BadValue, $"{f.Label} is a condition, not text", line, st, en); return; }
                SetCondition(n, line, f.Key, text, st);
                return;
            case K.VarWrite:
            case K.VarRead:
            case K.ListVar:
                if (!isStr && text.Length > 1 && text[0] == '$' && SrsCursor.IsIdent(text[1..])) { n.Json[f.Key] = text[1..]; return; }
                Error(TextProblemCodes.BadValue, $"{f.Label} is a variable: $name", line, st, en);
                return;
            case K.PointRef:   SetRef(n, line, SrsRefKind.Point, f.StoredProps[0], f.StoredProps.ElementAtOrDefault(1), text, st, en, f.Key, isStr); return;
            case K.ToolRef:    SetRef(n, line, SrsRefKind.Tool, "toolId", "toolName", text, st, en, f.Key, isStr); return;
            case K.LocalRef:   SetRef(n, line, SrsRefKind.Local, "localId", "localName", text, st, en, f.Key, isStr); return;
            case K.RoutineRef: SetRef(n, line, SrsRefKind.Routine, "routineId", "routineName", text, st, en, f.Key, isStr); return;
            case K.ProgramRef: SetRef(n, line, SrsRefKind.Program, f.StoredProps[0], f.StoredProps.ElementAtOrDefault(1), text, st, en, f.Key, isStr); return;
            case K.VisionRef:  SetRef(n, line, SrsRefKind.Vision, "visionProgramId", "visionProgramName", text, st, en, f.Key, isStr); return;
            case K.DeviceRef:  SetRef(n, line, SrsRefKind.Device, f.Key, null, text, st, en, f.Key, isStr); return;
            case K.CameraRef:  SetRef(n, line, SrsRefKind.Camera, f.Key, null, text, st, en, f.Key, isStr); return;
            case K.LabelRef:
                if (!isStr && !SrsCursor.IsIdent(text)) { Error(TextProblemCodes.BadValue, "Expected a label name", line, st, en); return; }
                n.Json["labelName"] = text;
                return;
            default:
                Error(TextProblemCodes.BadValue, $"{f.Label} cannot be written here", line, st, en);
                return;
        }
    }

    /// <summary>A literal (into <paramref name="key"/>) or an expression (into
    /// <c>expressions[exprKey]</c>). <c>(…)</c> always means an expression (the parentheses are
    /// dropped). A positional slot takes a bare expression only when it is a <c>$var</c> token.</summary>
    private void ApplyNumber(SrsLine line, StepNode n, string key, string? exprKey, bool integer, bool ms,
                             string text, int st, bool positional, string label, JsonObject? into = null)
    {
        var target = into ?? n.Json;
        int en = st + Math.Max(1, text.Length);
        if (SrsText.IsWrapped(text))
        {
            if (exprKey == null) { Error(TextProblemCodes.BadValue, $"{label} takes a number here, not an expression", line, st, en); return; }
            var inner = text[1..^1];
            int lead = inner.Length - inner.TrimStart().Length;
            SetExpr(n, line, exprKey, inner.Trim(), st + 1 + lead);
            return;
        }
        if (SrsText.TryLiteral(text, ms, out double v))
        {
            if (integer)
            {
                if (v != Math.Floor(v) || Math.Abs(v) > 9e15) { Error(TextProblemCodes.BadValue, $"{label} is a whole number", line, st, en); return; }
                target[key] = (long)v;
            }
            else target[key] = v;
            return;
        }
        if (exprKey == null) { Error(TextProblemCodes.BadValue, $"{label} takes a number" + (ms ? " (ms, or 1.5s)" : ""), line, st, en); return; }
        if (positional && !SrsText.IsVarRefToken(text))
        {
            Error(TextProblemCodes.BadValue, $"Write the expression in parentheses: ({Excerpt(text)})", line, st, en);
            return;
        }
        SetExpr(n, line, exprKey, text, st);
    }

    private void SetExpr(StepNode n, SrsLine line, string exprKey, string text, int st)
    {
        if (!CheckExpr(line, text, st)) return;
        if (n.Json["expressions"] is not JsonObject ex) n.Json["expressions"] = ex = new JsonObject();
        ex[exprKey] = text;
    }

    /// <summary>Syntax-checks an expression; problems point into it.</summary>
    private bool CheckExpr(SrsLine line, string text, int st)
    {
        if (string.IsNullOrWhiteSpace(text)) { Error(TextProblemCodes.ExpressionSyntax, "Expected an expression", line, st, st + 1); return false; }
        try { ExpressionEvaluator.Validate(text); return true; }
        catch (ExpressionParseException ex)
        {
            int p = Math.Clamp(ex.Position, 0, Math.Max(0, text.Length - 1));
            string code = ex.Code switch
            {
                "unknownFunction" => TextProblemCodes.UnknownFunction,
                "badArity" => TextProblemCodes.BadArity,
                _ => TextProblemCodes.ExpressionSyntax,
            };
            Error(code, ex.Message, line, st + p, st + p + 1);
            return false;
        }
    }

    private void SetCondition(StepNode n, SrsLine line, string key, string text, int st)
    {
        if (!CheckExpr(line, text, st)) return;
        var parts = ProgramNormalizer.SplitCondition(text) ?? [("", text)];
        var items = new JsonArray();
        for (int i = 0; i < parts.Count; i++)
        {
            var item = new JsonObject { ["id"] = "", ["expr"] = parts[i].Text };
            if (i > 0) item["join"] = parts[i].Join;
            items.Add(item);
        }
        n.Json[key] = new JsonObject { ["combinator"] = "ALL", ["items"] = items };
        n.FieldSpans.TryAdd(key, SpanOf(line, st, st + text.Length));
    }

    private void SetRef(StepNode n, SrsLine line, SrsRefKind kind, string idProp, string? nameProp,
                        string text, int st, int en, string fieldKey, bool isStr = true)
    {
        n.FieldSpans.TryAdd(fieldKey, SpanOf(line, st, en));
        var id = _names.IdOf(kind, text);
        if (nameProp != null)
        {
            if (id != null) { n.Json[idProp] = id; n.Json[nameProp] = _names.NameOf(kind, id); }
            else
            {
                n.Json[nameProp] = text;
                Warn(NameResolver.UnknownCode(kind), $"No {NameResolver.KindLabel(kind)} named '{text}' — the name is kept; create it, then the step links to it", line, st, en);
            }
            return;
        }
        if (id == null && _names.IdExists(kind, text)) id = text;
        n.Json[idProp] = id ?? text;
        if (id == null)
            Warn(NameResolver.UnknownCode(kind), $"No {NameResolver.KindLabel(kind)} named '{text}' — kept as written", line, st, en);
    }

    // ── Targets, tuples, lists ─────────────────────────────────────────────

    private void ParseTarget(SrsLine line, SrsCursor c, StepNode n, FieldSpec f, bool isVia)
    {
        c.Ws();
        int st = c.P;
        string idP = isVia ? "viaPointId" : "pointId", nameP = isVia ? "viaPointName" : "pointName",
               exprP = isVia ? "viaPointNameExpr" : "pointNameExpr";
        if (c.TryWord("here")) return;
        if (!isVia && (c.PeekIdent() is { } gw) && (gw.Equals("grid", StringComparison.OrdinalIgnoreCase) || gw.Equals("stack", StringComparison.OrdinalIgnoreCase)))
        {
            int save = c.P;
            c.TryIdent();
            c.Ws();
            if (c.Peek() == '"' || SrsCursor.IsIdentStart(c.Peek()))
            {
                bool grid = gw.Equals("grid", StringComparison.OrdinalIgnoreCase);
                int ns = c.P;
                var name = c.TryString(out _) ?? c.TryIdent()!;
                int ne = c.P;
                c.Ws();
                if (c.Peek() != '[') { Error(TextProblemCodes.Syntax, grid ? "Expected the cell: grid Name[row, col] or grid Name[index]" : "Expected the slot: stack Name[index]", c, c.P); return; }
                c.P++;
                var (i1, s1) = c.ScanExpr();
                string? i2 = null; int s2 = 0;
                if (c.TryChar(',')) (i2, s2) = c.ScanExpr();
                if (!c.TryChar(']')) { Error(TextProblemCodes.Syntax, "Expected ]", c, c.P); return; }
                var kind = grid ? SrsRefKind.Grid : SrsRefKind.Stack;
                var id = _names.IdOf(kind, name) ?? (_names.IdExists(kind, name) ? name : null);
                if (id == null) Warn(NameResolver.UnknownCode(kind), $"No {kind.ToString().ToLowerInvariant()} named '{name}' — kept as written", line, ns, ne);
                if (grid)
                {
                    var gp = new JsonObject { ["gridId"] = id ?? name };
                    if (i2 == null)
                    {
                        gp["useGridIndex"] = true;
                        IndexValue(line, n, gp, "gridIndex", "gridGridIndex", i1, s1);
                    }
                    else
                    {
                        gp["useGridIndex"] = false;
                        IndexValue(line, n, gp, "rowIndex", "gridRowIndex", i1, s1);
                        IndexValue(line, n, gp, "colIndex", "gridColIndex", i2, s2);
                    }
                    n.Json["gridPoint"] = gp;
                }
                else
                {
                    if (i2 != null) Error(TextProblemCodes.Syntax, "A stack slot has one index", line, s2, s2 + i2.Length);
                    var sp = new JsonObject { ["stackId"] = id ?? name };
                    IndexValue(line, n, sp, "index", "stackIndex", i1, s1);
                    n.Json["stackPoint"] = sp;
                }
                return;
            }
            c.P = save;
        }
        var tok = c.TryToken(out st, out bool isStr);
        if (tok == null) { Error(TextProblemCodes.Syntax, $"Expected a target: a point name, \"name\", $points[i], grid/stack cell or here", c, c.P); return; }
        if (isStr)
        {
            if (tok.Contains('{') || tok.Contains('$')) n.Json[exprP] = tok;
            else SetRef(n, line, SrsRefKind.Point, idP, nameP, tok, st, c.P, f.Key);
            return;
        }
        if (tok[0] == '$')
        {
            if (!SrsText.IsVarRefToken(tok)) { Error(TextProblemCodes.Syntax, "Expected $variable or $points[i]", line, st, c.P); return; }
            if (CheckExpr(line, tok, st)) n.Json[exprP] = tok;
            return;
        }
        if (SrsCursor.IsIdent(tok)) { SetRef(n, line, SrsRefKind.Point, idP, nameP, tok, st, c.P, f.Key); return; }
        Error(TextProblemCodes.Syntax, $"'{Excerpt(tok)}' is not a target — a point name with spaces goes in quotes", line, st, c.P);
    }

    private void IndexValue(SrsLine line, StepNode n, JsonObject into, string prop, string exprKey, string text, int st)
    {
        if (text.Length == 0) { Error(TextProblemCodes.Syntax, "Expected an index", line, st, st + 1); return; }
        if (SrsText.IsWrapped(text)) { SetExpr(n, line, exprKey, text[1..^1].Trim(), st + 1); return; }
        if (SrsText.TryLiteral(text, false, out double v)) { into[prop] = v; return; }
        SetExpr(n, line, exprKey, text, st);
    }

    private static readonly string[] AxisNames = ["x", "y", "z", "rx", "ry", "rz"];

    private void ParseAxes(SrsLine line, SrsCursor c, StepNode n, FieldSpec f)
    {
        int st = c.P;
        if (!c.TryChar('(')) { Error(TextProblemCodes.Syntax, $"{f.Arg} is a tuple: (x=…, z=…)", c, st); c.ScanExpr(); return; }
        string prefix = f.StoredProps[0][..^1]; // offsetX → offset
        if (c.TryChar(')')) return;
        while (true)
        {
            c.Ws();
            int as_ = c.P;
            var ax = c.TryIdent()?.ToLowerInvariant();
            if (ax == null || !AxisNames.Contains(ax) || !c.TryChar('='))
            {
                Error(TextProblemCodes.Syntax, "Expected an axis: x=, y=, z=, rx=, ry= or rz=", c, as_);
                c.ScanExpr(); c.TryChar(')');
                return;
            }
            var (t, ts) = c.ScanExpr();
            string prop = prefix + ax.ToUpperInvariant();
            ApplyNumber(line, n, prop, prop, false, false, t, ts, positional: false, $"{f.Label} {ax}");
            if (c.TryChar(',')) continue;
            if (c.TryChar(')')) return;
            Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P);
            return;
        }
    }

    private void ParseTextList(SrsLine line, SrsCursor c, StepNode n, FieldSpec f)
    {
        var arr = new JsonArray();
        int st = c.P;
        if (!c.TryChar('('))
        {
            var one = c.TryString(out _) ?? c.TryIdent();
            if (one == null) { Error(TextProblemCodes.Syntax, $"{f.Arg} is a list: (\"A\", \"B\")", c, st); c.ScanExpr(); return; }
            arr.Add(one);
            n.Json[f.Key] = arr;
            return;
        }
        if (!c.TryChar(')'))
        {
            while (true)
            {
                c.Ws();
                int es = c.P;
                var s = c.TryString(out _) ?? c.TryIdent();
                if (s == null) { Error(TextProblemCodes.Syntax, "Expected text in quotes or a word", c, es); c.ScanExpr(); c.TryChar(')'); return; }
                arr.Add(s);
                if (c.TryChar(',')) continue;
                if (c.TryChar(')')) break;
                Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P);
                return;
            }
        }
        n.Json[f.Key] = arr;
    }

    // ── $ statements: assignment and list operations ──────────────────────

    private static readonly string[] ListMethods = ["append", "insert", "removeAt", "clear", "sort", "reverse", "copyFrom"];

    private StepNode? ParseDollar(SrsLine line, SrsCursor c)
    {
        int st = c.P;
        c.P++;
        var name = c.TryIdent();
        if (name == null) { Error(TextProblemCodes.Syntax, "Expected a variable name after $", c, st); return null; }
        var n = new StepNode { Line = line.Line, EndLine = line.EndLine, Span = SpanOf(line, st, c.P) };
        n.FieldSpans["variableName"] = SpanOf(line, st, c.P);
        n.Json["variableName"] = name;

        if (c.Peek() == '[')
        {
            n.Type = StepType.ListOp;
            int ob = c.P;
            c.P++;
            var (idx, is_) = c.ScanExpr();
            if (!c.TryChar(']')) { Error(TextProblemCodes.Syntax, "Expected ]", c, c.P); return null; }
            ListIndex(line, n, idx, is_);
            string? field = null;
            if (c.Peek() == '.')
            {
                c.P++;
                field = c.TryIdent();
                if (field == null) { Error(TextProblemCodes.Syntax, "Expected a field name after .", c, c.P); return null; }
            }
            c.Ws();
            if (c.Peek() != '=' || c.Peek(1) == '=') { Error(TextProblemCodes.Syntax, "Expected = and a value", c, c.P); return null; }
            c.P++;
            var (rhs, rs) = c.ScanExpr(stopComma: false);
            if (field != null)
            {
                n.Json["listAction"] = "setField";
                if (CheckExpr(line, rhs, rs)) n.Json["listFields"] = new JsonObject { [field] = rhs };
            }
            else
            {
                n.Json["listAction"] = "set";
                if (IsTuple(rhs)) n.Json["listFields"] = ParseFieldTuple(line, rhs, rs);
                else if (CheckExpr(line, rhs, rs)) n.Json["listValueExpr"] = rhs;
            }
            ExpectEnd(c);
            return n;
        }

        c.Ws();
        if (c.Peek() == '=' && c.Peek(1) != '=')
        {
            n.Type = StepType.SetVariable;
            c.P++;
            c.Ws();
            int rs = c.P;
            var rest = c.Rest.TrimEnd();
            n.RhsSpan = SpanOf(line, rs, rs + rest.Length);
            n.FieldSpans["variableExpr"] = n.RhsSpan;
            if (rest.Length > 0 && rest[0] == '"')
            {
                var sc = new SrsCursor(line, rs);
                var s = sc.TryString(out _);
                if (s != null && sc.AtEnd) { n.Json["variableExpr"] = s; n.RhsIsString = true; return n; }
            }
            if (rest.Length == 0) { Error(TextProblemCodes.MissingArgument, "Expected a value after =", line, rs, rs + 1); return n; }
            if (CheckExpr(line, rest, rs)) n.Json["variableExpr"] = rest;
            return n;
        }

        if (c.Peek() == '.')
        {
            c.P++;
            int ms = c.P;
            var method = c.TryIdent();
            var canon = ListMethods.FirstOrDefault(m => m.Equals(method, StringComparison.OrdinalIgnoreCase));
            if (canon == null)
            {
                var guess = method != null ? SrsText.DidYouMean(method, ListMethods) : null;
                Error(TextProblemCodes.Syntax, $"Unknown list action '{method}'" + (guess != null ? $" — did you mean {guess}?" : "") +
                      $" ({string.Join(", ", ListMethods)})", line, ms, c.P);
                return null;
            }
            n.Type = StepType.ListOp;
            n.Json["listAction"] = canon;
            if (!c.TryChar('(')) { Error(TextProblemCodes.Syntax, $"Expected ( after {canon}", c, c.P); return null; }
            var args = new List<(string? Name, string Text, int Start)>();
            if (!c.TryChar(')'))
            {
                while (true)
                {
                    string? an = null;
                    if (c.AtNamedArg()) { an = c.TryIdent(); c.TryChar('='); }
                    var (t, ts) = c.ScanExpr();
                    args.Add((an, t, ts));
                    if (c.TryChar(',')) continue;
                    if (c.TryChar(')')) break;
                    Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P);
                    return null;
                }
            }
            ExpectEnd(c);
            ApplyListArgs(line, n, canon, args);
            return n;
        }

        Error(TextProblemCodes.Syntax, $"Expected = (assign), [i] = (set an element) or .append(…) and the like after ${name}", c, c.P);
        return null;
    }

    private void ListIndex(SrsLine line, StepNode n, string text, int st)
    {
        n.FieldSpans["listIndex"] = SpanOf(line, st, st + text.Length);
        ApplyNumber(line, n, "listIndex", "listIndex", true, false, text, st, positional: false, "Index");
    }

    private static bool IsTuple(string t)
    {
        if (!SrsText.IsWrapped(t)) return false;
        var c = new SrsCursor(new SrsLine { Text = t[1..^1], Map = new int[t.Length] });
        return c.AtNamedArg() || c.AtEnd;
    }

    private JsonObject ParseFieldTuple(SrsLine line, string t, int st)
    {
        var fields = new JsonObject();
        var c = new SrsCursor(line, st + 1);
        if (c.TryChar(')')) return fields;
        while (true)
        {
            c.Ws();
            int fs = c.P;
            var f = c.TryIdent();
            if (f == null || !c.TryChar('=')) { Error(TextProblemCodes.Syntax, "Expected field=value", c, fs); return fields; }
            var (v, vs) = c.ScanExpr();
            if (CheckExpr(line, v, vs)) fields[f] = v;
            if (c.TryChar(',')) continue;
            if (c.TryChar(')')) return fields;
            Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P);
            return fields;
        }
    }

    private void ApplyListArgs(SrsLine line, StepNode n, string action, List<(string? Name, string Text, int Start)> args)
    {
        var fields = new JsonObject();
        var positional = args.Where(a => a.Name == null).ToList();
        var named = args.Where(a => a.Name != null).ToList();
        void TooMany(int i) => Error(TextProblemCodes.Syntax, $"{action}() takes fewer values", line, positional[i].Start, positional[i].Start + Math.Max(1, positional[i].Text.Length));
        switch (action)
        {
            case "append":
            case "insert":
            {
                int k = 0;
                if (action == "insert")
                {
                    if (positional.Count == 0) { Error(TextProblemCodes.MissingArgument, "insert needs the index first: insert(i, value)", line, 0, line.Text.Length); break; }
                    ListIndex(line, n, positional[0].Text, positional[0].Start);
                    k = 1;
                }
                if (positional.Count > k + 1) TooMany(k + 1);
                if (positional.Count > k)
                {
                    if (named.Count > 0) Error(TextProblemCodes.Syntax, "Give either a value or fields (x=…), not both", line, named[0].Start, named[0].Start + 1);
                    if (CheckExpr(line, positional[k].Text, positional[k].Start)) n.Json["listValueExpr"] = positional[k].Text;
                }
                foreach (var (nm, t, s) in named) if (CheckExpr(line, t, s)) fields[nm!] = t;
                if (fields.Count > 0) n.Json["listFields"] = fields;
                break;
            }
            case "removeAt":
                if (positional.Count != 1 || named.Count > 0) { Error(TextProblemCodes.Syntax, "removeAt takes one index: removeAt(i)", line, 0, line.Text.Length); break; }
                ListIndex(line, n, positional[0].Text, positional[0].Start);
                break;
            case "clear":
            case "reverse":
                if (args.Count > 0) Error(TextProblemCodes.Syntax, $"{action}() takes no values", line, args[0].Start, args[0].Start + 1);
                break;
            case "copyFrom":
                if (positional.Count != 1 || !(positional[0].Text.StartsWith('$') && SrsCursor.IsIdent(positional[0].Text[1..])))
                { Error(TextProblemCodes.Syntax, "copyFrom takes the source list: copyFrom($other)", line, 0, line.Text.Length); break; }
                n.Json["listSourceVariable"] = positional[0].Text[1..];
                break;
            case "sort":
                foreach (var (nm, t, s) in args)
                {
                    if (nm == null && t.Equals("desc", StringComparison.OrdinalIgnoreCase)) { n.Json["listSortDescending"] = true; continue; }
                    if (nm != null && nm.Equals("desc", StringComparison.OrdinalIgnoreCase) && SrsText.IsBoolWord(t, out bool d)) { n.Json["listSortDescending"] = d; continue; }
                    if (nm != null && nm.Equals("by", StringComparison.OrdinalIgnoreCase))
                    {
                        string? f = SrsCursor.IsIdent(t) ? t : null;
                        if (f == null && t.Length > 1 && t[0] == '"')
                        {
                            var sc = new SrsCursor(new SrsLine { Text = t, Map = new int[t.Length + 1] });
                            var sv = sc.TryString(out _);
                            if (sv != null && sc.AtEnd) f = sv;
                        }
                        if (f != null) { n.Json["listSortField"] = f; continue; }
                    }
                    Error(TextProblemCodes.Syntax, "sort takes by=field and desc", line, s, s + Math.Max(1, t.Length));
                }
                break;
        }
    }

    // ── Flow blocks ───────────────────────────────────────────────────────

    private SrsLine? Peek() => _i < _lines.Count ? _lines[_i] : null;

    private static string ClauseWord(SrsLine l) => new SrsCursor(l).PeekIdent()?.ToLowerInvariant() ?? "";

    /// <summary>A body that ends at <c>end</c> only; other clauses are reported and skipped.</summary>
    private StepNode ParseBlockBody(SrsLine open, StepNode n, string role, int depth)
    {
        var all = new List<StepNode>();
        while (true)
        {
            var (steps, term) = ParseBody(depth + 1);
            all.AddRange(steps);
            if (term == null) { Error(TextProblemCodes.MissingEnd, "This block has no 'end'", open, 0, open.Text.Length); n.EndLine = _lines.Count > 0 ? _lines[^1].EndLine : n.Line; break; }
            var w = ClauseWord(term);
            if (w == "end") { CheckClauseTail(term); n.EndLine = term.EndLine; _i++; break; }
            Error(TextProblemCodes.MisplacedClause, $"'{w}' does not belong in this block", term, 0, w.Length);
            _i++;
        }
        n.Bodies.Add(new StepBody(role, null, all));
        return n;
    }

    private void CheckClauseTail(SrsLine term)
    {
        var c = new SrsCursor(term);
        c.TryIdent();
        if (!c.AtEnd) Error(TextProblemCodes.Syntax, $"Unexpected '{Excerpt(c.Rest)}'", term, c.P, term.Text.Length);
    }

    private StepNode ParseIf(SrsLine line, SrsCursor c, StepNode n, int depth)
    {
        var (t, st) = c.ScanExpr(stopComma: false);
        if (t.Length > 0) SetCondition(n, line, "condition", t, st);
        var branches = new JsonArray();
        string role = StepTree.Roles.IfSteps; int? index = null;
        var cur = new List<StepNode>();
        bool sawElse = false;
        while (true)
        {
            var (steps, term) = ParseBody(depth + 1);
            cur.AddRange(steps);
            if (term == null)
            {
                Error(TextProblemCodes.MissingEnd, "This if has no 'end'", line, 0, line.Text.Length);
                n.EndLine = _lines.Count > 0 ? _lines[^1].EndLine : n.Line;
                n.Bodies.Add(new StepBody(role, index, cur));
                break;
            }
            var w = ClauseWord(term);
            if (w == "elif" || (w == "else" && IsElseIf(term)))
            {
                if (sawElse) Error(TextProblemCodes.MisplacedClause, "elif after else", term, 0, w.Length);
                n.Bodies.Add(new StepBody(role, index, cur));
                cur = [];
                var tc = new SrsCursor(term);
                tc.TryIdent();
                if (w == "else") tc.TryIdent(); // "else if"
                var (ct, cs) = tc.ScanExpr(stopComma: false);
                var tmp = new StepNode();
                if (ct.Length > 0) SetCondition(tmp, term, "condition", ct, cs);
                else Error(TextProblemCodes.MissingArgument, "elif needs a condition", term, 0, term.Text.Length);
                branches.Add(new JsonObject { ["id"] = "", ["condition"] = tmp.Json["condition"]?.DeepClone() ?? new JsonObject { ["combinator"] = "ALL", ["items"] = new JsonArray() } });
                role = StepTree.Roles.ElseIfBranches; index = branches.Count - 1;
                _i++;
                continue;
            }
            if (w == "else")
            {
                if (sawElse) Error(TextProblemCodes.MisplacedClause, "A second else", term, 0, w.Length);
                CheckClauseTail(term);
                n.Bodies.Add(new StepBody(role, index, cur));
                cur = [];
                role = StepTree.Roles.ElseSteps; index = null; sawElse = true;
                _i++;
                continue;
            }
            if (w == "end")
            {
                CheckClauseTail(term);
                n.Bodies.Add(new StepBody(role, index, cur));
                n.EndLine = term.EndLine;
                _i++;
                break;
            }
            Error(TextProblemCodes.MisplacedClause, $"'{w}' does not belong in an if", term, 0, w.Length);
            _i++;
        }
        if (branches.Count > 0) n.Json["elseIfBranches"] = branches;
        return n;
    }

    private static bool IsElseIf(SrsLine l)
    {
        var c = new SrsCursor(l);
        c.TryIdent();
        return c.TryWord("if");
    }

    private StepNode ParseLoopTail(SrsLine line, SrsCursor c, StepNode n, int depth)
    {
        var spec = StepCatalog.Get(StepType.Loop)!;
        if (c.TryWord("forever")) n.Json["loopCount"] = 0L;
        else if (!c.AtEnd && c.Peek() != ',' && !(c.PeekIdent() is { } w && w.Equals("as", StringComparison.OrdinalIgnoreCase)))
        {
            var tok = c.TryToken(out int st, out bool isStr);
            if (tok == null || isStr) Error(TextProblemCodes.Syntax, "Expected the count: repeat 3, repeat $n, repeat ($n * 2) or repeat forever", c, c.P);
            else
            {
                n.FieldSpans["loopCount"] = SpanOf(line, st, c.P);
                ApplyNumber(line, n, "loopCount", "loopCount", true, false, tok, st, positional: true, "Count");
            }
        }
        if (c.TryWord("as"))
        {
            c.Ws();
            int vs = c.P;
            var tok = c.TryToken(out vs, out _);
            if (tok == null || tok.Length < 2 || tok[0] != '$' || !SrsCursor.IsIdent(tok[1..])) Error(TextProblemCodes.Syntax, "Expected the index variable: as $i", c, vs);
            else n.Json["forEachIndexVariableName"] = tok[1..];
        }
        ParseTrailingArgs(line, c, n, spec);
        return ParseBlockBody(line, n, StepTree.Roles.LoopSteps, depth);
    }

    /// <summary>Named args after a comma on a block header (try, repeat, catch …).</summary>
    private void ParseTrailingArgs(SrsLine line, SrsCursor c, StepNode n, StepSpec spec, HashSet<string>? only = null)
    {
        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        bool first = true;
        while (!c.AtEnd)
        {
            if (!first || c.Peek() == ',') { if (!c.TryChar(',') && !first) { Error(TextProblemCodes.Syntax, "Expected ,", c, c.P); return; } }
            first = false;
            if (c.AtEnd) return;
            if (!c.AtNamedArg()) { Error(TextProblemCodes.Syntax, $"Unexpected '{Excerpt(c.Rest)}'", line, c.P, c.S.Length); return; }
            c.Ws();
            int ns = c.P;
            var arg = c.TryIdent()!;
            c.TryChar('=');
            var f = FieldByArg(spec, arg);
            if (f == null || (only != null && !only.Contains(f.Arg)))
            {
                var pool = spec.Fields.Where(x => x.Arg.Length > 0 && Settable(x) && (only == null || only.Contains(x.Arg))).Select(x => x.Arg);
                var guess = SrsText.DidYouMean(arg, pool);
                Error(TextProblemCodes.UnknownArgument, $"No argument '{arg}' here" + (guess != null ? $" — did you mean {guess}?" : ""), line, ns, ns + arg.Length);
                c.ScanExpr();
                continue;
            }
            if (!seen.Add(f.Arg)) Error(TextProblemCodes.DuplicateArgument, $"'{arg}' is given twice", line, ns, ns + arg.Length);
            ReadNamedValue(line, c, n, spec, f);
        }
    }

    private StepNode ParseFor(SrsLine line, SrsCursor c, StepNode n, int depth)
    {
        n.Json["loopMode"] = "forEach";
        string? Var(out int at)
        {
            c.Ws();
            at = c.P;
            if (c.TryWord("_")) return "_";
            var tok = c.TryToken(out at, out bool s);
            return tok != null && !s && tok.Length > 1 && tok[0] == '$' && SrsCursor.IsIdent(tok[1..]) ? tok[1..] : null;
        }
        var v1 = Var(out int a1);
        if (v1 == null) { Error(TextProblemCodes.Syntax, "Expected for $value in $list (or for $value, $index in $list)", c, a1); return ParseBlockBody(line, n, StepTree.Roles.LoopSteps, depth); }
        string? v2 = null;
        if (c.TryChar(','))
        {
            v2 = Var(out int a2);
            if (v2 == null || v2 == "_") { Error(TextProblemCodes.Syntax, "Expected the index variable", c, a2); v2 = null; }
        }
        if (!c.TryWord("in")) { Error(TextProblemCodes.Syntax, "Expected 'in' and the list", c, c.P); return ParseBlockBody(line, n, StepTree.Roles.LoopSteps, depth); }
        var list = Var(out int a3);
        if (list == null || list == "_") Error(TextProblemCodes.Syntax, "Expected the list variable: in $list", c, a3);
        else { n.Json["forEachVariableName"] = list; n.FieldSpans["forEachVariableName"] = SpanOf(line, a3, c.P); }
        if (v1 != "_") n.Json["forEachValueVariableName"] = v1;
        if (v2 != null) n.Json["forEachIndexVariableName"] = v2;
        ParseTrailingArgs(line, c, n, StepCatalog.Get(StepType.Loop)!);
        return ParseBlockBody(line, n, StepTree.Roles.LoopSteps, depth);
    }

    private StepNode ParseSwitch(SrsLine line, SrsCursor c, StepNode n, int depth)
    {
        var (t, st) = c.ScanExpr(stopComma: false);
        if (t.Length > 0 && CheckExpr(line, t, st)) { n.Json["switchExpr"] = t; n.FieldSpans["switchExpr"] = SpanOf(line, st, st + t.Length); }
        var cases = new JsonArray();
        string? role = null; int? index = null;
        var cur = new List<StepNode>();
        bool sawDefault = false;
        while (true)
        {
            var (steps, term) = ParseBody(depth + 1);
            if (role == null && steps.Count > 0)
                Error(TextProblemCodes.MisplacedClause, "Steps before the first case are not part of any case", line, 0, line.Text.Length);
            else cur.AddRange(steps);
            if (term == null)
            {
                Error(TextProblemCodes.MissingEnd, "This switch has no 'end'", line, 0, line.Text.Length);
                if (role != null) n.Bodies.Add(new StepBody(role, index, cur));
                n.EndLine = _lines.Count > 0 ? _lines[^1].EndLine : n.Line;
                break;
            }
            var w = ClauseWord(term);
            if (w is "case" or "default" or "end" && role != null) { n.Bodies.Add(new StepBody(role, index, cur)); cur = []; }
            if (w == "case")
            {
                if (sawDefault) Error(TextProblemCodes.MisplacedClause, "case after default", term, 0, w.Length);
                var tc = new SrsCursor(term);
                tc.TryIdent();
                var values = new JsonArray();
                while (!tc.AtEnd)
                {
                    var (vt, vs) = tc.ScanExpr();
                    if (vt.Length > 0 && CheckExpr(term, vt, vs)) values.Add(vt);
                    if (!tc.TryChar(',')) break;
                }
                if (!tc.AtEnd) Error(TextProblemCodes.Syntax, $"Unexpected '{Excerpt(tc.Rest)}'", term, tc.P, term.Text.Length);
                if (values.Count == 0) Error(TextProblemCodes.MissingArgument, "case needs a value: case 1 or case 1, 2", term, 0, term.Text.Length);
                cases.Add(new JsonObject { ["id"] = "", ["values"] = values });
                role = StepTree.Roles.SwitchCases; index = cases.Count - 1;
                _i++;
                continue;
            }
            if (w == "default")
            {
                if (sawDefault) Error(TextProblemCodes.MisplacedClause, "A second default", term, 0, w.Length);
                CheckClauseTail(term);
                role = StepTree.Roles.SwitchDefault; index = null; sawDefault = true;
                _i++;
                continue;
            }
            if (w == "end")
            {
                CheckClauseTail(term);
                n.EndLine = term.EndLine;
                _i++;
                break;
            }
            Error(TextProblemCodes.MisplacedClause, $"'{w}' does not belong in a switch", term, 0, w.Length);
            _i++;
        }
        n.Json["switchCases"] = cases;
        return n;
    }

    private static readonly HashSet<string> TryArgs = new(StringComparer.OrdinalIgnoreCase) { "retry", "delay" };
    private static readonly HashSet<string> CatchArgs = new(StringComparer.OrdinalIgnoreCase) { "code", "only", "message" };

    private StepNode ParseTry(SrsLine line, SrsCursor c, StepNode n, int depth)
    {
        var spec = StepCatalog.Get(StepType.Try)!;
        ParseTrailingArgs(line, c, n, spec, TryArgs);
        string role = StepTree.Roles.TrySteps;
        var cur = new List<StepNode>();
        bool sawCatch = false;
        while (true)
        {
            var (steps, term) = ParseBody(depth + 1);
            cur.AddRange(steps);
            if (term == null)
            {
                Error(TextProblemCodes.MissingEnd, "This try has no 'end'", line, 0, line.Text.Length);
                n.Bodies.Add(new StepBody(role, null, cur));
                n.EndLine = _lines.Count > 0 ? _lines[^1].EndLine : n.Line;
                break;
            }
            var w = ClauseWord(term);
            if (w == "catch" && !sawCatch)
            {
                n.Bodies.Add(new StepBody(role, null, cur));
                cur = [];
                role = StepTree.Roles.CatchSteps; sawCatch = true;
                var tc = new SrsCursor(term);
                tc.TryIdent();
                tc.Ws();
                if (tc.Peek() == '$')
                {
                    var tok = tc.TryToken(out int vs, out _);
                    if (tok != null && tok.Length > 1 && SrsCursor.IsIdent(tok[1..])) n.Json["errorVariableName"] = tok[1..];
                    else Error(TextProblemCodes.Syntax, "Expected the message variable: catch $msg", term, vs, tc.P);
                }
                ParseTrailingArgs(term, tc, n, spec, CatchArgs);
                _i++;
                continue;
            }
            if (w == "end")
            {
                CheckClauseTail(term);
                n.Bodies.Add(new StepBody(role, null, cur));
                n.EndLine = term.EndLine;
                _i++;
                break;
            }
            Error(TextProblemCodes.MisplacedClause, $"'{w}' does not belong in a try", term, 0, w.Length);
            _i++;
        }
        return n;
    }

    private StepNode ParseGoto(SrsLine line, SrsCursor c, StepNode n)
    {
        c.Ws();
        int st = c.P;
        var name = c.TryString(out _) ?? c.TryIdent();
        if (name != null) { n.Json["labelName"] = name; n.FieldSpans["labelId"] = SpanOf(line, st, c.P); }
        ExpectEnd(c);
        return n;
    }

    private StepNode ParseCall(SrsLine line, SrsCursor c, StepNode n)
    {
        c.Ws();
        int st = c.P;
        var name = c.TryString(out _) ?? c.TryIdent();
        if (name == null) { if (!c.AtEnd) Error(TextProblemCodes.Syntax, "Expected the routine name", c, st); return n; }
        SetRef(n, line, SrsRefKind.Routine, "routineId", "routineName", name, st, c.P, "routineId");
        if (c.Peek() == '(')
        {
            c.P++;
            var args = new JsonObject();
            int as_ = c.P;
            if (!c.TryChar(')'))
            {
                while (true)
                {
                    c.Ws();
                    int ns = c.P;
                    var pn = c.TryIdent();
                    if (pn == null || !c.TryChar('=')) { Error(TextProblemCodes.Syntax, "Expected param=value", c, ns); return n; }
                    var (vt, vs) = c.ScanExpr();
                    var v = ExprOrTemplate(line, vt, vs);
                    if (v != null)
                    {
                        if (args.ContainsKey(pn)) Error(TextProblemCodes.DuplicateArgument, $"'{pn}' is given twice", line, ns, ns + pn.Length);
                        args[pn] = v;
                    }
                    if (c.TryChar(',')) continue;
                    if (c.TryChar(')')) break;
                    Error(TextProblemCodes.Syntax, "Expected , or )", c, c.P);
                    return n;
                }
            }
            n.Json["routineArgs"] = args;
            n.FieldSpans["routineArgs"] = SpanOf(line, as_ - 1, c.P);
        }
        var outs = new JsonArray();
        while (c.TryChar(','))
        {
            c.Ws();
            int os = c.P;
            if (!c.TryWord("out")) { Error(TextProblemCodes.Syntax, "Expected out param=$var", c, os); return n; }
            c.Ws();
            int ps = c.P;
            var pn = c.TryIdent();
            if (pn == null || !c.TryChar('=')) { Error(TextProblemCodes.Syntax, "Expected out param=$var", c, ps); return n; }
            c.Ws();
            var tok = c.TryToken(out int vs, out _);
            if (tok == null || tok.Length < 2 || tok[0] != '$' || !SrsCursor.IsIdent(tok[1..])) { Error(TextProblemCodes.Syntax, "Expected the variable: $name", c, vs); return n; }
            outs.Add(new JsonObject { ["key"] = pn, ["variableName"] = tok[1..] });
        }
        if (outs.Count > 0) n.Json["routineOutputs"] = outs;
        ExpectEnd(c);
        return n;
    }

    /// <summary>A value that is an expression, or text in quotes (a template): the stored text
    /// is the expression as written, or the string's content.</summary>
    private string? ExprOrTemplate(SrsLine line, string text, int st)
    {
        if (text.Length > 1 && text[0] == '"')
        {
            var sc = new SrsCursor(new SrsLine { Text = text, Map = new int[text.Length + 1] });
            var s = sc.TryString(out _);
            if (s != null && sc.AtEnd) return s;
        }
        return CheckExpr(line, text, st) ? text : null;
    }

    private StepNode ParseStatus(SrsLine line, SrsCursor c, StepNode n)
    {
        string sev = "Info";
        int ws = c.P;
        var w = c.PeekIdent();
        if (w != null && (w.Equals("info", StringComparison.OrdinalIgnoreCase) || w.Equals("warning", StringComparison.OrdinalIgnoreCase) ||
                          w.Equals("error", StringComparison.OrdinalIgnoreCase)) && !c.AtNamedArg())
        {
            c.TryIdent();
            sev = char.ToUpperInvariant(w[0]) + w[1..].ToLowerInvariant();
        }
        n.Json["statusSeverity"] = sev;
        c.Ws();
        int ms = c.P;
        if (c.Peek() == '"')
        {
            var s = c.TryString(out _);
            if (s != null)
            {
                string field = sev == "Warning" ? "statusWarning" : sev == "Error" ? "statusError" : "statusMessage";
                n.Json[field] = s;
                n.FieldSpans[field] = SpanOf(line, ms, c.P);
            }
        }
        var spec = StepCatalog.Get(StepType.StatusUpdate)!;
        ParseTrailingArgs(line, c, n, spec);
        return n;
    }

    private StepNode ParsePlugin(SrsLine line, SrsCursor c, StepNode n)
    {
        c.Ws();
        int st = c.P;
        if (!c.AtEnd && c.Peek() != ',')
        {
            var id = c.TryString(out _) ?? c.TryIdent();
            if (id == null || !c.TryChar('.')) { Error(TextProblemCodes.Syntax, "Expected plugin <pluginId>.<stepId>", c, st); c.P = c.S.Length; }
            else
            {
                c.Ws();
                var step = c.TryString(out _) ?? c.TryIdent();
                if (step == null) Error(TextProblemCodes.Syntax, "Expected the step id after the dot", c, c.P);
                else { n.Json["pluginId"] = id; n.Json["pluginStepId"] = step; n.FieldSpans["pluginId"] = SpanOf(line, st, c.P); }
            }
        }
        ParseTrailingArgs(line, c, n, StepCatalog.Get(StepType.Plugin)!);
        ParseMappingBody(line, n);
        return n;
    }

    // ── Mapping bodies ────────────────────────────────────────────────────

    private static bool LooksLikeMapLine(SrsLine l)
    {
        if (l.Kind != SrsLineKind.Content || l.Error != null) return false;
        var c = new SrsCursor(l);
        c.Ws();
        if (c.Peek() == '"') return true;
        var w = c.TryIdent();
        if (w == null) return false;
        c.Ws();
        char p = c.Peek();
        if ((p == '=' && c.Peek(1) != '=') || (p == '-' && c.Peek(1) == '>') || p == '.') return true;
        if (w.Equals("send", StringComparison.OrdinalIgnoreCase) || w.Equals("receive", StringComparison.OrdinalIgnoreCase))
            return p == '"' || SrsCursor.IsIdentStart(p);
        return false;
    }

    private void ParseMappingBody(SrsLine header, StepNode n)
    {
        if (Peek() is not { } first || !LooksLikeMapLine(first)) return;
        while (true)
        {
            var l = Peek();
            if (l == null) { Error(TextProblemCodes.MissingEnd, "These rows need an 'end'", header, 0, header.Text.Length); return; }
            if (l.Kind is SrsLineKind.Blank or SrsLineKind.Comment or SrsLineKind.Section) { _i++; continue; }
            if (l.Kind == SrsLineKind.Content && ClauseWord(l) == "end" && !LooksLikeMapLine(l))
            {
                CheckClauseTail(l);
                n.EndLine = l.EndLine;
                _i++;
                return;
            }
            if (!LooksLikeMapLine(l))
            {
                Error(TextProblemCodes.MissingEnd, "These rows need an 'end' before the next statement", header, 0, header.Text.Length);
                return;
            }
            ParseMapLine(l, n);
            _i++;
        }
    }

    private JsonArray Rows(StepNode n, string key)
    {
        if (n.Json[key] is not JsonArray a) n.Json[key] = a = new JsonArray();
        return a;
    }

    private void ParseMapLine(SrsLine l, StepNode n)
    {
        var c = new SrsCursor(l);
        c.Ws();
        string? lead = null;
        int save = c.P;
        var w = c.PeekIdent();
        if (w != null && (w.Equals("send", StringComparison.OrdinalIgnoreCase) || w.Equals("receive", StringComparison.OrdinalIgnoreCase)))
        {
            c.TryIdent();
            c.Ws();
            if (c.Peek() == '"' || SrsCursor.IsIdentStart(c.Peek())) lead = w.ToLowerInvariant();
            else c.P = save;
        }
        c.Ws();
        int ks = c.P;
        var key = c.TryString(out _) ?? c.TryIdent();
        if (key == null) { Error(TextProblemCodes.Syntax, "Expected a key", c, ks); return; }
        string? sub = null;
        if (c.Peek() == '.')
        {
            c.P++;
            sub = c.TryString(out _) ?? c.TryIdent();
            if (sub == null) { Error(TextProblemCodes.Syntax, "Expected the value name after the dot", c, c.P); return; }
        }
        c.Ws();
        bool arrow = c.Peek() == '-' && c.Peek(1) == '>';
        bool eq = c.Peek() == '=' && c.Peek(1) != '=';
        if (!arrow && !eq) { Error(TextProblemCodes.Syntax, "Expected -> $variable or = value", c, c.P); return; }
        c.P += arrow ? 2 : 1;

        string? OutVar()
        {
            c.Ws();
            var tok = c.TryToken(out int vs, out _);
            if (tok == null || tok.Length < 2 || tok[0] != '$' || !SrsCursor.IsIdent(tok[1..])) { Error(TextProblemCodes.Syntax, "Expected the variable: $name", c, vs); return null; }
            ExpectEnd(c);
            return tok[1..];
        }

        JsonObject? SendRow()
        {
            c.Ws();
            var row = new JsonObject { ["key"] = key, ["expr"] = "" };
            int vs = c.P;
            if (c.TryWord("list")) { var v = OutVar(); if (v == null) return null; row["listVar"] = v; return row; }
            if (c.TryWord("image")) { var v = OutVar(); if (v == null) return null; row["imageVar"] = v; return row; }
            var (t, ts) = c.ScanExpr(stopComma: false);
            if (!CheckExpr(l, t, ts)) return null;
            row["expr"] = t;
            return row;
        }

        switch (n.Type)
        {
            case StepType.RunVision:
            {
                if (!arrow || sub == null || lead != null) { Error(TextProblemCodes.Syntax, "A Run Vision row is: \"Tool\".value -> $variable", l, 0, l.Text.Length); return; }
                var v = OutVar();
                if (v == null) return;
                string vpId = n.Json["visionProgramId"]?.ToString() ?? "";
                var toolId = _names.ToolId(vpId, key);
                if (toolId == null)
                {
                    toolId = key;
                    if (_names.Tools(vpId).Count > 0 && !_names.Tools(vpId).Any(t => t.Id == key))
                        Warn(Validation.ValidationCodes.UnknownVisionTool, $"The vision program has no tool named '{key}' — kept as written", l, ks, ks + key.Length + 2);
                }
                Rows(n, "toolOutputs").Add(new JsonObject { ["toolId"] = toolId, ["value"] = sub, ["variable"] = v });
                return;
            }
            case StepType.HttpRequest:
            {
                if (lead == "receive" && arrow && sub == null) { var v = OutVar(); if (v != null) Rows(n, "jsonInbound").Add(new JsonObject { ["key"] = key, ["variableName"] = v }); return; }
                if (lead == "send" && eq && sub == null) { var r = SendRow(); if (r != null) Rows(n, "jsonOutbound").Add(r); return; }
                Error(TextProblemCodes.Syntax, "An HTTP row is: send \"key\" = value  or  receive \"key\" -> $variable", l, 0, l.Text.Length);
                return;
            }
            case StepType.HttpReceive:
            {
                if ((lead == "receive" || lead == null) && arrow && sub == null) { var v = OutVar(); if (v != null) Rows(n, "httpReceiveInbound").Add(new JsonObject { ["key"] = key, ["variableName"] = v }); return; }
                Error(TextProblemCodes.Syntax, "An HTTP Receive row is: receive \"key\" -> $variable", l, 0, l.Text.Length);
                return;
            }
            case StepType.ExportData:
            {
                if (lead == null && eq && sub == null) { var r = SendRow(); if (r != null) Rows(n, "exportColumns").Add(r); return; }
                Error(TextProblemCodes.Syntax, "An Export Data row is: \"column\" = value  (or = list $v / = image $v)", l, 0, l.Text.Length);
                return;
            }
            case StepType.Plugin:
            {
                if (lead != null || sub != null) { Error(TextProblemCodes.Syntax, "A plugin row is: param = value  or  output -> $variable", l, 0, l.Text.Length); return; }
                if (arrow) { var v = OutVar(); if (v != null) Rows(n, "pluginOutputs").Add(new JsonObject { ["key"] = key, ["variableName"] = v }); return; }
                var (t, ts) = c.ScanExpr(stopComma: false);
                var val = ExprOrTemplate(l, t, ts);
                if (val == null) return;
                if (n.Json["pluginParams"] is not JsonObject pp) n.Json["pluginParams"] = pp = new JsonObject();
                if (pp.ContainsKey(key)) Error(TextProblemCodes.DuplicateArgument, $"'{key}' is given twice", l, ks, ks + key.Length);
                pp[key] = val;
                return;
            }
        }
        Error(TextProblemCodes.Syntax, "This step has no rows", l, 0, l.Text.Length);
    }

    // ── Required fields ────────────────────────────────────────────────────

    private void CheckRequired(StepNode n)
    {
        if (n.With != null) return; // machine-written: the @with carries what the sugar cannot
        var spec = StepCatalog.Get(n.Type);
        if (spec == null) return;
        foreach (var f in spec.Fields)
        {
            if (!f.Required || !Visible(n, spec, f)) continue;
            if (f.Kind is K.Steps or K.Branches or K.Cases) continue;
            bool present = f.StoredProps.Any(p => n.Json[p] != null) ||
                           (f.ExprKey != null && n.Json["expressions"] is JsonObject ex && ex[f.ExprKey] != null) ||
                           (f.Kind == K.LabelRef && n.Json["labelName"] != null);
            if (!present)
            {
                var span = n.Span;
                _doc.Problems.Add(new TextProblem
                {
                    Line = span.Line, Col = span.Col, EndLine = span.EndLine, EndCol = span.EndCol, Severity = "error",
                    Code = TextProblemCodes.MissingArgument, Message = $"{spec.Label} needs {f.Label.ToLowerInvariant()}", Source = "parse",
                });
            }
        }
    }
}
