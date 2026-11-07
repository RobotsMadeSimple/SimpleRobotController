using System.Text.Json;
using System.Text.Json.Nodes;
using Controller.RobotControl.Programs;
using Controller.RobotControl.Validation;

namespace Controller.RobotControl.Language;

/// <summary>
/// Simple Robot Script — the text view of a <see cref="BuiltProgram"/> (docs/program-text.md).
/// <see cref="Format"/> prints a program; <see cref="Parse"/> reads text back into one, with
/// ids reconciled against a base program. The guarantee, for every program P:
/// <c>Normalize(Parse(Format(P), base: P)) == Normalize(P)</c> and Format is idempotent.
/// </summary>
public static class SrsLanguage
{
    public static FormatProgramTextResult Format(BuiltProgram program, ISrsNames names, bool includeIds = false, bool includeHeader = false)
    {
        var r = new NameResolver(names);
        var p = ProgramNormalizer.Normalize(program, r);
        return SrsPrinter.Print(p, r, includeIds, includeHeader);
    }

    /// <summary>The canonical JSON of a program (the round-trip comparison form).</summary>
    public static JsonObject NormalizeJson(BuiltProgram program, ISrsNames names) =>
        ProgramNormalizer.NormalizeJson(program, new NameResolver(names));

    /// <param name="validate">Runs the program validator on a parsed program (null = skip).</param>
    public static ParseProgramTextResult Parse(string text, BuiltProgram? @base, ISrsNames names,
                                               Func<BuiltProgram, IEnumerable<ValidationProblem>>? validate = null)
    {
        var r = new NameResolver(names);
        var source = new SrsSource(text ?? "");
        var doc = SrsParser.Parse(text ?? "", r);
        var problems = doc.Problems;

        // Variables
        var vars = new List<ProgramVariable>();
        var varNodes = new Dictionary<ProgramVariable, VarNode>(ReferenceEqualityComparer.Instance);
        foreach (var vn in doc.Vars)
        {
            var json = (JsonObject)vn.Json.DeepClone();
            if (vn.ExplicitId != null) json["id"] = vn.ExplicitId;
            if (vn.With != null) { DropUnknown(vn.With, SrsJson.VarProps, vn.Span, problems, "variable"); SrsJson.Merge(json, vn.With); }
            ProgramVariable v;
            try { v = SrsJson.Var(json); }
            catch (Exception ex) when (ex is JsonException or InvalidOperationException or FormatException)
            {
                Add(problems, vn.Span, "error", TextProblemCodes.Syntax, "@with does not fit this variable: " + FirstSentence(ex.Message));
                v = SrsJson.Var(vn.Json);
            }
            vars.Add(v);
            varNodes[v] = vn;
        }
        var kinds = new Dictionary<string, bool>(StringComparer.Ordinal);
        foreach (var v in vars) kinds.TryAdd(v.Name, v.IsString == true);

        // Steps
        var nodes = new Dictionary<ProgramStep, StepNode>(ReferenceEqualityComparer.Instance);
        foreach (var n in Walk(doc.Steps)) CheckStringAssignment(n, kinds, problems);
        var steps = doc.Steps.Select(n => BuildStep(n, nodes, problems, source)).ToList();

        // Explicit ids
        var claimed = new HashSet<string>(StringComparer.Ordinal);
        foreach (var s in StepTree.Walk(steps))
        {
            if (string.IsNullOrEmpty(s.Id)) continue;
            if (!claimed.Add(s.Id))
            {
                var span = nodes.TryGetValue(s, out var dn) ? dn.IdSpan ?? dn.Span : new SrsSpan(1, 1, 1, 2);
                Add(problems, span, "error", TextProblemCodes.DuplicateId, $"Two steps have the id \"{s.Id}\" — remove one @id");
            }
        }
        var claimedVars = vars.Where(v => !string.IsNullOrEmpty(v.Id)).Select(v => v.Id).ToHashSet();

        var baseNorm = @base != null ? ProgramNormalizer.Normalize(@base, r) : null;
        IdReconciler.Reconcile(baseNorm?.Steps, steps, claimed, st => nodes.TryGetValue(st, out var nn) ? nn : null);
        IdReconciler.ReconcileVariables(baseNorm?.Variables, vars, claimedVars);

        // goto → labelId, by the (reconciled) labels of this program
        var labels = ProgramNormalizer.LabelMap(steps);
        foreach (var s in StepTree.Walk(steps))
            if (s.Type == StepType.GoToLabel && nodes.TryGetValue(s, out var gn)) ResolveGoto(s, gn, labels, problems, source);

        // Program
        BuiltProgram program;
        if (@base != null)
        {
            program = SrsJson.Clone(@base);
        }
        else if (doc.ProgramHeader != null)
        {
            try { program = doc.ProgramHeader.Deserialize<BuiltProgram>(SrsJson.Options) ?? new BuiltProgram(); }
            catch (Exception ex) when (ex is JsonException or InvalidOperationException)
            {
                Add(problems, new SrsSpan(1, 1, 1, 2), "warning", TextProblemCodes.Syntax, "@program could not be read: " + FirstSentence(ex.Message));
                program = new BuiltProgram();
            }
            program.Id = "";
        }
        else program = new BuiltProgram();
        program.Steps = steps;
        program.Variables = vars.Count > 0 || @base?.Variables != null ? vars : null;

        var map = new List<ProgramTextMapEntry>();
        foreach (var (s, path) in StepTree.WalkWithPaths(steps))
            if (nodes.TryGetValue(s, out var n))
                map.Add(new ProgramTextMapEntry { StepId = s.Id, StepPath = path, Line = n.Line, EndLine = Math.Max(n.Line, n.EndLine) });

        bool ok = !problems.Any(p => p.Severity == "error");
        if (ok && validate != null)
        {
            var varSpans = vars.Select((v, i) => (i, varNodes.TryGetValue(v, out var vn) ? vn.Span : new SrsSpan(1, 1, 1, 2)))
                               .ToDictionary(x => x.i, x => x.Item2);
            var byId = new Dictionary<string, (ProgramStep Step, StepNode Node)>(StringComparer.Ordinal);
            foreach (var (s, n) in nodes) byId.TryAdd(s.Id, (s, n));
            foreach (var vp in validate(program))
                problems.Add(MapValidation(vp, byId, map, varSpans));
        }
        problems.Sort((a, b) => a.Line != b.Line ? a.Line.CompareTo(b.Line) : a.Col.CompareTo(b.Col));
        return new ParseProgramTextResult { Ok = ok, Program = ok ? program : null, Problems = problems, Map = map };
    }

    // ── Building steps ─────────────────────────────────────────────────────

    private static IEnumerable<StepNode> Walk(IEnumerable<StepNode> nodes)
    {
        foreach (var n in nodes)
        {
            yield return n;
            foreach (var b in n.Bodies)
                foreach (var c in Walk(b.Steps)) yield return c;
        }
    }

    /// <summary>A node's step: its JSON, the @with merged over it, then its bodies.</summary>
    internal static ProgramStep BuildStep(StepNode n, Dictionary<ProgramStep, StepNode>? nodes, List<TextProblem> problems, SrsSource? source)
    {
        var json = (JsonObject)n.Json.DeepClone();
        json["type"] = n.Type.ToString();
        if (n.ExplicitId != null) json["id"] = n.ExplicitId;
        if (n.With != null)
        {
            DropUnknown(n.With, SrsJson.StepProps, n.Span, problems, "step");
            SrsJson.Merge(json, n.With);
        }
        ProgramStep step;
        try { step = SrsJson.Step(json); }
        catch (Exception ex) when (ex is JsonException or InvalidOperationException or FormatException)
        {
            Add(problems, n.Span, "error", TextProblemCodes.Syntax, "@with does not fit this step: " + FirstSentence(ex.Message));
            var plain = (JsonObject)n.Json.DeepClone();
            plain["type"] = n.Type.ToString();
            try { step = SrsJson.Step(plain); } catch { step = new ProgramStep { Type = n.Type }; }
        }
        foreach (var b in n.Bodies)
        {
            if (n.With != null && n.With.ContainsKey(b.Role)) continue; // the @with replaced it
            var list = b.Steps.Select(c => BuildStep(c, nodes, problems, source)).ToList();
            switch (b.Role)
            {
                case StepTree.Roles.LoopSteps:     step.LoopSteps = list; break;
                case StepTree.Roles.IfSteps:       step.IfSteps = list; break;
                case StepTree.Roles.ElseSteps:     step.ElseSteps = list; break;
                case StepTree.Roles.SwitchDefault: step.SwitchDefault = list; break;
                case StepTree.Roles.TrySteps:      step.TrySteps = list; break;
                case StepTree.Roles.CatchSteps:    step.CatchSteps = list; break;
                case StepTree.Roles.ElseIfBranches:
                    if (step.ElseIfBranches != null && b.Index is int ei && ei < step.ElseIfBranches.Count) step.ElseIfBranches[ei].Steps = list;
                    break;
                case StepTree.Roles.SwitchCases:
                    if (step.SwitchCases != null && b.Index is int ci && ci < step.SwitchCases.Count) step.SwitchCases[ci].Steps = list;
                    break;
            }
        }
        if (nodes != null) nodes[step] = n;
        return step;
    }

    private static void DropUnknown(JsonObject with, HashSet<string> known, SrsSpan span, List<TextProblem> problems, string what)
    {
        foreach (var k in with.Select(kv => kv.Key).ToList())
            if (!known.Contains(k))
            {
                with.Remove(k);
                Add(problems, span, "warning", TextProblemCodes.UnknownField, $"@with: '{k}' is not a {what} field, so it is dropped");
            }
    }

    /// <summary>A <c>goto NAME</c>: the labelId of the program's label of that name (unless the
    /// @with already gives one).</summary>
    internal static void ResolveGoto(ProgramStep s, StepNode n, LabelTable labels, List<TextProblem> problems, SrsSource? source)
    {
        if (n.With != null && n.With.ContainsKey("labelId")) return;
        if (string.IsNullOrEmpty(s.LabelName)) return;
        var id = labels.IdOf(s.LabelName);
        if (id != null || labels.Has(s.LabelName))
        {
            s.LabelId = id;
            return;
        }
        s.LabelId = null;
        var span = n.FieldSpans.TryGetValue("labelId", out var sp) ? sp : n.Span;
        Add(problems, span, "warning", TextProblemCodes.UnknownLabel, $"No label named '{s.LabelName}' in this program");
    }

    /// <summary><c>stringRequired</c>: a string variable takes text in quotes (a template); a
    /// number or boolean variable takes an expression. Returns the number of problems added.</summary>
    internal static int CheckStringAssignment(StepNode n, IReadOnlyDictionary<string, bool> kinds, List<TextProblem> problems)
    {
        if (n.Type != StepType.SetVariable || n.With != null) return 0;
        var name = n.Json["variableName"]?.GetValue<string>();
        if (name == null || n.Json["variableExpr"] == null || !kinds.TryGetValue(name, out bool isString)) return 0;
        if (isString && !n.RhsIsString)
        {
            Add(problems, n.RhsSpan, "error", TextProblemCodes.StringRequired, $"${name} is a string variable: put its value in quotes, e.g. \"Bin {{$i}}\"");
            return 1;
        }
        if (!isString && n.RhsIsString)
        {
            Add(problems, n.RhsSpan, "error", TextProblemCodes.StringRequired, $"${name} holds a number: write an expression, not text in quotes");
            return 1;
        }
        return 0;
    }

    private static void Add(List<TextProblem> problems, SrsSpan span, string severity, string code, string message) =>
        problems.Add(new TextProblem
        {
            Line = span.Line, Col = span.Col, EndLine = span.EndLine, EndCol = span.EndCol,
            Severity = severity, Code = code, Message = message, Source = "parse",
        });

    private static string FirstSentence(string m)
    {
        int i = m.IndexOf(". ", StringComparison.Ordinal);
        return i > 0 ? m[..i] : m;
    }

    // ── Validation problems → text ─────────────────────────────────────────

    private static TextProblem MapValidation(ValidationProblem vp, Dictionary<string, (ProgramStep Step, StepNode Node)> byId,
                                             List<ProgramTextMapEntry> map, Dictionary<int, SrsSpan> varSpans)
    {
        SrsSpan span = new(1, 1, 1, 2);
        string? field = vp.Field;
        if (vp.StepId != null && byId.TryGetValue(vp.StepId, out var hit))
        {
            span = hit.Node.Span;
            if (field != null && FieldSpan(hit.Step.Type, hit.Node, field) is { } fs) span = fs;
        }
        else if (vp.StepPath.StartsWith("variables[", StringComparison.Ordinal) &&
                 int.TryParse(vp.StepPath["variables[".Length..].Split(']')[0], out int vi) && varSpans.TryGetValue(vi, out var vs))
            span = vs;
        else
        {
            // A step reached through a routine call (or not in the text): the longest matching path prefix.
            var best = map.Where(m => vp.StepPath.StartsWith(m.StepPath, StringComparison.Ordinal))
                          .OrderByDescending(m => m.StepPath.Length).FirstOrDefault();
            if (best != null && byId.TryGetValue(best.StepId, out var b)) span = b.Node.Span;
        }
        return new TextProblem
        {
            Line = span.Line, Col = span.Col, EndLine = span.EndLine, EndCol = span.EndCol,
            Severity = vp.Severity, Code = vp.Code, Message = vp.Message, StepId = vp.StepId, Field = vp.Field, Source = "validate",
        };
    }

    /// <summary>The span of a validator field path ("expressions.offsetZ", "condition.items[0].expr",
    /// "pointId") through the catalog key that stores it.</summary>
    private static SrsSpan? FieldSpan(StepType type, StepNode n, string field)
    {
        string head = field.Split('.', '[')[0];
        if (head == "expressions" && field.Length > "expressions.".Length)
        {
            var key = field["expressions.".Length..].Split('.', '[')[0];
            var spec = StepCatalog.Get(type);
            var f = spec?.Fields.FirstOrDefault(x => x.ExprKey == key || (x.ExprKeys?.Contains(key) ?? false));
            if (f != null && n.FieldSpans.TryGetValue(f.Key, out var es)) return es;
            if (n.FieldSpans.TryGetValue(key, out var ks)) return ks;
            return null;
        }
        if (n.FieldSpans.TryGetValue(head, out var s)) return s;
        var sp = StepCatalog.Get(type);
        var owner = sp?.Fields.FirstOrDefault(x => x.StoredProps.Contains(head));
        if (owner != null && n.FieldSpans.TryGetValue(owner.Key, out var os)) return os;
        return null;
    }
}
