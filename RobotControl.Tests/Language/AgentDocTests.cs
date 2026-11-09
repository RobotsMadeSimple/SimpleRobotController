using System.Globalization;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using Controller.RobotControl;
using Controller.RobotControl.Language;
using Controller.RobotControl.Programs;
using K = Controller.RobotControl.Programs.FieldKinds;

namespace RobotControl.Tests.Language;

/// <summary>
/// docs/AGENT-PROGRAMMING.md — the context file for AI agents that write programs — keeps three
/// generated blocks (step index, per-step field reference, error codes) equal to the step catalog,
/// and every complete example in it (a <c>```srs</c> block starting with <c>#!srs 1</c>) must parse
/// with no errors and already be in the formatter's canonical form. Regenerate with
/// <c>scripts/update-agent-doc.ps1</c> (or set <c>SRS_UPDATE_DOCS=1</c> and run these tests).
/// </summary>
public class AgentDocTests
{
    private static string DocPath([System.Runtime.CompilerServices.CallerFilePath] string here = "") =>
        Path.GetFullPath(Path.Combine(Path.GetDirectoryName(here)!, "..", "..", "docs", "AGENT-PROGRAMMING.md"));

    private static string Inv(object? v) => Convert.ToString(v, CultureInfo.InvariantCulture) ?? "";

    private static string Cell(string s) => s.Replace("|", "\\|").Replace("\n", " ");

    /// <summary>The text word for an enum value (stopwatch/gripper/conveyor have their own words).</summary>
    private static string Word(StepSpec s, FieldSpec f, string v) =>
        SrsParser.EnumWords.TryGetValue((Enum.Parse<StepType>(s.Type), f.Key), out var words)
            ? words.FirstOrDefault(w => w.Value == v).Word ?? v : v;

    private static string TextSlot(FieldSpec f) => f.Kind switch
    {
        K.Steps => "body",
        K.Branches => "`elif` blocks",
        K.Cases => "`case` blocks",
        K.Json => "`@with` only",
        _ when f.Hidden => "`@with` only",
        _ when f.Position != null => $"pos {f.Position + 1}",
        _ when f.Arg.Length > 0 => $"`{f.Arg}=`",
        K.ExprMap or K.MappingOut or K.Rows => "rows / sugar",
        _ => "sugar",
    };

    private static string KindText(FieldSpec f)
    {
        var k = f.Kind;
        if (f.VarKinds is { Count: > 0 } vk) k += " (" + string.Join(", ", vk) + ")";
        return k;
    }

    private static string DefaultText(FieldSpec f) => f.Default switch
    {
        null => "",
        bool b => b ? "true" : "false",
        string str => $"\"{str}\"",
        string[] a => "[" + string.Join(", ", a.Select(x => $"\"{x}\"")) + "]",
        var d => Inv(d),
    };

    private static string Allowed(StepSpec s, FieldSpec f)
    {
        var parts = new List<string>();
        if (f.Options is { Count: > 0 } opts && f.Kind != K.TextList)
            parts.Add(string.Join(" ", opts.Select(o =>
            {
                var v = Inv(o.Value);
                var w = f.Kind == K.Enum ? Word(s, f, v) : v;
                return w == v ? $"`{v}`" : $"`{v}`(text `{w}`)";
            })));
        if (f.Kind == K.TextList && f.Key == "catchCodes") parts.Add("catchable codes or your own raise codes");
        if (f.Min != null) parts.Add($"min {Inv(f.Min)}");
        if (f.Max != null) parts.Add($"max {Inv(f.Max)}");
        return string.Join("; ", parts);
    }

    private static string Notes(FieldSpec f)
    {
        var parts = new List<string>();
        if (f.Required) parts.Add("**required**");
        if (f.ShowIf is { } si) parts.Add($"when {si.Field} ∈ {{{string.Join(", ", si.In)}}}");
        if (f.Help is { } h) parts.Add(h);
        return string.Join("; ", parts);
    }

    private static string Expr(FieldSpec f) =>
        f.ExprKey != null ? $"`{f.ExprKey}`"
        : f.ExprKeys is { Count: > 0 } ks ? string.Join(" ", ks.Select(k => $"`{k}`")) : "";

    /// <summary>Generated block "steps": one row per step type.</summary>
    public static string GenerateIndex()
    {
        var sb = new StringBuilder("\n| Type | Text keyword | Category | Body | Runs in background |\n|---|---|---|---|---|\n");
        foreach (var s in StepCatalog.Steps)
        {
            var kw = s.Keyword switch
            {
                "$" when s.Type == nameof(StepType.ListOp) => "`$list.append(…)` `$list[i] = …` …",
                "$" => "`$var = …`",
                "#" => "`# …` / `## …`",
                var k => $"`{k}`",
            };
            if (s.Aliases.Count > 0) kw += " " + string.Join(" ", s.Aliases.Select(a => $"`{a}`"));
            sb.Append($"| `{s.Type}` | {kw} | {s.Category} | {s.Body ?? ""} | {(s.Background ? "yes" : "**no — skipped**")} |\n");
        }
        return sb.ToString();
    }

    /// <summary>Generated block "fields": per step, its text syntax and every field.</summary>
    public static string GenerateFields()
    {
        var sb = new StringBuilder();
        foreach (var cat in StepCatalog.Categories)
        {
            var steps = StepCatalog.Steps.Where(s => s.Category == cat.Key).ToList();
            if (steps.Count == 0) continue;
            sb.Append($"\n### Fields: {cat.Label}\n");
            foreach (var s in steps)
            {
                sb.Append($"\n#### STEP {s.Type} — {s.Label}\n\n");
                sb.Append($"{s.Description}. Background: {(s.Background ? "runs" : "skipped")}. Body: {s.Body ?? "none"}.  \n");
                sb.Append($"Text: `{s.Syntax}`\n\n");
                if (s.Fields.Count == 0) { sb.Append("No fields.\n"); continue; }
                sb.Append("| Text | JSON field(s) | Kind | Unit | Default | Allowed | Expr key | Notes |\n|---|---|---|---|---|---|---|---|\n");
                foreach (var f in s.Fields)
                {
                    var json = string.Join(" ", f.StoredProps.Select(p => $"`{p}`"));
                    sb.Append($"| {TextSlot(f)} | {json} | {Cell(KindText(f))} | {Cell(f.Unit ?? "")} | {Cell(DefaultText(f))} | {Cell(Allowed(s, f))} | {Expr(f)} | {Cell(Notes(f))} |\n");
                }
            }
        }
        return sb.ToString();
    }

    /// <summary>Generated block "errors": every runtime error code.</summary>
    public static string GenerateErrors()
    {
        var sb = new StringBuilder("\n| Code | Meaning | Try can catch |\n|---|---|---|\n");
        foreach (var e in ErrorCodes.All)
            sb.Append($"| `{e.Code}` | {e.Label} | {(e.Catchable ? "yes" : "**no — fatal**")} |\n");
        sb.Append("| any other identifier (`raise …, code=myCode`) | your own RaiseError code | yes |\n");
        return sb.ToString();
    }

    private static readonly (string Name, Func<string> Gen)[] Blocks =
        [("steps", GenerateIndex), ("fields", GenerateFields), ("errors", GenerateErrors)];

    private static string Begin(string name) => $"<!-- BEGIN GENERATED {name} (AgentDocTests) -->";
    private static string End(string name) => $"<!-- END GENERATED {name} -->";

    [Fact]
    public void TheGeneratedReferenceMatchesTheCatalog()
    {
        var path = DocPath();
        Assert.True(File.Exists(path), path);
        var doc = File.ReadAllText(path).Replace("\r\n", "\n");
        var updated = doc;
        foreach (var (name, gen) in Blocks)
        {
            int b = updated.IndexOf(Begin(name), StringComparison.Ordinal), e = updated.IndexOf(End(name), StringComparison.Ordinal);
            Assert.True(b >= 0 && e > b, $"markers for '{name}' missing in docs/AGENT-PROGRAMMING.md");
            updated = updated[..(b + Begin(name).Length)] + gen() + updated[e..];
        }
        if (Environment.GetEnvironmentVariable("SRS_UPDATE_DOCS") == "1" && updated != doc)
        {
            File.WriteAllText(path, updated);
            return;
        }
        Assert.Equal(updated, doc);
    }

    /// <summary>The repository the examples are written against (the doc's "Example cell").</summary>
    private static SrsNameTable ExampleNames()
    {
        var t = new SrsNameTable();
        foreach (var p in new[] { "Home", "Infeed", "TrayOrigin", "CameraClear", "PickZ", "Drop" })
            t.Add(SrsRefKind.Point, "pt-" + p, p);
        t.Add(SrsRefKind.Tool, "tool-gripper", "Gripper")
         .Add(SrsRefKind.Grid, "grid-tray", "Tray")
         .Add(SrsRefKind.Vision, "vp-find-parts", "Find Parts")
         .Add(SrsRefKind.Routine, "routine-place", "PlaceInSlot")
         .Add(SrsRefKind.Program, "bg-door", "DoorMonitor");
        t.AddTool("vp-find-parts", "t-parts", "Parts");
        return t;
    }

    public static IEnumerable<object[]> Examples()
    {
        var doc = File.ReadAllText(DocPath()).Replace("\r\n", "\n");
        int i = 0;
        foreach (Match m in Regex.Matches(doc, "```srs\n(.*?)```", RegexOptions.Singleline))
        {
            i++;
            if (m.Groups[1].Value.StartsWith("#!srs 1", StringComparison.Ordinal))
                yield return [i, m.Groups[1].Value];
        }
    }

    [Theory]
    [MemberData(nameof(Examples))]
    public void EveryCompleteExampleParsesAndIsCanonical(int block, string text)
    {
        var names = ExampleNames();
        var parsed = SrsLanguage.Parse(text, null, names);
        var problems = parsed.Problems.Where(p => p.Severity == "error" || p.Code.StartsWith("unknown", StringComparison.Ordinal)).ToList();
        Assert.True(parsed.Ok && problems.Count == 0,
            $"srs block {block} has problems:\n" + string.Join("\n", problems.Select(p => $"{p.Line}:{p.Col} {p.Severity} {p.Code}: {p.Message}")));
        var formatted = SrsLanguage.Format(parsed.Program!, names, includeHeader: text.Contains("\n@program ")).Text;
        Assert.Equal(text.TrimEnd('\n'), formatted.TrimEnd('\n'));
    }

    [Fact]
    public void TheDocumentHasCompleteExamples() => Assert.True(Examples().Count() >= 6);

    /// <summary>A <c>```json</c> program example (an object with "steps") must format to the
    /// <c>```srs</c> block just before it.</summary>
    [Fact]
    public void TheJsonExampleFormatsToItsText()
    {
        var doc = File.ReadAllText(DocPath()).Replace("\r\n", "\n");
        var blocks = Regex.Matches(doc, "```(srs|json)\n(.*?)```", RegexOptions.Singleline).ToList();
        int checkedCount = 0;
        for (int i = 1; i < blocks.Count; i++)
        {
            if (blocks[i].Groups[1].Value != "json" || !blocks[i].Groups[2].Value.Contains("\"steps\"")) continue;
            var srs = blocks[i - 1];
            Assert.Equal("srs", srs.Groups[1].Value);
            var program = JsonSerializer.Deserialize<BuiltProgram>(blocks[i].Groups[2].Value, SrsTestKit.Json)!;
            var text = SrsLanguage.Format(program, ExampleNames()).Text;
            Assert.Equal(srs.Groups[2].Value.TrimEnd('\n'), text.TrimEnd('\n'));
            checkedCount++;
        }
        Assert.True(checkedCount >= 1, "no JSON program example found");
    }
}
