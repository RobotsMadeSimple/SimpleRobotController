using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.RegularExpressions;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Programs;

namespace RobotControl.Tests;

/// <summary>
/// The step catalog's coverage tests (docs/program-language.md §5.2) — the vision rework's
/// "test every tool" pattern: the catalog must describe every step type and every ProgramStep
/// property, every expression key the executor reads, and every error code a handler fails with.
/// </summary>
public class StepCatalogTests
{
    private static readonly IReadOnlyList<StepSpec> Steps = StepCatalog.Steps;

    private static readonly HashSet<string> StepProps =
        typeof(ProgramStep).GetProperties(BindingFlags.Public | BindingFlags.Instance)
            .Where(p => p.GetCustomAttribute<JsonIgnoreAttribute>() == null)
            .Select(p => p.GetCustomAttribute<JsonPropertyNameAttribute>()?.Name ?? p.Name)
            .ToHashSet(StringComparer.Ordinal);

    private static readonly string[] Composites = ["target", "via", "offset", "override", "toolOffset"];

    private static string RepoRoot([CallerFilePath] string here = "") =>
        Path.GetFullPath(Path.Combine(Path.GetDirectoryName(here)!, ".."));

    private static IEnumerable<string> SourceFiles(string relDir) =>
        Directory.EnumerateFiles(Path.Combine(RepoRoot(), relDir), "*.cs", SearchOption.AllDirectories);

    // 1 ─────────────────────────────────────────────────────────────────────
    [Fact]
    public void EveryStepTypeExceptUnknownHasExactlyOneEntry()
    {
        var byType = Steps.GroupBy(s => s.Type).ToDictionary(g => g.Key, g => g.Count());
        var expected = Enum.GetNames<StepType>().Where(n => n != nameof(StepType.Unknown)).ToList();
        Assert.Equal(expected.OrderBy(x => x), byType.Keys.OrderBy(x => x));
        Assert.All(byType, kv => Assert.Equal(1, kv.Value));
        Assert.Null(StepCatalog.Get(StepType.Unknown));
        Assert.Equal("MoveC", StepCatalog.Get(StepType.MoveC)!.Type);
    }

    // 2 ─────────────────────────────────────────────────────────────────────
    [Fact]
    public void EveryFieldKeyResolvesToProgramStepProperties()
    {
        var bad = new List<string>();
        foreach (var s in Steps)
            foreach (var f in s.Fields)
            {
                if (Composites.Contains(f.Key) && f.Props == null) bad.Add($"{s.Type}.{f.Key}: composite without props");
                if (!Composites.Contains(f.Key) && !StepProps.Contains(f.Key)) bad.Add($"{s.Type}.{f.Key}: not a ProgramStep property");
                foreach (var p in f.StoredProps)
                    if (!StepProps.Contains(p)) bad.Add($"{s.Type}.{f.Key}: prop '{p}' is not a ProgramStep property");
            }
        Assert.True(bad.Count == 0, string.Join("\n", bad));
    }

    // 3 ─────────────────────────────────────────────────────────────────────
    [Fact]
    public void EveryProgramStepPropertyIsClaimed()
    {
        var claimed = Steps.SelectMany(s => s.Fields).SelectMany(f => f.StoredProps)
            .Concat(StepCatalog.CommonFields).Concat(StepCatalog.LegacyFields)
            .ToHashSet(StringComparer.Ordinal);
        var unclaimed = StepProps.Where(p => !claimed.Contains(p)).OrderBy(p => p).ToList();
        Assert.True(unclaimed.Count == 0,
            "ProgramStep properties no catalog field claims (add a FieldSpec, or list them as common/legacy): " +
            string.Join(", ", unclaimed));
        Assert.All(StepCatalog.CommonFields.Concat(StepCatalog.LegacyFields), p => Assert.Contains(p, StepProps));
    }

    // 4 ─────────────────────────────────────────────────────────────────────
    private static readonly Regex EvalKey =
        new(@"\.(EvalField|OptionalField)\(\s*\w+\s*,\s*""([A-Za-z]+)""", RegexOptions.Compiled);

    [Fact]
    public void EveryExpressionKeyTheExecutorReadsIsInTheCatalog()
    {
        var read = SourceFiles(Path.Combine("RobotControl", "Execution"))
            .SelectMany(f => EvalKey.Matches(File.ReadAllText(f)).Select(m => (File: Path.GetFileName(f), Key: m.Groups[2].Value)))
            .ToList();
        Assert.True(read.Count > 30, $"expected the executor to read many expression keys, found {read.Count} — did the regex rot?");

        var catalog = Steps.SelectMany(s => s.Fields)
            .SelectMany(f => (f.ExprKeys ?? []).Append(f.ExprKey ?? ""))
            .Where(k => k != "").ToHashSet(StringComparer.Ordinal);
        var missing = read.Where(r => !catalog.Contains(r.Key)).Select(r => $"{r.Key} ({r.File})").Distinct().ToList();
        Assert.True(missing.Count == 0, "Expression keys read by the executor but not in any catalog exprKey: " + string.Join(", ", missing));
    }

    // 5 ─────────────────────────────────────────────────────────────────────
    private static readonly Regex FailCode =
        new(@"\.Fail\(\s*(?:""(?<lit>[A-Za-z]+)""|ErrorCodes\.(?<const>\w+))", RegexOptions.Compiled);

    [Fact]
    public void EveryErrorCodeHandlersFailWithIsInTheCatalog()
    {
        var consts = typeof(ErrorCodes).GetFields(BindingFlags.Public | BindingFlags.Static)
            .Where(f => f.IsLiteral).ToDictionary(f => f.Name, f => (string)f.GetRawConstantValue()!);
        var listed = ErrorCodes.All.Select(e => e.Code).ToHashSet(StringComparer.Ordinal);
        Assert.Equal(consts.Values.OrderBy(x => x), listed.OrderBy(x => x));
        Assert.Equal(listed.Count, ErrorCodes.All.Count); // no duplicates

        var used = SourceFiles(Path.Combine("RobotControl", "Execution"))
            .SelectMany(f => FailCode.Matches(File.ReadAllText(f)))
            .Select(m => m.Groups["lit"].Success ? m.Groups["lit"].Value : consts.GetValueOrDefault(m.Groups["const"].Value, "?" + m.Groups["const"].Value))
            .Distinct().ToList();
        var missing = used.Where(c => !listed.Contains(c)).ToList();
        Assert.True(missing.Count == 0, "ctx.Fail codes not in ErrorCodes.All: " + string.Join(", ", missing));
    }

    [Fact]
    public void CatchabilityFollowsTheSpec()
    {
        Assert.True(ErrorCodes.IsCatchable(ErrorCodes.WaitTimeout));
        Assert.True(ErrorCodes.IsCatchable(ErrorCodes.UserError));
        Assert.True(ErrorCodes.IsCatchable("noParts")); // a RaiseError code of the user's own
        Assert.False(ErrorCodes.IsCatchable(ErrorCodes.UnknownVariable));
        Assert.False(ErrorCodes.IsCatchable(ErrorCodes.ExpressionSyntax));
        Assert.False(ErrorCodes.IsCatchable(ErrorCodes.UserStop));
        Assert.False(ErrorCodes.IsCatchable(ErrorCodes.MotionFault));
    }

    // Shape ─────────────────────────────────────────────────────────────────
    [Fact]
    public void FieldsAreWellFormed()
    {
        var categories = StepCatalog.Categories.Select(c => c.Key).ToHashSet();
        string?[] bodies = [null, "steps", "branches", "cases", "tryCatch", "mapping"];
        var bad = new List<string>();
        foreach (var s in Steps)
        {
            if (!categories.Contains(s.Category)) bad.Add($"{s.Type}: unknown category {s.Category}");
            if (!bodies.Contains(s.Body)) bad.Add($"{s.Type}: bad body {s.Body}");
            if (string.IsNullOrWhiteSpace(s.Label) || string.IsNullOrWhiteSpace(s.Description) || string.IsNullOrWhiteSpace(s.Syntax))
                bad.Add($"{s.Type}: label/description/syntax missing");
            if (s.Fields.Any(f => f.Kind is FieldKinds.Steps or FieldKinds.Branches or FieldKinds.Cases) && s.Body == null)
                bad.Add($"{s.Type}: has a body field but body is null");

            var keys = s.Fields.Select(f => f.Key).ToList();
            if (keys.Distinct().Count() != keys.Count) bad.Add($"{s.Type}: duplicate field keys");

            var args = s.Fields.Where(f => f.Arg != "").Select(f => f.Arg.ToLowerInvariant()).ToList();
            if (args.Distinct().Count() != args.Count) bad.Add($"{s.Type}: duplicate args {string.Join(",", args.GroupBy(a => a).Where(g => g.Count() > 1).Select(g => g.Key))}");

            var positions = s.Fields.Where(f => f.Position != null).Select(f => f.Position!.Value).OrderBy(p => p).ToList();
            if (!positions.SequenceEqual(Enumerable.Range(0, positions.Count))) bad.Add($"{s.Type}: positions not 0..n-1 ({string.Join(",", positions)})");

            foreach (var f in s.Fields)
            {
                string at = $"{s.Type}.{f.Key}";
                if (!FieldKinds.All.Contains(f.Kind)) bad.Add($"{at}: unknown kind {f.Kind}");
                if (f.Kind == FieldKinds.Enum && (f.Options == null || f.Options.Count == 0)) bad.Add($"{at}: enum without options");
                if (f.Kind == FieldKinds.Enum && f.Default is string d && !f.Options!.Any(o => (string)o.Value == d))
                    bad.Add($"{at}: default '{d}' is not an option");
                if (f.ShowIf != null)
                {
                    var target = s.Fields.FirstOrDefault(x => x.Key == f.ShowIf.Field);
                    if (target == null) bad.Add($"{at}: showIf on unknown field {f.ShowIf.Field}");
                    else if (target.Kind == FieldKinds.Enum && f.ShowIf.In.Any(v => !target.Options!.Any(o => (string)o.Value == v)))
                        bad.Add($"{at}: showIf value not an option of {target.Key}");
                    if (f.ShowIf.In.Count == 0) bad.Add($"{at}: showIf with no values");
                }
                if (f.ExprKey != null && f.Kind is not (FieldKinds.Number or FieldKinds.Integer or FieldKinds.Bool))
                    bad.Add($"{at}: exprKey on a non-numeric field");
                if (f.VarKinds != null && f.Kind is not (FieldKinds.VarWrite or FieldKinds.VarRead or FieldKinds.ListVar))
                    bad.Add($"{at}: varKinds on a non-variable field");
            }
        }
        Assert.True(bad.Count == 0, string.Join("\n", bad));
    }

    [Fact]
    public void KeywordsAndAliasesAreUnique()
    {
        // "$" (assignment and list methods) and "#" (notes) are statement forms, not keywords.
        var words = Steps.SelectMany(s => s.Aliases.Prepend(s.Keyword))
            .Where(w => w is not ("$" or "#")).Select(w => w.ToLowerInvariant()).ToList();
        var dupes = words.GroupBy(w => w).Where(g => g.Count() > 1).Select(g => g.Key).ToList();
        Assert.True(dupes.Count == 0, "Duplicate keywords: " + string.Join(", ", dupes));
    }

    [Fact]
    public void BackgroundFlagMatchesTheExecutor()
    {
        Assert.False(StepCatalog.Get(StepType.MoveL)!.Background);
        Assert.False(StepCatalog.Get(StepType.MoveC)!.Background);
        Assert.False(StepCatalog.Get(StepType.SetSpeedFactor)!.Background);
        Assert.True(StepCatalog.Get(StepType.Prompt)!.Background);
        Assert.True(StepCatalog.Get(StepType.SetVariable)!.Background);
    }

    [Fact]
    public void TryCatchCodesOfferEveryCatchableCode()
    {
        var f = StepCatalog.Get(StepType.Try)!.Fields.Single(x => x.Key == "catchCodes");
        Assert.Equal(ErrorCodes.All.Where(e => e.Catchable).Select(e => e.Code), f.Options!.Select(o => (string)o.Value));
    }

    // Wire ──────────────────────────────────────────────────────────────────
    [Fact]
    public void CatalogSerializesWithTheFrozenNames()
    {
        // Serialized as the WebSocket server does (no naming policy — the names come from the attributes).
        using var doc = JsonDocument.Parse(JsonSerializer.Serialize(StepCatalog.Info()));
        var root = doc.RootElement;
        Assert.Equal(StepCatalog.CatalogVersion, root.GetProperty("catalogVersion").GetInt32());
        Assert.Equal(StepCatalog.LanguageVersion, root.GetProperty("languageVersion").GetInt32());
        Assert.True(root.GetProperty("categories").GetArrayLength() >= 8);
        Assert.Equal(ErrorCodes.All.Count, root.GetProperty("errorCodes").GetArrayLength());
        Assert.True(root.GetProperty("errorCodes")[0].GetProperty("catchable").GetBoolean());

        var moveL = root.GetProperty("steps").EnumerateArray().First(s => s.GetProperty("type").GetString() == "MoveL");
        Assert.Equal("motion", moveL.GetProperty("category").GetString());
        Assert.Equal(JsonValueKind.Null, moveL.GetProperty("body").ValueKind);
        var target = moveL.GetProperty("fields")[0];
        Assert.Equal("target", target.GetProperty("key").GetString());
        Assert.Equal("", target.GetProperty("arg").GetString());
        Assert.Equal(0, target.GetProperty("position").GetInt32());
        Assert.Equal("target", target.GetProperty("kind").GetString());
        Assert.Contains("pointId", target.GetProperty("props").EnumerateArray().Select(e => e.GetString()));
        Assert.False(target.TryGetProperty("unit", out _)); // nulls of optional metadata are omitted

        var loop = root.GetProperty("steps").EnumerateArray().First(s => s.GetProperty("type").GetString() == "Loop");
        var count = loop.GetProperty("fields").EnumerateArray().First(f => f.GetProperty("key").GetString() == "loopCount");
        Assert.Equal("loopCount", count.GetProperty("exprKey").GetString());
        Assert.Equal("loopMode", count.GetProperty("showIf").GetProperty("field").GetString());
        Assert.Equal("count", count.GetProperty("showIf").GetProperty("in")[0].GetString());
    }

    [Fact]
    public void CatalogAndStubCommandsAreRegistered()
    {
        var d = new CommandDispatcher();
        new StepCatalogCommands().Register(d);
        new LanguageCommands().Register(d);
        new DebugCommands(null).Register(d);
        Assert.True(d.TryGet("GetStepCatalog", out var get));
        Assert.IsType<StepCatalogInfo>(get(new CommandMessage()).Result);

        foreach (var name in new[] { "FormatProgramText", "ParseProgramText", "StepBuiltProgram", "SetBreakpoints" })
        {
            Assert.True(d.TryGet(name, out var h), name);
            var ex = Assert.Throws<CommandNotImplementedException>(() => h(new CommandMessage()).GetAwaiter().GetResult());
            Assert.StartsWith("notImplemented:", ex.Message);
        }
    }
}
