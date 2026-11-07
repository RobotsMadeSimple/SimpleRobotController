using Controller.RobotControl;
using System.Reflection;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace RobotControl.Tests;

/// <summary>
/// Cross-repo drift guard for the ProgramStep wire model.
///
/// Golden/programstep.golden.json contains EVERY field of the app-side ProgramStep
/// type (SimpleRobotApp/src/models/robotModels.tsx). The app repo carries a mirror
/// (src/models/programStepGolden.check.ts) type-checked by tsc. Update BOTH files
/// together whenever the step model changes.
///
/// If this test fails with an unmapped-member error, the app has a ProgramStep field
/// the controller model doesn't know about (it would be silently dropped on the wire).
/// </summary>
public class GoldenProgramStepTests
{
    private static string GoldenPath =>
        Path.Combine(AppContext.BaseDirectory, "Golden", "programstep.golden.json");

    private static string GoldenFile(string name) => Path.Combine(AppContext.BaseDirectory, "Golden", name);

    /// <summary>The JSON names of a model type's serialized properties.</summary>
    private static List<string> JsonNames(Type t) =>
        t.GetProperties(BindingFlags.Public | BindingFlags.Instance)
         .Where(p => p.GetCustomAttribute<JsonIgnoreAttribute>() == null)
         .Select(p => p.GetCustomAttribute<JsonPropertyNameAttribute>()?.Name ?? p.Name)
         .ToList();

    private static HashSet<string> Keys(JsonElement obj) =>
        obj.EnumerateObject().Select(p => p.Name).ToHashSet(StringComparer.Ordinal);

    private static readonly JsonSerializerOptions Options = new()
    {
        PropertyNameCaseInsensitive = true,
        // The controller normally IGNORES unknown members (that's the silent-drift
        // failure mode) — the whole point of this test is to disallow them here.
        UnmappedMemberHandling = System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow,
    };

    [Fact]
    public void EveryAppFieldDeserializesIntoTheControllerModel()
    {
        var json = File.ReadAllText(GoldenPath);
        var step = JsonSerializer.Deserialize<ProgramStep>(json, Options);
        Assert.NotNull(step);
        Assert.Equal("step-golden-1", step!.Id);
        Assert.Equal(StepType.MoveL, step.Type);
    }

    [Fact]
    public void PluginStepFieldsDeserialize()
    {
        var step = JsonSerializer.Deserialize<ProgramStep>(File.ReadAllText(GoldenPath), Options)!;
        Assert.Equal("scale", step.PluginId);
        Assert.Equal("weigh", step.PluginStepId);
        Assert.Equal(6, step.PluginParams!.Count);
        Assert.Equal("$n * 2", step.PluginParams["samples"]);
        Assert.Equal("Bin {$i}", step.PluginParams["label"]);
        Assert.Equal(2, step.PluginOutputs!.Count);
        Assert.Equal("where", step.PluginOutputs[1].Key);
        Assert.Equal("pickPts", step.PluginOutputs[1].VariableName);
        Assert.Equal(5000, step.PluginTimeoutMs);
        Assert.Equal(StepType.Plugin, JsonSerializer.Deserialize<ProgramStep>("""{"id":"p","type":"Plugin"}""", Options)!.Type);
    }

    [Fact]
    public void RunVisionFieldsDeserialize()
    {
        var step = JsonSerializer.Deserialize<ProgramStep>(File.ReadAllText(GoldenPath), Options)!;
        Assert.Equal("vp-1", step.VisionProgramId);
        Assert.Equal("visionOk", step.PassVariable);
        Assert.Equal(2, step.ToolOutputs!.Count);
        Assert.Equal("t5", step.ToolOutputs[1].ToolId);
        Assert.Equal("cells", step.ToolOutputs[1].Value);
        Assert.Equal("whiteCells", step.ToolOutputs[1].Variable);
        Assert.Equal("robot", step.OutputFrame);
    }

    [Fact]
    public void RoundTripPreservesEveryGoldenKey()
    {
        var json = File.ReadAllText(GoldenPath);
        var step = JsonSerializer.Deserialize<ProgramStep>(json, Options)!;

        var reserialized = JsonSerializer.Serialize(step, new JsonSerializerOptions());
        using var output = JsonDocument.Parse(reserialized);
        var outputKeys = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        foreach (var prop in output.RootElement.EnumerateObject())
            outputKeys.Add(prop.Name);

        using var golden = JsonDocument.Parse(json);
        var missing = new List<string>();
        foreach (var prop in golden.RootElement.EnumerateObject())
            if (!outputKeys.Contains(prop.Name))
                missing.Add(prop.Name);

        Assert.True(missing.Count == 0,
            $"Fields lost on round-trip (check [JsonPropertyName] casing): {string.Join(", ", missing)}");
    }

    /// <summary>
    /// The other direction: every ProgramStep property the controller has is in the golden, so a
    /// field added here without the app mirror (programStepGolden.check.ts) fails a test too.
    /// </summary>
    [Fact]
    public void EveryControllerFieldIsInTheGolden()
    {
        using var golden = JsonDocument.Parse(File.ReadAllText(GoldenPath));
        var keys = Keys(golden.RootElement);
        var missing = JsonNames(typeof(ProgramStep)).Where(n => !keys.Contains(n)).ToList();
        Assert.True(missing.Count == 0,
            $"ProgramStep fields missing from Golden/programstep.golden.json (add them there and to the app's programStepGolden.check.ts): {string.Join(", ", missing)}");
    }

    [Fact]
    public void NewBlockFieldsDeserialize()
    {
        var step = JsonSerializer.Deserialize<ProgramStep>(File.ReadAllText(GoldenPath), Options)!;
        Assert.True(step.Enabled);
        Assert.Equal("1", step.SwitchCases![0].Values[0]);
        Assert.Equal("nested-case-1", step.SwitchCases[0].Steps[0].Id);
        Assert.Equal(StepType.Note, step.CatchSteps![0].Type);
        Assert.Equal(["waitTimeout", "httpFailed"], step.CatchCodes!);
        Assert.Equal("$n", step.RoutineArgs!["count"]);
        Assert.Equal("ok", step.RoutineOutputs![0].VariableName);
        Assert.Equal("$robot.x", step.ListFields!["x"]);
        Assert.Equal(["Continue", "Skip"], step.PromptButtons!);
        Assert.Equal(0, step.PromptTimeoutButton);
        Assert.Equal("photo", step.JsonOutbound![1].ImageVar);
        Assert.Equal("x", step.HttpReceiveInbound![0].VariableName);
    }

    /// <summary>Golden/steptypes.golden.json is the full StepType list, in enum order; the app's
    /// STEP_TYPES_ALL mirrors it (type-checked against the StepType union).</summary>
    [Fact]
    public void StepTypeEnumMatchesTheGolden()
    {
        var golden = JsonSerializer.Deserialize<List<string>>(File.ReadAllText(GoldenFile("steptypes.golden.json")))!;
        Assert.Equal(golden, Enum.GetNames<StepType>().ToList());
    }

    /// <summary>The program-level fields (editorMode, variables[].paramMode, …) round-trip too.</summary>
    [Fact]
    public void BuiltProgramGoldenDeserializesAndCoversTheModel()
    {
        var json = File.ReadAllText(GoldenFile("builtprogram.golden.json"));
        var options = new JsonSerializerOptions(Options) { Converters = { new JsonStringEnumConverter() } };
        var program = JsonSerializer.Deserialize<BuiltProgram>(json, options)!;
        Assert.Equal("text", program.EditorMode);
        Assert.Equal(ParamModes.InOut, program.Variables![0].ParamMode);

        using var doc = JsonDocument.Parse(json);
        var programKeys = Keys(doc.RootElement);
        var variableKeys = Keys(doc.RootElement.GetProperty("variables")[0]);
        // Legacy list fields (values / points / objects) are read-only and stay out of the golden.
        string[] legacyVariableFields = ["values", "points", "objects"];
        var missing = JsonNames(typeof(BuiltProgram)).Where(n => !programKeys.Contains(n) && n != "displayPageLayout")
            .Concat(JsonNames(typeof(ProgramVariable)).Where(n => !variableKeys.Contains(n) && !legacyVariableFields.Contains(n))
                .Select(n => "variables[]." + n))
            .ToList();
        Assert.True(missing.Count == 0, $"Missing from Golden/builtprogram.golden.json: {string.Join(", ", missing)}");
    }
}
