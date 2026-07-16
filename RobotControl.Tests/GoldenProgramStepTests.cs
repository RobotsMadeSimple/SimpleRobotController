using System.Text.Json;

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
}
