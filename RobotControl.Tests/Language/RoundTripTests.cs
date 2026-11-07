using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Language;
using Controller.RobotControl.Programs;
using static RobotControl.Tests.Language.SrsTestKit;

namespace RobotControl.Tests.Language;

/// <summary>
/// The text language's round-trip guarantee (docs/program-text.md): for every program P,
/// <c>Normalize(Parse(Format(P), base: P)) == Normalize(P)</c> and <c>Format</c> is idempotent.
/// Fixtures: the golden step (every field at once), every step type, the samples, and a
/// catalog-driven random program generator over 500 seeds.
/// </summary>
public class RoundTripTests
{
    private static string Golden(string name) => Path.Combine(AppContext.BaseDirectory, "Golden", name);

    [Fact]
    public void TheGoldenStepRoundTrips()
    {
        var step = JsonSerializer.Deserialize<ProgramStep>(File.ReadAllText(Golden("programstep.golden.json")), SrsTestKit.Json)!;
        AssertRoundTrip(Program(step));
    }

    [Fact]
    public void TheGoldenStepRoundTripsAsEveryType()
    {
        // The golden step carries every field; as each type, most of them ride in @with.
        var json = File.ReadAllText(Golden("programstep.golden.json"));
        foreach (StepType t in Enum.GetValues<StepType>())
        {
            var step = JsonSerializer.Deserialize<ProgramStep>(json, SrsTestKit.Json)!;
            step.Type = t;
            AssertRoundTrip(Program(step));
        }
    }

    [Fact]
    public void EveryStepTypeRoundTripsEmpty()
    {
        foreach (StepType t in Enum.GetValues<StepType>())
            AssertRoundTrip(Program(new ProgramStep { Id = "s1", Type = t }));
    }

    [Fact]
    public void EveryStepTypeRoundTripsWithCatalogDefaults()
    {
        foreach (var spec in StepCatalog.Steps)
        {
            var j = new System.Text.Json.Nodes.JsonObject { ["id"] = "s1", ["type"] = spec.Type };
            foreach (var f in spec.Fields)
                if (f.Default != null && f.Props == null && f.Kind is not FieldKinds.TextList)
                    j[f.Key] = System.Text.Json.Nodes.JsonValue.Create(f.Default is bool b ? b : f.Default is string s ? (object)s : Convert.ToDouble(f.Default));
            var step = j.Deserialize<ProgramStep>(SrsTestKit.Json)!;
            AssertRoundTrip(Program(step));
        }
    }

    [Fact]
    public void TheSamplesRoundTrip()
    {
        foreach (var (name, text) in Samples.All)
        {
            var names = Names();
            var parsed = SrsLanguage.Parse(text, null, names);
            Assert.True(parsed.Ok, name + ":\n" + string.Join("\n", parsed.Problems.Select(p => $"{p.Line}:{p.Col} {p.Code} {p.Message}")));
            AssertRoundTrip(parsed.Program!, names);
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void RandomProgramsRoundTrip(int seed)
    {
        var p = new Gen(seed).Program();
        AssertRoundTrip(p);
    }

    public static IEnumerable<object[]> Seeds() => Enumerable.Range(1, 500).Select(i => new object[] { i });

    [Theory]
    [InlineData(7)]
    [InlineData(42)]
    [InlineData(99)]
    public void RandomProgramsRoundTripWithIds(int seed)
    {
        var p = new Gen(seed).Program();
        var text = AssertRoundTrip(p, includeIds: true);
        // With ids written out, even a parse without a base keeps every step id.
        var parsed = SrsLanguage.Parse(text, null, Names());
        var want = StepTree.Walk(p.Steps).Select(s => s.Id).ToList();
        var got = StepTree.Walk(parsed.Program!.Steps).Select(s => s.Id).ToList();
        Assert.Equal(want, got);
    }

    [Fact]
    public void OnAndOffAssignmentsRoundTripAsTyped()
    {
        const string text = "#!srs 1\nvar ok: bool = off\n\n$ok = on\n$ok = off\n$ok = $ok and not off\n";
        var parsed = SrsLanguage.Parse(text, null, Names());
        Assert.True(parsed.Ok, string.Join("\n", parsed.Problems.Select(p => $"{p.Line}:{p.Col} {p.Message}")));
        Assert.Equal(["on", "off", "$ok and not off"], parsed.Program!.Steps.Select(s => s.VariableExpr));
        Assert.Equal(text, AssertRoundTrip(parsed.Program!));
        Assert.Equal(1, ExpressionEvaluator.Evaluate(parsed.Program!.Steps[0].VariableExpr!, new Dictionary<string, double>()));
    }

    [Fact]
    public void AnEmptyProgramRoundTrips()
    {
        var text = AssertRoundTrip(new BuiltProgram { Id = "p", Name = "Empty" });
        Assert.Equal("#!srs 1\n", text);
    }
}
