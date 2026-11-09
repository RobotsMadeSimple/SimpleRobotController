using Controller.RobotControl;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>
/// ProgramValidator.Steps.cs (package C): MoveC (via missing / unknown / same as the end,
/// unsupportedOnStep), SavePosition (destination, mode, pointWriteInBackground), SetSpeedFactor
/// (range), ListOp (what each action needs for the list's element type) and Prompt (1–4
/// buttons, input, timeout button) — plus the implicit variables Prompt and SavePosition create.
/// </summary>
public class StepsValidatorTests
{
    private static readonly HashSet<string> KnownPoints = new(StringComparer.OrdinalIgnoreCase) { "P1", "P2", "P3" };

    private static readonly ValidationContext Ctx = new()
    {
        PointExists = (id, name) => (id != null && KnownPoints.Contains(id)) || (name != null && KnownPoints.Contains(name)),
    };

    private static List<ValidationProblem> Validate(IEnumerable<ProgramVariable> vars, ProgramStep step, bool background = false,
                                                    params ProgramStep[] more) =>
        ProgramValidator.Validate(new BuiltProgram
        {
            Id = "p", Name = "P", IsBackground = background,
            Steps = new[] { step }.Concat(more).ToList(), Variables = vars.ToList(),
        }, Ctx);

    private static List<ValidationProblem> Validate(ProgramStep step, bool background = false) => Validate([], step, background);

    private static ProgramStep S(StepType t, Action<ProgramStep> init) => StepsRig.Step(t, init);

    private static void Has(List<ValidationProblem> problems, string code, string field, string severity = ValidationSeverity.Error) =>
        Assert.Contains(problems, p => p.Code == code && p.Field == field && p.Severity == severity);

    private static void Clean(List<ValidationProblem> problems) =>
        Assert.DoesNotContain(problems, p => p.Severity == ValidationSeverity.Error);

    // ── MoveC ─────────────────────────────────────────────────────────────────

    private static ProgramStep Arc(Action<ProgramStep>? init = null) => S(StepType.MoveC, s =>
    {
        s.ViaPointName = "P1"; s.PointName = "P2";
        init?.Invoke(s);
    });

    [Fact]
    public void AGoodArcIsClean() => Assert.Empty(Validate(Arc()));

    [Fact]
    public void ArcWithoutVia() => Has(Validate(Arc(s => s.ViaPointName = null)), ValidationCodes.MissingField, "via");

    [Fact]
    public void ArcWithAnUnknownVia() => Has(Validate(Arc(s => s.ViaPointName = "Nowhere")), ValidationCodes.UnknownPoint, "via");

    [Fact]
    public void ArcWithAnUnknownEnd() => Has(Validate(Arc(s => s.PointName = "Nowhere")), ValidationCodes.UnknownPoint, "pointName");

    [Fact]
    public void ArcThroughItsOwnEnd()
    {
        Has(Validate(Arc(s => s.ViaPointName = "P2")), ValidationCodes.MissingField, "via");
        Has(Validate(Arc(s => { s.ViaPointId = "P3"; s.PointId = "P3"; })), ValidationCodes.MissingField, "via");
    }

    [Fact]
    public void ArcViaTemplateAndPointsList()
    {
        Clean(Validate([StepsRig.Num("i")], Arc(s => { s.ViaPointName = null; s.ViaPointNameExpr = "P{$i}"; })));
        Has(Validate(Arc(s => { s.ViaPointName = null; s.ViaPointNameExpr = "P{$nope}"; })), ValidationCodes.UnknownVariable, "via");
        Clean(Validate([StepsRig.PointList("pts")], Arc(s => { s.ViaPointName = null; s.ViaPointNameExpr = "$pts[0]"; })));
        Has(Validate(Arc(s => { s.ViaPointName = null; s.ViaPointNameExpr = "Nowhere"; })), ValidationCodes.UnknownPoint, "via");
    }

    [Fact]
    public void ArcUnsupportedOptions()
    {
        var problems = Validate(Arc(s =>
        {
            s.MoveStopCondition = new ConditionGroup { Items = [new ConditionItem { Id = "c", Expr = "1" }] };
            s.ToolOffsetZ = 5;
            s.Blend = true;
        }));
        Has(problems, ValidationCodes.UnsupportedOnStep, "moveStopCondition");
        Has(problems, ValidationCodes.UnsupportedOnStep, "toolOffset", ValidationSeverity.Warning);
        Has(problems, ValidationCodes.UnsupportedOnStep, "blend", ValidationSeverity.Warning);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    [InlineData(0.0001)]
    public void ArcToleranceWarnings(double tol) =>
        Has(Validate(Arc(s => s.ArcTolerance = tol)), ValidationCodes.MissingField, "arcTolerance", ValidationSeverity.Warning);

    [Fact]
    public void ArcUnknownLocal()
    {
        var ctx = new ValidationContext { LocalExists = (_, _) => false };
        var problems = ProgramValidator.Validate(new BuiltProgram { Id = "p", Name = "P", Steps = [Arc(s => s.LocalName = "Fx")] }, ctx);
        Has(problems, ValidationCodes.UnknownLocal, "localId");
    }

    // ── SavePosition ──────────────────────────────────────────────────────────

    private static ProgramStep Save(Action<ProgramStep>? init = null) => S(StepType.SavePosition, s =>
    {
        s.VariableName = "pts";
        init?.Invoke(s);
    });

    [Fact]
    public void SaveToAPointsList()
    {
        Assert.Empty(Validate([StepsRig.PointList("pts")], Save()));
        // Undeclared is fine: the step creates the Points list (and later reads of it resolve).
        Clean(Validate([StepsRig.Num("x")], Save(), false, StepsRig.Set("x", "$pts[0].x")));
    }

    [Fact]
    public void SaveToTheWrongKindOfVariable()
    {
        Has(Validate([StepsRig.Numbers("pts")], Save()), ValidationCodes.UnknownVariable, "variableName");
        Has(Validate([StepsRig.Num("pts")], Save()), ValidationCodes.UnknownVariable, "variableName");
        Has(Validate(Save(s => s.VariableName = "")), ValidationCodes.MissingField, "variableName");
        Has(Validate(Save(s => s.VariableName = "robot.x")), ValidationCodes.ReadOnlyProperty, "variableName");
    }

    [Fact]
    public void SaveModesAndIndex()
    {
        Has(Validate([StepsRig.PointList("pts")], Save(s => s.SavePositionMode = "prepend")), ValidationCodes.MissingField, "savePositionMode");
        Has(Validate([StepsRig.PointList("pts")], Save(s => { s.SavePositionMode = "set"; s.SavePositionIndex = -1; })),
            ValidationCodes.MissingField, "savePositionIndex");
        Has(Validate([StepsRig.PointList("pts")], Save(s => s.SavePositionSource = "joints")), ValidationCodes.MissingField, "savePositionSource");
        Has(Validate([StepsRig.PointList("pts")], Save(s => s.SavePositionTo = "file")), ValidationCodes.MissingField, "savePositionTo");
        Clean(Validate([StepsRig.PointList("pts"), StepsRig.Num("i")],
            Save(s => { s.SavePositionMode = "insert"; s.Expressions = new() { ["savePositionIndex"] = "$i" }; })));
        Has(Validate([StepsRig.PointList("pts")],
            Save(s => { s.SavePositionMode = "insert"; s.Expressions = new() { ["savePositionIndex"] = "$nope" }; })),
            ValidationCodes.UnknownVariable, "expressions.savePositionIndex");
    }

    [Fact]
    public void SaveToAPoint()
    {
        Assert.Empty(Validate(Save(s => { s.SavePositionTo = "point"; s.PointName = "New point"; })));   // created
        Has(Validate(Save(s => { s.SavePositionTo = "point"; s.VariableName = null; })), ValidationCodes.MissingField, "pointId");
        Has(Validate(Save(s => { s.SavePositionTo = "point"; s.PointId = "gone"; })), ValidationCodes.UnknownPoint, "pointId");
    }

    [Fact]
    public void SaveToAPointInABackgroundProgramWarns()
    {
        var problems = Validate(Save(s => { s.SavePositionTo = "point"; s.PointName = "P1"; }), background: true);
        Has(problems, ValidationCodes.PointWriteInBackground, "pointId", ValidationSeverity.Warning);
        Assert.DoesNotContain(Validate([StepsRig.PointList("pts")], Save(), background: true),
            p => p.Code == ValidationCodes.PointWriteInBackground);
    }

    // ── SetSpeedFactor ────────────────────────────────────────────────────────

    [Theory]
    [InlineData(0.5)]
    [InlineData(150.0)]
    [InlineData(null)]
    public void SpeedFactorOutOfRangeOrMissingWarns(double? f) =>
        Has(Validate(S(StepType.SetSpeedFactor, s => s.SpeedFactor = f)), ValidationCodes.MissingField, "speedFactor", ValidationSeverity.Warning);

    [Fact]
    public void SpeedFactorInRangeOrAnExpressionIsClean()
    {
        Assert.Empty(Validate(S(StepType.SetSpeedFactor, s => s.SpeedFactor = 50)));
        Assert.Empty(Validate([StepsRig.Num("f")], S(StepType.SetSpeedFactor, s => s.Expressions = new() { ["speedFactor"] = "$f" })));
    }

    // ── ListOp ────────────────────────────────────────────────────────────────

    private static ProgramStep Op(string action, Action<ProgramStep>? init = null) =>
        S(StepType.ListOp, s => { s.VariableName = "v"; s.ListAction = action; init?.Invoke(s); });

    [Fact]
    public void GoodListOpsAreClean()
    {
        ProgramVariable[] vars = [StepsRig.Numbers("v"), StepsRig.PointList("p"), StepsRig.Records("r"), StepsRig.Num("i")];
        foreach (var step in new[]
        {
            Op("append", s => s.ListValueExpr = "$i"),
            Op("insert", s => { s.ListIndex = 0; s.ListValueExpr = "1"; }),
            Op("removeAt", s => s.Expressions = new() { ["listIndex"] = "$i" }),
            Op("clear"), Op("reverse"), Op("sort"),
            Op("set", s => { s.ListIndex = 0; s.ListValueExpr = "2"; }),
            Op("copyFrom", s => s.ListSourceVariable = "$v"),
            Op("append", s => s.VariableName = "p"),                                         // append Here
            Op("setField", s => { s.VariableName = "p"; s.ListIndex = 0; s.ListFields = new() { ["z"] = "$i" }; }),
            Op("sort", s => { s.VariableName = "p"; s.ListSortField = "x"; }),
            Op("append", s => { s.VariableName = "r"; s.ListFields = new() { ["score"] = "1" }; }),
            Op("copyFrom", s => { s.VariableName = "r"; s.ListSourceVariable = "p"; }),
        })
            Assert.DoesNotContain(Validate(vars, step), p => p.Code != ValidationCodes.UnusedVariable);
    }

    [Fact]
    public void ListOpOnSomethingThatIsNotAList()
    {
        Has(Validate(Op("clear")), ValidationCodes.UnknownVariable, "variableName");
        Has(Validate([StepsRig.Num("v")], Op("clear")), ValidationCodes.UnknownVariable, "variableName");
        Has(Validate(Op("clear", s => s.VariableName = null)), ValidationCodes.MissingField, "variableName");
        Has(Validate([StepsRig.Numbers("v")], Op("shuffle")), ValidationCodes.MissingField, "listAction");
    }

    [Fact]
    public void ListOpNumberListNeeds()
    {
        ProgramVariable[] v = [StepsRig.Numbers("v")];
        Has(Validate(v, Op("append")), ValidationCodes.MissingField, "listValueExpr");
        Has(Validate(v, Op("append", s => { s.ListValueExpr = "1"; s.ListFields = new() { ["x"] = "1" }; })), ValidationCodes.UnsupportedOnStep, "listFields");
        Has(Validate(v, Op("setField", s => { s.ListIndex = 0; s.ListFields = new() { ["x"] = "1" }; })), ValidationCodes.UnsupportedOnStep, "listAction");
        Has(Validate(v, Op("sort", s => s.ListSortField = "x")), ValidationCodes.UnsupportedOnStep, "listSortField");
        Has(Validate(v, Op("append", s => s.ListValueExpr = "$typo")), ValidationCodes.UnknownVariable, "listValueExpr");
        Clean(Validate(v, Op("append", s => s.ListFields = new() { ["value"] = "3" })));
    }

    [Fact]
    public void ListOpIndexNeeds()
    {
        ProgramVariable[] v = [StepsRig.Numbers("v")];
        Has(Validate(v, Op("removeAt")), ValidationCodes.MissingField, "listIndex", ValidationSeverity.Warning);
        Has(Validate(v, Op("removeAt", s => s.ListIndex = -1)), ValidationCodes.MissingField, "listIndex");
    }

    [Fact]
    public void ListOpPointAndRecordNeeds()
    {
        ProgramVariable[] v = [StepsRig.PointList("p"), StepsRig.Records("r")];
        Has(Validate(v, Op("insert", s => { s.VariableName = "p"; s.ListIndex = 0; })), ValidationCodes.MissingField, "listFields");
        Has(Validate(v, Op("append", s => { s.VariableName = "p"; s.ListValueExpr = "1"; })), ValidationCodes.UnsupportedOnStep, "listValueExpr");
        Has(Validate(v, Op("append", s => { s.VariableName = "p"; s.ListFields = new() { ["w"] = "1" }; })), ValidationCodes.MissingField, "listFields.w");
        Has(Validate(v, Op("append", s => { s.VariableName = "r"; s.ListFields = new() { ["w"] = "" }; })),
            ValidationCodes.MissingField, "listFields.w", ValidationSeverity.Warning);
        Has(Validate(v, Op("append", s => s.VariableName = "r")), ValidationCodes.MissingField, "listFields");
        Has(Validate(v, Op("setField", s => { s.VariableName = "r"; s.ListIndex = 0; })), ValidationCodes.MissingField, "listFields");
        Has(Validate(v, Op("sort", s => s.VariableName = "r")), ValidationCodes.MissingField, "listSortField");
        Has(Validate(v, Op("sort", s => { s.VariableName = "p"; s.ListSortField = "w"; })), ValidationCodes.MissingField, "listSortField");
    }

    [Fact]
    public void ListOpCopyNeeds()
    {
        ProgramVariable[] v = [StepsRig.PointList("p"), StepsRig.Records("r"), StepsRig.Num("n")];
        Has(Validate(v, Op("copyFrom", s => s.VariableName = "p")), ValidationCodes.MissingField, "listSourceVariable");
        Has(Validate(v, Op("copyFrom", s => { s.VariableName = "p"; s.ListSourceVariable = "r"; })), ValidationCodes.UnsupportedOnStep, "listSourceVariable");
        Has(Validate(v, Op("copyFrom", s => { s.VariableName = "p"; s.ListSourceVariable = "n"; })), ValidationCodes.UnknownVariable, "listSourceVariable");
        Has(Validate(v, Op("copyFrom", s => { s.VariableName = "p"; s.ListSourceVariable = "nope"; })), ValidationCodes.UnknownVariable, "listSourceVariable");
    }

    [Fact]
    public void ListOpMarksItsListsUsed()
    {
        var problems = Validate([StepsRig.Numbers("v"), StepsRig.Numbers("src")], Op("copyFrom", s => s.ListSourceVariable = "src"));
        Assert.DoesNotContain(problems, p => p.Code == ValidationCodes.UnusedVariable);
    }

    // ── Prompt ────────────────────────────────────────────────────────────────

    private static ProgramStep Ask(Action<ProgramStep>? init = null) => S(StepType.Prompt, s =>
    {
        s.PromptMessage = "Go?";
        init?.Invoke(s);
    });

    [Fact]
    public void AGoodPromptIsClean()
    {
        Assert.Empty(Validate([StepsRig.Num("c"), StepsRig.Num("q"), StepsRig.Num("n")], Ask(s =>
        {
            s.PromptButtons = ["Go", "Skip"]; s.PromptResultVariableName = "c";
            s.PromptInput = "number"; s.PromptDefault = "$n"; s.PromptMin = 0; s.PromptMax = 10; s.PromptInputVariableName = "q";
            s.PromptTimeoutMs = 1000; s.PromptTimeoutButton = 1;
        })));
    }

    [Fact]
    public void PromptMessageAndButtons()
    {
        Has(Validate(Ask(s => s.PromptMessage = " ")), ValidationCodes.MissingField, "promptMessage");
        Has(Validate(Ask(s => s.PromptButtons = ["a", "b", "c", "d", "e"])), ValidationCodes.MissingField, "promptButtons");
        Has(Validate(Ask(s => s.PromptButtons = ["a", ""])), ValidationCodes.MissingField, "promptButtons", ValidationSeverity.Warning);
        Has(Validate(Ask(s => s.PromptMessage = "{$nope}")), ValidationCodes.UnknownVariable, "promptMessage", ValidationSeverity.Warning);
    }

    [Fact]
    public void PromptInputs()
    {
        Has(Validate(Ask(s => s.PromptInput = "slider")), ValidationCodes.MissingField, "promptInput");
        Has(Validate(Ask(s => { s.PromptInput = "number"; s.PromptMin = 5; s.PromptMax = 1; })), ValidationCodes.MissingField, "promptMin");
        Has(Validate(Ask(s => { s.PromptInput = "number"; s.PromptDefault = "$nope"; })), ValidationCodes.UnknownVariable, "promptDefault");
        Has(Validate([StepsRig.Num("q")], Ask(s => s.PromptInputVariableName = "q")),
            ValidationCodes.UnsupportedOnStep, "promptInputVariableName", ValidationSeverity.Warning);
    }

    [Fact]
    public void PromptVariables()
    {
        Has(Validate([StepsRig.Numbers("c")], Ask(s => s.PromptResultVariableName = "c")), ValidationCodes.UnknownVariable, "promptResultVariableName");
        Has(Validate(Ask(s => s.PromptResultVariableName = "robot.x")), ValidationCodes.ReadOnlyProperty, "promptResultVariableName");
        // An undeclared result variable is created by the step, so reading it later is fine.
        Clean(Validate([StepsRig.Num("x")], Ask(s => s.PromptResultVariableName = "choice"), false, StepsRig.Set("x", "$choice + 1")));
        Clean(Validate([StepsRig.Num("x")], Ask(s => { s.PromptInput = "number"; s.PromptInputVariableName = "qty"; }), false, StepsRig.Set("x", "$qty")));
    }

    [Fact]
    public void PromptTimeoutButton()
    {
        Has(Validate(Ask(s => { s.PromptButtons = ["a", "b"]; s.PromptTimeoutMs = 10; s.PromptTimeoutButton = 2; })),
            ValidationCodes.MissingField, "promptTimeoutButton");
        Has(Validate(Ask(s => s.PromptTimeoutButton = 0)), ValidationCodes.UnsupportedOnStep, "promptTimeoutButton", ValidationSeverity.Warning);
        Has(Validate(Ask(s => s.PromptTimeoutMs = -5)), ValidationCodes.MissingField, "promptTimeoutMs");
    }
}
