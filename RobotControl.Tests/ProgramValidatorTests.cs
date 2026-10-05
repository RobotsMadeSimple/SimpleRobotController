using Controller.RobotControl;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>ProgramValidator — the rules of docs/expressions-and-variables.md §5.</summary>
public class ProgramValidatorTests
{
    // ── Builders ─────────────────────────────────────────────────────────────

    private static int _ids;
    private static ProgramStep Step(StepType type, Action<ProgramStep>? init = null)
    {
        var s = new ProgramStep { Id = $"s{++_ids}", Type = type };
        init?.Invoke(s);
        return s;
    }

    private static ProgramVariable Num(string name, double value = 0, string? expr = null) =>
        new() { Id = name, Name = name, Value = value, ValueExpression = expr };

    private static ProgramVariable List(string name, ListElementType type) =>
        new() { Id = name, Name = name, Items = [], ElementType = type };

    private static BuiltProgram Prog(IEnumerable<ProgramStep> steps, params ProgramVariable[] vars) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars.ToList() };

    private static ConditionGroup Cond(string left, string op, string right) =>
        new() { Items = [new ConditionItem { Id = "c", Left = left, Operator = op, Right = right }] };

    private static readonly HashSet<string> Points = new(StringComparer.OrdinalIgnoreCase) { "Home", "Pick" };

    private static ValidationContext Ctx(params BuiltProgram[] others) => new()
    {
        PointExists         = Points.Contains,
        ToolExists          = n => n == "Gripper",
        LocalExists         = n => n == "Fixture",
        GridExists          = id => id == "grid1",
        StackExists         = id => id == "stack1",
        VisionProgramExists = id => id == "vp1",
        FindProgram         = (id, name) => others.FirstOrDefault(p =>
            (id != null && p.Id == id) || (id == null && string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase))),
    };

    private static List<ValidationProblem> Validate(BuiltProgram p, ValidationContext? ctx = null) =>
        ProgramValidator.Validate(p, ctx ?? Ctx());

    private static ValidationProblem Single(List<ValidationProblem> problems, string code)
    {
        var hits = problems.Where(p => p.Code == code).ToList();
        Assert.True(hits.Count == 1, $"expected one {code}, got: {string.Join(" | ", problems)}");
        return hits[0];
    }

    private static void None(List<ValidationProblem> problems, string code) =>
        Assert.True(problems.All(p => p.Code != code), $"unexpected {code}: {string.Join(" | ", problems)}");

    private static void Clean(List<ValidationProblem> problems) =>
        Assert.True(problems.Count == 0, "expected no problems, got: " + string.Join(" | ", problems));

    // ── A clean program ─────────────────────────────────────────────────────

    [Fact]
    public void AWellFormedProgramHasNoProblems()
    {
        var p = Prog(
        [
            Step(StepType.MoveL, s => { s.PointName = "Home"; s.Expressions = new() { ["offsetZ"] = "$clearance + max($robot.z, 0)" }; }),
            Step(StepType.Loop, s =>
            {
                s.LoopCount = 3;
                s.ForEachIndexVariableName = "i";
                s.LoopSteps = [Step(StepType.SetVariable, v => { v.VariableName = "count"; v.VariableExpr = "$count + $i"; })];
            }),
            Step(StepType.IfCondition, s =>
            {
                s.Condition = Cond("$count", ">", "2");
                s.IfSteps = [Step(StepType.StatusUpdate, v => v.StatusMessage = "count={$count} at {$robot.x}")];
            }),
        ], Num("clearance", 10), Num("count"));
        Clean(Validate(p));
    }

    // ── Expressions ─────────────────────────────────────────────────────────

    [Fact]
    public void SyntaxErrorsAreReportedWithFieldAndPath()
    {
        var p = Prog([Step(StepType.Loop, s =>
        {
            s.LoopCount = 1;
            s.LoopSteps = [Step(StepType.MoveL, m => { m.PointName = "Home"; m.Expressions = new() { ["offsetX"] = "(1 + 2" }; })];
        })]);
        var prob = Single(Validate(p), ValidationCodes.ExpressionSyntax);
        Assert.Equal("steps[0].loopSteps[0]", prob.StepPath);
        Assert.Equal("expressions.offsetX", prob.Field);
        Assert.Equal(ValidationSeverity.Error, prob.Severity);
        Assert.Equal(p.Steps[0].LoopSteps![0].Id, prob.StepId);
    }

    [Fact]
    public void UnknownFunctionAndBadArityUseTheirOwnCodes()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "a"; s.VariableExpr = "nope(1)"; }),
                      Step(StepType.SetVariable, s => { s.VariableName = "a"; s.VariableExpr = "clamp(1, 2)"; })], Num("a"));
        var probs = Validate(p);
        Assert.Equal("variableExpr", Single(probs, ValidationCodes.UnknownFunction).Field);
        Assert.Equal("steps[1]", Single(probs, ValidationCodes.BadArity).StepPath);
    }

    [Fact]
    public void UnknownVariablesPropertiesAndIo()
    {
        var p = Prog([Step(StepType.MoveL, s =>
        {
            s.PointName = "Home";
            s.Expressions = new()
            {
                ["offsetX"] = "$typo",
                ["offsetY"] = "$robot.nope",
                ["offsetZ"] = "$stb.in1 + $relay.2 + $nano.Board.pin3 + $time_ms + $program.runCount + $time.hour",
            };
        })]);
        var probs = Validate(p);
        Assert.Equal("expressions.offsetX", Single(probs, ValidationCodes.UnknownVariable).Field);
        Assert.Equal("expressions.offsetY", Single(probs, ValidationCodes.UnknownProperty).Field);
    }

    [Fact]
    public void ListReferencesMustBeLists()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "a"; s.VariableExpr = "$a[0] + len($pts) + $pts.length + $pts[0].x + $pts.x"; })],
                     Num("a"), List("pts", ListElementType.Point));
        var probs = Validate(p).Where(x => x.Code == ValidationCodes.UnknownVariable).ToList();
        Assert.Equal(2, probs.Count);                               // $a[0] (not a list) and $pts.x
        Assert.Contains(probs, x => x.Message.Contains("'$a' is not a list"));
        Assert.Contains(probs, x => x.Message.Contains("$pts.x"));
    }

    [Fact]
    public void StringVariablesHaveNoNumericValue()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "a"; s.VariableExpr = "$label + 1"; })],
                     Num("a"), new ProgramVariable { Name = "label", IsString = true });
        Assert.Contains("text", Single(Validate(p), ValidationCodes.UnknownVariable).Message);
    }

    [Fact]
    public void BareWordsAreAWarning()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "a"; s.VariableExpr = "a + 1"; })], Num("a"));
        var prob = Single(Validate(p), ValidationCodes.ExpressionSyntax);
        Assert.Equal(ValidationSeverity.Warning, prob.Severity);
        Assert.Contains("did you mean $a", prob.Message);
    }

    [Fact]
    public void ConditionGroupsAreChecked()
    {
        var p = Prog([Step(StepType.IfCondition, s =>
        {
            s.Condition = Cond("$x", ">", "");
            s.IfSteps = [Step(StepType.PauseProgram)];
            s.ElseIfBranches = [new ElseIfBranch { Id = "e", Condition = Cond("$missing", "==", "1"), Steps = [Step(StepType.PauseProgram)] }];
        })], Num("x"));
        var probs = Validate(p);
        Assert.Equal("condition.items[0].right", Single(probs, ValidationCodes.MissingField).Field);
        Assert.Equal("elseIfBranches[0].condition.items[0].left", Single(probs, ValidationCodes.UnknownVariable).Field);
    }

    [Fact]
    public void StringConditionsTreatSidesAsText()
    {
        var p = Prog([Step(StepType.IfCondition, s =>
        {
            s.Condition = Cond("$name", "==", "hello world");
            s.IfSteps = [Step(StepType.PauseProgram)];
        }),
        Step(StepType.IfCondition, s =>
        {
            s.Condition = Cond("{$name}", "contains", "ell");
            s.IfSteps = [Step(StepType.PauseProgram)];
        })], new ProgramVariable { Name = "name", IsString = true });
        Clean(Validate(p));
    }

    [Fact]
    public void WhileAndWaitConditionsAreRequiredAndChecked()
    {
        var p = Prog(
        [
            Step(StepType.Loop, s => { s.LoopMode = "while"; s.LoopSteps = [Step(StepType.PauseProgram)]; }),
            Step(StepType.Loop, s => { s.LoopMode = "while"; s.LoopWhileCondition = Cond("$nope", "<", "3"); s.LoopSteps = [Step(StepType.PauseProgram)]; }),
            Step(StepType.Wait, s => { s.WaitMode = "condition"; s.WaitCondition = Cond("$stb.in1", "==", "1"); }),
        ]);
        var probs = Validate(p);
        Assert.Equal("loopWhileCondition", Single(probs, ValidationCodes.MissingField).Field);
        Assert.Equal("loopWhileCondition.items[0].left", Single(probs, ValidationCodes.UnknownVariable).Field);
    }

    [Fact]
    public void TemplatesWarnAboutUnknownNames()
    {
        var p = Prog([Step(StepType.StatusUpdate, s => s.StatusMessage = "at $robot.x — {$nope + 1} {$ok}")], Num("ok"));
        var probs = Validate(p).Where(x => x.Code == ValidationCodes.UnknownVariable).ToList();
        Assert.Equal(2, probs.Count);
        Assert.All(probs, x => Assert.Equal(ValidationSeverity.Warning, x.Severity));
        Assert.Contains(probs, x => x.Message.Contains("properties need braces"));
    }

    [Fact]
    public void ForgottenSigilInBracesIsAWarningOnlyWhenItLooksLikeAnExpression()
    {
        var p = Prog([Step(StepType.StatusUpdate, s => s.StatusMessage = "{i + $j} {\"json\": 1} {plain words}")], Num("j"));
        var prob = Single(Validate(p), ValidationCodes.ExpressionSyntax);
        Assert.Contains("'i'", prob.Message);
    }

    [Fact]
    public void VariableInitialValuesSeeOnlyEarlierDeclarations()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "b"; s.VariableExpr = "$a + $c"; })],
                     Num("a", 1, "$c * 2"), Num("b", 0, "$a + 1 + $robot.x"), Num("c", 5));
        var prob = Single(Validate(p), ValidationCodes.UnknownVariable);
        Assert.Equal("variables[0]", prob.StepPath);
        Assert.Equal("valueExpression", prob.Field);
        Assert.Null(prob.StepId);
        Assert.Contains("declaration order", prob.Message);
    }

    [Fact]
    public void CncSpecExpressionsAreChecked()
    {
        var p = Prog([Step(StepType.CncProgram, s => s.CncSpec = new CncSpec { Expressions = new() { ["holeDepth"] = "$depth +" } })]);
        Assert.Equal("cncSpec.expressions.holeDepth", Single(Validate(p), ValidationCodes.ExpressionSyntax).Field);
    }

    [Fact]
    public void EmptyFieldExpressionIsAWarning()
    {
        var p = Prog([Step(StepType.MoveL, s => { s.PointName = "Home"; s.Expressions = new() { ["offsetZ"] = " " }; })]);
        var prob = Single(Validate(p), ValidationCodes.MissingField);
        Assert.Equal(ValidationSeverity.Warning, prob.Severity);
    }

    // ── Read-only properties ────────────────────────────────────────────────

    [Theory]
    [InlineData("robot.x")]
    [InlineData("$program.runCount")]
    [InlineData("time.hour")]
    [InlineData("stb.out1")]
    public void AssigningAPropertyOrIoIsRejected(string target)
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = target; s.VariableExpr = "1"; })]);
        var prob = Single(Validate(p), ValidationCodes.ReadOnlyProperty);
        Assert.Equal("variableName", prob.Field);
        None(Validate(p), ValidationCodes.UnknownVariable);
    }

    [Fact]
    public void DeclaringAVariableNamedLikeAPropertyIsRejected()
    {
        var p = Prog([], Num("robot.x", 0, "1"));
        Assert.Equal("variables[0]", Single(Validate(p), ValidationCodes.ReadOnlyProperty).StepPath);
    }

    [Fact]
    public void LoopAndVisionTargetsCannotBeProperties()
    {
        var p = Prog([
            Step(StepType.Loop, s => { s.LoopMode = "forEach"; s.ForEachVariableName = "vals"; s.ForEachValueVariableName = "robot.z"; s.LoopSteps = [Step(StepType.PauseProgram)]; }),
            Step(StepType.RunVision, s => { s.VisionProgramId = "vp1"; s.VisionOutputs = [new VisionStepOutput { InspectionId = "a", CountVar = "time.now" }]; }),
        ], List("vals", ListElementType.Number));
        Assert.Equal(2, Validate(p).Count(x => x.Code == ValidationCodes.ReadOnlyProperty));
    }

    [Fact]
    public void SetVariableTargetMustBeDeclared()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "ghost"; s.VariableExpr = "1"; }),
                      Step(StepType.SetVariable, s => { s.VariableName = "real"; s.VariableExpr = "$ghost"; })], Num("real"));
        var prob = Single(Validate(p), ValidationCodes.UnknownVariable);   // reading $ghost later is not reported twice
        Assert.Equal("steps[0]", prob.StepPath);
    }

    // ── Scoped names ───────────────────────────────────────────────────────

    [Fact]
    public void LoopVariablesAreInScopeOnlyInsideTheBody()
    {
        var p = Prog(
        [
            Step(StepType.Loop, s =>
            {
                s.LoopMode = "forEach"; s.ForEachVariableName = "pts"; s.ForEachIndexVariableName = "k"; s.ForEachValueVariableName = "v";
                s.LoopSteps = [Step(StepType.MoveL, m => m.PointNameExpr = "$pts[$k]")];
            }),
            Step(StepType.SetVariable, s => { s.VariableName = "n"; s.VariableExpr = "$k"; }),
        ], List("pts", ListElementType.Point), Num("n"));
        var prob = Single(Validate(p), ValidationCodes.UnknownVariable);
        Assert.Equal("steps[1]", prob.StepPath);
    }

    [Fact]
    public void ImplicitOutputsAreKnown()
    {
        var p = Prog([
            Step(StepType.RunVision, s => { s.VisionProgramId = "vp1"; s.VisionOutputs = [new VisionStepOutput { InspectionId = "a", CountVar = "blobCount", PointsVar = "blobs" }]; }),
            Step(StepType.MoveL, s => { s.PointNameExpr = "$blobs[0]"; s.Expressions = new() { ["offsetZ"] = "$blobCount" }; }),
        ]);
        Clean(Validate(p));
    }

    // ── Repositories ───────────────────────────────────────────────────────

    [Fact]
    public void ReferencedGeometryMustExist()
    {
        var p = Prog(
        [
            Step(StepType.MoveL, s => s.PointName = "Nowhere"),
            Step(StepType.MoveJ, s => s.PointName = "pick"),        // case follows the repository
            Step(StepType.MoveL, s => s.PointNameExpr = "Missing"),
            Step(StepType.MoveL, s => s.GridPoint = new GridPointRef { GridId = "gridX" }),
            Step(StepType.MoveL, s => s.StackPoint = new StackPointRef { StackId = "stackX" }),
            Step(StepType.MoveL, s => { s.PointName = "Home"; s.LocalName = "Nope"; }),
            Step(StepType.SetTool, s => s.ToolName = "Hammer"),
            Step(StepType.SetTool, s => s.ToolName = ""),              // None
            Step(StepType.SetLocal, s => s.LocalName = "Other"),
            Step(StepType.RunVision, s => s.VisionProgramId = "vpX"),
            Step(StepType.MoveL, s => { s.PointName = "Home"; s.GridPoint = new GridPointRef { GridId = "grid1" }; }),
        ]);
        var probs = Validate(p);
        Assert.Equal(2, probs.Count(x => x.Code == ValidationCodes.UnknownPoint));
        Single(probs, ValidationCodes.UnknownGrid);
        Single(probs, ValidationCodes.UnknownStack);
        Assert.Equal(2, probs.Count(x => x.Code == ValidationCodes.UnknownLocal));
        Single(probs, ValidationCodes.UnknownTool);
        Single(probs, ValidationCodes.UnknownVisionProgram);
    }

    [Fact]
    public void PointsVariableTargetsMustBePointLists()
    {
        var p = Prog([Step(StepType.MoveL, s => s.VarPointName = "vals"),
                      Step(StepType.MoveL, s => s.VarPointName = "nothing")],
                     List("vals", ListElementType.Number));
        Assert.Equal(2, Validate(p).Count(x => x.Code == ValidationCodes.UnknownVariable));
    }

    [Fact]
    public void OfflineContextSkipsRepositoryChecks()
    {
        var p = Prog([Step(StepType.MoveL, s => s.PointName = "Anything"),
                      Step(StepType.CallRoutine, s => s.RoutineName = "Whatever")]);
        Clean(ProgramValidator.Validate(p, ValidationContext.Offline()));
    }

    // ── Routines ───────────────────────────────────────────────────────────

    [Fact]
    public void RoutinesAreResolvedByIdThenNameAndTheirBodiesChecked()
    {
        var routine = new BuiltProgram
        {
            Id = "r1", Name = "Pick", IsRoutine = true,
            Variables = [Num("grip", 0, "$speed * 2")],
            Steps = [Step(StepType.MoveL, s => { s.PointName = "Pick"; s.Expressions = new() { ["offsetZ"] = "$grip + $oops" }; })],
        };
        var p = Prog([Step(StepType.CallRoutine, s => { s.RoutineId = "r1"; s.RoutineName = "Renamed"; }),
                      Step(StepType.CallRoutine, s => s.RoutineName = "pick"),   // same routine again, by name
                      Step(StepType.CallRoutine, s => s.RoutineName = "Missing")],
                     Num("speed", 5));
        var probs = Validate(p, Ctx(routine));
        var bad = Single(probs, ValidationCodes.UnknownVariable);       // reported once, not per call
        Assert.Equal("steps[0].routine(Pick).steps[0]", bad.StepPath);
        Single(probs, ValidationCodes.UnknownRoutine);
    }

    [Fact]
    public void RoutineRecursionIsDetected()
    {
        var a = new BuiltProgram { Id = "a", Name = "A", IsRoutine = true };
        var b = new BuiltProgram { Id = "b", Name = "B", IsRoutine = true };
        a.Steps = [Step(StepType.CallRoutine, s => s.RoutineId = "b")];
        b.Steps = [Step(StepType.CallRoutine, s => s.RoutineId = "a")];
        var p = Prog([Step(StepType.CallRoutine, s => s.RoutineId = "a")]);
        var prob = Single(Validate(p, Ctx(a, b)), ValidationCodes.RoutineRecursion);
        Assert.Equal("steps[0].routine(A).steps[0].routine(B).steps[0]", prob.StepPath);
        Assert.Contains("A → B → A", prob.Message);
    }

    [Fact]
    public void AProgramCallingItselfIsRecursion()
    {
        var p = Prog([Step(StepType.CallRoutine, s => s.RoutineId = "main")]);
        Single(Validate(p, ValidationContext.Offline()), ValidationCodes.RoutineRecursion);
    }

    [Fact]
    public void RoutineVariablesAreVisibleToTheCaller()
    {
        var routine = new BuiltProgram { Id = "r", Name = "R", Variables = [Num("result")], Steps = [Step(StepType.PauseProgram)] };
        var p = Prog([Step(StepType.CallRoutine, s => s.RoutineId = "r"),
                      Step(StepType.SetVariable, s => { s.VariableName = "x"; s.VariableExpr = "$result"; })], Num("x"));
        Clean(Validate(p, Ctx(routine)));
    }

    [Fact]
    public void BackgroundProgramsMustExist()
    {
        var p = Prog([Step(StepType.StartBackground, s => s.BackgroundProgramName = "Conveyor")]);
        Single(Validate(p), ValidationCodes.UnknownProgram);
    }

    // ── Labels ─────────────────────────────────────────────────────────────

    [Fact]
    public void GoToLabelMustReachALabelInItsListOrAnEnclosingOne()
    {
        var p = Prog(
        [
            Step(StepType.Label, s => { s.LabelId = "top"; s.LabelName = "Top"; }),
            Step(StepType.Loop, s =>
            {
                s.LoopCount = 2;
                s.LoopSteps =
                [
                    Step(StepType.Label, l => { l.LabelId = "inner"; l.LabelName = "Inner"; }),
                    Step(StepType.GoToLabel, g => { g.LabelId = "top"; g.LabelName = "Top"; }),   // enclosing: ok
                ];
            }),
            Step(StepType.GoToLabel, s => { s.LabelId = "inner"; s.LabelName = "Inner"; }),      // nested: not reachable
        ]);
        var probs = Validate(p);
        Assert.Equal("steps[2]", Single(probs, ValidationCodes.UnknownLabel).StepPath);
    }

    [Fact]
    public void DuplicateLabelsAreReported()
    {
        var p = Prog([Step(StepType.Label, s => { s.LabelId = "a"; s.LabelName = "Start"; }),
                      Step(StepType.Label, s => { s.LabelId = "b"; s.LabelName = "start"; }),
                      Step(StepType.Label, s => { s.LabelId = "a"; s.LabelName = "Other"; })]);
        Assert.Equal(2, Validate(p).Count(x => x.Code == ValidationCodes.DuplicateLabel));
    }

    // ── Structure ──────────────────────────────────────────────────────────

    [Fact]
    public void EmptyLoopsAndBranches()
    {
        var p = Prog([Step(StepType.Loop, s => s.LoopCount = 2),
                      Step(StepType.IfCondition, s => s.Condition = Cond("1", "==", "1")),
                      Step(StepType.IfCondition, s => { s.Condition = Cond("1", "==", "1"); s.ElseSteps = [Step(StepType.PauseProgram)]; })]);
        var probs = Validate(p);
        Assert.Equal("steps[0]", Single(probs, ValidationCodes.EmptyLoop).StepPath);
        Assert.Equal("steps[1]", Single(probs, ValidationCodes.EmptyBranch).StepPath);
    }

    [Fact]
    public void MissingRequiredFields()
    {
        var p = Prog([Step(StepType.SetOutput),
                      Step(StepType.CallRoutine),
                      Step(StepType.GoToLabel),
                      Step(StepType.RunVision),
                      Step(StepType.SetVariable, s => s.VariableName = "a"),
                      Step(StepType.Loop, s => { s.LoopMode = "forEach"; s.LoopSteps = [Step(StepType.PauseProgram)]; }),
                      Step(StepType.IfCondition, s => s.IfSteps = [Step(StepType.PauseProgram)])], Num("a"));
        var fields = Validate(p).Where(x => x.Code == ValidationCodes.MissingField).Select(x => x.Field).ToList();
        Assert.Equal(["outputNumber", "routineName", "labelId", "visionProgramId", "variableExpr", "forEachVariableName", "condition"], fields);
    }

    [Fact]
    public void DisabledStepsAreAWarningAndNotCheckedFurther()
    {
        var p = Prog([Step(StepType.MoveL, s => { s.Enabled = false; s.PointName = "Nowhere"; })]);
        var probs = Validate(p);
        var prob = Single(probs, ValidationCodes.DisabledStep);
        Assert.Equal(ValidationSeverity.Warning, prob.Severity);
        None(probs, ValidationCodes.UnknownPoint);
    }

    [Fact]
    public void StepsAfterAGoToOrInfiniteLoopAreUnreachableUntilALabel()
    {
        var p = Prog(
        [
            Step(StepType.Label, s => s.LabelId = "a"),
            Step(StepType.GoToLabel, s => s.LabelId = "a"),
            Step(StepType.PauseProgram),                               // unreachable
            Step(StepType.PauseProgram),                               // same run — not reported again
            Step(StepType.Label, s => s.LabelId = "b"),
            Step(StepType.Loop, s => { s.LoopCount = 0; s.LoopSteps = [Step(StepType.PauseProgram)]; }),
            Step(StepType.PauseProgram),                               // unreachable
        ]);
        var probs = Validate(p).Where(x => x.Code == ValidationCodes.UnreachableStep).ToList();
        Assert.Equal(["steps[2]", "steps[6]"], probs.Select(x => x.StepPath));
        Assert.All(probs, x => Assert.Equal(ValidationSeverity.Warning, x.Severity));
    }

    [Fact]
    public void UnusedVariablesAreAWarningExceptGlobals()
    {
        var p = Prog([Step(StepType.SetVariable, s => { s.VariableName = "used"; s.VariableExpr = "1"; })],
                     Num("used"), Num("idle"), new ProgramVariable { Name = "shared", IsGlobal = true });
        var prob = Single(Validate(p), ValidationCodes.UnusedVariable);
        Assert.Equal("variables[1]", prob.StepPath);
        Assert.Equal(ValidationSeverity.Warning, prob.Severity);
    }

    [Fact]
    public void UnknownStepTypesAreAWarning()
    {
        var p = Prog([Step(StepType.Unknown, s => s.UnknownStepType = "Teleport")]);
        Assert.Equal(ValidationSeverity.Warning, Single(Validate(p), ValidationCodes.UnknownStepType).Severity);
    }

    [Fact]
    public void ElseIfAndElseBodiesAreWalked()
    {
        var p = Prog([Step(StepType.IfCondition, s =>
        {
            s.Condition = Cond("1", "==", "1");
            s.IfSteps = [Step(StepType.PauseProgram)];
            s.ElseIfBranches = [new ElseIfBranch { Id = "e", Condition = Cond("1", "==", "2"), Steps = [Step(StepType.MoveL, m => m.PointName = "Bad1")] }];
            s.ElseSteps = [Step(StepType.MoveL, m => m.PointName = "Bad2")];
        })]);
        var paths = Validate(p).Where(x => x.Code == ValidationCodes.UnknownPoint).Select(x => x.StepPath).ToList();
        Assert.Equal(["steps[0].elseIfBranches[0].steps[0]", "steps[0].elseSteps[0]"], paths);
    }
}
