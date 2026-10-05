using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>Computed variables (user-defined properties) — docs/expressions-and-variables.md §7.</summary>
public class ComputedVariableTests
{
    private static BuiltProgram Program(params ProgramVariable[] vars) =>
        new() { Id = "p1", Name = "P", Variables = [.. vars] };

    private static ProgramVariable Num(string name, double value = 0) => new() { Id = name, Name = name, Value = value };

    private static ProgramVariable Computed(string name, string formula, bool global = false, bool boolean = false,
                                            bool display = false) =>
        new()
        {
            Id = name, Name = name, IsComputed = true, ValueExpression = formula,
            IsGlobal = global ? true : null, IsBoolean = boolean ? true : null, DisplayOnMonitor = display ? true : null,
        };

    private static double Eval(VariableScope scope, string expr) => scope.Eval.Evaluate(expr);

    // ── Model ────────────────────────────────────────────────────────────────

    [Fact]
    public void IsComputedRoundTripsAsJson()
    {
        var v = JsonSerializer.Deserialize<ProgramVariable>("""{"name":"area","isComputed":true,"valueExpression":"$w * $h"}""")!;
        Assert.True(v.IsComputed);
        Assert.Equal("$w * $h", v.ValueExpression);
        Assert.Contains("\"isComputed\":true", JsonSerializer.Serialize(v));
    }

    // ── Reading ──────────────────────────────────────────────────────────────

    [Fact]
    public void ReadEvaluatesTheFormulaEveryTime()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Num("w", 2), Num("h", 3), Computed("area", "$w * $h")));

        Assert.Equal(6, Eval(scope, "$area"));
        Assert.False(scope.MergedVars().ContainsKey("area")); // never stored

        scope.Set("w", 10);
        Assert.Equal(30, Eval(scope, "$area"));
        Assert.Equal(31, Eval(scope, "$area + 1"));
    }

    [Fact]
    public void ReadInsideATickSeesAWriteMadeEarlierInTheSameTick()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Num("w", 2), Computed("twice", "$w * 2")));
        scope.Eval.BeginTick();
        try
        {
            Assert.Equal(4, Eval(scope, "$twice"));
            scope.Set("w", 5);
            Assert.Equal(10, Eval(scope, "$twice"));
        }
        finally { scope.Eval.EndTick(); }
    }

    [Fact]
    public void NestedComputedVariablesResolve()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Num("w", 2), Num("h", 3),
            Computed("area", "$w * $h"), Computed("volume", "$area * $depth"), Num("depth", 4),
            Computed("big", "$volume > 20", boolean: true)));

        Assert.Equal(24, Eval(scope, "$volume"));
        Assert.Equal(1, Eval(scope, "$big"));
        scope.Set("depth", 1);
        Assert.Equal(0, Eval(scope, "$big"));
    }

    [Fact]
    public void ComputedVariablesWorkInConditionsTemplatesAndIndexes()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Num("n", 1), Computed("next", "$n + 1"), Computed("ok", "$n > 0", boolean: true),
            new ProgramVariable { Name = "vals", Items = [ObjectRecord.FromScalar(10), ObjectRecord.FromScalar(20), ObjectRecord.FromScalar(30)],
                                  ElementType = ListElementType.Number }));

        Assert.Equal(30, Eval(scope, "$vals[$next]"));
        var cond = new ConditionGroup { Items = [new ConditionItem { Left = "$next", Operator = "==", Right = "2" }] };
        Assert.True(scope.Eval.EvaluateCondition(cond));
        Assert.Equal("next=2 ok=True sum=3", scope.Interpolate("next=$next ok=$ok sum={$next + 1}"));
    }

    [Fact]
    public void ComputedVariablesChainInFrontOfProperties()
    {
        var scope = new VariableScope { Properties = new FakeProps(("robot.z", 40)) };
        scope.Initialize(Program(Computed("clearance", "$robot.z + 10")));
        Assert.Equal(50, Eval(scope, "$clearance"));
        Assert.Equal(40, Eval(scope, "$robot.z"));
        Assert.Throws<UnknownVariableException>(() => Eval(scope, "$robot.nope"));
    }

    [Fact]
    public void AnUnknownVariableInAFormulaPropagates()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Computed("bad", "$missing + 1")));
        var ex = Assert.Throws<UnknownVariableException>(() => Eval(scope, "$bad"));
        Assert.Equal("missing", ex.VariableName);
    }

    [Fact]
    public void ACycleThrowsComputedCycleInsteadOfRecursing()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Computed("a", "$b + 1"), Computed("b", "$a + 1"), Computed("self", "$self")));

        var ex = Assert.Throws<ExpressionParseException>(() => Eval(scope, "$a"));
        Assert.Equal("computedCycle", ex.Code);
        Assert.Contains("$a → $b → $a", ex.Message);
        Assert.Equal("computedCycle", Assert.Throws<ExpressionParseException>(() => Eval(scope, "$self")).Code);

        // The guard is released after a failure: an unrelated read still works.
        scope.Set("x", 3);
        Assert.Equal(3, Eval(scope, "$x"));
    }

    // ── Writing ──────────────────────────────────────────────────────────────

    [Fact]
    public void WritingAComputedVariableIsRefused()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Num("w", 1), Computed("area", "$w * 2")));

        var ex = Assert.Throws<ComputedVariableWriteException>(() => scope.Set("AREA", 5));
        Assert.Equal("AREA", ex.VariableName);
        Assert.Throws<ComputedVariableWriteException>(() => scope.SetList("area", ListVar.OfNumbers([1])));
        Assert.Throws<ComputedVariableWriteException>(() => scope.ControlStopwatch("area", "Start"));
        Assert.Equal(2, Eval(scope, "$area"));
    }

    [Fact]
    public void ClearForgetsComputedVariables()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(Computed("k", "7")));
        scope.Clear();
        Assert.Throws<UnknownVariableException>(() => Eval(scope, "$k"));
        scope.Set("k", 1); // an ordinary variable again
    }

    // ── Global ───────────────────────────────────────────────────────────────

    [Fact]
    public void GlobalComputedIsReadableFromAnotherProgram()
    {
        var globals = new GlobalVariableStore();
        var owner = new VariableScope(globals);
        owner.Initialize(Program(new ProgramVariable { Name = "parts", Value = 4, IsGlobal = true },
                                 Computed("boxes", "ceil($parts / 3)", global: true)));

        var reader = new VariableScope(globals);          // declares nothing
        reader.Initialize(Program(Num("parts", 100)));    // a local of the same name does not leak in
        Assert.Equal(2, Eval(reader, "$boxes"));

        owner.Set("parts", 10);
        Assert.Equal(4, Eval(reader, "$boxes"));

        Assert.Throws<ComputedVariableWriteException>(() => reader.Set("boxes", 1));
        Assert.True(globals.TryGetComputed("BOXES", out var f));
        Assert.Equal("ceil($parts / 3)", f);

        globals.Clear();
        Assert.False(globals.TryGetComputed("boxes", out _));
    }

    [Fact]
    public void GlobalComputedDoesNotSeeTheReadersLocals()
    {
        var globals = new GlobalVariableStore();
        var owner = new VariableScope(globals);
        owner.Initialize(Program(Computed("g", "$localOnly + 1", global: true)));

        var reader = new VariableScope(globals);
        reader.Initialize(Program(Num("localOnly", 5)));
        Assert.Throws<UnknownVariableException>(() => Eval(reader, "$g"));
    }

    [Fact]
    public void LocalComputedWinsOverGlobalOfTheSameName()
    {
        var globals = new GlobalVariableStore();
        new VariableScope(globals).Initialize(Program(Computed("v", "1", global: true)));
        var local = new VariableScope(globals);
        local.Initialize(Program(Computed("v", "2")));
        Assert.Equal(2, Eval(local, "$v"));
    }

    // ── Monitor / symbols ────────────────────────────────────────────────────

    [Fact]
    public void DisplayAndSnapshotIncludeComputedValues()
    {
        var globals = new GlobalVariableStore();
        var scope = new VariableScope(globals);
        var program = Program(Num("w", 3),
            Computed("area", "$w * $w", display: true),
            Computed("ready", "$w > 1", boolean: true, display: true),
            Computed("broken", "$nope", display: true),
            Computed("hidden", "1"),
            Computed("shared", "$w2 + 1", global: true, display: true),
            new ProgramVariable { Name = "w2", Value = 1, IsGlobal = true });
        scope.Initialize(program);

        var display = scope.GetDisplayVariables(program);
        Assert.Contains(("area", 9.0, false), display);
        Assert.Contains(("ready", 1.0, true), display);
        Assert.Contains(("shared", 2.0, false), display);
        var broken = Assert.Single(display, d => d.Name == "broken");
        Assert.True(double.IsNaN(broken.Value));
        Assert.DoesNotContain(display, d => d.Name == "hidden");

        var snap = scope.SnapshotValues();
        Assert.Equal(9.0, snap["area"]);
        Assert.Equal(1.0, snap["hidden"]);
        Assert.Equal(2.0, snap["shared"]);
        Assert.Null(snap["broken"]);
    }

    [Fact]
    public void SymbolsListComputedVariables()
    {
        var globals = new GlobalVariableStore();
        var program = Program(Num("w", 3), Computed("area", "$w * $w"), Computed("big", "$area > 5", boolean: true),
                              Computed("g", "2 + 2", global: true));
        var props = ComputedPropertySource.ForGlobals(globals, null, null);

        var scope = new VariableScope(globals);
        scope.Initialize(program);
        var syms = JsonSerializer.SerializeToElement(
            BuiltProgramCommands.VariableSymbols(program, scope.SnapshotValues(), globals, props));

        var area = syms.EnumerateArray().Single(e => e.GetProperty("name").GetString() == "area");
        Assert.Equal("computed", area.GetProperty("kind").GetString());
        Assert.Equal("$w * $w", area.GetProperty("expression").GetString());
        Assert.Equal(9, area.GetProperty("value").GetDouble());
        Assert.False(area.GetProperty("isGlobal").GetBoolean());
        var big = syms.EnumerateArray().Single(e => e.GetProperty("name").GetString() == "big");
        Assert.True(big.GetProperty("isBoolean").GetBoolean());
        var g = syms.EnumerateArray().Single(e => e.GetProperty("name").GetString() == "g");
        Assert.True(g.GetProperty("isGlobal").GetBoolean());
        Assert.Equal(4, g.GetProperty("value").GetDouble());

        // Not held by an executor: no live value for a local, the store's for a global.
        var idle = JsonSerializer.SerializeToElement(BuiltProgramCommands.VariableSymbols(program, null, globals, props));
        Assert.Equal(JsonValueKind.Null,
            idle.EnumerateArray().Single(e => e.GetProperty("name").GetString() == "area").GetProperty("value").ValueKind);
        Assert.Equal(4, idle.EnumerateArray().Single(e => e.GetProperty("name").GetString() == "g").GetProperty("value").GetDouble());

        // No program: global values and global computed formulas.
        var none = JsonSerializer.SerializeToElement(BuiltProgramCommands.VariableSymbols(null, null, globals, props));
        var gg = Assert.Single(none.EnumerateArray(), e => e.GetProperty("kind").GetString() == "computed");
        Assert.Equal("2 + 2", gg.GetProperty("expression").GetString());
        Assert.Equal(4, gg.GetProperty("value").GetDouble());
    }

    // ── Validation ───────────────────────────────────────────────────────────

    private static int _ids;
    private static ProgramStep Step(StepType type, Action<ProgramStep>? init = null)
    {
        var s = new ProgramStep { Id = $"c{++_ids}", Type = type };
        init?.Invoke(s);
        return s;
    }

    private static List<ValidationProblem> Validate(IEnumerable<ProgramStep> steps, params ProgramVariable[] vars) =>
        ProgramValidator.Validate(new BuiltProgram { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars.ToList() },
                                  ValidationContext.Offline());

    private static ProgramStep Uses(string expr) =>
        Step(StepType.StatusUpdate, s => s.StatusMessage = "{" + expr + "}");

    [Fact]
    public void ValidComputedVariablesHaveNoErrors()
    {
        var problems = Validate(
            [Step(StepType.SetVariable, s => { s.VariableName = "w"; s.VariableExpr = "$area + $robot.z"; })],
            Num("w", 1), Computed("area", "$w * $half"), Computed("half", "$w / 2 + $stb.in1 + $time.hour"),
            new ProgramVariable { Name = "gw", IsGlobal = true }, Computed("g", "$gw * 2 + $robot.x", global: true));
        Assert.DoesNotContain(problems, p => p.Severity == ValidationSeverity.Error);
    }

    [Fact]
    public void FormulaReferencesAreResolved()
    {
        var problems = Validate([Uses("$c")], Computed("c", "$nope + sqrt("));
        Assert.Contains(problems, p => p.Code == ValidationCodes.ExpressionSyntax && p.StepPath == "variables[0]"
                                       && p.Field == "valueExpression");
        problems = Validate([Uses("$c")], Computed("c", "$nope + 1"));
        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownVariable && p.StepPath == "variables[0]");
        problems = Validate([Uses("$c")], Computed("c", ""));
        Assert.Contains(problems, p => p.Code == ValidationCodes.MissingField && p.Field == "valueExpression");
    }

    [Fact]
    public void AFormulaMayReferenceVariablesDeclaredBelowIt()
    {
        var problems = Validate([Uses("$c")], Computed("c", "$later * 2"), Num("later", 1));
        Assert.DoesNotContain(problems, p => p.Severity == ValidationSeverity.Error);
    }

    [Fact]
    public void CyclesAreReported()
    {
        var problems = Validate([Uses("$a + $c")],
            Computed("a", "$b + 1"), Computed("b", "$a + 1"), Computed("c", "$c"), Computed("d", "$a"));
        var cycles = problems.Where(p => p.Code == ValidationCodes.ComputedCycle).ToList();
        Assert.Equal(["variables[0]", "variables[1]", "variables[2]"], cycles.Select(c => c.StepPath).ToArray());
        Assert.Contains("$a → $b → $a", cycles[0].Message);
        Assert.All(cycles, c => Assert.Equal(ValidationSeverity.Error, c.Severity));
    }

    [Fact]
    public void WritingAComputedVariableIsAnError()
    {
        var problems = Validate(
        [
            Step(StepType.SetVariable, s => { s.VariableName = "c"; s.VariableExpr = "1"; }),
            Step(StepType.Loop, s => { s.LoopCount = 2; s.ForEachIndexVariableName = "c"; s.LoopSteps = [Uses("1")]; }),
            Step(StepType.Loop, s =>
            {
                s.LoopMode = "forEach"; s.ForEachVariableName = "vals"; s.ForEachValueVariableName = "c";
                s.LoopSteps = [Uses("1")];
            }),
            Step(StepType.StopwatchControl, s => { s.StopwatchVariableName = "c"; s.StopwatchAction = "Start"; }),
            Step(StepType.HttpRequest, s => { s.JsonUrl = "http://x"; s.JsonInbound = [new JsonInboundMapping { Key = "k", VariableName = "c" }]; }),
            Step(StepType.RunVision, s => { s.VisionOutputs = [new VisionStepOutput { CountVar = "c" }]; }),
        ], Computed("c", "1"), new ProgramVariable { Name = "vals", Items = [], ElementType = ListElementType.Number });

        var writes = problems.Where(p => p.Code == ValidationCodes.ComputedVariable).ToList();
        Assert.Equal(["steps[0]", "steps[1]", "steps[2]", "steps[3]", "steps[4]", "steps[5]"],
                     writes.Select(w => w.StepPath).ToArray());
        Assert.All(writes, w => Assert.Equal(ValidationSeverity.Error, w.Severity));
    }

    [Fact]
    public void GlobalComputedMayOnlyUseGlobals()
    {
        var problems = Validate([Uses("$g + $ok")],
            Num("local", 1), new ProgramVariable { Name = "shared", IsGlobal = true },
            new ProgramVariable { Name = "vals", Items = [], ElementType = ListElementType.Number },
            Computed("lc", "1"),
            Computed("g", "$local + $shared + $lc + len($vals)", global: true),
            Computed("ok", "$shared + $stb.in1 + $robot.x + $g", global: true));
        var scope = problems.Where(p => p.Code == ValidationCodes.ComputedGlobalScope).ToList();
        Assert.Equal(3, scope.Count);
        Assert.All(scope, p => Assert.Equal("variables[4]", p.StepPath));
        Assert.Contains(scope, p => p.Message.Contains("$local"));
        Assert.Contains(scope, p => p.Message.Contains("$lc"));
        Assert.Contains(scope, p => p.Message.Contains("$vals"));
    }

    [Fact]
    public void StoredKindFlagsConflictWithComputed()
    {
        var bad = Computed("c", "1");
        bad.IsPersistent = true;
        bad.IsStopwatch = true;
        var problems = Validate([Uses("$c + $l")], bad,
            new ProgramVariable { Name = "l", IsComputed = true, ValueExpression = "1", Items = [] },
            new ProgramVariable { Name = "s", IsComputed = true, ValueExpression = "1", IsString = true },
            new ProgramVariable { Name = "i", IsComputed = true, ValueExpression = "1", IsImage = true });
        var conflicts = problems.Where(p => p.Code == ValidationCodes.ComputedKindConflict).ToList();
        Assert.Equal(4, conflicts.Count);
        Assert.Contains("persistent", conflicts[0].Message);
        Assert.Contains("stopwatch", conflicts[0].Message);
    }

    private sealed class FakeProps(params (string Name, double Value)[] props) : IPropertySource
    {
        public bool TryGet(string name, out double value)
        {
            foreach (var p in props)
                if (string.Equals(p.Name, name, StringComparison.OrdinalIgnoreCase)) { value = p.Value; return true; }
            value = 0;
            return false;
        }

        public IEnumerable<(string Name, string Description, string Type)> List() =>
            props.Select(p => (p.Name, "", "number"));
    }
}
