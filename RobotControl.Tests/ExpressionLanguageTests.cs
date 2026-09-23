using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>
/// The September 2026 additions to the expression language (docs/expressions-and-variables.md §2):
/// %, ^, ?:, functions, parse errors, TryParse / ReferencedNames and properties.
/// The original syntax is pinned by ExpressionEvaluatorTests.
/// </summary>
public class ExpressionLanguageTests
{
    private static Dictionary<string, double> Vars(params (string k, double v)[] entries)
    {
        var d = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        foreach (var (k, v) in entries) d[k] = v;
        return d;
    }

    private static double Eval(string expr, Dictionary<string, double>? vars = null,
                               Dictionary<string, ListVar>? lists = null, IPropertySource? props = null) =>
        ExpressionEvaluator.Evaluate(expr, vars ?? Vars(), lists, props);

    private static Dictionary<string, ListVar> Nums(string name, params double[] v) =>
        new(StringComparer.OrdinalIgnoreCase) { [name] = ListVar.OfNumbers(v) };

    // -- Operators ---------------------------------------------------------------

    [Theory]
    [InlineData("7 % 3", 1)]
    [InlineData("-7 % 3", -1)]          // remainder takes the dividend's sign
    [InlineData("7.5 % 2", 1.5)]
    [InlineData("5 % 0", 0)]            // like / — 0 rather than NaN
    [InlineData("2 + 7 % 3 * 2", 4)]    // % binds with * /
    [InlineData("2 ^ 10", 1024)]
    [InlineData("2 ^ 3 ^ 2", 512)]      // right-assoc: 2 ^ 9
    [InlineData("-2 ^ 2", -4)]          // ^ tighter than unary minus
    [InlineData("(-2) ^ 2", 4)]
    [InlineData("2 ^ -1", 0.5)]         // the exponent may carry a sign
    [InlineData("2 * 3 ^ 2", 18)]
    [InlineData("- -5", 5)]
    [InlineData("+5", 5)]
    [InlineData("1e3 + 1", 1001)]
    public void ArithmeticOperators(string expr, double expected) => Assert.Equal(expected, Eval(expr), 9);

    [Theory]
    [InlineData("1 ? 10 : 20", 10)]
    [InlineData("0 ? 10 : 20", 20)]
    [InlineData("$a > 5 ? $a * 2 : -1", 16)]
    [InlineData("$a > 50 ? 1 : $a > 5 ? 2 : 3", 2)]       // right-assoc chain
    [InlineData("1 + 1 ? 7 : 9", 7)]                       // lowest precedence
    [InlineData("0 or 0 ? 7 : 9", 9)]
    [InlineData("(1 ? 2 : 3) + 10", 12)]
    [InlineData("1 ? 0 ? 5 : 6 : 7", 6)]
    public void Conditional(string expr, double expected) => Assert.Equal(expected, Eval(expr, Vars(("a", 8))), 9);

    [Fact]
    public void NotAsAnOperandBindsLikeUnaryMinus()
    {
        Assert.Equal(1, Eval("$a == not $b", Vars(("a", 1), ("b", 0))), 9);
        Assert.Equal(0, Eval("not $a > 5", Vars(("a", 8))), 9);   // leading not still wraps the comparison
    }

    // -- Functions ---------------------------------------------------------------

    [Theory]
    [InlineData("abs(-3)", 3)]
    [InlineData("ABS(-3)", 3)]                 // case-insensitive
    [InlineData("sign(-9)", -1)]
    [InlineData("sqrt(16)", 4)]
    [InlineData("pow(2, 5)", 32)]
    [InlineData("min(4, 2, 8)", 2)]
    [InlineData("max(4, 2, 8, 1)", 8)]
    [InlineData("min(3)", 3)]
    [InlineData("clamp(15, 0, 10)", 10)]
    [InlineData("clamp(-5, 0, 10)", 0)]
    [InlineData("round(2.5)", 3)]              // half away from zero, not banker's
    [InlineData("round(-2.5)", -3)]
    [InlineData("round(3.14159, 2)", 3.14)]
    [InlineData("round(1.005, 1)", 1.0)]
    [InlineData("round(1234, -2)", 1200)]
    [InlineData("floor(-1.5)", -2)]
    [InlineData("ceil(1.2)", 2)]
    [InlineData("trunc(-1.7)", -1)]
    [InlineData("mod(7, 3)", 1)]
    [InlineData("sin(90)", 1)]
    [InlineData("cos(180)", -1)]
    [InlineData("tan(45)", 1)]
    [InlineData("asin(1)", 90)]
    [InlineData("acos(0)", 90)]
    [InlineData("atan(1)", 45)]
    [InlineData("atan2(1, 1)", 45)]
    [InlineData("atan2(0, -1)", 180)]
    [InlineData("deg(3.141592653589793)", 180)]
    [InlineData("rad(180)", 3.141592653589793)]
    [InlineData("hypot(3, 4)", 5)]
    [InlineData("dist(0, 0, 3, 4)", 5)]
    [InlineData("dist3(1, 1, 1, 3, 4, 7)", 7)]
    [InlineData("if(1, 5, 6)", 5)]
    [InlineData("if(0, 5, 6)", 6)]
    [InlineData("map(5, 0, 10, 100, 200)", 150)]
    [InlineData("map(5, 3, 3, 100, 200)", 100)]  // degenerate input range → outLo
    [InlineData("lerp(10, 20, 0.25)", 12.5)]
    [InlineData("max(1, 2) + min(5, 3) * 2", 8)]
    [InlineData("abs(-2) ^ 3", 8)]
    public void BuiltInFunctions(string expr, double expected) => Assert.Equal(expected, Eval(expr), 9);

    [Fact]
    public void FunctionsTakeExpressionsAndVariables() =>
        Assert.Equal(7, Eval("max($a, $b + 1, min(3, $a))", Vars(("a", 5), ("b", 6))), 9);

    [Fact]
    public void ListFunctions()
    {
        var lists = Nums("vals", 4, 1, 7);
        Assert.Equal(3,  Eval("len($vals)",   lists: lists), 9);
        Assert.Equal(12, Eval("sum($vals)",   lists: lists), 9);
        Assert.Equal(4,  Eval("avg($vals)",   lists: lists), 9);
        Assert.Equal(1,  Eval("minOf($vals)", lists: lists), 9);
        Assert.Equal(7,  Eval("MAXOF($vals)", lists: lists), 9);

        var empty = Nums("e");
        Assert.Equal(0, Eval("avg($e) + minOf($e) + maxOf($e) + sum($e) + len($e)", lists: empty), 9);

        var pts = new Dictionary<string, ListVar> { ["pts"] = ListVar.OfPoints([new Vector6Val(), new Vector6Val()]) };
        Assert.Equal(2, Eval("len($pts)", lists: pts), 9);
    }

    [Fact]
    public void ListFunctionOnAnUnknownListThrowsUnknownVariable()
    {
        var ex = Assert.Throws<UnknownVariableException>(() => Eval("len($nope)"));
        Assert.Equal("nope", ex.VariableName);
    }

    [Fact]
    public void RandStaysInRange()
    {
        for (int i = 0; i < 200; i++)
        {
            var r = Eval("rand()");
            Assert.InRange(r, 0, 1);
            var s = Eval("rand(10, 20)");
            Assert.InRange(s, 10, 20);
        }
    }

    // -- Parse errors ------------------------------------------------------------

    [Theory]
    [InlineData("(1 + 2",        "expressionSyntax", 0)]
    [InlineData("1 + 2)",        "expressionSyntax", 5)]
    [InlineData("1 2",           "expressionSyntax", 2)]
    [InlineData("2 +",           "expressionSyntax", 3)]
    [InlineData("3 # 4",         "expressionSyntax", 2)]
    [InlineData("foo(1)",        "unknownFunction",  0)]
    [InlineData("1 + nope(2)",   "unknownFunction",  4)]
    [InlineData("abs(1, 2)",     "badArity",         0)]
    [InlineData("clamp(1)",      "badArity",         0)]
    [InlineData("min()",         "badArity",         0)]
    [InlineData("rand(5)",       "badArity",         0)]
    [InlineData("len(5)",        "expressionSyntax", 4)]
    [InlineData("len($a[0])",    "expressionSyntax", 4)]
    [InlineData("1 ? 2",         "expressionSyntax", 5)]
    [InlineData("$a[1",          "expressionSyntax", 4)]
    [InlineData("max(1,",        "expressionSyntax", 6)]
    [InlineData("1.2.3",         "expressionSyntax", 0)]
    [InlineData("\"text\"",      "expressionSyntax", 0)]
    public void SyntaxErrorsThrowWithPositionAndCode(string expr, string code, int position)
    {
        var ex = Assert.Throws<ExpressionParseException>(() => Eval(expr, Vars(("a", 1))));
        Assert.Equal(code, ex.Code);
        Assert.Equal(position, ex.Position);
        Assert.Equal(expr, ex.Expression);

        Assert.False(ExpressionEvaluator.TryParse(expr, out var error, out var pos));
        Assert.Equal(ex.Message, error);
        Assert.Equal(position, pos);
    }

    [Theory]
    [InlineData("")]
    [InlineData("   ")]
    [InlineData("{}")]
    [InlineData("$a + max(1, 2) ? 1 : 0")]
    [InlineData("$undeclared * 2")]          // syntax only — names are not resolved
    [InlineData("x + 2.5")]                  // bare words are legal (they read as 0)
    public void TryParseAcceptsValidSyntax(string expr)
    {
        Assert.True(ExpressionEvaluator.TryParse(expr, out var error, out var pos));
        Assert.Null(error);
        Assert.Equal(-1, pos);
        Assert.True(ExpressionEvaluator.TryParse(expr, out _));
    }

    [Fact]
    public void EmptyExpressionEvaluatesToZero() => Assert.Equal(0, Eval("  "), 9);

    // -- Introspection -------------------------------------------------------------

    [Fact]
    public void ReferencedNamesListsEveryDollarName()
    {
        var names = ExpressionEvaluator.ReferencedNames(
            "$a + $stb.in1 * $pts[$i + $j].x + len($vals) + $robot.targetX + $list.length + $A").ToList();
        Assert.Equal(["a", "stb.in1", "pts", "i", "j", "vals", "robot.targetX", "list.length"], names);
    }

    [Fact]
    public void ReferencedNamesThrowsOnSyntaxError() =>
        Assert.Throws<ExpressionParseException>(() => ExpressionEvaluator.ReferencedNames("(($a").ToList());

    [Theory]
    [InlineData("$a > 1", true)]
    [InlineData("$a == 1 or $b", true)]
    [InlineData("not $a", true)]
    [InlineData("!$a", true)]
    [InlineData("$a && $b", true)]
    [InlineData("$a + 1", false)]
    [InlineData("($a > 1) + 1", false)]
    [InlineData("$a > 1 ? 1 : 0", false)]
    [InlineData("abs($a)", false)]
    [InlineData("", false)]
    public void IsBooleanExpression(string expr, bool expected) =>
        Assert.Equal(expected, ExpressionEvaluator.IsBooleanExpression(expr));

    [Fact]
    public void FunctionTableIsDocumented()
    {
        foreach (var f in ExpressionEvaluator.Functions)
        {
            Assert.False(string.IsNullOrWhiteSpace(f.Signature));
            Assert.False(string.IsNullOrWhiteSpace(f.Description));
            Assert.True(ExpressionEvaluator.IsFunctionName(f.Name.ToUpperInvariant()));
        }
        Assert.Contains(ExpressionEvaluator.Functions, f => f.Name == "minOf" && f.TakesList);
    }

    // -- Properties ------------------------------------------------------------------

    private sealed class FakeProps(params (string Name, double Value)[] values) : IPropertySource
    {
        public int Lookups;
        private readonly Dictionary<string, double> _v =
            values.ToDictionary(v => v.Name, v => v.Value, StringComparer.OrdinalIgnoreCase);

        public bool TryGet(string name, out double value) { Lookups++; return _v.TryGetValue(name, out value); }
        public IEnumerable<(string Name, string Description, string Type)> List() =>
            _v.Keys.Select(k => (k, "", "number"));
    }

    [Fact]
    public void PropertiesResolveWhenNotAVariable()
    {
        var props = new FakeProps(("robot.x", 12.5), ("time.hour", 9));
        Assert.Equal(13.5, Eval("$robot.x + 1", props: props), 9);
        Assert.Equal(12.5, Eval("$ROBOT.X", props: props), 9);
        Assert.Equal(1, Eval("$time.hour >= 9 and $time.hour < 17", props: props), 9);
    }

    [Fact]
    public void VariablesAndIoAreConsultedBeforeProperties()
    {
        var props = new FakeProps(("stb.in1", 5));
        Assert.Equal(1, Eval("$stb.in1", Vars(("stb.in1", 1)), props: props), 9);
        Assert.Equal(0, props.Lookups); // never asked
    }

    [Fact]
    public void UnknownPropertyThrowsUnknownVariableWithTheFullName()
    {
        var ex = Assert.Throws<UnknownVariableException>(() => Eval("$robot.nope", props: new FakeProps()));
        Assert.Equal("robot.nope", ex.VariableName);
    }

    [Fact]
    public void PropertySourceProgramAndTimeValues()
    {
        var run = new FakeRun { RunCount = 3, StepIndex = 4, StepCount = 10, ElapsedMs = 1234, LoopDepth = 2 };
        var src = new RobotPropertySource(null, run);
        Assert.Equal(3,    Eval("$program.runCount",  props: src), 9);
        Assert.Equal(4,    Eval("$program.stepIndex", props: src), 9);
        Assert.Equal(10,   Eval("$program.STEPCOUNT", props: src), 9);
        Assert.Equal(1234, Eval("$program.elapsedMs", props: src), 9);
        Assert.Equal(2,    Eval("$program.loopDepth", props: src), 9);

        var now = DateTime.Now;
        Assert.InRange(Eval("$time.hour", props: src), 0, 23);
        Assert.Equal((int)now.DayOfWeek, Eval("$time.dayOfWeek", props: src), 9);
        Assert.True(Eval("$time.now", props: src) > 1.7e12);

        // No robot: robot.* is unknown rather than a silent 0.
        Assert.False(src.TryGet("robot.x", out _));
        Assert.Throws<UnknownVariableException>(() => Eval("$robot.x", props: src));
        // No run: program.* reads 0.
        Assert.Equal(0, Eval("$program.runCount", props: new RobotPropertySource(null, null)), 9);
    }

    [Fact]
    public void PropertySourceListsStaticNames()
    {
        var names = RobotPropertySource.StaticNames.ToHashSet(StringComparer.OrdinalIgnoreCase);
        foreach (var n in new[] { "robot.x", "robot.targetRz", "robot.moving", "robot.homed", "robot.faulted",
                                  "robot.driverConnected", "robot.speedS", "robot.decelJ", "robot.speedOverride",
                                  "robot.joint1", "robot.joint2x", "robot.joint2z", "robot.joint4",
                                  "program.runCount", "program.stepIndex", "program.stepCount", "program.elapsedMs",
                                  "program.loopDepth", "time.now", "time.hour", "time.minute", "time.second",
                                  "time.dayOfWeek", "time.dayOfYear" })
            Assert.Contains(n, names);
        // Without a robot only the program/time ones are listed.
        Assert.All(new RobotPropertySource(null, null).List(), p => Assert.False(p.Name.StartsWith("robot.")));
    }

    private sealed class FakeRun : IProgramRunInfo
    {
        public int  RunCount  { get; set; }
        public int  StepIndex { get; set; }
        public int  StepCount { get; set; }
        public long ElapsedMs { get; set; }
        public int  LoopDepth { get; set; }
    }

    // -- Wiring through VariableScope ---------------------------------------------

    [Fact]
    public void ScopePropertiesReachStepFieldsConditionsAndTemplates()
    {
        var scope = new VariableScope { Properties = new FakeProps(("robot.z", 40)) };
        scope.Set("a", 2);
        var step = new ProgramStep { Type = StepType.MoveL, Expressions = new() { ["offsetZ"] = "$robot.z + $a" } };
        Assert.Equal(42, scope.Eval.EvalField(step, "offsetZ", 0), 9);

        var cond = new ConditionGroup { Items = [new ConditionItem { Left = "$robot.z", Operator = ">", Right = "30" }] };
        Assert.True(scope.Eval.EvaluateCondition(cond));

        Assert.Equal("z=40 r=3.14", scope.Interpolate("z={$robot.z} r={round(3.14159, 2)}"));
    }

    [Fact]
    public void TemplatesEvaluateFunctionsButStillLeaveForgottenSigils()
    {
        var scope = new VariableScope();
        scope.Set("x", 2.5);
        Assert.Equal("3", scope.Interpolate("{round($x)}"));
        Assert.Equal("{x}", scope.Interpolate("{x}"));
        Assert.Equal("{round}", scope.Interpolate("{round}"));   // a function name alone is still a bare word
        Assert.Equal("1", scope.Interpolate("{$x > 1 and $x < 3}"));
    }

    [Fact]
    public void SyntaxErrorInAStepFieldPropagatesInsteadOfFallingBack()
    {
        var scope = new VariableScope();
        var step = new ProgramStep { Type = StepType.MoveL, OffsetZ = 5, Expressions = new() { ["offsetZ"] = "(1 + 2" } };
        Assert.Throws<ExpressionParseException>(() => scope.Eval.EvalField(step, "offsetZ", 5));

        var cond = new ConditionGroup { Items = [new ConditionItem { Left = "1 +", Operator = "==", Right = "1" }] };
        Assert.Throws<ExpressionParseException>(() => scope.Eval.EvaluateCondition(cond));
    }

    [Fact]
    public void InitialValueSyntaxErrorPropagates()
    {
        var scope = new VariableScope();
        var prog = new BuiltProgram { Id = "p", Name = "P", Variables = [new ProgramVariable { Name = "a", Value = 3, ValueExpression = "max(" }] };
        Assert.Throws<ExpressionParseException>(() => scope.Initialize(prog));
    }
}
