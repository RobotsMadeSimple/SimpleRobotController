using Controller.RobotControl;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>
/// The expression additions of docs/program-language.md §4.3: the zone functions
/// <c>inBox</c> / <c>inCircle</c> and the text helpers that return numbers —
/// <c>$s.length</c>, <c>$s.indexOf("x")</c>, <c>$s.toNumber()</c> (bad text = 0) — through the
/// evaluator, the per-tick evaluation context, condition rows and the validator.
/// </summary>
public class ExpressionFunctionTests
{
    private static double Eval(string expr, Dictionary<string, string>? strings = null,
                               Dictionary<string, double>? vars = null) =>
        ExpressionEvaluator.Evaluate(expr, vars ?? new(StringComparer.OrdinalIgnoreCase), null, null,
            strings: r => r.StartsWith('$') && strings != null && strings.TryGetValue(r[1..], out var s) ? s : r,
            stringVariables: n => strings != null && strings.TryGetValue(n, out var s) ? s : null);

    // ── inBox ─────────────────────────────────────────────────────────────────

    [Theory]
    [InlineData("inBox(5, 5, 5, 0, 0, 0, 10, 10, 10)", 1)]
    [InlineData("inBox(0, 10, 0, 0, 0, 0, 10, 10, 10)", 1)]      // on the faces counts as inside
    [InlineData("inBox(5, 5, 5, 10, 10, 10, 0, 0, 0)", 1)]       // corners in either order
    [InlineData("inBox(5, 5, 5, 10, 0, 10, 0, 10, 0)", 1)]       // mixed corner order per axis
    [InlineData("inBox(11, 5, 5, 0, 0, 0, 10, 10, 10)", 0)]
    [InlineData("inBox(5, -0.1, 5, 0, 0, 0, 10, 10, 10)", 0)]
    [InlineData("inBox(5, 5, 10.5, 0, 0, 0, 10, 10, 10)", 0)]    // z outside alone fails it
    [InlineData("inBox(3, 3, 3, 3, 3, 3, 3, 3, 3)", 1)]          // a zero-size box holds its own point
    public void InBox(string expr, double expected) => Assert.Equal(expected, Eval(expr));

    [Theory]
    [InlineData("inCircle(0, 0, 0, 0, 5)", 1)]
    [InlineData("inCircle(3, 4, 0, 0, 5)", 1)]                   // exactly on the edge
    [InlineData("inCircle(3, 4.01, 0, 0, 5)", 0)]
    [InlineData("inCircle(12, 10, 10, 10, 2)", 1)]
    [InlineData("inCircle(0, 0, 0, 0, 0)", 1)]                   // zero radius: only the centre
    [InlineData("inCircle(0, 0, 0, 0, -1)", 0)]                  // a negative radius holds nothing
    public void InCircle(string expr, double expected) => Assert.Equal(expected, Eval(expr));

    [Theory]
    [InlineData("inBox(1, 2, 3)")]
    [InlineData("inCircle(1, 2, 3, 4)")]
    [InlineData("inCircle(1, 2, 3, 4, 5, 6)")]
    public void ZoneFunctionsCheckTheirArity(string expr)
    {
        var ex = Assert.Throws<ExpressionParseException>(() => Eval(expr));
        Assert.Equal("badArity", ex.Code);
    }

    [Fact]
    public void ZoneFunctionsTakeExpressions() =>
        Assert.Equal(1, Eval("inCircle($x + 1, $y, 10, 10, 2) and not inBox($x, $y, 0, 0, 0, 0, 5, 5, 5)",
            vars: new(StringComparer.OrdinalIgnoreCase) { ["x"] = 9, ["y"] = 10 }));

    [Fact]
    public void ZoneFunctionsAreInTheFunctionTable()
    {
        Assert.Contains(ExpressionEvaluator.Functions, f => f.Name == "inBox" && f.MinArgs == 9 && f.MaxArgs == 9);
        Assert.Contains(ExpressionEvaluator.Functions, f => f.Name == "inCircle" && f.MinArgs == 5 && f.MaxArgs == 5);
        Assert.True(ExpressionEvaluator.IsFunctionName("INBOX")); // case-insensitive like the rest
    }

    // ── Text helpers ──────────────────────────────────────────────────────────

    private static readonly Dictionary<string, string> Texts = new(StringComparer.OrdinalIgnoreCase)
    {
        ["s"] = "Hello world", ["empty"] = "", ["num"] = " 12.5 ", ["exp"] = "1e3", ["bad"] = "12abc",
        ["neg"] = "-4", ["comma"] = "1,5", ["nan"] = "NaN", ["inf"] = "Infinity", ["needle"] = "world",
    };

    [Theory]
    [InlineData("$s.length", 11)]
    [InlineData("$empty.length", 0)]
    [InlineData("$s.indexOf(\"world\")", 6)]
    [InlineData("$s.indexOf(\"o\")", 4)]                // the first match
    [InlineData("$s.indexOf(\"xyz\")", -1)]
    [InlineData("$s.indexOf(\"WORLD\")", -1)]           // case-sensitive, like contains
    [InlineData("$s.indexOf(\"\")", 0)]
    [InlineData("$s.indexOf($needle)", 6)]              // the argument may be a string variable
    [InlineData("$num.toNumber()", 12.5)]               // surrounding spaces ignored
    [InlineData("$exp.toNumber()", 1000)]
    [InlineData("$neg.toNumber() * 2", -8)]
    [InlineData("$bad.toNumber()", 0)]                  // not a number → 0 (open question 6)
    [InlineData("$empty.toNumber()", 0)]
    [InlineData("$comma.toNumber()", 0)]                // invariant culture: ',' is not a decimal point
    [InlineData("$nan.toNumber()", 0)]                  // not finite → 0
    [InlineData("$inf.toNumber()", 0)]
    [InlineData("$s.length > 5 and $s.indexOf(\"H\") == 0", 1)]
    public void TextHelpers(string expr, double expected) => Assert.Equal(expected, Eval(expr, Texts));

    [Fact]
    public void ComparisonHelpersStayBooleanAndNumberHelpersDoNot()
    {
        Assert.True(ExpressionEvaluator.IsBooleanExpression("$s.contains(\"a\")"));
        Assert.False(ExpressionEvaluator.IsBooleanExpression("$s.indexOf(\"a\")"));
        Assert.False(ExpressionEvaluator.IsBooleanExpression("$s.toNumber()"));
    }

    [Theory]
    [InlineData("$s.toNumber(1)")]
    [InlineData("$s.toNumber(")]
    [InlineData("$s.indexOf()")]
    [InlineData("$s.indexOf(3)")]
    public void TextHelperSyntaxErrors(string expr) => Assert.False(ExpressionEvaluator.TryParse(expr, out _));

    [Fact]
    public void LengthOfAnUnknownNameIsStillAnError() =>
        Assert.Throws<UnknownVariableException>(() => Eval("$nope.length", Texts));

    [Fact]
    public void ListLengthAndNumberVariablesKeepTheirMeaning()
    {
        var lists = new Dictionary<string, ListVar>(StringComparer.OrdinalIgnoreCase) { ["vals"] = ListVar.OfNumbers([1, 2, 3]) };
        var vars  = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase) { ["n"] = 7 };
        Assert.Equal(3, ExpressionEvaluator.Evaluate("$vals.length", vars, lists, null, _ => "", _ => null));
        // A number variable answers with its own value, as a dotted name always has.
        Assert.Equal(7, ExpressionEvaluator.Evaluate("$n.length", vars, lists, null, _ => "", _ => null));
    }

    [Fact]
    public void WithOnlyTheTemplateResolverTheTextIsStillFound()
    {
        // Condition rows hand the evaluator a template resolver only; a name that is not a
        // string variable comes back as written, which is how .length tells the two apart.
        double v = ExpressionEvaluator.Evaluate("$s.length", new(StringComparer.OrdinalIgnoreCase), null, null,
            r => r == "$s" ? "abc" : r);
        Assert.Equal(3, v);
        Assert.Throws<UnknownVariableException>(() =>
            ExpressionEvaluator.Evaluate("$t.length", new(StringComparer.OrdinalIgnoreCase), null, null, r => r));
    }

    [Fact]
    public void StringMethodsAreListed()
    {
        var names = ExpressionEvaluator.StringMethods.Select(m => m.Name).ToList();
        Assert.Equal(["contains", "startsWith", "endsWith", "indexOf", "toNumber", "length"], names);
        Assert.All(ExpressionEvaluator.StringMethods, m => Assert.False(string.IsNullOrWhiteSpace(m.Description)));
    }

    [Theory]
    [InlineData("12", 12)]
    [InlineData("-0.5", -0.5)]
    [InlineData("  3  ", 3)]
    [InlineData("1e-2", 0.01)]
    [InlineData("abc", 0)]
    [InlineData(null, 0)]
    [InlineData("1.7976931348623157E+309", 0)] // overflows to infinity → 0
    public void TextToNumber(string? text, double expected) => Assert.Equal(expected, ExpressionEvaluator.TextToNumber(text));

    // ── Through a program's variables ────────────────────────────────────────

    [Fact]
    public void FieldExpressionsReadStringVariables()
    {
        var scope = new VariableScope();
        scope.SetString("code", "A-42");
        scope.Set("n", 2);
        Assert.Equal(4, scope.Eval.Evaluate("$code.length"));
        Assert.Equal(1, scope.Eval.Evaluate("$code.indexOf(\"-\")"));
        Assert.Equal(1, scope.Eval.Evaluate("$code.contains(\"42\")"));   // text comparisons now work in fields too
        Assert.Equal(2, scope.Eval.Evaluate("$n.toNumber()"));            // a number's text reads back as itself
        scope.SetString("qty", "17");
        Assert.Equal(34, scope.Eval.Evaluate("$qty.toNumber() * $n"));
    }

    [Fact]
    public void ConditionRowsReadStringLength()
    {
        var scope = new VariableScope();
        scope.SetString("code", "A-42");
        var cond = new ConditionGroup { Items = [new ConditionItem { Id = "c", Expr = "$code.length == 4 and $code.toNumber() == 0" }] };
        Assert.True(scope.EvaluateCondition(cond, scope.EvalVars()));
    }

    // ── Validator ─────────────────────────────────────────────────────────────

    private static List<ValidationProblem> Validate(params ProgramStep[] steps) =>
        ProgramValidator.Validate(new BuiltProgram
        {
            Id = "p", Name = "P", Steps = steps.ToList(),
            Variables = [StepsRig.Str("code", "A"), StepsRig.Num("n")],
        }, ValidationContext.Offline());

    [Fact]
    public void ValidatorAcceptsTheNewHelpers()
    {
        var problems = Validate(
            StepsRig.Set("n", "$code.length + $code.indexOf(\"x\") + $code.toNumber() + inBox(1,2,3,0,0,0,5,5,5) + inCircle(1,1,0,0,2)"));
        Assert.DoesNotContain(problems, p => p.Severity == ValidationSeverity.Error);
    }

    [Fact]
    public void ValidatorStillFlagsUnknownNamesAndTextAsANumber()
    {
        var problems = Validate(StepsRig.Set("n", "$nope.length"), StepsRig.Set("n", "$code + 1"));
        Assert.Equal(2, problems.Count(p => p.Code == ValidationCodes.UnknownVariable));
    }
}
