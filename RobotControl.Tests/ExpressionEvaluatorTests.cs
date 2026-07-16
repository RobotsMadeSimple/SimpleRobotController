using Controller.RobotControl;

namespace RobotControl.Tests;

public class ExpressionEvaluatorTests
{
    // Matches ProgramExecutor.MergedVars() - case-insensitive keys.
    private static Dictionary<string, double> Vars(params (string k, double v)[] entries)
    {
        var d = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        foreach (var (k, v) in entries) d[k] = v;
        return d;
    }

    private static double Eval(string expr,
        Dictionary<string, double>? vars = null,
        Dictionary<string, List<double>>? lists = null,
        Dictionary<string, List<Vector6Val>>? pts = null)
        => ExpressionEvaluator.Evaluate(expr, vars ?? Vars(), lists, pts);

    // -- Arithmetic ------------------------------------------------------------

    [Theory]
    [InlineData("42", 42)]
    [InlineData("-5", -5)]
    [InlineData("2 + 3 * 4", 14)]        // precedence
    [InlineData("(2 + 3) * 4", 20)]      // grouping
    [InlineData("10 - 4 - 3", 3)]        // left assoc
    [InlineData("7 / 2", 3.5)]
    [InlineData("5 / 0", 0)]             // div-by-zero -> 0 (intentional)
    [InlineData("true", 1)]
    [InlineData("FALSE", 0)]
    public void Arithmetic(string expr, double expected)
        => Assert.Equal(expected, Eval(expr), 9);

    // -- Scalar variables --------------------------------------------------------

    [Fact]
    public void ScalarVariableResolves()
        => Assert.Equal(12.5, Eval("$x + 2.5", Vars(("x", 10))), 9);

    [Fact]
    public void VariableLookupIsCaseInsensitive()
        => Assert.Equal(10, Eval("$ApproachZ", Vars(("approachz", 10))), 9);

    // -- Lists and points --------------------------------------------------------

    [Fact]
    public void ListIndexAndLength()
    {
        var lists = new Dictionary<string, List<double>> { ["vals"] = [5, 7, 9] };
        Assert.Equal(7, Eval("$vals[1]", lists: lists), 9);
        Assert.Equal(3, Eval("$vals.length", lists: lists), 9);
        Assert.Equal(3, Eval("$vals.count", lists: lists), 9);
        Assert.Equal(0, Eval("$vals[99]", lists: lists), 9);   // out of range -> 0
        Assert.Equal(0, Eval("$vals", lists: lists), 9);       // bare list name -> 0, no throw
    }

    [Fact]
    public void PointComponents()
    {
        var pts = new Dictionary<string, List<Vector6Val>>
        {
            ["blobs"] = [new Vector6Val { X = 1, Y = 2, Z = 3, RX = 4, RY = 5, RZ = 6 }],
        };
        Assert.Equal(1, Eval("$blobs[0].x", pts: pts), 9);
        Assert.Equal(3, Eval("$blobs[0][2]", pts: pts), 9);
        Assert.Equal(1, Eval("$blobs.length", pts: pts), 9);
        Assert.Equal(0, Eval("$blobs", pts: pts), 9);          // bare points name -> 0, no throw
    }

    // -- Dotted IO variables -----------------------------------------------------

    [Theory]
    [InlineData("$stb.in1", "stb.in1", 1)]
    [InlineData("$relay.1", "relay.1", 1)]
    [InlineData("$nano.Board.pin1", "nano.Board.pin1", 1)]
    public void DottedIoNamesResolve(string expr, string key, double value)
        => Assert.Equal(value, Eval(expr, Vars((key, value))), 9);

    [Fact]
    public void DottedLookupPrefersLongestMatch()
        => Assert.Equal(7, Eval("$stb.in1", Vars(("stb", 99), ("stb.in1", 7))), 9);

    [Fact]
    public void DottedIoNamesWorkInsideExpressions()
        => Assert.Equal(2, Eval("$stb.in1 + $stb.in2", Vars(("stb.in1", 1), ("stb.in2", 1))), 9);

    // -- Unknown variables throw ---------------------------------------------------

    [Fact]
    public void UnknownVariableThrows()
    {
        var ex = Assert.Throws<UnknownVariableException>(() => Eval("$typo + 1", Vars(("real", 5))));
        Assert.Equal("typo", ex.VariableName);
    }

    [Fact]
    public void UnknownVariableWithIndexThrows()
        => Assert.Throws<UnknownVariableException>(() => Eval("$typo[0]"));

    [Fact]
    public void UnknownDottedVariableThrows()
        => Assert.Throws<UnknownVariableException>(() => Eval("$stb.in9", Vars(("stb.in1", 1))));
}
