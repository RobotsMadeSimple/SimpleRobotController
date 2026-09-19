using Controller.RobotControl;

namespace RobotControl.Tests;

/// <summary>
/// The point target field takes one expression that resolves either to coordinates
/// (an indexed points variable) or to the name of a saved point. This is the split
/// between the two, so a mistake here silently sends a move somewhere else.
/// </summary>
public class PointsRefParseTests
{
    [Theory]
    [InlineData("$pts[0]",        "pts", "0")]
    [InlineData("$pts[$i]",       "pts", "$i")]
    [InlineData("  $pts[ $i ] ",  "pts", "$i")]      // surrounding and inner padding
    [InlineData("{$pts[$i]}",     "pts", "$i")]      // braced form
    [InlineData("$pts[$row * 3]", "pts", "$row * 3")] // index is itself an expression
    [InlineData("$pts[]",         "pts", "")]        // empty index — caller defaults to 0
    public void ParsesAnIndexedReference(string expr, string name, string idx)
    {
        Assert.True(ProgramExecutor.TryParsePointsRef(expr, out var n, out var i));
        Assert.Equal(name, n);
        Assert.Equal(idx, i);
    }

    // Anything that is not *only* a reference is text being assembled into a point
    // name, and has to fall through to the name lookup instead.
    [Theory]
    [InlineData("bin$pts[0]")]      // prefixed
    [InlineData("$pts[0]x")]        // suffixed
    [InlineData("$target")]         // string variable, no index
    [InlineData("{$prefix}{$i}")]   // two references joined
    [InlineData("pts[0]")]          // missing sigil
    [InlineData("")]
    public void RejectsAnythingElse(string expr)
        => Assert.False(ProgramExecutor.TryParsePointsRef(expr, out _, out _));

    [Fact]
    public void RejectsNull()
        => Assert.False(ProgramExecutor.TryParsePointsRef(null!, out _, out _));
}
