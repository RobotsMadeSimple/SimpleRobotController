using Controller.RobotControl;
using Controller.RobotControl.Language;
using Controller.RobotControl.Validation;
using static RobotControl.Tests.Language.SrsTestKit;

namespace RobotControl.Tests.Language;

/// <summary>Every parse problem code, with its exact 1-based line and UTF-16 column span
/// (endCol exclusive), plus line-based error recovery.</summary>
public class ParserErrorTests
{
    private static ParseProgramTextResult Parse(string text) => SrsLanguage.Parse(text, null, Names());

    /// <summary>The problem with <paramref name="code"/>; asserts it sits on <paramref name="line"/>
    /// starting at the first occurrence of <paramref name="at"/> on that line.</summary>
    private static TextProblem Expect(string text, string code, int line, string? at = null, string severity = "error", int? length = null)
    {
        var r = Parse(text);
        var p = r.Problems.FirstOrDefault(x => x.Code == code);
        Assert.True(p != null, $"no {code} in: " + string.Join("; ", r.Problems.Select(x => $"{x.Line}:{x.Col} {x.Code} {x.Message}")));
        Assert.Equal(severity, p!.Severity);
        Assert.Equal("parse", p.Source);
        Assert.Equal(line, p.Line);
        if (at != null)
        {
            var src = text.Split('\n')[line - 1];
            int col = src.IndexOf(at, StringComparison.Ordinal) + 1;
            Assert.True(col > 0, $"'{at}' not on line {line}");
            Assert.Equal(col, p.Col);
            if (length != null) Assert.Equal(col + length.Value, p.EndCol);
        }
        Assert.True(p.EndLine > p.Line || p.EndCol > p.Col, "a problem spans at least one column");
        if (severity == "error") Assert.False(r.Ok);
        return p;
    }

    [Fact] public void UnknownKeyword()
    {
        var p = Expect("#!srs 1\nMoveQ Pick1", TextProblemCodes.UnknownKeyword, 2, "MoveQ", length: 5);
        Assert.Contains("did you mean", p.Message);
    }

    [Fact] public void UnknownArgumentSuggestsTheClosest()
    {
        var p = Expect("MoveL Pick1, sped=3", TextProblemCodes.UnknownArgument, 1, "sped", length: 4);
        Assert.Contains("speed", p.Message);
    }

    [Fact] public void DuplicateArgument()
    {
        var text = "MoveL Pick1, speed=3, speed=4";
        var p = Expect(text, TextProblemCodes.DuplicateArgument, 1);
        Assert.Equal(text.LastIndexOf("speed", StringComparison.Ordinal) + 1, p.Col);
    }

    [Fact] public void MissingArgument() => Expect("SetSpeedL", TextProblemCodes.MissingArgument, 1, "SetSpeedL");

    [Fact] public void MissingArgumentIsNotRaisedWhenWithFillsIt()
    {
        var r = Parse("@with {\"speed\": 5}\nSetSpeedL");
        Assert.True(r.Ok);
        Assert.Equal(5, r.Program!.Steps[0].Speed);
    }

    [Fact] public void BadValueOnAnOutput() => Expect("SetOutput stb 1 maybe", TextProblemCodes.BadValue, 1, "maybe");
    [Fact] public void BadValueOnAnEnum() => Expect("Wait until $a, onTimeout=explode", TextProblemCodes.BadValue, 1, "explode");
    [Fact] public void BadValueNeedsParenthesesForAPositionalExpression() => Expect("SetSpeedL 2 * 3", TextProblemCodes.BadValue, 1, "*");
    [Fact] public void BadValueOnAWholeNumber() => Expect("RotaryIndex 1, count=2.5", TextProblemCodes.BadValue, 1, "2.5");

    [Fact] public void ExpressionSyntaxPointsIntoTheExpression() => Expect("$x = 3 + * 4", TextProblemCodes.ExpressionSyntax, 1, "*");
    [Fact] public void UnknownFunction() => Expect("$x = foo(1)", TextProblemCodes.UnknownFunction, 1, "foo");
    [Fact] public void BadArity() => Expect("$x = abs(1, 2)", TextProblemCodes.BadArity, 1, "abs");

    [Fact] public void ExpressionErrorInANamedArgument() =>
        Expect("MoveL Pick1, speed=($v +)", TextProblemCodes.ExpressionSyntax, 1);

    [Fact] public void MissingEnd() => Expect("pause\nif $a\n  pause", TextProblemCodes.MissingEnd, 2, "if");
    [Fact] public void MissingEndAfterRows() => Expect("HttpReceive \"x\"\n  receive \"a\" -> $a\nMoveL Pick1", TextProblemCodes.MissingEnd, 1, "HttpReceive");
    [Fact] public void UnexpectedEnd() => Expect("#!srs 1\npause\nend", TextProblemCodes.UnexpectedEnd, 3, "end", length: 3);
    [Fact] public void MisplacedElse() => Expect("repeat 3\n  pause\nelse\nend", TextProblemCodes.MisplacedClause, 3, "else");
    [Fact] public void MisplacedCaseAtTopLevel() => Expect("case 1", TextProblemCodes.MisplacedClause, 1, "case");
    [Fact] public void StepsBeforeTheFirstCase() => Expect("switch $a\n  pause\ncase 1\nend", TextProblemCodes.MisplacedClause, 1);

    [Fact] public void DeclarationKind() => Expect("persistent computed x = 1", TextProblemCodes.DeclarationKind, 1);
    [Fact] public void ComputedNeedsAFormula() => Expect("computed x", TextProblemCodes.MissingArgument, 1);

    [Fact] public void StringRequiredForAStringVariable() =>
        Expect("var s: string\n$s = 1 + 2", TextProblemCodes.StringRequired, 2, "1 + 2", length: 5);

    [Fact] public void StringRequiredForANumberVariable() =>
        Expect("var n = 0\n$n = \"five\"", TextProblemCodes.StringRequired, 2, "\"five\"");

    [Fact] public void DuplicateId() =>
        Expect("@id \"a\"\npause\n@id \"a\"\npause", TextProblemCodes.DuplicateId, 3, "@id");

    [Fact] public void UnknownLabelIsAWarning()
    {
        var p = Expect("goto nowhere", TextProblemCodes.UnknownLabel, 1, "nowhere", severity: "warning");
        Assert.True(Parse("goto nowhere").Ok);
        Assert.Contains("nowhere", p.Message);
    }

    [Fact] public void UnknownPointIsAWarningAndKeepsTheName()
    {
        Expect("MoveL Nowhere", ValidationCodes.UnknownPoint, 1, "Nowhere", severity: "warning", length: 7);
        var r = Parse("MoveL Nowhere");
        Assert.True(r.Ok);
        Assert.Equal("Nowhere", r.Program!.Steps[0].PointName);
        Assert.Null(r.Program.Steps[0].PointId);
    }

    [Fact] public void UnknownDeviceKeepsTheTextAsTheId()
    {
        Expect("Tare device=\"Ghost\", pin=1", TextProblemCodes.UnknownDevice, 1, "\"Ghost\"", severity: "warning");
        Assert.Equal("Ghost", Parse("Tare device=\"Ghost\", pin=1").Program!.Steps[0].LoadCellDeviceId);
    }

    [Fact] public void UnknownAttribute() => Expect("@shiny\npause", TextProblemCodes.Syntax, 1, "@shiny");

    [Fact] public void BadWithJson() => Expect("@with {\"speed\": }\nSetSpeedL 1", TextProblemCodes.Syntax, 1, "{");

    [Fact] public void UnknownWithFieldIsDropped()
    {
        var p = Expect("@with {\"sped\": 3}\nSetSpeedL 1", TextProblemCodes.UnknownField, 2, severity: "warning");
        Assert.Contains("sped", p.Message);
    }

    [Fact] public void WithOfTheWrongShape() => Expect("@with {\"speed\": \"fast\"}\nSetSpeedL 1", TextProblemCodes.Syntax, 2);

    [Fact] public void UnterminatedString() => Expect("status \"oops", TextProblemCodes.Syntax, 1, "\"oops");

    [Fact] public void UnclosedBracketStopsAtABlankLine()
    {
        var r = Parse("MoveL Pick1, offset=(z=1\n\nMoveL Pick1, sped=1");
        Assert.Contains(r.Problems, p => p.Code == TextProblemCodes.Syntax && p.Line == 1 && p.Col == 21);
        Assert.Contains(r.Problems, p => p.Code == TextProblemCodes.UnknownArgument && p.Line == 3);
    }

    [Fact] public void ColumnsCountUtf16CodeUnits()
    {
        var text = "status \"🤖 robot\", oops=1";
        var p = Expect(text, TextProblemCodes.UnknownArgument, 1, "oops", length: 4);
        Assert.Equal(text.IndexOf("oops", StringComparison.Ordinal) + 1, p.Col); // the emoji counts as 2
    }

    [Fact] public void RecoveryKeepsParsingLineByLine()
    {
        var r = Parse("MoveQ\nMoveL Pick1\nrepeat 2\n  MoveL Pick1, sped=1\n  bogus\nend\nend\nSetSpeedL fast");
        var codes = r.Problems.Where(p => p.Severity == "error").Select(p => (p.Line, p.Code)).ToList();
        Assert.Contains((1, TextProblemCodes.UnknownKeyword), codes);
        Assert.Contains((4, TextProblemCodes.UnknownArgument), codes);
        Assert.Contains((5, TextProblemCodes.UnknownKeyword), codes);
        Assert.Contains((7, TextProblemCodes.UnexpectedEnd), codes);
        Assert.Contains((8, TextProblemCodes.BadValue), codes);
        Assert.False(r.Ok);
        Assert.Null(r.Program);
        Assert.NotEmpty(r.Map); // the statements that did parse are still mapped
    }

    [Fact] public void ContinuationLinesMakeOneStatement()
    {
        var r = Parse("MoveL Pick1,\n  speed=200,\n  offset=(x=1,\n          z=2)");
        Assert.True(r.Ok, string.Join("; ", r.Problems.Select(p => p.Message)));
        var s = Assert.Single(r.Program!.Steps);
        Assert.Equal(200, s.Speed);
        Assert.Equal(2, s.OffsetZ);
        Assert.Equal(1, r.Map[0].Line);
        Assert.Equal(4, r.Map[0].EndLine);
    }

    [Fact] public void CommentsBecomeStepCommentsOrNotes()
    {
        var r = Parse("# about the move\nMoveL Pick1  # and more\n\n# a free note\n\n## Section\npause");
        Assert.True(r.Ok);
        var steps = r.Program!.Steps;
        Assert.Equal(StepType.MoveL, steps[0].Type);
        Assert.Equal("about the move\nand more", steps[0].Comment);
        Assert.Equal(StepType.Note, steps[1].Type);
        Assert.Equal("a free note", steps[1].Comment);
        Assert.Null(steps[1].NoteStyle);
        Assert.Equal("section", steps[2].NoteStyle);
        Assert.Equal("Section", steps[2].Comment);
        Assert.Equal(StepType.PauseProgram, steps[3].Type);
    }

    [Fact] public void KeywordsAreCaseInsensitive()
    {
        var r = Parse("movel Pick1, SPEED=3\nIF $a\n  PAUSE\nEND");
        Assert.True(r.Ok, string.Join("; ", r.Problems.Select(p => p.Message)));
        Assert.Equal(3, r.Program!.Steps[0].Speed);
        Assert.Equal(StepType.IfCondition, r.Program.Steps[1].Type);
    }

    [Fact] public void ValidationProblemsAreMappedOntoTheText()
    {
        var text = "pause\nMoveL here, speed=$nope * 2";
        var r = SrsLanguage.Parse(text, null, Names(), p => ProgramValidator.Validate(p, new ValidationContext()));
        Assert.True(r.Ok);
        var v = Assert.Single(r.Problems, p => p.Source == "validate" && p.Code == ValidationCodes.UnknownVariable);
        Assert.Equal(2, v.Line);
        Assert.Equal(text.Split('\n')[1].IndexOf("$nope", StringComparison.Ordinal) + 1, v.Col);
        Assert.NotNull(v.StepId);
    }
}
