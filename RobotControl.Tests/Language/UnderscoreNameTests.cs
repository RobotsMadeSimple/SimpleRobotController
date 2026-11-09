using Controller.RobotControl;
using Controller.RobotControl.Language;
using static RobotControl.Tests.Language.SrsTestKit;

namespace RobotControl.Tests.Language;

/// <summary>
/// Names with spaces written as identifiers in their underscore form (<c>"Pick 1"</c> â†’
/// <c>Pick_1</c>): resolution, ambiguity, keyword clashes, printing and round trips for every
/// reference kind (docs/program-text.md, "Names").
/// </summary>
public class UnderscoreNameTests
{
    /// <summary>One spaced name per kind, plus a vision tool and a label in the program.</summary>
    private static SrsNameTable Spaced()
    {
        var t = new SrsNameTable();
        t.Add(SrsRefKind.Point, "pt-1", "Pick 1").Add(SrsRefKind.Point, "pt-2", "Place - left")
         .Add(SrsRefKind.Tool, "tool-1", "Vacuum Cup")
         .Add(SrsRefKind.Local, "local-1", "Fixture 2")
         .Add(SrsRefKind.Grid, "grid-1", "Big Tray")
         .Add(SrsRefKind.Stack, "stack-1", "Bin Stack")
         .Add(SrsRefKind.Routine, "routine-1", "Pick Part")
         .Add(SrsRefKind.Program, "bg-1", "Conveyor Loop")
         .Add(SrsRefKind.Vision, "vp-1", "Detect Parts")
         .Add(SrsRefKind.Device, "nano-1", "Arduino Nano")
         .Add(SrsRefKind.Camera, "cam-1", "Side Cam");
        t.AddTool("vp-1", "t1", "Top edge");
        return t;
    }

    private const string AllKindsUnderscore = """
#!srs 1
var a = 0
label "Try again"
SetTool Vacuum_Cup
SetLocal Fixture_2
MoveL Pick_1
MoveJ place_left
JumpJ grid Big_Tray[1, 2], z=10
JumpL stack Bin_Stack[0], z=10
SavePosition point Pick_1
call Pick_Part
StartBackground Conveyor_Loop
RunVision Detect_Parts
  Top_edge.angle -> $a
end
CaptureImage $a, camera=Side_Cam
Tare device=Arduino_Nano, pin=2
goto Try_again
""";

    private static ParseProgramTextResult Parse(string text, ISrsNames? names = null) =>
        SrsLanguage.Parse(text, null, names ?? Spaced());

    private static string Problems(ParseProgramTextResult r) =>
        string.Join("; ", r.Problems.Select(p => $"{p.Line}:{p.Col} {p.Severity} {p.Code} {p.Message}"));

    // â”€â”€ Resolution â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    [Fact]
    public void EveryKindResolvesItsUnderscoreForm()
    {
        var r = Parse(AllKindsUnderscore);
        Assert.True(r.Ok, Problems(r));
        Assert.DoesNotContain(r.Problems, p => p.Severity == "warning" && p.Code.StartsWith("unknown"));
        var s = StepTree.Walk(r.Program!.Steps).ToList();
        ProgramStep Of(StepType t) => s.First(x => x.Type == t);

        Assert.Equal(("tool-1", "Vacuum Cup"), (Of(StepType.SetTool).ToolId, Of(StepType.SetTool).ToolName));
        Assert.Equal(("local-1", "Fixture 2"), (Of(StepType.SetLocal).LocalId, Of(StepType.SetLocal).LocalName));
        Assert.Equal(("pt-1", "Pick 1"), (Of(StepType.MoveL).PointId, Of(StepType.MoveL).PointName));
        Assert.Equal(("pt-2", "Place - left"), (Of(StepType.MoveJ).PointId, Of(StepType.MoveJ).PointName));
        Assert.Equal("grid-1", Of(StepType.JumpJ).GridPoint!.GridId);
        Assert.Equal("stack-1", Of(StepType.JumpL).StackPoint!.StackId);
        Assert.Equal("pt-1", Of(StepType.SavePosition).PointId);
        Assert.Equal(("routine-1", "Pick Part"), (Of(StepType.CallRoutine).RoutineId, Of(StepType.CallRoutine).RoutineName));
        Assert.Equal("bg-1", Of(StepType.StartBackground).BackgroundProgramId);
        var rv = Of(StepType.RunVision);
        Assert.Equal("vp-1", rv.VisionProgramId);
        Assert.Equal("t1", rv.ToolOutputs![0].ToolId);
        Assert.Equal("cam-1", Of(StepType.CaptureImage).CaptureImageCameraId);
        var go = Of(StepType.GoToLabel);
        Assert.Equal("Try again", go.LabelName);
        Assert.Equal(s.First(x => x.Type == StepType.Label).LabelId, go.LabelId);
    }

    [Fact]
    public void TheUnderscoreFormIsCaseInsensitive()
    {
        var r = Parse("#!srs 1\nMoveL PICK_1\nSetTool vacuum_cup");
        Assert.True(r.Ok, Problems(r));
        Assert.Equal("pt-1", r.Program!.Steps[0].PointId);
        Assert.Equal("tool-1", r.Program!.Steps[1].ToolId);
    }

    [Fact]
    public void AnExactNameWinsOverTheUnderscoreForm()
    {
        var names = new SrsNameTable().Add(SrsRefKind.Point, "pt-space", "Pick 1").Add(SrsRefKind.Point, "pt-under", "Pick_1");
        var r = Parse("#!srs 1\nMoveL Pick_1\nMoveL \"Pick 1\"", names);
        Assert.True(r.Ok, Problems(r));
        Assert.Equal("pt-under", r.Program!.Steps[0].PointId);
        Assert.Equal("pt-space", r.Program!.Steps[1].PointId);

        // â€¦so the printer quotes "Pick 1" (its underscore form names the other point).
        var text = SrsLanguage.Format(r.Program!, names).Text;
        Assert.Contains("MoveL Pick_1\n", text);
        Assert.Contains("MoveL \"Pick 1\"\n", text);
        AssertRoundTrip(r.Program!, names);
    }

    [Fact]
    public void StoredJsonNamesDoNotUseTheUnderscoreForm()
    {
        // Only the text resolves underscore forms; a stored pointName is matched as written.
        var p = Program(new ProgramStep { Id = "s1", Type = StepType.MoveL, PointName = "Pick_1" });
        var n = SrsLanguage.NormalizeJson(p, Spaced());
        Assert.Null(n["steps"]![0]!["pointId"]);
    }

    // â”€â”€ Ambiguity â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    [Fact]
    public void AnAmbiguousUnderscoreFormIsAnErrorThatSaysToQuote()
    {
        var names = new SrsNameTable().Add(SrsRefKind.Point, "a", "Pick 1").Add(SrsRefKind.Point, "b", "Pick-1");
        var text = "#!srs 1\nMoveL Pick_1";
        var r = Parse(text, names);
        Assert.False(r.Ok);
        var p = Assert.Single(r.Problems, x => x.Code == TextProblemCodes.AmbiguousName);
        Assert.Equal("error", p.Severity);
        Assert.Equal((2, 7, 13), (p.Line, p.Col, p.EndCol));
        Assert.Contains("\"Pick 1\"", p.Message);
        Assert.Contains("\"Pick-1\"", p.Message);
        Assert.Contains("quotes", p.Message);

        // The quoted names still work, and print quoted.
        var q = Parse("#!srs 1\nMoveL \"Pick 1\"\nMoveL \"Pick-1\"", names);
        Assert.True(q.Ok, Problems(q));
        Assert.Equal(["a", "b"], q.Program!.Steps.Select(s => s.PointId));
        var f = AssertRoundTrip(q.Program!, names);
        Assert.Contains("MoveL \"Pick 1\"", f);
        Assert.Contains("MoveL \"Pick-1\"", f);
    }

    [Theory]
    [InlineData("SetTool Cup_A", SrsRefKind.Tool)]
    [InlineData("SetLocal Cup_A", SrsRefKind.Local)]
    [InlineData("call Cup_A", SrsRefKind.Routine)]
    [InlineData("StartBackground Cup_A", SrsRefKind.Program)]
    [InlineData("RunVision Cup_A", SrsRefKind.Vision)]
    [InlineData("CaptureImage $x, camera=Cup_A", SrsRefKind.Camera)]
    [InlineData("Tare device=Cup_A, pin=1", SrsRefKind.Device)]
    [InlineData("JumpJ grid Cup_A[1], z=1", SrsRefKind.Grid)]
    [InlineData("JumpJ stack Cup_A[1], z=1", SrsRefKind.Stack)]
    public void AmbiguityIsReportedForEveryKind(string stmt, SrsRefKind kind)
    {
        var names = new SrsNameTable().Add(kind, "x1", "Cup A").Add(kind, "x2", "Cup/A");
        var r = Parse("#!srs 1\nvar x: image\n" + stmt, names);
        var p = Assert.Single(r.Problems, x => x.Code == TextProblemCodes.AmbiguousName);
        Assert.Equal(3, p.Line);
        Assert.Equal(stmt.IndexOf("Cup_A", StringComparison.Ordinal) + 1, p.Col);
    }

    [Fact]
    public void AmbiguousLabelsAndVisionTools()
    {
        var names = new SrsNameTable().Add(SrsRefKind.Vision, "vp", "V");
        names.AddTool("vp", "t1", "Top edge").AddTool("vp", "t2", "Top-edge");
        var r = Parse("#!srs 1\nvar a = 0\nlabel \"go on\"\nlabel \"go-on\"\ngoto go_on\nRunVision V\n  Top_edge.angle -> $a\nend", names);
        var codes = r.Problems.Where(p => p.Code == TextProblemCodes.AmbiguousName).Select(p => p.Line).OrderBy(x => x).ToList();
        Assert.Equal([5, 7], codes);
    }

    // â”€â”€ Printing â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    [Fact]
    public void ThePrinterWritesTheUnderscoreFormForEveryKind()
    {
        // Parse the quoted spelling, print: every reference comes out in its underscore form.
        var quoted = """
#!srs 1
var a = 0
label "Try again"
SetTool "Vacuum Cup"
SetLocal "Fixture 2"
MoveL "Pick 1"
MoveJ "Place - left"
JumpJ grid "Big Tray"[1, 2], z=10
JumpL stack "Bin Stack"[0], z=10
call "Pick Part"
StartBackground "Conveyor Loop"
RunVision "Detect Parts"
  "Top edge".angle -> $a
end
CaptureImage $a, camera="Side Cam"
Tare device="Arduino Nano", pin=2
goto "Try again"
""";
        var r = Parse(quoted);
        Assert.True(r.Ok, Problems(r));
        var text = AssertRoundTrip(r.Program!, Spaced());
        foreach (var t in new[]
                 {
                     "label \"Try again\"", "SetTool Vacuum_Cup", "SetLocal Fixture_2", "MoveL Pick_1", "MoveJ Place_left",
                     "grid Big_Tray[1, 2]", "stack Bin_Stack[0]", "call Pick_Part", "StartBackground Conveyor_Loop",
                     "RunVision Detect_Parts", "Top_edge.angle -> $a", "camera=Side_Cam", "device=Arduino_Nano", "goto Try_again",
                 })
            Assert.Contains(t, text);
    }

    [Fact]
    public void TheUnderscoreTextRoundTripsAndIsCanonical()
    {
        var r = Parse(AllKindsUnderscore);
        Assert.True(r.Ok, Problems(r));
        var text = AssertRoundTrip(r.Program!, Spaced());
        Assert.Contains("MoveJ Place_left", text); // the canonical underscore spelling, not what was typed
    }

    [Fact]
    public void AnUnknownSpacedNameStaysQuoted()
    {
        // Not in the repository: its underscore form would not read back as the same name.
        var p = Program(new ProgramStep { Id = "s1", Type = StepType.MoveL, PointName = "Ghost Point" });
        var text = AssertRoundTrip(p, Spaced());
        Assert.Contains("MoveL \"Ghost Point\"", text);
    }

    [Fact]
    public void ADanglingIdKeepsTheQuotedName()
    {
        // The id is missing from the repository; Pick_1 would relink it to pt-1.
        var p = Program(new ProgramStep { Id = "s1", Type = StepType.MoveL, PointId = "gone", PointName = "Pick 1" });
        var text = AssertRoundTrip(p, Spaced());
        Assert.DoesNotContain("MoveL Pick_1", text);
    }

    [Fact]
    public void AnUnderscoreFormThatIsAReservedWordOrNoIdentifierIsQuoted()
    {
        var names = new SrsNameTable().Add(SrsRefKind.Point, "p1", " ").Add(SrsRefKind.Point, "p2", "1st pick")
                                      .Add(SrsRefKind.Point, "p3", "CafÃ©");
        var p = Program(
            new ProgramStep { Id = "s1", Type = StepType.MoveL, PointId = "p1" },
            new ProgramStep { Id = "s2", Type = StepType.MoveL, PointId = "p2" },
            new ProgramStep { Id = "s3", Type = StepType.MoveL, PointId = "p3" });
        var text = AssertRoundTrip(p, names);
        Assert.Contains("MoveL \" \"", text);        // "_" is reserved
        Assert.Contains("MoveL \"1st pick\"", text); // 1st_pick is not an identifier
        Assert.Contains("MoveL Caf_", text);         // Ã© is not an identifier character
    }

    [Theory]
    [InlineData("MoveL")]
    [InlineData("end")]
    [InlineData("here")]
    [InlineData("var")]
    [InlineData("on")]
    [InlineData("elif")]
    [InlineData("call")]
    public void KeywordsAreRecognisedCaseInsensitively(string keyword)
    {
        Assert.True(SrsText.IsReservedOrKeyword(keyword));
        Assert.True(SrsText.IsReservedOrKeyword(keyword.ToUpperInvariant()));
        Assert.False(SrsText.IsReservedOrKeyword(keyword + "_x"));
    }

    [Fact]
    public void AnUnderscoreFormThatIsAKeywordIsQuoted()
    {
        // "-" and " " both become "_", which is reserved (the for-loop's ignore name).
        Assert.Equal("\"-\"", SrsNameMatch.Text("-", _ => true));
        Assert.Equal("\" \"", SrsNameMatch.Text(" ", _ => true));
    }

    // â”€â”€ The helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    [Theory]
    [InlineData("Pick 1", "Pick_1")]
    [InlineData("Bin - left", "Bin_left")]
    [InlineData("  padded  ", "_padded_")]
    [InlineData("a_b c", "a_b_c")]
    [InlineData("tab\there", "tab_here")]
    [InlineData("Pick1", "Pick1")]
    public void UnderscoreReplacesEachRunOfNonIdentifierCharacters(string name, string want) =>
        Assert.Equal(want, SrsNameMatch.Underscore(name));

    [Fact]
    public void TextChoosesBareUnderscoreOrQuoted()
    {
        Assert.Equal("Pick1", SrsNameMatch.Text("Pick1", _ => false));
        Assert.Equal("Pick_1", SrsNameMatch.Text("Pick 1", u => u == "Pick_1"));
        Assert.Equal("\"Pick 1\"", SrsNameMatch.Text("Pick 1", _ => false));
        Assert.Equal("\"end\"", SrsNameMatch.Text("end", _ => true));
        Assert.Null(SrsNameMatch.Text("", _ => true));
    }

    [Fact]
    public void FindPrefersExactThenCaseThenUnderscore()
    {
        var list = new List<SrsNamed> { new("1", "Pick 1"), new("2", "pick 1"), new("3", "Pick-2"), new("4", "Pick 2") };
        Assert.Equal("1", SrsNameMatch.Find(list, "Pick 1", true).Item?.Id);
        Assert.Equal("2", SrsNameMatch.Find(list, "pick 1", true).Item?.Id);
        Assert.Equal("1", SrsNameMatch.Find(list, "Pick_1", true).Item?.Id);  // case-sensitive underscore first
        Assert.Equal("2", SrsNameMatch.Find(list, "pick_1", true).Item?.Id);
        var amb = SrsNameMatch.Find(list, "Pick_2", true);
        Assert.True(amb.Ambiguous);
        Assert.Equal(["3", "4"], amb.Candidates.Select(c => c.Id));
        Assert.Null(SrsNameMatch.Find(list, "Nope", true).Item);
        Assert.False(SrsNameMatch.Find(list, "Nope", true).Ambiguous);
    }
}
