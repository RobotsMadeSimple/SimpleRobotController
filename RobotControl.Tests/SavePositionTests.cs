using Controller.RobotControl;

namespace RobotControl.Tests;

/// <summary>
/// SavePosition ("Here", docs/program-language.md §4.3): the TCP pose into a Points list —
/// replace / append / insert / set, the index as a literal or an expression, out-of-range
/// indexes — or into a saved point (by id, by name, created), with the points history
/// recording it; the actual vs commanded source; and the active local frame.
/// </summary>
[Collection("RobotController")]
public sealed class SavePositionTests : IDisposable
{
    private readonly StepsRig _rig = new();
    public void Dispose() => _rig.Dispose();

    private Vector6 Here => _rig.Robot.GetCurrentPosition();

    private static ProgramStep Save(string mode = "replace", int? index = null, Action<ProgramStep>? init = null) =>
        StepsRig.Step(StepType.SavePosition, s =>
        {
            s.VariableName = "pts"; s.SavePositionMode = mode; s.SavePositionIndex = index;
            init?.Invoke(s);
        });

    private static ProgramVariable Pts(int count) =>
        StepsRig.PointList("pts", Enumerable.Range(0, count).Select(i => StepsRig.P(i, i, i)).ToArray());

    private void AssertIsHere(int index)
    {
        var h = Here;
        Assert.Equal(h.X,  _rig.Live($"$pts[{index}].x"), 9);
        Assert.Equal(h.Y,  _rig.Live($"$pts[{index}].y"), 9);
        Assert.Equal(h.Z,  _rig.Live($"$pts[{index}].z"), 9);
        Assert.Equal(h.RZ, _rig.Live($"$pts[{index}].rz"), 9);
    }

    // ── Points list ───────────────────────────────────────────────────────────

    [Fact]
    public void ReplaceLeavesExactlyThePose()
    {
        _rig.Run(StepsRig.Prog([Pts(3)], Save()));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Count("pts"));
        AssertIsHere(0);
    }

    [Fact]
    public void DefaultModeIsReplace()
    {
        _rig.Run(StepsRig.Prog([Pts(2)], Save(init: s => s.SavePositionMode = null)));
        Assert.Equal(1, _rig.Count("pts"));
    }

    [Fact]
    public void AppendAddsAtTheEnd()
    {
        _rig.Run(StepsRig.Prog([Pts(2)], Save("append"), Save("append")));
        _rig.AssertComplete();
        Assert.Equal(4, _rig.Count("pts"));
        Assert.Equal(1, _rig.Live("$pts[1].x"));
        AssertIsHere(2);
        AssertIsHere(3);
    }

    [Fact]
    public void InsertAtTheFrontAndAtTheEnd()
    {
        _rig.Run(StepsRig.Prog([Pts(2)], Save("insert", 0), Save("insert", 3)));
        _rig.AssertComplete();
        Assert.Equal(4, _rig.Count("pts"));
        AssertIsHere(0);
        Assert.Equal(0, _rig.Live("$pts[1].x"));
        Assert.Equal(1, _rig.Live("$pts[2].x"));
        AssertIsHere(3);
    }

    [Fact]
    public void SetReplacesOneElement()
    {
        _rig.Run(StepsRig.Prog([Pts(3)], Save("set", 1)));
        _rig.AssertComplete();
        Assert.Equal(3, _rig.Count("pts"));
        Assert.Equal(0, _rig.Live("$pts[0].x"));
        AssertIsHere(1);
        Assert.Equal(2, _rig.Live("$pts[2].x"));
    }

    [Fact]
    public void IndexMayBeAnExpression()
    {
        var step = Save("set", 0, s => s.Expressions = new() { ["savePositionIndex"] = "$i + 1" });
        _rig.Run(StepsRig.Prog([Pts(3), StepsRig.Num("i", 1)], step));
        _rig.AssertComplete();
        AssertIsHere(2);
        Assert.Equal(1, _rig.Live("$pts[1].x"));
    }

    [Theory]
    [InlineData("insert", 3)]
    [InlineData("insert", -1)]
    [InlineData("set", 2)]
    [InlineData("set", -1)]
    public void OutOfRangeIndexFails(string mode, int index)
    {
        _rig.Run(StepsRig.Prog([Pts(2)], Save(mode, index), StepsRig.Set("after", "1")));
        _rig.AssertError($"index {index} is outside '$pts'");
        Assert.Equal(2, _rig.Count("pts")); // unchanged
    }

    [Fact]
    public void SetOnAnEmptyListFails()
    {
        _rig.Run(StepsRig.Prog([Pts(0)], Save("set", 0)));
        _rig.AssertError("index 0 is outside");
    }

    [Fact]
    public void AnUndeclaredVariableBecomesAPointsList()
    {
        _rig.Run(StepsRig.Prog(Save("append", init: s => s.VariableName = "$taught")));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Count("taught"));
        Assert.Equal(Here.X, _rig.Live("$taught[0].x"), 9);
    }

    [Fact]
    public void ANumberListIsRefused()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Numbers("pts", 1, 2)], Save("append")));
        _rig.AssertError("not a Points list");
    }

    [Fact]
    public void UnknownModeIsRefused()
    {
        _rig.Run(StepsRig.Prog([Pts(1)], Save("prepend")));
        _rig.AssertError("unknown mode 'prepend'");
    }

    [Fact]
    public void NoVariableEndsTheRun()
    {
        _rig.Run(StepsRig.Prog(Save(init: s => s.VariableName = " ")));
        _rig.AssertError("no Points variable");
    }

    [Fact]
    public void TheProgramDeclarationIsNotChanged()
    {
        var program = StepsRig.Prog([Pts(2)], Save("append"));
        _rig.Run(program);
        Assert.Equal(2, program.Variables[0].Items!.Count);
        _rig.Run(program); // a second run starts from the declared two again
        Assert.Equal(3, _rig.Count("pts"));
    }

    // ── Source and frame ──────────────────────────────────────────────────────

    [Fact]
    public void TargetSourceSavesTheCommandedTarget()
    {
        _rig.Run(StepsRig.Prog([Pts(0)], Save(init: s => s.SavePositionSource = "target")));
        _rig.AssertComplete();
        var t = _rig.Robot.LiveTargetPosition;
        Assert.Equal(t.X, _rig.Live("$pts[0].x"), 9);
        Assert.Equal(t.Z, _rig.Live("$pts[0].z"), 9);
    }

    [Fact]
    public void ThePoseIsInTheActiveLocalFrame()
    {
        var frame = _rig.Locals.SaveItem("Fixture", new Vector6(100, -50, 10, 0, 0, 90));
        var setLocal = StepsRig.Step(StepType.SetLocal, s => { s.LocalId = frame.Id; s.LocalName = "Fixture"; });
        _rig.Run(StepsRig.Prog([Pts(0)], setLocal, Save()));
        _rig.AssertComplete();

        var expected = LocalFrame.Inverse(new Vector6(100, -50, 10, 0, 0, 90), Here);
        Assert.Equal(expected.X,  _rig.Live("$pts[0].x"), 9);
        Assert.Equal(expected.Y,  _rig.Live("$pts[0].y"), 9);
        Assert.Equal(expected.Z,  _rig.Live("$pts[0].z"), 9);
        Assert.Equal(expected.RZ, _rig.Live("$pts[0].rz"), 9);
        // …so moving to it under the same local lands back where the robot is.
        var world = LocalFrame.Apply(new Vector6(100, -50, 10, 0, 0, 90), expected);
        Assert.Equal(Here.X, world.X, 9);
        Assert.Equal(Here.Y, world.Y, 9);
    }

    // ── Saved point ───────────────────────────────────────────────────────────

    private static ProgramStep ToPoint(string? id, string? name) =>
        StepsRig.Step(StepType.SavePosition, s => { s.SavePositionTo = "point"; s.PointId = id; s.PointName = name; });

    [Fact]
    public void OverwritesASavedPointByIdKeepingItsIdAndRecordingHistory()
    {
        var p = _rig.Points.SavePoint("Pick", new Vector6(1, 2, 3, 0, 0, 0));
        // Linked by id; the stored name is stale (the point was renamed) — the id wins.
        _rig.Run(StepsRig.Prog(ToPoint(p.Id, "OldName")));
        _rig.AssertComplete();

        var now = _rig.Points.Get("Pick")!;
        Assert.Equal(p.Id, now.Id);
        Assert.Equal(Here.X, now.X, 9);
        Assert.Equal(Here.Z, now.Z, 9);
        Assert.Null(_rig.Points.Get("OldName"));
        var history = File.ReadAllText(Path.Combine(_rig.Tmp, "pointsHistory.json"));
        Assert.Contains("\"Pick\"", history);
        Assert.Equal(2, System.Text.Json.JsonDocument.Parse(history).RootElement.GetProperty("Pick").GetArrayLength());
    }

    [Fact]
    public void OverwritesBySavedNameWhenThereIsNoId()
    {
        _rig.Points.SavePoint("Place", new Vector6(9, 9, 9, 0, 0, 0));
        _rig.Run(StepsRig.Prog(ToPoint(null, "Place")));
        _rig.AssertComplete();
        Assert.Equal(Here.Y, _rig.Points.Get("Place")!.Y, 9);
    }

    [Fact]
    public void CreatesAPointThatDoesNotExistYet()
    {
        _rig.Run(StepsRig.Prog(ToPoint(null, " Taught 1 ")));
        _rig.AssertComplete();
        var p = _rig.Points.Get("Taught 1");
        Assert.NotNull(p);
        Assert.False(string.IsNullOrEmpty(p!.Id));
        Assert.Equal(Here.X, p.X, 9);
    }

    [Fact]
    public void ADeletedPointLinkedOnlyByIdFails()
    {
        var p = _rig.Points.SavePoint("Gone", new Vector6(1, 1, 1, 0, 0, 0));
        _rig.Points.DeletePoint("Gone");
        _rig.Run(StepsRig.Prog(ToPoint(p.Id, null)));
        _rig.AssertError("no longer exists");
    }

    [Fact]
    public void NoPointSelectedEndsTheRun()
    {
        _rig.Run(StepsRig.Prog(ToPoint(null, "")));
        _rig.AssertError("no saved point");
    }
}
