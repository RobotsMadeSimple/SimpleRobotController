using Controller.RobotControl;
using Controller.RobotControl.Language;
using static RobotControl.Tests.Language.SrsTestKit;

namespace RobotControl.Tests.Language;

/// <summary>Ids survive text edits (docs/program-text.md, "Ids"): matched steps keep their id,
/// their condition-item / branch / case ids, their shadowed literals and their label id.</summary>
public class ReconcilerTests
{
    private static BuiltProgram Base(string text)
    {
        var r = SrsLanguage.Parse(text, null, Names());
        Assert.True(r.Ok, string.Join("; ", r.Problems.Select(p => p.Message)));
        return r.Program!;
    }

    private static BuiltProgram Edit(BuiltProgram @base, string text)
    {
        var r = SrsLanguage.Parse(text, @base, Names());
        Assert.True(r.Ok, string.Join("; ", r.Problems.Select(p => p.Message)));
        return r.Program!;
    }

    private static List<string> Ids(BuiltProgram p) => p.Steps.Select(s => s.Id).ToList();

    [Fact]
    public void UnchangedTextKeepsEveryId()
    {
        var b = Base("MoveL Pick1\nrepeat 3\n  MoveL Place1\n  pause\nend\nstatus \"done\"");
        var e = Edit(b, SrsLanguage.Format(b, Names()).Text);
        Assert.Equal(StepTree.Walk(b.Steps).Select(s => s.Id), StepTree.Walk(e.Steps).Select(s => s.Id));
    }

    [Fact]
    public void InsertingAStepKeepsTheOthers()
    {
        var b = Base("MoveL Pick1\nMoveL Place1\npause");
        var e = Edit(b, "MoveL Pick1\nWait 100ms\nMoveL Place1\npause");
        Assert.Equal(b.Steps[0].Id, e.Steps[0].Id);
        Assert.Equal(b.Steps[1].Id, e.Steps[2].Id);
        Assert.Equal(b.Steps[2].Id, e.Steps[3].Id);
        Assert.DoesNotContain(e.Steps[1].Id, Ids(b));
        Assert.True(Guid.TryParse(e.Steps[1].Id, out _));
    }

    [Fact]
    public void DeletingAStepKeepsTheOthers()
    {
        var b = Base("MoveL Pick1\nMoveL Place1\npause");
        var e = Edit(b, "MoveL Pick1\npause");
        Assert.Equal([b.Steps[0].Id, b.Steps[2].Id], Ids(e));
    }

    [Fact]
    public void EditingAFieldKeepsTheId()
    {
        var b = Base("MoveL Pick1, speed=100\nMoveL Place1");
        var e = Edit(b, "MoveL Pick1, speed=250, blend=on\nMoveL Place1");
        Assert.Equal(Ids(b), Ids(e));
    }

    [Fact]
    public void RetargetingAMoveKeepsItsId()
    {
        var b = Base("pause\nMoveL Pick1\nstatus \"x\"");
        var e = Edit(b, "pause\nMoveL Place1\nstatus \"x\"");
        Assert.Equal(Ids(b), Ids(e)); // paired by type between the anchors
    }

    [Fact]
    public void AMovedStepKeepsItsId()
    {
        var b = Base("MoveL Pick1\nMoveL Place1\nWait 1s\nstatus \"x\"");
        var e = Edit(b, "MoveL Place1\nWait 1s\nstatus \"x\"\nMoveL Pick1");
        Assert.Equal(b.Steps[0].Id, e.Steps[3].Id);
        Assert.Equal(b.Steps[1].Id, e.Steps[0].Id);
    }

    [Fact]
    public void NestedBodiesReconcileByRole()
    {
        var b = Base("if $a\n  MoveL Pick1\nelif $b\n  MoveL Place1\nelse\n  pause\nend");
        var e = Edit(b, "if $a\n  MoveL Pick1\n  Wait 1s\nelif $b and $c\n  MoveL Place1\nelse\n  pause\nend");
        var bi = b.Steps[0]; var ei = e.Steps[0];
        Assert.Equal(bi.Id, ei.Id);
        Assert.Equal(bi.IfSteps![0].Id, ei.IfSteps![0].Id);
        Assert.Equal(bi.ElseIfBranches![0].Id, ei.ElseIfBranches![0].Id);
        Assert.Equal(bi.ElseIfBranches[0].Steps[0].Id, ei.ElseIfBranches[0].Steps[0].Id);
        Assert.Equal(bi.ElseSteps![0].Id, ei.ElseSteps![0].Id);
        Assert.Equal(bi.Condition!.Items[0].Id, ei.Condition!.Items[0].Id);
        Assert.Equal(bi.ElseIfBranches[0].Condition.Items[0].Id, ei.ElseIfBranches[0].Condition.Items[0].Id);
        Assert.Equal(2, ei.ElseIfBranches[0].Condition.Items.Count);
    }

    [Fact]
    public void SwitchCasesKeepTheirIds()
    {
        var b = Base("switch $n\ncase 1\n  pause\ncase 2\n  pause\nend");
        var e = Edit(b, "switch $n\ncase 1, 3\n  pause\ncase 2\n  return\nend");
        Assert.Equal(b.Steps[0].SwitchCases!.Select(c => c.Id), e.Steps[0].SwitchCases!.Select(c => c.Id));
    }

    [Fact]
    public void ExplicitIdsWin()
    {
        var b = Base("MoveL Pick1");
        var e = Edit(b, "@id \"mine\"\nMoveL Pick1\nMoveL Pick1");
        Assert.Equal("mine", e.Steps[0].Id);
        Assert.NotEqual("mine", e.Steps[1].Id);
    }

    [Fact]
    public void AnExplicitIdIsNeverInheritedByAnotherStep()
    {
        var b = Base("MoveL Pick1");
        var id = b.Steps[0].Id;
        var e = Edit(b, $"MoveL Pick1\n@id \"{id}\"\nMoveL Place1");
        Assert.Equal(id, e.Steps[1].Id);
        Assert.NotEqual(id, e.Steps[0].Id);
    }

    [Fact]
    public void ShadowedLiteralsAreInherited()
    {
        var b = Base("MoveL Pick1, speed=200");
        b.Steps[0].Expressions = new() { ["speed"] = "$v" }; // the literal 200 stays as the fallback
        var e = Edit(b, SrsLanguage.Format(b, Names()).Text);
        Assert.Equal(200, e.Steps[0].Speed);
        Assert.Equal("$v", e.Steps[0].Expressions!["speed"]);
        var fresh = SrsLanguage.Parse(SrsLanguage.Format(b, Names()).Text, null, Names()).Program!;
        Assert.Null(fresh.Steps[0].Speed); // no base: nothing to inherit
    }

    [Fact]
    public void LabelsKeepTheirLabelIdAndGotosFollowThem()
    {
        var b = Base("label top\npause\ngoto top");
        var lid = b.Steps[0].LabelId;
        Assert.NotNull(lid);
        Assert.Equal(lid, b.Steps[2].LabelId);
        var e = Edit(b, "label top\nWait 1s\npause\ngoto top");
        Assert.Equal(lid, e.Steps[0].LabelId);
        Assert.Equal(lid, e.Steps[3].LabelId);
    }

    [Fact]
    public void RenamingALabelKeepsItsLinks()
    {
        var b = Base("label top\npause\ngoto top");
        var e = Edit(b, "label start\npause\ngoto start");
        Assert.Equal(b.Steps[0].LabelId, e.Steps[0].LabelId);
        Assert.Equal(b.Steps[0].LabelId, e.Steps[2].LabelId);
    }

    [Fact]
    public void VariablesMatchByName()
    {
        var b = Base("var a = 1\nvar b = $robot.z");
        b.Variables![1].Value = 42; // last known value
        var e = Edit(b, "var b = $robot.z\nvar c = 3\nvar a = 2");
        Assert.Equal(b.Variables[1].Id, e.Variables![0].Id);
        Assert.Equal(42, e.Variables[0].Value);
        Assert.Equal(b.Variables[0].Id, e.Variables[2].Id);
        Assert.DoesNotContain(e.Variables[1].Id, b.Variables.Select(v => v.Id));
    }

    [Fact]
    public void BaseMetadataIsKept()
    {
        var b = Base("pause");
        b.Id = "prog-9"; b.Name = "Keep me"; b.IsRoutine = true; b.EditorMode = "text"; b.DisplayPageHtml = "<p>x</p>";
        var e = Edit(b, "return");
        Assert.Equal(("prog-9", "Keep me", true, "text", "<p>x</p>"), (e.Id, e.Name, e.IsRoutine, e.EditorMode, e.DisplayPageHtml));
    }

    [Fact]
    public void TheProgramHeaderIsReadWithoutABase()
    {
        var p = new BuiltProgram { Id = "x", Name = "Exported", IsBackground = true, Steps = [new ProgramStep { Id = "s", Type = StepType.PauseProgram }] };
        var text = SrsLanguage.Format(p, Names(), includeHeader: true).Text;
        Assert.Contains("@program {", text);
        var r = SrsLanguage.Parse(text, null, Names());
        Assert.True(r.Ok);
        Assert.Equal(("Exported", true, ""), (r.Program!.Name, r.Program.IsBackground, r.Program.Id));
    }
}
