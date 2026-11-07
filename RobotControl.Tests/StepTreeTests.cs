using Controller.RobotControl;

namespace RobotControl.Tests;

/// <summary>StepTree: the one walker over nested step lists (docs/program-language.md §4.1 #4).</summary>
public class StepTreeTests
{
    private static ProgramStep W(string id) => new() { Id = id, Type = StepType.Wait, WaitMs = 1 };

    /// <summary>A step carrying every kind of body, each holding one marked step.</summary>
    private static ProgramStep Everything() => new()
    {
        Id = "all", Type = StepType.IfCondition,
        LoopSteps = [W("loop")],
        IfSteps = [W("if")],
        ElseIfBranches = [new() { Id = "b0", Steps = [W("elif0")] }, new() { Id = "b1", Steps = [W("elif1")] }],
        ElseSteps = [W("else")],
        SwitchCases = [new() { Id = "c0", Values = ["1"], Steps = [W("case0")] }],
        SwitchDefault = [W("default")],
        TrySteps = [W("try")],
        CatchSteps = [W("catch")],
        CncProgramSteps = [W("cnc")],
    };

    [Fact]
    public void ChildLists_YieldsEveryBodyInRoleOrder()
    {
        var lists = Everything().ChildLists().ToList();
        Assert.Equal(
            ["loopSteps", "ifSteps", "elseIfBranches[0].steps", "elseIfBranches[1].steps", "elseSteps",
             "switchCases[0].steps", "switchDefault", "trySteps", "catchSteps", "cncProgramSteps"],
            lists.Select(l => l.Path).ToList());
        Assert.Equal(["loop", "if", "elif0", "elif1", "else", "case0", "default", "try", "catch", "cnc"],
            lists.Select(l => l.Steps[0].Id).ToList());
        Assert.True(lists[0].IsLoopBody);
        Assert.All(lists.Skip(1), l => Assert.False(l.IsLoopBody));
    }

    [Fact]
    public void RolesAll_MatchesTheOrderOfChildLists()
    {
        var roles = Everything().ChildLists().Select(l => l.Role).Distinct().ToList();
        Assert.Equal(StepTree.Roles.All, roles);
    }

    [Fact]
    public void ChildLists_SkipsNullListsAndNullBranches()
    {
        var s = new ProgramStep { Type = StepType.Switch, SwitchCases = [null!, new() { Steps = [W("x")] }] };
        var lists = s.ChildLists().ToList();
        Assert.Single(lists);
        Assert.Equal("switchCases[1].steps", lists[0].Path);
        Assert.Empty(new ProgramStep { Type = StepType.Wait }.ChildLists());
    }

    [Fact]
    public void ExecutableOnly_DropsBakedCncStepsWhenTheSpecWins()
    {
        var s = new ProgramStep { Type = StepType.CncProgram, CncProgramSteps = [W("baked")] };
        Assert.Single(s.ChildLists(executableOnly: true));
        s.CncSpec = new CncSpec();
        Assert.Empty(s.ChildLists(executableOnly: true));
        Assert.Single(s.ChildLists());
    }

    [Fact]
    public void Walk_IsPreOrderAndCanSkipDisabledSubtrees()
    {
        var root = Everything();
        var steps = new List<ProgramStep> { W("first"), root, W("last") };
        Assert.Equal(13, StepTree.Walk(steps).Count());
        Assert.Equal("first", StepTree.Walk(steps).First().Id);
        Assert.Equal("all", StepTree.Walk(steps).ElementAt(1).Id);
        Assert.Equal("last", StepTree.Walk(steps).Last().Id);

        root.Enabled = false;
        Assert.Equal(["first", "last"], StepTree.Walk(steps, enabledOnly: true).Select(s => s.Id).ToList());
    }

    [Fact]
    public void WalkWithPaths_BuildsValidatorStepPaths()
    {
        var steps = new List<ProgramStep> { W("a"), Everything() };
        var paths = StepTree.WalkWithPaths(steps).ToDictionary(x => x.Step.Id, x => x.Path);
        Assert.Equal("steps[0]", paths["a"]);
        Assert.Equal("steps[1]", paths["all"]);
        Assert.Equal("steps[1].switchCases[0].steps[0]", paths["case0"]);
        Assert.Equal("steps[1].elseIfBranches[1].steps[0]", paths["elif1"]);
        Assert.Equal("steps[1].trySteps[0]", paths["try"]);
    }
}
