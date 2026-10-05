using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

public class StepExecutionStructureTests
{
    [Fact]
    public void EveryStepType_HasAHandler()
    {
        var missing = Enum.GetValues<StepType>().Where(t => !StepHandlers.TryGet(t, out _)).ToList();
        Assert.True(missing.Count == 0, "No handler for: " + string.Join(", ", missing));
    }

    [Fact]
    public void FrameStack_TracksLoopDepthThroughPushAndPop()
    {
        var steps = new List<ProgramStep>();
        var frames = new FrameStack();
        frames.Push(StepListFrame.Plain(steps));
        Assert.Equal(0, frames.LoopDepth);

        frames.Push(StepListFrame.CountLoop(steps, 3, 3, ""));
        frames.Push(StepListFrame.Plain(steps));          // an if-branch inside the loop
        frames.Push(StepListFrame.WhileLoop(steps, new ConditionGroup()));
        Assert.Equal(2, frames.LoopDepth);

        frames.Pop();
        Assert.Equal(1, frames.LoopDepth);
        frames.Pop();
        Assert.Equal(1, frames.LoopDepth);
        frames.Pop();
        Assert.Equal(0, frames.LoopDepth);

        frames.Push(StepListFrame.ForEach(steps, 2, 0, "list", "", ""));
        frames.Clear();
        Assert.Equal(0, frames.LoopDepth);
        Assert.Equal(0, frames.Count);
    }

    [Fact]
    public void Frames_OnlyLoopKindsAreLoops()
    {
        var steps = new List<ProgramStep>();
        Assert.False(StepListFrame.Plain(steps).IsLoop);
        Assert.False(StepListFrame.Cnc(steps).IsLoop);
        Assert.True(StepListFrame.CountLoop(steps, 1, 1, "").IsLoop);
        Assert.True(StepListFrame.WhileLoop(steps, new ConditionGroup()).IsLoop);
        Assert.True(StepListFrame.ForEach(steps, 1, 0, "l", "", "").IsLoop);
    }

    [Fact]
    public void NextPass_CopiesTheLoopSettingsAndRestartsAtTheTop()
    {
        var steps = new List<ProgramStep> { new() { Type = StepType.Label } };

        var count = StepListFrame.CountLoop(steps, 5, 5, "i");
        count.Index = 1;
        count.LoopRemaining--;
        var nextCount = count.NextPass();
        Assert.Equal(FrameKind.CountLoop, nextCount.Kind);
        Assert.Same(steps, nextCount.Steps);
        Assert.Equal(0, nextCount.Index);
        Assert.Equal(4, nextCount.LoopRemaining);
        Assert.Equal(5, nextCount.LoopTotal);
        Assert.Equal("i", nextCount.IndexVar);

        var each = StepListFrame.ForEach(steps, 3, 0, "src", "v", "i");
        each.ForEachCurrentIndex++;
        var nextEach = each.NextPass();
        Assert.Equal(1, nextEach.ForEachCurrentIndex);
        Assert.Equal(3, nextEach.ForEachCount);
        Assert.Equal("src", nextEach.ForEachSourceVar);
        Assert.Equal("v", nextEach.ForEachValueVar);

        var cond = new ConditionGroup();
        Assert.Same(cond, StepListFrame.WhileLoop(steps, cond).NextPass().WhileCondition);

        Assert.Throws<InvalidOperationException>(() => StepListFrame.Plain(steps).NextPass());
    }

    [Fact]
    public void StepDescription_UsesNameAndTarget()
    {
        Assert.Equal("MoveL → P1", ProgressReporter.StepDescription(new ProgramStep { Type = StepType.MoveL, PointName = "P1" }));
        Assert.Equal("Pick  (MoveJ → current position)",
            ProgressReporter.StepDescription(new ProgramStep { Type = StepType.MoveJ, Name = "Pick" }));
    }
}
