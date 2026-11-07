using System.Collections.Generic;

namespace Controller.RobotControl;

/// <summary>
/// One nested step list of a step, with the role (JSON property) it sits under. Branches
/// (<c>elseIfBranches</c>, <c>switchCases</c>) carry their index; <see cref="Path"/> is the
/// stepPath segment the validator and the text language use: <c>loopSteps</c>,
/// <c>elseIfBranches[1].steps</c>, <c>switchCases[0].steps</c>.
/// </summary>
public readonly record struct StepChildList(string Role, int? Index, List<ProgramStep> Steps)
{
    public string Path => Index is int i ? $"{Role}[{i}].steps" : Role;

    /// <summary>A loop body: its steps run once per iteration (and do not add to the step count).</summary>
    public bool IsLoopBody => Role == StepTree.Roles.LoopSteps;
}

/// <summary>
/// The one place that knows which <see cref="ProgramStep"/> properties hold nested step lists
/// (docs/program-language.md §4.1 #4). Everything that walks a program — step counting, the
/// validator, the id reconciler, the text printer, paste — goes through here, so a new block
/// body is added once.
/// </summary>
public static class StepTree
{
    /// <summary>The roles, in the order <see cref="ChildLists"/> yields them (frozen: paths use it).</summary>
    public static class Roles
    {
        public const string LoopSteps       = "loopSteps";
        public const string IfSteps         = "ifSteps";
        public const string ElseIfBranches  = "elseIfBranches";
        public const string ElseSteps       = "elseSteps";
        public const string SwitchCases     = "switchCases";
        public const string SwitchDefault   = "switchDefault";
        public const string TrySteps        = "trySteps";
        public const string CatchSteps      = "catchSteps";
        public const string CncProgramSteps = "cncProgramSteps";

        public static readonly IReadOnlyList<string> All =
        [
            LoopSteps, IfSteps, ElseIfBranches, ElseSteps, SwitchCases, SwitchDefault,
            TrySteps, CatchSteps, CncProgramSteps,
        ];
    }

    /// <summary>
    /// Every non-null nested step list of <paramref name="s"/>, in <see cref="Roles.All"/> order.
    /// </summary>
    /// <param name="executableOnly">Skip lists the executor never runs: legacy baked
    /// <c>cncProgramSteps</c> when the step also has a <c>cncSpec</c> (the spec wins).</param>
    public static IEnumerable<StepChildList> ChildLists(this ProgramStep s, bool executableOnly = false)
    {
        if (s.LoopSteps != null) yield return new(Roles.LoopSteps, null, s.LoopSteps);
        if (s.IfSteps   != null) yield return new(Roles.IfSteps,   null, s.IfSteps);
        if (s.ElseIfBranches != null)
            for (int i = 0; i < s.ElseIfBranches.Count; i++)
                if (s.ElseIfBranches[i]?.Steps is { } b) yield return new(Roles.ElseIfBranches, i, b);
        if (s.ElseSteps != null) yield return new(Roles.ElseSteps, null, s.ElseSteps);
        if (s.SwitchCases != null)
            for (int i = 0; i < s.SwitchCases.Count; i++)
                if (s.SwitchCases[i]?.Steps is { } c) yield return new(Roles.SwitchCases, i, c);
        if (s.SwitchDefault != null) yield return new(Roles.SwitchDefault, null, s.SwitchDefault);
        if (s.TrySteps   != null) yield return new(Roles.TrySteps,   null, s.TrySteps);
        if (s.CatchSteps != null) yield return new(Roles.CatchSteps, null, s.CatchSteps);
        if (s.CncProgramSteps != null && !(executableOnly && s.CncSpec != null))
            yield return new(Roles.CncProgramSteps, null, s.CncProgramSteps);
    }

    /// <summary>Every step in <paramref name="steps"/> and below, pre-order (parent before its
    /// bodies). Null entries are skipped. Routine bodies are not followed.</summary>
    /// <param name="enabledOnly">Do not yield a disabled step or anything inside it.</param>
    public static IEnumerable<ProgramStep> Walk(IEnumerable<ProgramStep>? steps, bool enabledOnly = false,
                                                bool executableOnly = false)
    {
        if (steps == null) yield break;
        foreach (var s in steps)
        {
            if (s == null) continue;
            if (enabledOnly && !s.IsEnabled) continue;
            yield return s;
            foreach (var child in s.ChildLists(executableOnly))
                foreach (var c in Walk(child.Steps, enabledOnly, executableOnly))
                    yield return c;
        }
    }

    /// <summary>Every step with its stepPath (<c>steps[2].switchCases[0].steps[1]</c>), pre-order.</summary>
    public static IEnumerable<(ProgramStep Step, string Path)> WalkWithPaths(List<ProgramStep>? steps, string root = "steps")
    {
        if (steps == null) yield break;
        for (int i = 0; i < steps.Count; i++)
        {
            var s = steps[i];
            if (s == null) continue;
            var path = $"{root}[{i}]";
            yield return (s, path);
            foreach (var child in s.ChildLists())
                foreach (var c in WalkWithPaths(child.Steps, $"{path}.{child.Path}"))
                    yield return c;
        }
    }
}
