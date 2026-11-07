namespace Controller.RobotControl.Validation
{
    // Package B (docs/program-language.md §8): checks for the flow blocks — LoopControl
    // (loopControlOutsideLoop), Switch (duplicateCaseValue, emptyBranch), Try (emptyBranch),
    // RaiseError, EndProgram and routine parameters (unknownParam, paramMissing, paramOutputType).
    //
    // Implement here:
    //     partial void CheckFlowExt(ProgramStep s, string sp, At at, List<List<ProgramStep>> chain, Body body) { … }
    // Nested bodies (switchCases[i].steps, switchDefault, trySteps, catchSteps) are already walked
    // by ValidateStep through StepTree; this hook only adds the per-step checks.
    internal static partial class ProgramValidator
    {
        private sealed partial class Walker
        {
        }
    }
}
