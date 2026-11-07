namespace Controller.RobotControl.Validation
{
    // Package C (docs/program-language.md §8): checks for MoveC (via point missing,
    // unsupportedOnStep), SavePosition (pointWriteInBackground), SetSpeedFactor, ListOp (the
    // fields each listAction needs) and Prompt (1–4 buttons).
    //
    // Implement here:
    //     partial void CheckStepsExt(ProgramStep s, At at) { … }
    internal static partial class ProgramValidator
    {
        private sealed partial class Walker
        {
        }
    }
}
