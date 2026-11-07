namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 motion blocks (docs/program-language.md §4.3): MoveC, SavePosition,
    /// SetSpeedFactor. Owned by package C — placeholders until it lands them.
    /// </summary>
    internal static class MotionExtSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.MoveC]          = NotImplementedStep.Instance;
            r[StepType.SavePosition]   = NotImplementedStep.Instance;
            r[StepType.SetSpeedFactor] = NotImplementedStep.Instance;
        }
    }
}
