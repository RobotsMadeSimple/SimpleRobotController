namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 data blocks (docs/program-language.md §4.3): ListOp. Owned by
    /// package C — a placeholder until it lands.
    /// </summary>
    internal static class DataSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.ListOp] = NotImplementedStep.Instance;
        }
    }
}
