namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 operator blocks (docs/program-language.md §4.3): Prompt. Owned by
    /// package C — a placeholder until it lands (the status DTO is
    /// <see cref="ProgramPromptInfo"/>, published with <c>ProgramCycleManager.SetPrompt</c>).
    /// </summary>
    internal static class OperatorSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Prompt] = NotImplementedStep.Instance;
        }
    }
}
