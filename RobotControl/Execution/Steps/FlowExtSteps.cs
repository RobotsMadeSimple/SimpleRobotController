namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 flow blocks (docs/program-language.md §4.1–4.2): Note, LoopControl,
    /// Return, EndProgram, RaiseError, Switch, Try. Owned by package B — every type but Note is
    /// a <see cref="NotImplementedStep"/> placeholder until B lands it.
    /// </summary>
    internal static class FlowExtSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Note]        = new NoteStep();
            r[StepType.LoopControl] = NotImplementedStep.Instance;
            r[StepType.Return]      = NotImplementedStep.Instance;
            r[StepType.EndProgram]  = NotImplementedStep.Instance;
            r[StepType.RaiseError]  = NotImplementedStep.Instance;
            r[StepType.Switch]      = NotImplementedStep.Instance;
            r[StepType.Try]         = NotImplementedStep.Instance;
        }

        /// <summary>A note or section header: does nothing, and is not a step — it moves the frame
        /// on without reporting a completion, so it never adds to the step count.</summary>
        private sealed class NoteStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                frame.Index++;
                return StepOutcome.Yield;
            }
        }
    }
}
