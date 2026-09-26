namespace Controller.RobotControl.Execution
{
    /// <summary>Background programs: StartBackground, StopBackground, WaitForBackground.</summary>
    internal static class BackgroundSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.StartBackground]   = new StartBackgroundStep();
            r[StepType.StopBackground]    = new StopBackgroundStep();
            r[StepType.WaitForBackground] = new WaitForBackgroundStep();
        }

        private static BuiltProgram? Resolve(ProgramStep step, ExecutionContext ctx)
        {
            if (!string.IsNullOrEmpty(step.BackgroundProgramId))
                return ctx.BuiltPrograms.GetById(step.BackgroundProgramId);
            if (!string.IsNullOrEmpty(step.BackgroundProgramName))
                return ctx.BuiltPrograms.Get(step.BackgroundProgramName);
            return null;
        }

        private sealed class StartBackgroundStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (ctx.Background != null)
                {
                    var prog = Resolve(step, ctx);
                    if (prog != null) ctx.Background.TryStart(prog);
                }
                return StepOutcome.Advance;
            }
        }

        private sealed class StopBackgroundStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (ctx.Background != null)
                {
                    var prog = Resolve(step, ctx);
                    if (prog != null) ctx.Background.Stop(prog.Id);
                }
                return StepOutcome.Advance;
            }
        }

        private sealed class WaitForBackgroundStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (ctx.Background == null) return StepOutcome.Advance;

                var prog = Resolve(step, ctx);
                if (prog == null || !ctx.Background.IsRunning(prog.Id)) return StepOutcome.Advance;

                // Still running — set the wait flag and yield; Update() blocks until the
                // program stops, then clears the flag and re-runs this step, which advances.
                ctx.WaitingForBackground = prog.Id;
                return StepOutcome.Yield;
            }
        }
    }
}
