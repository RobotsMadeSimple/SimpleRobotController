using Controller.RobotControl.Io;

namespace Controller.RobotControl.Execution
{
    /// <summary>Load cell (HX711) program steps: Tare (zero) and ReadLoadCell (into a variable).</summary>
    internal static class LoadCellSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Tare]          = new TareStep();
            r[StepType.ReadLoadCell]  = new ReadLoadCellStep();
        }

        /// <summary>Zero the selected load cell (capture its current raw count as the tare offset).</summary>
        private sealed class TareStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (!string.IsNullOrEmpty(step.LoadCellDeviceId) && step.LoadCellPin is int pin)
                    ctx.Controller.NanoManager.TareLoadCell(step.LoadCellDeviceId, pin);
                return StepOutcome.Advance;
            }
        }

        /// <summary>Read the load cell's weight into a program variable, in the step's chosen unit (default grams).</summary>
        private sealed class ReadLoadCellStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (!string.IsNullOrEmpty(step.VariableName) &&
                    !string.IsNullOrEmpty(step.LoadCellDeviceId) && step.LoadCellPin is int pin)
                {
                    double grams = ctx.Controller.NanoManager.GetLoadCellGrams(step.LoadCellDeviceId, pin) ?? 0;
                    ctx.Vars.Set(step.VariableName, LoadCellUnits.GramsToUnit(grams, step.LoadCellUnit));
                }
                return StepOutcome.Advance;
            }
        }
    }
}
