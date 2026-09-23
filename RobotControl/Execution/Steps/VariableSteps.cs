namespace Controller.RobotControl.Execution
{
    /// <summary>Variable writes: SetVariable and StopwatchControl.</summary>
    internal static class VariableSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.SetVariable]      = new SetVariableStep();
            r[StepType.StopwatchControl] = new StopwatchControlStep();
        }

        private sealed class SetVariableStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (!string.IsNullOrEmpty(step.VariableName) && !string.IsNullOrEmpty(step.VariableExpr))
                {
                    var vars = ctx.Vars;
                    if (vars.IsString(step.VariableName))
                    {
                        // String variable — treat expr as a template and interpolate $var tokens
                        vars.SetString(step.VariableName, vars.Interpolate(step.VariableExpr));
                    }
                    else
                    {
                        try
                        {
                            vars.Set(step.VariableName, ctx.Eval.Evaluate(step.VariableExpr));
                        }
                        catch (UnknownVariableException)
                        {
                            throw; // errors the program via the dispatch-level handler
                        }
                        catch (ExpressionParseException)
                        {
                            throw; // a syntax error errors the program too
                        }
                        catch
                        {
                            // Malformed expression — leave variable unchanged
                        }
                    }
                }
                return StepOutcome.Advance;
            }
        }

        private sealed class StopwatchControlStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var varName = step.StopwatchVariableName;
                if (!string.IsNullOrEmpty(varName))
                    ctx.Vars.ControlStopwatch(varName, step.StopwatchAction);

                ctx.Progress.StepStarted(step);
                return StepOutcome.Advance;
            }
        }
    }
}
