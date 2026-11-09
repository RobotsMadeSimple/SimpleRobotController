using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 flow blocks (docs/program-language.md §4.1–4.2): Note, LoopControl
    /// (break / continue), Return, EndProgram, RaiseError, Switch and Try.
    /// </summary>
    internal static class FlowExtSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Note]        = new NoteStep();
            r[StepType.LoopControl] = new LoopControlStep();
            r[StepType.Return]      = new ReturnStep();
            r[StepType.EndProgram]  = new EndProgramStep();
            r[StepType.RaiseError]  = new RaiseErrorStep();
            r[StepType.Switch]      = new SwitchStep();
            r[StepType.Try]         = new TryStep();
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

        /// <summary>
        /// Break: leave the nearest enclosing loop. Continue (Next iteration): end this pass and
        /// let the loop decide on the next one (count left, next element, condition re-checked).
        /// Frames above the loop (branches, Try bodies) are abandoned; a routine boundary is never
        /// crossed, so a Break in a routine called from a loop is an error.
        /// </summary>
        private sealed class LoopControlStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                bool next = step.LoopControl == "continue";
                var frames = ctx.Frames.ToArray(); // innermost first
                for (int i = 0; i < frames.Length; i++)
                {
                    var f = frames[i];
                    if (f.Kind == FrameKind.Routine) break;
                    if (!f.IsLoop) continue;

                    // Completed while still inside the loop, so it counts like any loop body step.
                    ctx.CompleteNow(step, frame);
                    for (int k = 0; k < i; k++) ctx.Frames.Pop();
                    if (next) f.Index = f.Steps.Count; // exhausted: the executor runs the next-pass logic
                    else      ctx.Frames.Pop();        // the loop is done
                    return StepOutcome.Yield;
                }
                return ctx.Finish(ProgramStatus.Error,
                    $"{ProgressReporter.StepDescription(step)}: not inside a loop" +
                    (frames.Any(f => f.Kind == FrameKind.Routine) ? " in this routine" : ""));
            }
        }

        /// <summary>In a routine: leave it now (out-parameters are copied). Outside one: the
        /// program is complete.</summary>
        private sealed class ReturnStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                ctx.CompleteNow(step, frame);
                var frames = ctx.Frames.ToArray();
                for (int i = 0; i < frames.Length; i++)
                {
                    if (frames[i].Kind != FrameKind.Routine) continue;
                    for (int k = 0; k <= i; k++) ctx.Frames.Pop();
                    ctx.LeaveRoutine(frames[i]);
                    return StepOutcome.Yield;
                }
                return ctx.Finish(ProgramStatus.Complete, "Complete");
            }
        }

        /// <summary>Finish the program as Complete from any depth, with an optional message
        /// (a template). A background program ends only itself.</summary>
        private sealed class EndProgramStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var message = string.IsNullOrWhiteSpace(step.StatusMessage)
                    ? "Complete"
                    : ctx.Vars.Interpolate(step.StatusMessage);
                ctx.CompleteNow(step, frame);
                return ctx.Finish(ProgramStatus.Complete, message);
            }
        }

        /// <summary>Fail with the step's message (a template) and code (default userError):
        /// caught by an enclosing Try, otherwise the program ends with the message.</summary>
        private sealed class RaiseErrorStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var code = string.IsNullOrWhiteSpace(step.ErrorCode) ? ErrorCodes.UserError : step.ErrorCode.Trim();
                var message = string.IsNullOrWhiteSpace(step.ErrorMessage)
                    ? $"Error raised ({code})"
                    : ctx.Vars.Interpolate(step.ErrorMessage);
                return ctx.Fail(code, message);
            }
        }

        /// <summary>Evaluates switchExpr once; the first case with a value equal to it (within
        /// 1e-9) runs, otherwise the default. No fall-through.</summary>
        private sealed class SwitchStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (string.IsNullOrWhiteSpace(step.SwitchExpr))
                    return ctx.Finish(ProgramStatus.Error, $"{ProgressReporter.StepDescription(step)}: no value to switch on");

                // Like If: counted before the expressions are evaluated.
                ctx.CompleteNow(step, frame);
                double value = ctx.Eval.Evaluate(step.SwitchExpr);

                foreach (var c in step.SwitchCases ?? [])
                {
                    if (c == null) continue;
                    foreach (var v in c.Values ?? [])
                    {
                        if (string.IsNullOrWhiteSpace(v)) continue;
                        if (Math.Abs(ctx.Eval.Evaluate(v) - value) > 1e-9) continue;
                        if (c.Steps is { Count: > 0 }) ctx.Frames.Push(StepListFrame.SwitchBody(c.Steps));
                        return StepOutcome.Yield;
                    }
                }
                if (step.SwitchDefault is { Count: > 0 } def) ctx.Frames.Push(StepListFrame.SwitchBody(def));
                return StepOutcome.Yield;
            }
        }

        /// <summary>Runs trySteps in a Try frame; <see cref="ExecutionContext.Fail"/> unwinds to it.</summary>
        private sealed class TryStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                ctx.CompleteNow(step, frame);
                if (step.TrySteps is { Count: > 0 }) ctx.Frames.Push(StepListFrame.TryBody(step));
                return StepOutcome.Yield;
            }
        }
    }
}
