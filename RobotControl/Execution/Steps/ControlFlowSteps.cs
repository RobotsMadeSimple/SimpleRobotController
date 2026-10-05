namespace Controller.RobotControl.Execution
{
    /// <summary>Program flow: Loop, Label, GoToLabel, IfCondition, CallRoutine,
    /// PauseProgram, StatusUpdate.</summary>
    internal static class ControlFlowSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Loop]         = new LoopStep();
            r[StepType.Label]        = new LabelStep();
            r[StepType.GoToLabel]    = new GoToLabelStep();
            r[StepType.IfCondition]  = new IfConditionStep();
            r[StepType.CallRoutine]  = new CallRoutineStep();
            r[StepType.PauseProgram] = new PauseProgramStep();
            r[StepType.StatusUpdate] = new StatusUpdateStep();
        }

        private sealed class LoopStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var innerSteps = step.LoopSteps ?? new();
                if (innerSteps.Count == 0) return StepOutcome.Advance;

                // The loop header itself is done (and counts) before its body frame goes on
                // the stack; body steps count separately.
                ctx.CompleteNow(step, frame);

                if (step.LoopMode == "while" && step.LoopWhileCondition != null)
                {
                    // Pre-check: if condition is already false, skip the body entirely
                    if (!ctx.EvalWhileCondition(step.LoopWhileCondition))
                        return StepOutcome.Yield;
                    ctx.Frames.Push(StepListFrame.WhileLoop(innerSteps, step.LoopWhileCondition));
                }
                else if (step.LoopMode == "forEach" && !string.IsNullOrEmpty(step.ForEachVariableName))
                {
                    // Determine iteration count from the source collection
                    int count = ctx.Vars.Lists.TryGetValue(step.ForEachVariableName, out var lst)
                        ? lst.Count : 0;

                    if (count == 0) return StepOutcome.Yield; // empty — skip body

                    var bodyFrame = StepListFrame.ForEach(innerSteps, count, currentIndex: 0,
                        sourceVar: step.ForEachVariableName,
                        valueVar:  step.ForEachValueVariableName ?? "",
                        indexVar:  step.ForEachIndexVariableName ?? "");
                    ctx.Frames.Push(bodyFrame);
                    ctx.InjectForEachVars(bodyFrame);
                }
                else
                {
                    int count     = (int)ctx.Eval.EvalField(step, "loopCount", step.LoopCount ?? 1);
                    int remaining = count == 0 ? int.MaxValue : count; // 0 = infinite

                    // Initialise the count-loop index variable to 0 on first entry
                    string indexVar = step.ForEachIndexVariableName ?? "";
                    if (!string.IsNullOrEmpty(indexVar))
                        ctx.Vars.Set(indexVar, 0);

                    ctx.Frames.Push(StepListFrame.CountLoop(innerSteps, remaining,
                        total: count == 0 ? int.MaxValue : count, indexVar));
                }
                return StepOutcome.Yield;
            }
        }

        private sealed class LabelStep : IStepHandler
        {
            // Labels are no-ops at runtime — they are only markers for GoToLabel
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx) =>
                StepOutcome.Advance;
        }

        private sealed class GoToLabelStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (string.IsNullOrEmpty(step.LabelId))
                    return ctx.Finish(ProgramStatus.Error, "GoToLabel: no label ID set");

                // Search current frame first, then walk up the stack toward the top-level frame.
                // frames[0] = current (deepest), frames[last] = top-level.
                var frames = ctx.Frames.ToArray();
                for (int fi = 0; fi < frames.Length; fi++)
                {
                    var target = frames[fi];
                    for (int i = 0; i < target.Steps.Count; i++)
                    {
                        if (target.Steps[i].Type == StepType.Label &&
                            target.Steps[i].LabelId == step.LabelId)
                        {
                            // Unwind any frames above the target (abandoned loops leave the
                            // loop depth as they are popped).
                            for (int k = 0; k < fi; k++) ctx.Frames.Pop();
                            ctx.Progress.StepCompleted(step);
                            target.Index = i; // Label step advances past itself on the next tick
                            return StepOutcome.Yield;
                        }
                    }
                }

                return ctx.Finish(ProgramStatus.Error,
                    $"GoToLabel: label '{step.LabelName ?? step.LabelId}' not found");
            }
        }

        private sealed class IfConditionStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                // Reported before the conditions are evaluated, so a bad condition errors
                // the program with this step already counted.
                ctx.CompleteNow(step, frame);

                if (step.Condition != null && ctx.Eval.EvaluateCondition(step.Condition))
                {
                    var body = step.IfSteps ?? new();
                    if (body.Count > 0) ctx.Frames.Push(StepListFrame.Plain(body));
                    return StepOutcome.Yield;
                }

                foreach (var elif in step.ElseIfBranches ?? [])
                {
                    if (ctx.Eval.EvaluateCondition(elif.Condition))
                    {
                        if (elif.Steps.Count > 0) ctx.Frames.Push(StepListFrame.Plain(elif.Steps));
                        return StepOutcome.Yield;
                    }
                }

                var elseBody = step.ElseSteps ?? new();
                if (elseBody.Count > 0) ctx.Frames.Push(StepListFrame.Plain(elseBody));
                return StepOutcome.Yield;
            }
        }

        private sealed class CallRoutineStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                // Resolve by id first (survives renames), falling back to name for legacy steps.
                var repo    = ctx.BuiltPrograms;
                var routine = !string.IsNullOrEmpty(step.RoutineId) ? repo.GetById(step.RoutineId!) : null;
                routine ??= repo.Get(step.RoutineName ?? "");
                if (routine is null)
                    return ctx.Finish(ProgramStatus.Error, $"Routine not found: {step.RoutineName}");
                if (routine.Steps.Count == 0) return StepOutcome.Advance;

                // Register the routine's own variables so they can be used inside the routine.
                // Caller variables remain available (shared scope); a routine variable with
                // the same name as a caller's takes the routine's declared default.
                ctx.Vars.Initialize(routine);

                // Push the routine's steps as a plain (non-loop) frame. The call step itself
                // then completes on Advance (a plain frame leaves the loop depth unchanged, so
                // it counts exactly as if reported first); routine steps count separately.
                ctx.Frames.Push(StepListFrame.Plain(routine.Steps));
                return StepOutcome.Advance;
            }
        }

        private sealed class PauseProgramStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                ctx.CompleteNow(step, frame);
                // Pause without clearing the frame stack so Resume() can continue from the next step
                ctx.Pause();
                return StepOutcome.Yield;
            }
        }

        private sealed class StatusUpdateStep : IStepHandler
        {
            // The status message is emitted as this step's completion report — see
            // ProgressReporter.StepCompleted.
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx) =>
                StepOutcome.Advance;
        }
    }
}
