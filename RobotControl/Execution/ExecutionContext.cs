using System.Collections.Concurrent;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Execution
{
    /// <summary>What a step handler tells the executor after one call.</summary>
    internal enum StepOutcome
    {
        /// <summary>The step is done: the executor advances past it and reports it completed.</summary>
        Advance,

        /// <summary>
        /// The executor takes no further action for this step this tick. Either the step is
        /// still in progress (the handler is called again next tick, e.g. a Wait), or the
        /// handler has already moved the program on itself through one of the explicit
        /// <see cref="ExecutionContext"/> helpers — <see cref="ExecutionContext.CompleteNow"/>
        /// when completion must be reported before later effects (Loop, If, Pause, GoToLabel),
        /// or <see cref="ExecutionContext.AwaitMove"/> and friends when completion is reported
        /// later, once dispatched motion finishes.
        /// </summary>
        Yield,

        /// <summary>The handler ended the program (normally with an error) via <see cref="ExecutionContext.Finish"/>.</summary>
        Finished,
    }

    /// <summary>Executes one kind of program step. Implementations are stateless singletons;
    /// per-run state lives on the frame or in a state object on the context.</summary>
    internal interface IStepHandler
    {
        StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx);
    }

    /// <summary>
    /// Everything a step handler can reach for one executor: the controller, the saved
    /// geometry and programs, variables, progress reporting, motion dispatch, the frame
    /// stack, and the small state objects multi-tick handlers keep between ticks.
    /// </summary>
    /// <remarks>
    /// One per <see cref="ProgramExecutor"/>. Not thread-safe: only touched under the
    /// executor's control lock (the one exception, <see cref="PendingActions"/>, is a
    /// concurrent queue fed by HTTP completions and drained on the loop thread).
    /// </remarks>
    internal sealed class ExecutionContext : IProgramRunInfo
    {
        private readonly Action<ProgramStatus, string> _finish;
        private readonly Action _pause;

        public ExecutionContext(
            RobotController controller, ProgramCycleManager programManager,
            PointRepository pointRepo, LocalRepository localRepo, BuiltProgramRepository builtProgramRepo,
            GridRepository gridRepo, StackRepository stackRepo,
            VariableScope vars, bool isBackground, BackgroundProgramManager? backgroundManager,
            Action<ProgramStatus, string> finish, Action pause)
        {
            Controller        = controller;
            Locals            = localRepo;
            BuiltPrograms     = builtProgramRepo;
            TargetSources     = new MoveTargetSources(pointRepo, gridRepo, stackRepo, localRepo);
            Vars              = vars;
            IsBackground      = isBackground;
            Background        = backgroundManager;
            Frames            = new FrameStack();
            Progress          = new ProgressReporter(programManager, Frames, vars)
            {
                IsBackground = isBackground,
                Plugins      = () => controller.PluginManager,
            };
            Motion            = new MotionDispatcher(controller);
            LivePosition      = controller.GetCurrentPosition;
            _finish           = finish;
            _pause            = pause;
            // $robot.* / $program.* / $time.* / $aux.* and the plugins' $<id>.* — read live at
            // lookup time (plugin properties via the controller's PluginManager).
            vars.Properties   = new RobotPropertySource(controller, this);
        }

        /// <summary>The controller's plugin manager (read live, so tests can swap it); may be null.</summary>
        public global::Controller.RobotControl.Plugins.PluginManager? Plugins => Controller.PluginManager;

        // ── $program.* properties ─────────────────────────────────────────────

        int  IProgramRunInfo.RunCount  => Progress.RunCount;
        int  IProgramRunInfo.StepIndex => Progress.GlobalStepIndex;
        int  IProgramRunInfo.StepCount => Progress.StepCount;
        long IProgramRunInfo.ElapsedMs => Progress.ElapsedMs;
        int  IProgramRunInfo.LoopDepth => Frames.LoopDepth;

        // ── Collaborators ─────────────────────────────────────────────────────

        /// <summary>The controller — the sink for IO, motion, vision, cameras and webhooks.</summary>
        public RobotController          Controller    { get; }
        public LocalRepository          Locals        { get; }
        public BuiltProgramRepository   BuiltPrograms { get; }
        public MoveTargetSources        TargetSources { get; }
        public VariableScope            Vars          { get; }
        public EvalContext              Eval          => Vars.Eval;
        public ProgressReporter         Progress      { get; }
        public MotionDispatcher         Motion        { get; }
        public FrameStack               Frames        { get; }
        public bool                     IsBackground  { get; }
        public BackgroundProgramManager? Background   { get; }

        /// <summary>Reads the robot's live position (cached delegate, no per-call allocation).</summary>
        public Func<Vector6> LivePosition { get; }

        // ── Per-run state ─────────────────────────────────────────────────────

        /// <summary>The step whose handler is running right now (set by the executor around each
        /// dispatch); null between steps. An Error ending marks this step as the one that failed.</summary>
        public ProgramStep? CurrentStep { get; set; }

        /// <summary>The program being run (set on Start; kept after it finishes).</summary>
        public BuiltProgram? Program { get; set; }

        /// <summary>Active local during program execution — null = no local (zero offset).</summary>
        public Vector6? ActiveLocal { get; set; }

        /// <summary>WaitForBackground: the program id being waited on, or null.</summary>
        public string? WaitingForBackground { get; set; }

        /// <summary>
        /// Bumped whenever run state is reset, so async completions from a previous run
        /// (fire-and-continue HttpRequest) are discarded instead of applied.
        /// </summary>
        public int RunGeneration { get; set; }

        /// <summary>Variable writes queued by async completions, applied at the top of the next tick.</summary>
        public ConcurrentQueue<Action> PendingActions { get; } = new();

        /// <summary>
        /// Non-blocking SetOutput pulses: the reverting flip, due at DueMs. Processed at the
        /// top of Update() on the loop thread, so Stop/Reset/e-stop can cancel them.
        /// </summary>
        public List<(long DueMs, Action Flip)> OutputFlips { get; } = new();

        // Multi-tick handler state
        public JumpState   Jump   { get; } = new();
        public ThreadMoveState ThreadMove { get; } = new();
        public HomingState Homing { get; } = new();
        public VisionState Vision { get; } = new();
        public HttpState   Http   { get; } = new();
        public PluginStepState PluginState { get; } = new();

        // Active program recordings: channelKey → (IoHistory recording id, target list-variable name).
        // Set by StartRecording, consumed + cleared by StopRecording.
        public Dictionary<string, (string RecordingId, string VarName)> ProgramRecordings { get; } = new();

        // ── Control ───────────────────────────────────────────────────────────

        /// <summary>Ends the program; returns <see cref="StepOutcome.Finished"/> for the handler to return.</summary>
        public StepOutcome Finish(ProgramStatus status, string description)
        {
            _finish(status, description);
            return StepOutcome.Finished;
        }

        /// <summary>
        /// A <em>runtime</em> failure with a catalog error code (<see cref="Programs.ErrorCodes"/>;
        /// a RaiseError step may use its own identifier). Handlers call it instead of
        /// <c>Finish(Error, …)</c> for anything a Try should be able to catch
        /// (docs/program-language.md §4.2, "Catchable vs fatal errors").
        /// </summary>
        /// <remarks>
        /// When the code is catchable and an enclosing Try (at any depth, routines included)
        /// accepts it, the frames above that Try are abandoned and the Try either retries its
        /// body or runs its catch steps — see <see cref="TryCatch"/>; the handler's step is then
        /// not completed, and the program goes on next tick. Otherwise the program ends with the
        /// error. Returns <see cref="StepOutcome.Yield"/> when caught, <see cref="StepOutcome.Finished"/>
        /// when not — either way the handler just returns it.
        /// </remarks>
        public StepOutcome Fail(string code, string message)
        {
            if (string.IsNullOrWhiteSpace(code)) code = Programs.ErrorCodes.UserError;
            if (Programs.ErrorCodes.IsCatchable(code) && TryCatch(code, message)) return StepOutcome.Yield;
            Progress.Failing(CurrentStep, code);
            return Finish(ProgramStatus.Error, message);
        }

        /// <summary>
        /// Bumped whenever a Try catches a failure (the frame stack was rewritten). The debugger
        /// uses it so a retried step counts as a fresh step for step mode and breakpoints.
        /// </summary>
        public int CatchGeneration { get; private set; }

        /// <summary>Whether the Try <paramref name="tryStep"/> catches <paramref name="code"/>:
        /// an empty catchCodes list catches every catchable code.</summary>
        internal static bool Catches(ProgramStep tryStep, string code)
        {
            var codes = tryStep.CatchCodes;
            if (codes == null || codes.Count == 0 || codes.All(string.IsNullOrWhiteSpace)) return true;
            return codes.Any(c => string.Equals(c?.Trim(), code, StringComparison.Ordinal));
        }

        /// <summary>
        /// Unwinds to the innermost Try body frame that catches <paramref name="code"/>. With
        /// retries left it restarts the body (after tryRetryDelayMs); otherwise it pops the Try,
        /// writes the error variables and pushes the catch steps (an empty catch: carry on after
        /// the block). False when no Try on the stack catches the code.
        /// </summary>
        private bool TryCatch(string code, string message)
        {
            var frames = Frames.ToArray(); // innermost first
            for (int i = 0; i < frames.Length; i++)
            {
                var tf = frames[i];
                if (tf.Kind != FrameKind.Try || tf.TryStep is not { } t || !Catches(t, code)) continue;

                // Abandon everything inside the Try: loops, branches and whole routine calls
                // (whose out-parameters are not copied back — the call did not finish).
                for (int k = 0; k < i; k++) Frames.Pop();
                ClearStepState();
                CatchGeneration++;

                int retries = Math.Max(0, t.TryRetries ?? 0); // literal only (the catalog marks no exprKey)
                if (tf.TryAttempt < retries)
                {
                    tf.TryAttempt++;
                    tf.Index       = 0;
                    tf.WaitStarted = false;
                    int delay = Math.Max(0, t.TryRetryDelayMs ?? 0);
                    tf.ResumeAtMs = delay > 0 ? DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() + delay : 0;
                    Progress.ErrorCaught(t, code, message, $"retry {tf.TryAttempt} of {retries}");
                    return true;
                }

                Frames.Pop(); // the Try body itself
                if (!string.IsNullOrWhiteSpace(t.ErrorVariableName))
                    Vars.SetString(t.ErrorVariableName.Trim().TrimStart('$'), message);
                if (!string.IsNullOrWhiteSpace(t.ErrorCodeVariableName))
                    Vars.SetString(t.ErrorCodeVariableName.Trim().TrimStart('$'), code);
                var catchSteps = t.CatchSteps ?? new();
                if (catchSteps.Count > 0) Frames.Push(StepListFrame.Plain(catchSteps));
                Progress.ErrorCaught(t, code, message, catchSteps.Count > 0 ? "running the catch steps" : "ignored");
                return true;
            }
            return false;
        }

        /// <summary>
        /// Per-step multi-tick state the failed step may have left behind (a vision processor
        /// still running, a Jump between legs). Pending async work of <em>earlier</em> steps (a
        /// fire-and-continue HTTP request, output pulses, recordings) is left alone.
        /// </summary>
        private void ClearStepState()
        {
            Vision.Reset(Controller);
            Jump.Reset();
            ThreadMove.Reset();
        }

        /// <summary>
        /// Leaves a routine frame (its body ran out, or a Return): copies each mapped out / inOut
        /// parameter into the caller's variable. A mapping whose key is not an out / inOut
        /// parameter of the routine is ignored (the validator reports it).
        /// </summary>
        public void LeaveRoutine(StepListFrame routineFrame)
        {
            if (routineFrame.Kind != FrameKind.Routine || routineFrame.RoutineOutputs is not { Count: > 0 } outs) return;
            var routine = routineFrame.Routine;
            foreach (var m in outs)
            {
                if (m == null || string.IsNullOrWhiteSpace(m.Key) || string.IsNullOrWhiteSpace(m.VariableName)) continue;
                var param = routine?.Variables?.FirstOrDefault(v =>
                    string.Equals(v.Name, m.Key.Trim(), StringComparison.OrdinalIgnoreCase));
                if (param?.ParamMode is not (ParamModes.Out or ParamModes.InOut)) continue;
                Vars.CopyVariable(param.Name, m.VariableName.Trim().TrimStart('$'));
            }
        }

        /// <summary>Pauses the program in place (PauseProgram step).</summary>
        public void Pause() => _pause();

        /// <summary>
        /// Advances past <paramref name="step"/> and reports it completed — exactly what the
        /// executor does on <see cref="StepOutcome.Advance"/>. For handlers whose completion
        /// must be reported before their remaining effects; they then return
        /// <see cref="StepOutcome.Yield"/>.
        /// </summary>
        public void CompleteNow(ProgramStep step, StepListFrame frame)
        {
            frame.Index++;
            Progress.StepCompleted(step);
        }

        /// <summary>
        /// A robot move for <paramref name="step"/> was dispatched: report it started, move
        /// on, and report it completed when the move finishes.
        /// </summary>
        public void AwaitMove(ProgramStep step, StepListFrame frame)
        {
            Motion.AwaitingMove = true;
            Motion.PendingStep  = step; // completion is reported in Update() once the move finishes
            // Announce the step is in progress without counting it yet
            Progress.StepStarted(step);
            frame.Index++;
        }

        /// <summary>A blended run covering <paramref name="run"/> (consecutive steps from the frame) was dispatched.</summary>
        public void AwaitBlendedRun(List<ProgramStep> run, StepListFrame frame)
        {
            Motion.AwaitingMove = true;
            Motion.PendingSteps = run;
            foreach (var s in run) Progress.StepStarted(s);
            frame.Index += run.Count;
        }

        /// <summary>An aux move with WaitForDone was dispatched; completion is reported when it stops.</summary>
        public void AwaitAuxMove(ProgramStep step, StepListFrame frame)
        {
            Progress.StepStarted(step);
            frame.Index++;
            Motion.AwaitingAux    = true;
            Motion.PendingAuxStep = step;
        }

        /// <summary>
        /// <see cref="AwaitAuxMove(ProgramStep, StepListFrame)"/> for a step that may carry a
        /// stop condition (AuxMove, GripperAction): the watch needs the axis to read its position
        /// from and <paramref name="decelHz"/> (steps/s²) to stop it smoothly with.
        /// </summary>
        public void AwaitAuxMove(ProgramStep step, StepListFrame frame, string deviceId, int axis, int decelHz)
        {
            AwaitAuxMove(step, frame);
            Motion.ArmWatch(MoveStopConditions.Create(step, isAux: true, deviceId, axis, Math.Max(1, decelHz)));
        }

        // ── Move stop condition ───────────────────────────────────────────────

        /// <summary>
        /// Arms the stop condition of a robot move step (MoveL/MoveJ single move, Jump at its
        /// first leg) that was just dispatched. No condition on the step = no watch.
        /// </summary>
        public void ArmMoveWatch(ProgramStep step) =>
            Motion.ArmWatch(MoveStopConditions.Create(step, isAux: false));

        /// <summary>
        /// One tick of the armed watch: evaluates the condition (against this tick's fresh IO)
        /// and, the first time it is true, captures the position and — in stop mode — ends the
        /// move. Returns false when the program was finished with an error (an unknown
        /// variable or a syntax error in the condition), true otherwise.
        /// </summary>
        public bool TickMoveWatch()
        {
            if (Motion.Watch is not { Fired: false } w) return true;

            bool hit;
            try
            {
                hit = Eval.EvaluateCondition(w.Condition);
            }
            catch (UnknownVariableException ex)
            {
                Finish(ProgramStatus.Error,
                    $"Stop condition on {ProgressReporter.StepDescription(w.Step)}: unknown variable '${ex.VariableName}'");
                return false;
            }
            catch (ExpressionParseException ex)
            {
                Finish(ProgramStatus.Error,
                    $"Stop condition on {ProgressReporter.StepDescription(w.Step)}: {ProgressReporter.DescribeParseError(ex)}");
                return false;
            }
            if (!hit) return true;

            w.Fired = true;
            if (w.IsAux)
            {
                var aux = Controller.AuxAxisManager;
                string dev = w.AuxDeviceId ?? "";
                if (w.PositionVar != null)
                {
                    // Physical unit when the axis has one, raw steps otherwise — the same
                    // convention as auxDistance/auxUnit on the move itself.
                    long   steps = aux.GetPosition(dev, w.AuxAxis);
                    double spu   = aux.GetAxisConfig(dev, w.AuxAxis)?.StepsPerUnit() ?? 0;
                    Vars.Set(w.PositionVar, spu > 0 ? steps / spu : steps);
                }
                if (w.Mode == MoveStopMode.Stop) aux.StopSmooth(dev, w.AuxAxis, w.AuxDecelHz);
            }
            else
            {
                if (w.PositionVar != null)
                {
                    // Exactly one element, so $var[0] is the contact pose whatever the list held.
                    var p = LivePosition();
                    Vars.SetList(w.PositionVar, ListVar.OfPoints([new Vector6Val
                    {
                        X = p.X, Y = p.Y, Z = p.Z, RX = p.RX, RY = p.RY, RZ = p.RZ,
                    }]));
                }
                if (w.Mode == MoveStopMode.Stop) Controller.RequestMotionStop();
            }
            return true;
        }

        /// <summary>The watched move completed: writes the fired flag (1 / 0) and clears the watch.</summary>
        public void CompleteMoveWatch(MoveWatch w)
        {
            // Cleared first: a refused flag write (computed variable) ends the program anyway,
            // and the watch must not outlive its move either way.
            Motion.ClearWatch();
            if (w.FlagVar != null) Vars.Set(w.FlagVar, w.Fired ? 1 : 0);
        }

        // ── Shared helpers ────────────────────────────────────────────────────

        /// <summary>Resolves a move target (see <see cref="MoveTargetResolver"/>); on failure
        /// fails the step and returns false. A computed target (<c>pointNameExpr</c>: a template
        /// naming a saved point, or a points-variable element) that resolves to nothing fails with
        /// the catchable <c>pointNotFound</c>; a missing saved point, grid or stack picked in the
        /// editor is an authoring error and ends the program.</summary>
        public bool ResolveMoveTarget(ProgramStep step, out Vector6 target, Vector6? currentPos = null)
        {
            if (TryResolveMoveTarget(step, out target, out var error, currentPos)) return true;
            if (IsComputedTarget(step)) Fail(Programs.ErrorCodes.PointNotFound, error);
            else Finish(ProgramStatus.Error, error);
            return false;
        }

        /// <summary>Resolves a move target with no side effects (false + the error text).</summary>
        public bool TryResolveMoveTarget(ProgramStep step, out Vector6 target, out string error, Vector6? currentPos = null) =>
            MoveTargetResolver.TryResolve(step, TargetSources, Vars, ActiveLocal, LivePosition,
                                          out target, out error, currentPos);

        /// <summary>The target comes from <c>pointNameExpr</c> (the resolver's precedence: grid,
        /// stack and the legacy variable point win over it).</summary>
        private static bool IsComputedTarget(ProgramStep step) =>
            step.GridPoint == null && step.StackPoint == null && string.IsNullOrEmpty(step.VarPointName)
            && !string.IsNullOrEmpty(step.PointNameExpr);

        /// <summary>Evaluates a while-loop condition; an unknown variable or a syntax error
        /// finishes the program with an error and reads as false.</summary>
        public bool EvalWhileCondition(ConditionGroup condition)
        {
            try
            {
                return Eval.EvaluateCondition(condition);
            }
            catch (UnknownVariableException ex)
            {
                // While-loop re-checks run outside the step dispatch, so error here directly.
                Finish(ProgramStatus.Error, $"Unknown variable '${ex.VariableName}' in while-loop condition");
                return false; // exit the loop — the program is already finishing with an error
            }
            catch (ExpressionParseException ex)
            {
                Finish(ProgramStatus.Error, $"Expression error in while-loop condition: {ProgressReporter.DescribeParseError(ex)}");
                return false;
            }
        }

        /// <summary>Writes a ForEach frame's index/value variables for its current iteration.</summary>
        public void InjectForEachVars(StepListFrame frame)
        {
            if (frame.Kind != FrameKind.ForEach) return;

            int idx = frame.ForEachCurrentIndex;

            // Write index variable if configured
            if (!string.IsNullOrEmpty(frame.IndexVar))
                Vars.Set(frame.IndexVar, idx);

            // Write value variable — only an element that is itself a value has one to give.
            // A boolean arrives as the 0/1 it is stored as, which is what a boolean variable
            // holds anyway. For point/record lists the element is not a number, so the value
            // variable gets the index instead and the element is read with $name[$i].field.
            if (!string.IsNullOrEmpty(frame.ForEachValueVar))
            {
                if (Vars.Lists.TryGetValue(frame.ForEachSourceVar, out var list) &&
                    list.HasScalarElements)
                    Vars.Set(frame.ForEachValueVar, idx < list.Count ? list.Items[idx].Scalar : 0);
                else
                    Vars.Set(frame.ForEachValueVar, idx); // point/object array or unknown: expose index
            }
        }

        /// <summary>
        /// Resets every per-run field to its idle value and releases anything a run may still
        /// hold (vision processor, webhook subscription, pending async work, output pulses).
        /// Variables are not touched. An outstanding plugin step is cancelled with
        /// <paramref name="pluginCancelReason"/> (<c>reset</c>, or <c>stopped</c> when a user Stop ended the run).
        /// </summary>
        public void ResetRunState(string pluginCancelReason = "reset")
        {
            RunGeneration++;

            // Plugin step — step.cancel, so the plugin can stop and a late reply is discarded
            PluginState.Reset(pluginCancelReason);

            Frames.Clear();
            Progress.Reset();
            ActiveLocal          = null;
            WaitingForBackground = null;

            Motion.Reset();
            Jump.Reset();
            ThreadMove.Reset();
            Homing.Reset();

            // Vision — a processor left running would keep grabbing and analysing frames
            Vision.Reset(Controller);

            // Async I/O
            Http.PendingJsonTask = null;
            while (PendingActions.TryDequeue(out _)) { }
            Http.ReleaseWebhook(Controller);

            // Output pulses not yet reverted are cancelled, not fired
            OutputFlips.Clear();

            Controller.ActiveCncToolpath = null;
        }
    }

    /// <summary>Maps each step type to its handler.</summary>
    internal static class StepHandlers
    {
        private static readonly Dictionary<StepType, IStepHandler> Registry = Build();

        public static bool TryGet(StepType type, out IStepHandler handler) =>
            Registry.TryGetValue(type, out handler!);

        private static Dictionary<StepType, IStepHandler> Build()
        {
            var r = new Dictionary<StepType, IStepHandler>();
            MotionSteps.Register(r);
            IoSteps.Register(r);
            LoadCellSteps.Register(r);
            RecordingSteps.Register(r);
            ControlFlowSteps.Register(r);
            VariableSteps.Register(r);
            AuxSteps.Register(r);
            VisionSteps.Register(r);
            HttpSteps.Register(r);
            PluginSteps.Register(r);
            BackgroundSteps.Register(r);
            CncSteps.Register(r);
            // Program editor v2 (docs/program-language.md §8): flow (package B), motion / data /
            // operator (package C). Until those land each new type errors "not implemented yet".
            FlowExtSteps.Register(r);
            MotionExtSteps.Register(r);
            DataSteps.Register(r);
            OperatorSteps.Register(r);
            // Steps the controller does not recognise (from a newer app) are skipped.
            r[StepType.Unknown] = NoOpStep.Instance;
            return r;
        }

        private sealed class NoOpStep : IStepHandler
        {
            public static readonly NoOpStep Instance = new();
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx) => StepOutcome.Advance;
        }
    }

    /// <summary>
    /// Placeholder handler for a step type whose contract exists but whose behaviour has not
    /// landed yet: it ends the program with a clear error rather than skipping silently (a
    /// skipped MoveC or Try would run the rest of the program in a state the author did not
    /// intend). Replaced type by type as packages B and C register the real handlers.
    /// </summary>
    internal sealed class NotImplementedStep : IStepHandler
    {
        public static readonly NotImplementedStep Instance = new();
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx) =>
            ctx.Finish(ProgramStatus.Error,
                $"{ProgressReporter.StepDescription(step)}: {step.Type} steps are not implemented on this controller yet");
    }
}
