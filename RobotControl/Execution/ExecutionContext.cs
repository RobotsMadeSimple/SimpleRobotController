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
    internal sealed class ExecutionContext
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
            Progress          = new ProgressReporter(programManager, Frames, vars);
            Motion            = new MotionDispatcher(controller);
            LivePosition      = controller.GetCurrentPosition;
            _finish           = finish;
            _pause            = pause;
        }

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

        // ── Control ───────────────────────────────────────────────────────────

        /// <summary>Ends the program; returns <see cref="StepOutcome.Finished"/> for the handler to return.</summary>
        public StepOutcome Finish(ProgramStatus status, string description)
        {
            _finish(status, description);
            return StepOutcome.Finished;
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

        // ── Shared helpers ────────────────────────────────────────────────────

        /// <summary>Resolves a move target (see <see cref="MoveTargetResolver"/>); on failure
        /// finishes the program with the error and returns false.</summary>
        public bool ResolveMoveTarget(ProgramStep step, out Vector6 target, Vector6? currentPos = null)
        {
            if (MoveTargetResolver.TryResolve(step, TargetSources, Vars, ActiveLocal, LivePosition,
                                              out target, out var error, currentPos))
                return true;
            Finish(ProgramStatus.Error, error);
            return false;
        }

        /// <summary>Evaluates a while-loop condition; an unknown variable finishes the program
        /// with an error and reads as false.</summary>
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
        /// Variables are not touched.
        /// </summary>
        public void ResetRunState()
        {
            RunGeneration++;

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
            ControlFlowSteps.Register(r);
            VariableSteps.Register(r);
            AuxSteps.Register(r);
            VisionSteps.Register(r);
            HttpSteps.Register(r);
            BackgroundSteps.Register(r);
            CncSteps.Register(r);
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
}
