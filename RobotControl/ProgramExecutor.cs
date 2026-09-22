using Controller.RobotControl.Execution;
using Controller.RobotControl.Persistence;
using System.Text.Json;
using ExecutionContext = Controller.RobotControl.Execution.ExecutionContext;

namespace Controller.RobotControl
{
    /// <summary>
    /// Executes a BuiltProgram step-by-step inside the main control loop.
    /// Call Update() on every Loop() tick. Each step is run by its <see cref="IStepHandler"/>
    /// (see <see cref="StepHandlers"/>); this class owns the run lifecycle — start, stop/pause,
    /// resume, reset, finish — the per-tick waits, and the frame stack walk.
    /// </summary>
    internal class ProgramExecutor
    {
        // ── State ────────────────────────────────────────────────────────────
        // Every piece of per-run state is reset in one place, ResetRunState() (most of it
        // lives on the ExecutionContext). Variables are the exception: they outlive Finish()
        // so the monitor keeps showing final values, and are cleared on Start/Reset.
        //
        // Threading: Update() runs on the ProgramLoop thread; Start/Stop/Resume/Reset
        // and the GetDisplay* readers are called from WebSocket (and occasionally the
        // motion) threads. All of them hold _controlLock for their whole body, so a
        // control call always observes — and leaves — a consistent executor state.
        // Nothing under Execution/ is thread-safe on its own; it relies on this lock.
        private readonly object _controlLock = new();
        // volatile: IsRunning/IsPaused are read lock-free by other threads
        private volatile bool   _running;
        private volatile bool   _isPaused;

        private readonly ExecutionContext _ctx;
        private readonly VariableScope    _vars;
        private readonly bool             _isBackground;

        private BuiltProgram? Program => _ctx.Program;

        public bool IsRunning => _running;
        public bool IsPaused  => _isPaused;
        public string? CurrentProgramName => _ctx.Program?.Name;
        public string  CurrentStepDescription => _ctx.Progress.CurrentStepDescription;

        public ProgramExecutor(
            RobotController controller, ProgramCycleManager programManager,
            PointRepository pointRepo, ToolRepository toolRepo, LocalRepository localRepo,
            BuiltProgramRepository builtProgramRepo, GridRepository gridRepo, StackRepository stackRepo,
            bool isBackground = false, GlobalVariableStore? globalVars = null,
            GlobalImageStore? globalImages = null, BackgroundProgramManager? backgroundManager = null)
        {
            _isBackground = isBackground;
            _vars = new VariableScope(globalVars, globalImages, io => AddIoVariables(controller, io));
            _ctx  = new ExecutionContext(
                controller, programManager, pointRepo, localRepo, builtProgramRepo, gridRepo, stackRepo,
                _vars, isBackground, backgroundManager, Finish, PauseInPlace);
        }

        // ── Monitor display ──────────────────────────────────────────────────

        /// <summary>Returns current values for all scalar variables flagged DisplayOnMonitor.</summary>
        public IReadOnlyList<(string Name, double Value, bool IsBoolean)> GetDisplayVariables()
        {
            // Called from WebSocket threads; the loop thread mutates the variable scope under this lock.
            lock (_controlLock) return _vars.GetDisplayVariables(Program);
        }

        /// <summary>
        /// Names and write-counts of image variables flagged DisplayOnMonitor.
        /// </summary>
        /// <remarks>
        /// The bytes are not included on purpose. This rides along with the variable poll,
        /// which the monitor runs several times a second; a base64 camera frame is a few
        /// hundred kilobytes, so inlining one would turn a cheap poll into a steady
        /// megabyte-a-second stream of a picture that usually has not changed. The revision
        /// is enough for the monitor to notice a change and ask for the image itself.
        /// </remarks>
        public IReadOnlyList<(string Name, long Revision)> GetDisplayImages()
        {
            lock (_controlLock) return _vars.GetDisplayImages(Program);
        }

        /// <summary>
        /// The base64 bytes of one DisplayOnMonitor image variable, or "" if there is no
        /// such variable or nothing has been written to it.
        /// </summary>
        /// <remarks>
        /// Gated on the same two flags as <see cref="GetDisplayImages"/> rather than reading
        /// any image variable by name, so what can be fetched is exactly what was listed —
        /// a program that captures a frame for its own use does not publish it by accident.
        /// </remarks>
        public string GetDisplayImage(string name)
        {
            lock (_controlLock) return _vars.GetDisplayImage(Program, name);
        }

        // ── Public control ───────────────────────────────────────────────────

        // Start/Stop/Resume/Reset are called from WebSocket threads (and Stop from the
        // motion thread on a joint-limit fault) while Update() runs on the ProgramLoop
        // thread. Each public entry point holds _controlLock for its whole body — the
        // same lock Update() holds — so callers get synchronous semantics: when Stop()
        // or Reset() returns, the next tick sees the stopped state, and IsRunning is
        // already true when Start() returns (BackgroundProgramManager and
        // WaitForBackground rely on that).
        public void Start(BuiltProgram program, string? imageBase64 = null)
        {
            lock (_controlLock) StartCore(program, imageBase64);
        }

        public void Resume()
        {
            lock (_controlLock) ResumeCore();
        }

        public void Stop()
        {
            lock (_controlLock) StopCore();
        }

        /// <summary>
        /// Immediately halts execution and clears all state — no status update is emitted.
        /// The caller is responsible for pushing a final status (e.g. Ready) to programManager.
        /// </summary>
        public void Reset()
        {
            lock (_controlLock) ResetCore();
        }

        private void StartCore(BuiltProgram program, string? imageBase64)
        {
            // Pulses left over from a run that completed normally finish now rather than
            // being dropped (which would leave the output stuck in its pulsed state).
            if (!_isPaused) FireAllOutputFlips();

            // Tear down whatever the previous run left behind (vision processor,
            // webhook subscription, pending async work) before anything else.
            ResetRunState();
            // Discard any motion error latched before this run started.
            _ctx.Controller.ConsumeMotionError();

            _ctx.Progress.Starting(program, imageBase64, CountSteps(program.Steps));

            _ctx.Frames.Push(StepListFrame.Plain(program.Steps));

            // Initialise variables from the program definition
            _vars.Clear();

            // Set before initialising, because a variable whose initial value is an
            // expression can now fail here — and reporting that failure needs the program.
            _ctx.Program              = program;
            _ctx.Progress.ProgramName = program.Name;
            try
            {
                _vars.Initialize(program);
            }
            catch (UnknownVariableException ex)
            {
                Finish(ProgramStatus.Error,
                    $"Unknown variable '${ex.VariableName}' in a variable's initial value");
                return;
            }

            // Programs start in the robot's active local (set from the jog page);
            // SetLocal / ClearLocal steps override it during the run.
            _ctx.ActiveLocal = _ctx.Controller.ActiveLocalOffset;
            _running         = true;
        }

        private void ResumeCore()
        {
            if (!_isPaused) return;
            _isPaused = false;
            // Any interrupted motion is re-dispatched by Update() on the control
            // loop thread — Resume() runs on the WebSocket thread, where calling
            // StartContinuousMove would race the motion thread. Armed before
            // _running so the first tick sees it ahead of any step execution.
            _ctx.Motion.ArmResume();
            // A vision wait spanning the pause needs a result from after the resume
            // (the part may have moved), and its timeout restarts from here.
            if (_ctx.Vision.Awaiting) _ctx.Vision.StartMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            _running = true;

            _ctx.Progress.Resuming();
        }

        private void StopCore()
        {
            if (!_running) return;

            // Background executors stop dead — a paused background program would
            // never signal OnExecutorFinished and WaitForBackground would hang.
            // Finish() tears down every wait (move, aux, vision, background, webhook).
            if (_isBackground)
            {
                Finish(ProgramStatus.Stopped, "Stopped by user");
                return;
            }

            // Main program: halt motion but PAUSE — the frame stack (including a
            // CNC block's generated steps and its captured origin anchor) stays
            // intact so Continue resumes from exactly where the program stopped.

            // Request a queue drain on the control loop thread — calling Clear() directly
            // here would race with RunCommands() which reads QueuedCommands on the loop thread.
            _ctx.Controller.RequestQueueDrain();
            // Hard-stop any active motion profiler so IsMoving clears immediately.
            _ctx.Controller.HardStop();

            // Snapshot the interrupted motion for Resume() and drop the move wait.
            _ctx.Motion.CaptureResumeSnapshot();

            _vars.SavePersistent();
            _running  = false;
            _isPaused = true;
            _ctx.Progress.StoppedByUser();
        }

        private void ResetCore()
        {
            ResetRunState();
            _vars.Clear();
        }

        /// <summary>A PauseProgram step: stop ticking but keep the frame stack for Continue.</summary>
        private void PauseInPlace()
        {
            _running  = false;
            _isPaused = true;
            _ctx.Motion.ClearWaitsForPause();
            _ctx.Progress.Paused();
        }

        /// <summary>
        /// Resets every per-run field to its idle value and releases anything a run may
        /// still hold (vision processor, webhook subscription, pending async work, output
        /// pulses). Called from Start(), Reset() and Finish(). Variables are not touched.
        /// </summary>
        private void ResetRunState()
        {
            _running  = false;
            _isPaused = false;
            _ctx.ResetRunState();
        }

        /// <summary>Fires the reverting flip of every output pulse that is due (all of them
        /// when <paramref name="all"/>), in the order they were scheduled.</summary>
        private void ProcessOutputFlips(bool all = false)
        {
            var flips = _ctx.OutputFlips;
            if (flips.Count == 0) return;
            long now = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            for (int i = 0; i < flips.Count;)
            {
                var (due, flip) = flips[i];
                if (!all && due > now) { i++; continue; }
                flips.RemoveAt(i);
                try { flip(); }
                catch (Exception ex) { Console.WriteLine($"[ProgramExecutor] Output pulse revert failed: {ex}"); }
            }
        }

        private void FireAllOutputFlips() => ProcessOutputFlips(all: true);

        // ── Main update — called every control loop tick ──────────────────────

        public void Update()
        {
            lock (_controlLock)
            {
                // One variable snapshot per tick (rebuilt only after a write) instead of one
                // per evaluated field — see EvalContext.
                _vars.Eval.BeginTick();
                try { UpdateCore(); }
                finally { _vars.Eval.EndTick(); }
            }
        }

        private void UpdateCore()
        {
            // Revert due output pulses. Runs even after a normal completion (Finish keeps
            // a Complete run's pending pulses) but not while paused: a Stop/e-stop holds
            // the pulsed state rather than flipping outputs behind the operator's back.
            if (!_isPaused) ProcessOutputFlips();

            if (!_running || Program is null) return;

            // Drain variable writes queued by fire-and-continue JsonExchange completions.
            while (_ctx.PendingActions.TryDequeue(out var action)) action();

            var motion = _ctx.Motion;

            // Re-dispatch motion interrupted by a pause — done here so it runs on
            // the control loop thread, before any step execution can advance.
            if (motion.ResumeDispatchPending) motion.DispatchResume();

            // Blocking on a background program finishing
            if (_ctx.WaitingForBackground != null)
            {
                var bg = _ctx.Background;
                if (bg == null || !bg.IsRunning(_ctx.WaitingForBackground))
                    _ctx.WaitingForBackground = null; // done or never started — advance
                else
                    return;
            }

            // Refresh stopwatch variable values so expressions always see the current elapsed time
            _vars.RefreshStopwatches();

            // If we dispatched a robot move, wait until the queue is clear and the robot is idle
            if (motion.AwaitingMove)
            {
                if (!motion.MoveFinished) return;

                motion.AwaitingMove = false;
                // A move the motion thread could not execute also leaves the queue
                // empty and the robot idle — never mistake it for a completed one.
                var motionError = _ctx.Controller.ConsumeMotionError();
                if (motionError != null)
                {
                    Finish(ProgramStatus.Error, motionError);
                    return;
                }
                // Report the step as completed now that the move has finished
                if (motion.PendingStep is { } pending)
                {
                    _ctx.Progress.StepCompleted(pending);
                    motion.PendingStep = null;
                }
                // Report every step consumed by a blended run
                if (motion.PendingSteps is { } run)
                {
                    foreach (var s in run) _ctx.Progress.StepCompleted(s);
                    motion.PendingSteps = null;
                }
            }

            // If we dispatched an aux indexed move with WaitForDone=true, block until it completes
            if (motion.AwaitingAux)
            {
                if (_ctx.Controller.IsAuxMoving) return;

                motion.AwaitingAux = false;
                Diag.AuxWaitDone();
                if (motion.PendingAuxStep is { } auxStep)
                {
                    _ctx.Progress.StepCompleted(auxStep);
                    motion.PendingAuxStep = null;
                }
            }

            // Nothing left to execute?
            if (_ctx.Frames.Count == 0)
            {
                Finish(ProgramStatus.Complete, "Complete");
                return;
            }

            var frame = _ctx.Frames.Peek();

            // Frame exhausted — pop and continue
            if (frame.Index >= frame.Steps.Count)
            {
                PopExhaustedFrame(frame);
                return; // Re-enter next tick with updated stack
            }

            var step = frame.Steps[frame.Index];
            if (Diag.Enabled) Diag.StepStart(frame.Index, step.Type);
            long execTs = Diag.Enabled ? Diag.Now() : 0;
            ExecuteStep(step, frame);
            if (Diag.Enabled) Diag.StepExec(step.Type, frame.Index, execTs);
        }

        /// <summary>
        /// Pops a finished step list and does what its kind asks: a loop re-pushes a fresh
        /// frame for its next pass, a CNC block clears its toolpath preview. The frame stack
        /// keeps the loop depth in step with the pop and push.
        /// </summary>
        private void PopExhaustedFrame(StepListFrame frame)
        {
            var frames = _ctx.Frames;
            frames.Pop();

            switch (frame.Kind)
            {
                case FrameKind.Cnc:
                    // CNC block finished — its toolpath preview is no longer active
                    _ctx.Controller.ActiveCncToolpath = null;
                    break;

                case FrameKind.WhileLoop:
                    // Re-push only if the condition still holds (a condition error has
                    // already finished the program)
                    if (_ctx.EvalWhileCondition(frame.WhileCondition!))
                        frames.Push(frame.NextPass());
                    break;

                case FrameKind.ForEach:
                    frame.ForEachCurrentIndex++;
                    if (frame.ForEachCurrentIndex < frame.ForEachCount)
                    {
                        var next = frame.NextPass();
                        frames.Push(next);
                        _ctx.InjectForEachVars(next);
                    }
                    break;

                case FrameKind.CountLoop:
                    frame.LoopRemaining--;
                    if (frame.LoopRemaining != 0)
                    {
                        // Increment count-loop index variable if configured
                        if (!string.IsNullOrEmpty(frame.IndexVar))
                            _vars.Set(frame.IndexVar, frame.LoopTotal - frame.LoopRemaining); // iteration number (0-based)
                        frames.Push(frame.NextPass());
                    }
                    break;

                case FrameKind.Plain:
                    break;
            }
        }

        // ── Step execution ────────────────────────────────────────────────────

        private void ExecuteStep(ProgramStep step, StepListFrame frame)
        {
            try
            {
                ExecuteStepInner(step, frame);
            }
            catch (UnknownVariableException ex)
            {
                // A typo'd variable in any step field or condition stops the program with
                // a clear error instead of silently evaluating to 0 and moving the robot.
                Finish(ProgramStatus.Error, $"Unknown variable '${ex.VariableName}' in step: {ProgressReporter.StepDescription(step)}");
            }
            catch (Exception ex)
            {
                // Anything else a step throws (device I/O, a bad repository entry, a bug)
                // errors the program cleanly instead of escaping to the ProgramLoop, which
                // would log it and re-run the same step on every tick.
                var desc = ProgressReporter.StepDescription(step);
                Console.WriteLine($"[ProgramExecutor] Step '{desc}' threw: {ex}");
                Finish(ProgramStatus.Error, $"{desc}: {ex.Message}");
            }
        }

        private void ExecuteStepInner(ProgramStep step, StepListFrame frame)
        {
            // Background programs skip motion/tool/homing steps rather than error
            if (_isBackground && IsRestrictedInBackground(step.Type))
            {
                _ctx.Progress.SkippedInBackground(step);
                _ctx.CompleteNow(step, frame);
                return;
            }

            if (!StepHandlers.TryGet(step.Type, out var handler))
            {
                Finish(ProgramStatus.Error, $"Unsupported step type {step.Type}");
                return;
            }

            if (handler.Execute(step, frame, _ctx) == StepOutcome.Advance)
                _ctx.CompleteNow(step, frame);
        }

        private static bool IsRestrictedInBackground(StepType type) => type switch
        {
            StepType.MoveL or StepType.MoveJ or StepType.JumpL or StepType.JumpJ => true,
            StepType.SetTool or StepType.SetSpeedL or StepType.SetSpeedJ => true,
            StepType.SetLocal or StepType.ClearLocal or StepType.RunHoming or StepType.ThreadMove => true,
            _ => false,
        };

        private void Finish(ProgramStatus status, string description)
        {
            _vars.SavePersistent();
            int finalStepIndex = _ctx.Progress.GlobalStepIndex;

            // A pulse still pending when the program completes normally is part of its
            // last step, so it is kept (the main executor keeps ticking and reverts it on
            // time; a background executor stops ticking, so it reverts now). Any other
            // ending cancels it along with the rest of the run state.
            List<(long DueMs, Action Flip)>? keptFlips = null;
            if (status == ProgramStatus.Complete && _ctx.OutputFlips.Count > 0)
            {
                if (_isBackground) FireAllOutputFlips();
                else keptFlips = new(_ctx.OutputFlips);
            }

            ResetRunState();
            if (keptFlips != null) _ctx.OutputFlips.AddRange(keptFlips);

            // Main program finishing: optionally kill all background programs
            if (!_isBackground && (Program?.KillBackgroundOnStop ?? true))
                _ctx.Background?.StopAll();

            // Notify manager so it removes this executor from the running set (keyed by ID)
            _ctx.Background?.OnExecutorFinished(Program?.Id ?? "");

            _ctx.Progress.Finished(status, description, finalStepIndex);
        }

        /// <summary>Writes live IO values (stb.inN/outN, relay.N, nano.name.pin) into <paramref name="io"/>.</summary>
        private static void AddIoVariables(RobotController controller, Dictionary<string, double> io)
        {
            io["stb.in1"]  = controller.stb.Input1  ? 1.0 : 0.0;
            io["stb.in2"]  = controller.stb.Input2  ? 1.0 : 0.0;
            io["stb.in3"]  = controller.stb.Input3  ? 1.0 : 0.0;
            io["stb.in4"]  = controller.stb.Input4  ? 1.0 : 0.0;
            io["stb.out1"] = controller.stb.Output1 ? 1.0 : 0.0;
            io["stb.out2"] = controller.stb.Output2 ? 1.0 : 0.0;
            io["stb.out3"] = controller.stb.Output3 ? 1.0 : 0.0;
            io["stb.out4"] = controller.stb.Output4 ? 1.0 : 0.0;

            var relays = controller.RelayManager.GetRelayStates();
            if (relays != null)
                for (int ri = 0; ri < relays.Length && ri < 4; ri++)
                    io[$"relay.{ri + 1}"] = relays[ri] ? 1.0 : 0.0;

            foreach (var nano in controller.NanoManager.GetAllStates())
                if (!string.IsNullOrEmpty(nano.Name))
                    foreach (var pin in nano.Pins)
                        if (!string.IsNullOrEmpty(pin.Name))
                            io[$"nano.{nano.Name}.{pin.Name}"] = pin.Value ? 1.0 : 0.0;
        }

        // ── Static API kept for callers and tests ─────────────────────────────

        internal static int CountSteps(List<ProgramStep> steps, BuiltProgramRepository? repo = null) =>
            ProgressReporter.CountSteps(steps, repo);

        internal static List<ProgramStep> GenerateCncSteps(CncSpec spec, Vector6? anchor = null,
            Func<string, double, double>? resolve = null) =>
            CncStepGenerator.Generate(spec, anchor, resolve);

        internal static int FindResumeIndex(Vector6 pos, Vector6 start, List<Vector6> waypoints) =>
            MoveTargetResolver.FindResumeIndex(pos, start, waypoints);

        internal static bool TryParsePointsRef(string expr, out string name, out string indexExpr) =>
            MoveTargetResolver.TryParsePointsRef(expr, out name, out indexExpr);

        internal static object ListToJson(ListVar lv) => JsonVariableCodec.ListToJson(lv);

        internal static double ScalarFromJson(JsonElement el) => JsonVariableCodec.ScalarFromJson(el);

        internal static ListVar ListFromJson(JsonElement arr, ListElementType type) =>
            JsonVariableCodec.ListFromJson(arr, type);
    }
}
