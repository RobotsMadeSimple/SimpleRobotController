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
        private readonly ProgramCycleManager _programManager;

        // ── Debugger (docs/program-language.md §4.2 "Debugging") ─────────────
        // Breakpoints pause BEFORE a step; "pause at next step" is step mode and the Step
        // command. A step is "fresh" the first tick it is dispatched — multi-tick steps (Wait,
        // a Jump's legs, vision) are re-dispatched every tick and must not pause mid-way, so
        // the last dispatched (step, frame, index) is remembered. Background executors never
        // pause for the debugger.
        private readonly HashSet<string> _breakpoints = new(StringComparer.Ordinal);
        private bool           _pauseAtNextStep;
        private ProgramStep?   _lastStep;
        private StepListFrame? _lastFrame;
        private int            _lastIndex = -1;
        private int            _lastCatchGeneration;

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
            _isBackground   = isBackground;
            _programManager = programManager;
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

        // ── Expression tools (EvaluateExpression / GetExpressionSymbols) ─────

        /// <summary>
        /// Evaluates <paramref name="expr"/> against this executor's live variables, IO and
        /// properties — what a step evaluating it right now would see. Throws like
        /// <see cref="ExpressionEvaluator.Evaluate"/>. Goes through the scope's
        /// <see cref="Execution.EvalContext"/> so the text helpers (<c>$s.length</c>,
        /// <c>$s.contains("x")</c> …) resolve string variables exactly as a running step does.
        /// Ticks run under the same lock, so this never sees a tick's cached snapshot.
        /// </summary>
        internal double EvaluateLive(string expr)
        {
            lock (_controlLock)
                return _vars.Eval.Evaluate(expr);
        }

        /// <summary>Current values of every variable in scope: number, string, or a list's element count.</summary>
        internal Dictionary<string, object?> SnapshotLiveValues()
        {
            lock (_controlLock) return _vars.SnapshotValues();
        }

        // ── Plugin variables.get / variables.set (docs/plugins.md §4.3) ──────

        /// <summary>Scalars (computed ones evaluated), lists as JSON-ready values, and strings.</summary>
        internal Plugins.VariablesSnapshot PluginSnapshot()
        {
            lock (_controlLock) return _vars.PluginSnapshot();
        }

        /// <summary>
        /// Validates every value, then applies the writes: queued onto the loop thread
        /// (<see cref="ExecutionContext.PendingActions"/>, dropped if the run ends first) while
        /// running, directly under the control lock otherwise. Returns <c>computedVariable</c>
        /// or <c>badValue</c> (nothing is written then), or null.
        /// </summary>
        internal string? PluginSetVariables(Dictionary<string, JsonElement> values)
        {
            lock (_controlLock)
            {
                var writes = new List<Action<VariableScope>>();
                foreach (var (name, value) in values)
                {
                    if (string.IsNullOrWhiteSpace(name)) return "badValue";
                    if (_vars.IsComputed(name)) return "computedVariable";
                    if (PluginVariables.Write(name, value) is not { } write) return "badValue";
                    writes.Add(write);
                }
                if (!_running)
                {
                    foreach (var w in writes) w(_vars);
                    return null;
                }
                int gen = _ctx.RunGeneration;
                foreach (var w in writes)
                    _ctx.PendingActions.Enqueue(() =>
                    {
                        if (gen != _ctx.RunGeneration) return;
                        try { w(_vars); }
                        catch (Exception ex) { Console.WriteLine($"[ProgramExecutor] Plugin variable write failed: {ex.Message}"); }
                    });
                return null;
            }
        }

        // ── Public control ───────────────────────────────────────────────────

        // Start/Stop/Resume/Reset are called from WebSocket threads (and Stop from the
        // motion thread on a joint-limit fault) while Update() runs on the ProgramLoop
        // thread. Each public entry point holds _controlLock for its whole body — the
        // same lock Update() holds — so callers get synchronous semantics: when Stop()
        // or Reset() returns, the next tick sees the stopped state, and IsRunning is
        // already true when Start() returns (BackgroundProgramManager and
        // WaitForBackground rely on that).
        public void Start(BuiltProgram program, string? imageBase64 = null,
                          IReadOnlyDictionary<string, JsonElement>? inputs = null,
                          DebugOptions? debug = null)
        {
            lock (_controlLock) StartCore(program, imageBase64, inputs, debug);
        }

        /// <summary>Continue: resumes a paused program and leaves step mode (it then runs to the
        /// next breakpoint, pause step or the end).</summary>
        public void Resume()
        {
            lock (_controlLock)
            {
                _pauseAtNextStep = false;
                ResumeCore();
            }
        }

        // ── Debugger ─────────────────────────────────────────────────────────

        /// <summary>The ExecuteBuiltProgram debug options.</summary>
        public sealed record DebugOptions(string? StartAtStepId, IReadOnlyCollection<string>? Breakpoints, bool StepMode);

        /// <summary>
        /// StepBuiltProgram: a paused program runs its next step and pauses again before the one
        /// after (pauseReason "step"); a running one pauses before its next step. False when no
        /// program is running or paused.
        /// </summary>
        public bool StepOnce()
        {
            lock (_controlLock)
            {
                if (_isPaused)
                {
                    _pauseAtNextStep = true;
                    ResumeCore();
                    return true;
                }
                if (!_running) return false;
                _pauseAtNextStep = true;
                return true;
            }
        }

        /// <summary>SetBreakpoints: replaces the breakpoints (step ids) of this executor's run.</summary>
        public int SetBreakpoints(IEnumerable<string>? stepIds)
        {
            lock (_controlLock)
            {
                _breakpoints.Clear();
                foreach (var id in stepIds ?? [])
                    if (!string.IsNullOrWhiteSpace(id)) _breakpoints.Add(id);
                return _breakpoints.Count;
            }
        }

        /// <summary>The current breakpoints (a copy).</summary>
        public IReadOnlyCollection<string> Breakpoints
        {
            get { lock (_controlLock) return _breakpoints.ToList(); }
        }

        /// <summary>
        /// Where <paramref name="stepId"/> sits in <paramref name="steps"/> (routine bodies are not
        /// searched): each enclosing step with the body that holds the next level, then the step's
        /// index in its own list. False when the id is not found.
        /// </summary>
        internal static bool TryFindStepPath(List<ProgramStep> steps, string stepId,
            out List<(ProgramStep Parent, int IndexInList, StepChildList Body)> ancestors, out int index)
        {
            ancestors = new();
            index = -1;
            return Find(steps, stepId, ancestors, ref index);

            static bool Find(List<ProgramStep> list, string id,
                             List<(ProgramStep, int, StepChildList)> path, ref int found)
            {
                for (int i = 0; i < list.Count; i++)
                {
                    var s = list[i];
                    if (s == null) continue;
                    if (s.Id == id) { found = i; return true; }
                    foreach (var child in s.ChildLists(executableOnly: true))
                    {
                        path.Add((s, i, child));
                        if (Find(child.Steps, id, path, ref found)) return true;
                        path.RemoveAt(path.Count - 1);
                    }
                }
                return false;
            }
        }

        /// <summary>
        /// Why the program cannot start at <paramref name="stepId"/>, or null when it can: the
        /// step must be in the program itself (not a routine), and not inside a disabled step or
        /// a CNC block's generated steps.
        /// </summary>
        internal static string? CheckStartAt(BuiltProgram program, string stepId)
        {
            if (!TryFindStepPath(program.Steps, stepId, out var ancestors, out _))
                return $"step '{stepId}' is not in program '{program.Name}' (a routine's steps cannot be started directly)";
            foreach (var (parent, _, body) in ancestors)
            {
                if (!parent.IsEnabled) return $"step '{stepId}' is inside a disabled step";
                if (body.Role == StepTree.Roles.CncProgramSteps) return $"step '{stepId}' is inside a CNC block";
            }
            return null;
        }

        /// <summary>
        /// Replaces the frame stack with one that runs from <paramref name="stepId"/>: each
        /// enclosing block is entered without being evaluated — a loop at its first pass (count
        /// index 0, for-each element 0, a while condition not checked), an If / Switch branch or a
        /// Try / catch body directly. Returns an error text, or null.
        /// </summary>
        private string? BuildStartStack(BuiltProgram program, string stepId)
        {
            if (CheckStartAt(program, stepId) is { } why) return why;
            TryFindStepPath(program.Steps, stepId, out var ancestors, out int index);

            var frames = _ctx.Frames;
            frames.Clear();
            var root = StepListFrame.Plain(program.Steps);
            root.Index = ancestors.Count > 0 ? ancestors[0].IndexInList + 1 : index;
            frames.Push(root);

            for (int k = 0; k < ancestors.Count; k++)
            {
                var (parent, _, body) = ancestors[k];
                StepListFrame f;
                switch (body.Role)
                {
                    case StepTree.Roles.LoopSteps:
                        if (parent.LoopMode == "while" && parent.LoopWhileCondition != null)
                            f = StepListFrame.WhileLoop(body.Steps, parent.LoopWhileCondition);
                        else if (parent.LoopMode == "forEach" && !string.IsNullOrEmpty(parent.ForEachVariableName))
                        {
                            int n = _vars.Lists.TryGetValue(parent.ForEachVariableName, out var lst) ? lst.Count : 0;
                            if (n == 0) return $"the for-each list '${parent.ForEachVariableName}' around step '{stepId}' is empty";
                            f = StepListFrame.ForEach(body.Steps, n, 0, parent.ForEachVariableName,
                                parent.ForEachValueVariableName ?? "", parent.ForEachIndexVariableName ?? "");
                        }
                        else
                        {
                            int count = (int)_ctx.Eval.EvalField(parent, "loopCount", parent.LoopCount ?? 1);
                            int remaining = count == 0 ? int.MaxValue : count;
                            string indexVar = parent.ForEachIndexVariableName ?? "";
                            if (!string.IsNullOrEmpty(indexVar)) _vars.Set(indexVar, 0);
                            f = StepListFrame.CountLoop(body.Steps, remaining, remaining, indexVar);
                        }
                        break;
                    case StepTree.Roles.SwitchCases:
                    case StepTree.Roles.SwitchDefault:
                        f = StepListFrame.SwitchBody(body.Steps);
                        break;
                    case StepTree.Roles.TrySteps:
                        f = StepListFrame.TryBody(parent);
                        break;
                    default: // if / else-if / else branches, catch steps
                        f = StepListFrame.Plain(body.Steps);
                        break;
                }
                f.Index = k + 1 < ancestors.Count ? ancestors[k + 1].IndexInList + 1 : index;
                frames.Push(f);
                _ctx.InjectForEachVars(f);
            }
            return null;
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

        private void StartCore(BuiltProgram program, string? imageBase64,
                               IReadOnlyDictionary<string, JsonElement>? inputs = null,
                               DebugOptions? debug = null)
        {
            // Debug state belongs to one run: a plain start clears it.
            _breakpoints.Clear();
            _pauseAtNextStep = false;
            ForgetLastDispatch();
            if (debug != null && !_isBackground)
            {
                foreach (var id in debug.Breakpoints ?? [])
                    if (!string.IsNullOrWhiteSpace(id)) _breakpoints.Add(id);
                _pauseAtNextStep = debug.StepMode;
            }

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
                _vars.SeedInputs(program, inputs); // launch-time input overrides win over defaults
            }
            catch (UnknownVariableException ex)
            {
                Finish(ProgramStatus.Error,
                    $"Unknown variable '${ex.VariableName}' in a variable's initial value");
                return;
            }
            catch (ExpressionParseException ex)
            {
                Finish(ProgramStatus.Error,
                    $"Expression error in a variable's initial value: {ProgressReporter.DescribeParseError(ex)}");
                return;
            }
            catch (ComputedVariableWriteException ex)
            {
                Finish(ProgramStatus.Error,
                    $"'${ex.VariableName}' is declared both as a computed variable and as a stored one");
                return;
            }

            // Run from a step: the stack is built down its path once the variables exist (a
            // count loop's index variable is written, a for-each reads its list).
            if (!string.IsNullOrEmpty(debug?.StartAtStepId))
            {
                string? error;
                try { error = BuildStartStack(program, debug!.StartAtStepId!); }
                catch (UnknownVariableException ex) { error = $"unknown variable '${ex.VariableName}' in an enclosing loop count"; }
                catch (ExpressionParseException ex) { error = ProgressReporter.DescribeParseError(ex); }
                if (error != null)
                {
                    Finish(ProgramStatus.Error, $"Cannot run from this step: {error}");
                    return;
                }
            }

            // Programs start in the robot's active local (set from the jog page);
            // SetLocal / ClearLocal steps override it during the run.
            _ctx.ActiveLocal = _ctx.Controller.ActiveLocalOffset;
            _running         = true;
        }

        private void ForgetLastDispatch()
        {
            _lastStep  = null;
            _lastFrame = null;
            _lastIndex = -1;
        }

        /// <summary>
        /// The debugger's check before a step runs: pauses (and returns true) at a breakpoint or
        /// in step mode, the first tick a step is dispatched. Notes and disabled steps never
        /// pause; background programs never do.
        /// </summary>
        private bool PauseBeforeStep(ProgramStep step, StepListFrame frame)
        {
            bool fresh = !ReferenceEquals(step, _lastStep) || !ReferenceEquals(frame, _lastFrame)
                         || frame.Index != _lastIndex || _ctx.CatchGeneration != _lastCatchGeneration;
            _lastStep  = step;
            _lastFrame = frame;
            _lastIndex = frame.Index;
            _lastCatchGeneration = _ctx.CatchGeneration;
            if (!fresh || _isBackground || step.Type == StepType.Note || !step.IsEnabled) return false;

            string? reason = _pauseAtNextStep ? PauseReasons.Step
                           : _breakpoints.Contains(step.Id) ? PauseReasons.Breakpoint
                           : null;
            if (reason == null) return false;

            // Paused before the step: Continue / Step dispatch it next tick, and because it is
            // the step remembered above, it does not pause again.
            _pauseAtNextStep = false;
            _running  = false;
            _isPaused = true;
            _ctx.Progress.Paused();
            _programManager.SetPauseState(_ctx.Progress.ProgramName, reason, step.Id);
            return true;
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

            // An outstanding plugin step is cancelled (step.cancel reason "stopped"; a late
            // reply is discarded) and re-sent when Continue reaches it again.
            _ctx.PluginState.Reset("stopped");

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
        /// <param name="pluginCancelReason">The <c>step.cancel</c> reason for an outstanding plugin step.</param>
        private void ResetRunState(string pluginCancelReason = "reset")
        {
            _running  = false;
            _isPaused = false;
            // A recording the run started but never stopped (an error, a reset after Stop, or
            // no StopRecording step) ends with the run; it stays in the Recordings browser. A
            // pause keeps it going, so a resumed test is still one recording.
            foreach (var (_, rec) in _ctx.ProgramRecordings)
                _ctx.Controller.IoHistory.StopRecording(rec.RecordingId);
            _ctx.ProgramRecordings.Clear();
            _ctx.ResetRunState(pluginCancelReason);
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
                catch (ComputedVariableWriteException ex)
                {
                    // Writes outside step dispatch — loop index/forEach variables as a loop
                    // re-enters, queued HTTP inbound mappings — end the program the same way.
                    Finish(ProgramStatus.Error, $"Cannot assign computed variable '${ex.VariableName}'");
                }
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
                // The move's stop condition, evaluated every tick it is in flight (the
                // snapshot is rebuilt per tick, so each evaluation sees fresh IO).
                if (motion.Watch is { Fired: false, IsAux: false } && !_ctx.TickMoveWatch()) return;

                if (!motion.MoveFinished) return;

                motion.AwaitingMove = false;
                // A move the motion thread could not execute also leaves the queue
                // empty and the robot idle — never mistake it for a completed one.
                var motionError = _ctx.Controller.ConsumeMotionError();
                if (motionError != null)
                {
                    _ctx.Progress.Failing(motion.PendingStep ?? motion.PendingSteps?.FirstOrDefault(), Programs.ErrorCodes.MotionFault);
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
                if (motion.Watch is { IsAux: false } watch) CompleteRobotMoveWatch(watch);
            }

            // If we dispatched an aux indexed move with WaitForDone=true, block until it completes
            if (motion.AwaitingAux)
            {
                if (motion.Watch is { Fired: false, IsAux: true } && !_ctx.TickMoveWatch()) return;

                if (_ctx.Controller.IsAuxMoving) return;

                motion.AwaitingAux = false;
                Diag.AuxWaitDone();
                if (motion.PendingAuxStep is { } auxStep)
                {
                    _ctx.Progress.StepCompleted(auxStep);
                    motion.PendingAuxStep = null;
                }
                if (motion.Watch is { IsAux: true } watch) _ctx.CompleteMoveWatch(watch);
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

            // A Try retry waiting out its delay.
            if (frame.ResumeAtMs > 0)
            {
                if (DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() < frame.ResumeAtMs) return;
                frame.ResumeAtMs = 0;
            }

            var step = frame.Steps[frame.Index];
            if (PauseBeforeStep(step, frame)) return;
            if (Diag.Enabled) Diag.StepStart(frame.Index, step.Type);
            long execTs = Diag.Enabled ? Diag.Now() : 0;
            ExecuteStep(step, frame);
            if (Diag.Enabled) Diag.StepExec(step.Type, frame.Index, execTs);
        }

        /// <summary>
        /// A robot move leg finished with a stop condition armed. A single move (or a blended
        /// Jump) is done: write the flag and clear. A stepped Jump still has legs to dispatch
        /// (<see cref="JumpState.SubStep"/> 1 or 2): a watch that fired in stop mode ends the
        /// Jump here — the remaining legs are skipped and the step is completed — while an
        /// unfired or record-mode watch stays armed across the next leg.
        /// </summary>
        private void CompleteRobotMoveWatch(MoveWatch watch)
        {
            var jump = _ctx.Jump;
            bool legsRemain = jump.SubStep is 1 or 2;
            if (!legsRemain)
            {
                _ctx.CompleteMoveWatch(watch);
                return;
            }
            if (!watch.Fired || watch.Mode != MoveStopMode.Stop) return;

            jump.Reset();
            _ctx.CompleteMoveWatch(watch);
            // The stepped Jump never advanced the frame (it re-runs its handler per leg), so
            // the frame index still points at the step: complete it so the next step runs.
            if (_ctx.Frames.Count > 0)
            {
                var frame = _ctx.Frames.Peek();
                if (frame.Index < frame.Steps.Count && ReferenceEquals(frame.Steps[frame.Index], watch.Step))
                    _ctx.CompleteNow(watch.Step, frame);
            }
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

                case FrameKind.Routine:
                    // The routine ran to its end: its out / inOut parameters go back to the caller.
                    _ctx.LeaveRoutine(frame);
                    break;

                case FrameKind.Plain:
                case FrameKind.Try:     // the body finished without a (caught) failure
                case FrameKind.Switch:
                    break;
            }
        }

        // ── Step execution ────────────────────────────────────────────────────

        private void ExecuteStep(ProgramStep step, StepListFrame frame)
        {
            _ctx.CurrentStep = step;
            try
            {
                ExecuteStepInner(step, frame);
            }
            catch (UnknownVariableException ex)
            {
                _ctx.Progress.Failing(step, Programs.ErrorCodes.UnknownVariable);
                // A typo'd variable in any step field or condition stops the program with
                // a clear error instead of silently evaluating to 0 and moving the robot.
                Finish(ProgramStatus.Error, $"Unknown variable '${ex.VariableName}' in step: {ProgressReporter.StepDescription(step)}");
            }
            catch (ComputedVariableWriteException ex)
            {
                // A computed variable is a formula; writing it would silently do nothing useful.
                _ctx.Progress.Failing(step, Programs.ErrorCodes.ComputedVariable);
                Finish(ProgramStatus.Error,
                    $"Cannot assign computed variable '${ex.VariableName}' in step: {ProgressReporter.StepDescription(step)}");
            }
            catch (PluginFunctionCallException ex)
            {
                // A plugin function failed (not running, timed out, returned an error) — the
                // plugin's message, not a syntax-error framing. A runtime failure, so a Try
                // can catch it (pluginFailed).
                var message = $"{_ctx.Progress.Describe(step)}: {ex.Message}";
                try { _ctx.Fail(Programs.ErrorCodes.PluginFailed, message); }
                catch (Exception inner) { Finish(ProgramStatus.Error, $"{message} ({inner.Message})"); }
            }
            catch (ExpressionParseException ex)
            {
                // A syntax error the validator would have reported — a clear message rather
                // than a stack trace, and never a silent fallback value.
                _ctx.Progress.Failing(step, Programs.ErrorCodes.ExpressionSyntax);
                Finish(ProgramStatus.Error,
                    $"Expression error in step {ProgressReporter.StepDescription(step)}: {ProgressReporter.DescribeParseError(ex)}");
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
            finally
            {
                _ctx.CurrentStep = null;
            }
        }

        private void ExecuteStepInner(ProgramStep step, StepListFrame frame)
        {
            // A disabled step (enabled: false) is skipped with its whole body. A disabled Note is
            // still just a note — nothing to report.
            if (!step.IsEnabled)
            {
                if (step.Type == StepType.Note) { frame.Index++; return; }
                _ctx.Progress.SkippedDisabled(step);
                _ctx.CompleteNow(step, frame);
                return;
            }

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

        /// <summary>Step types a background program skips (motion, tool, homing). The step catalog's
        /// <c>background</c> flag is the negation of this.</summary>
        internal static bool IsRestrictedInBackground(StepType type) => type switch
        {
            StepType.MoveL or StepType.MoveJ or StepType.JumpL or StepType.JumpJ or StepType.MoveC => true,
            StepType.SetTool or StepType.SetSpeedL or StepType.SetSpeedJ or StepType.SetSpeedFactor => true,
            StepType.SetLocal or StepType.ClearLocal or StepType.RunHoming or StepType.ThreadMove => true,
            _ => false,
        };

        private void Finish(ProgramStatus status, string description)
        {
            // An error while a step's handler runs is that step's: the monitor marks it.
            if (status == ProgramStatus.Error && _ctx.CurrentStep is { } failing)
                _ctx.Progress.Failing(failing, null);
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

            ResetRunState(status == ProgramStatus.Stopped ? "stopped" : "reset");
            if (keptFlips != null) _ctx.OutputFlips.AddRange(keptFlips);

            // Main program finishing: optionally kill all background programs
            if (!_isBackground && (Program?.KillBackgroundOnStop ?? true))
                _ctx.Background?.StopAll();

            // Notify manager so it removes this executor from the running set (keyed by ID)
            _ctx.Background?.OnExecutorFinished(Program?.Id ?? "");

            _ctx.Progress.Finished(status, description, finalStepIndex);
        }

        /// <summary>
        /// Writes live IO values (stb.inN/outN, relay.N, io.nano.pin) into <paramref name="io"/>.
        /// Nano pins follow <see cref="IoVariableNames"/>: a load cell reads in grams, and a name
        /// that is not an identifier is also published under its identifier-safe alias.
        /// </summary>
        internal static void AddIoVariables(RobotController controller, Dictionary<string, double> io)
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

            IoVariableNames.AddPins(controller.NanoManager.GetAllStates(), io);
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
