using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace Controller.RobotControl
{
    /// <summary>
    /// Executes a BuiltProgram step-by-step inside the main control loop.
    /// Call Update() on every Loop() tick. Enqueues RobotCommands for moves
    /// and reports progress via ProgramCycleManager.
    /// </summary>
    internal class ProgramExecutor
    {
        // ── Injected dependencies ─────────────────────────────────────────────
        private readonly RobotController          _controller;
        private readonly ProgramCycleManager      _programManager;
        private readonly PointRepository          _pointRepo;
        private readonly ToolRepository           _toolRepo;
        private readonly LocalRepository          _localRepo;
        private readonly BuiltProgramRepository   _builtProgramRepo;
        private readonly GridRepository           _gridRepo;
        private readonly StackRepository          _stackRepo;

        // ── Execution state ──────────────────────────────────────────────────
        private BuiltProgram?   _program;
        private bool            _running;
        // volatile so the control-loop thread always sees writes from the WebSocket thread
        private volatile bool   _stopRequested;
        private volatile bool   _isPaused;

        // Stack for nested step lists (supports Loop)
        private readonly Stack<StepListFrame> _frameStack = new();

        // For Wait steps
        private long _waitStartMs;

        // Whether we have dispatched a move command and are awaiting completion
        private bool _awaitingMove;

        // Step that was dispatched asynchronously; reported complete when the move finishes
        private ProgramStep? _pendingStep;

        // Steps consumed by a single blended (continuous) move; all reported on completion.
        private List<ProgramStep>? _pendingSteps;

        // Program default blend radius (set by SetBlendRadius); per-move BlendRadius overrides it.
        private double _defaultBlendRadius = 0;

        // Whether we are awaiting an aux axis indexed move with WaitForDone=true
        private bool _awaitingAuxMove;
        private ProgramStep? _pendingAuxStep;

        // RunVision: waiting for a fresh inspection result from VisionProcessor
        private bool   _awaitingVision;
        private long   _visionStartMs;
        private string? _visionProgramId;

        // Active local during program execution — null = no local (zero offset)
        private Vector6? _activeLocal;

        // Program variables — initialised from BuiltProgram.Variables on Start(), mutated by SetVariable steps
        private readonly Dictionary<string, double>            _variables        = new();
        private readonly Dictionary<string, List<double>>      _listVariables    = new();
        private readonly Dictionary<string, List<Vector6Val>>  _pointVariables   = new();
        private readonly HashSet<string>                       _booleanVariables = new(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, string>            _stringVariables  = new(StringComparer.OrdinalIgnoreCase);

        // Background execution support
        private readonly bool                    _isBackground;
        private readonly GlobalVariableStore?    _globalVars;
        private readonly BackgroundProgramManager? _backgroundManager;
        private readonly HashSet<string>         _globalVarNames = new(StringComparer.OrdinalIgnoreCase);

        // WaitForBackground: set to a program name while blocking on it
        private string? _waitingForBackground;

        // Persistent variables — names saved here; values written to disk on Finish()
        private readonly HashSet<string> _persistentVarNames = new(StringComparer.OrdinalIgnoreCase);
        private static readonly string   _persistPath        = "persistent_vars.json";
        private static readonly object   _persistLock        = new();

        // Stopwatch state per variable name
        private struct StopwatchEntry { public bool Running; public long AccumMs; public long StartTick; }
        private readonly Dictionary<string, StopwatchEntry> _stopwatches = new(StringComparer.OrdinalIgnoreCase);

        public bool IsRunning => _running;
        public bool IsPaused  => _isPaused;
        public string? CurrentProgramName => _program?.Name;
        public string  CurrentStepDescription { get; private set; } = "";

        /// <summary>Returns current values for all scalar variables flagged DisplayOnMonitor.</summary>
        public IReadOnlyList<(string Name, double Value, bool IsBoolean)> GetDisplayVariables()
        {
            if (_program?.Variables == null) return [];
            var merged = MergedVars();
            var result = new List<(string, double, bool)>();
            foreach (var v in _program.Variables)
            {
                if (v.DisplayOnMonitor != true) continue;
                if (v.Values != null || v.Points != null || v.IsString == true) continue; // non-scalar types not supported in numeric display
                merged.TryGetValue(v.Name, out double val);
                result.Add((v.Name, val, v.IsBoolean == true));
            }
            return result;
        }

        public ProgramExecutor(
            RobotController controller, ProgramCycleManager programManager,
            PointRepository pointRepo, ToolRepository toolRepo, LocalRepository localRepo,
            BuiltProgramRepository builtProgramRepo, GridRepository gridRepo, StackRepository stackRepo,
            bool isBackground = false, GlobalVariableStore? globalVars = null, BackgroundProgramManager? backgroundManager = null)
        {
            _controller        = controller;
            _programManager    = programManager;
            _pointRepo         = pointRepo;
            _toolRepo          = toolRepo;
            _localRepo         = localRepo;
            _builtProgramRepo  = builtProgramRepo;
            _gridRepo          = gridRepo;
            _stackRepo         = stackRepo;
            _isBackground      = isBackground;
            _globalVars        = globalVars;
            _backgroundManager = backgroundManager;
        }

        /// <summary>Returns a merged snapshot of local + global variables for expression evaluation.
        /// Always injects the built-in <c>time_ms</c> variable (current Unix timestamp in milliseconds).</summary>
        private Dictionary<string, double> MergedVars()
        {
            var merged = new Dictionary<string, double>(_variables, StringComparer.OrdinalIgnoreCase);
            if (_globalVars != null)
                foreach (var kv in _globalVars.Snapshot()) merged[kv.Key] = kv.Value;
            merged["time_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            return merged;
        }

        // ── Persistent variable helpers ───────────────────────────────────────

        private static Dictionary<string, double> LoadPersistentVars()
        {
            lock (_persistLock)
            {
                try
                {
                    if (!System.IO.File.Exists(_persistPath)) return new();
                    var json = System.IO.File.ReadAllText(_persistPath);
                    return System.Text.Json.JsonSerializer.Deserialize<Dictionary<string, double>>(json)
                           ?? new();
                }
                catch { return new(); }
            }
        }

        private static void WritePersistentVars(Dictionary<string, double> values)
        {
            lock (_persistLock)
            {
                try
                {
                    var json = System.Text.Json.JsonSerializer.Serialize(values);
                    System.IO.File.WriteAllText(_persistPath, json);
                }
                catch { /* best-effort */ }
            }
        }

        private void SavePersistentVars()
        {
            if (_persistentVarNames.Count == 0) return;
            var existing = LoadPersistentVars();
            var prefix = string.IsNullOrEmpty(_program?.Id) ? "" : _program.Id + ":";
            foreach (var name in _persistentVarNames)
            {
                var val = _globalVarNames.Contains(name) && _globalVars != null && _globalVars.TryGet(name, out var gv)
                    ? gv
                    : _variables.TryGetValue(name, out var v) ? v : 0;
                existing[prefix + name] = val;
            }
            WritePersistentVars(existing);
        }

        // ── Variable write helper — respects global store ─────────────────────

        private void SetVariable(string name, double value)
        {
            if (_globalVarNames.Contains(name) && _globalVars != null)
                _globalVars.Set(name, value);
            else
                _variables[name] = value;
        }

        // ── ForEach helpers ───────────────────────────────────────────────────

        private bool EvalWhileCondition(ConditionGroup condition)
        {
            try
            {
                return EvaluateConditionGroup(condition, EvalVars());
            }
            catch (UnknownVariableException ex)
            {
                // While-loop re-checks run outside the step dispatch, so error here directly.
                Finish(global::ProgramStatus.Error, $"Unknown variable '${ex.VariableName}' in while-loop condition");
                return false; // exit the loop — the program is already finishing with an error
            }
        }

        private void InjectForEachVars(StepListFrame frame)
        {
            if (!frame.IsForEach) return;

            int idx = frame.ForEachCurrentIndex;

            // Write index variable if configured
            if (!string.IsNullOrEmpty(frame.ForEachIndexVar))
                SetVariable(frame.ForEachIndexVar, idx);

            // Write value variable — for point arrays the value is the index itself
            if (!string.IsNullOrEmpty(frame.ForEachValueVar))
            {
                if (_listVariables.TryGetValue(frame.ForEachSourceVar, out var list))
                    SetVariable(frame.ForEachValueVar, idx < list.Count ? list[idx] : 0);
                else
                    SetVariable(frame.ForEachValueVar, idx); // point array or unknown: expose index
            }
        }

        private static bool IsRestrictedInBackground(StepType type) => type switch
        {
            StepType.MoveL or StepType.MoveJ or StepType.JumpL or StepType.JumpJ => true,
            StepType.SetTool or StepType.SetSpeedL or StepType.SetSpeedJ => true,
            StepType.SetLocal or StepType.ClearLocal or StepType.RunHoming or StepType.ThreadMove => true,
            _ => false,
        };

        // ── Public control ───────────────────────────────────────────────────

        public void Start(BuiltProgram program, string? imageBase64 = null)
        {
            // Register program in the cycle manager so the monitor tab can see it
            _programManager.SetAvailablePrograms(new()
            {
                new() { Name = program.Name, Description = program.Description, Image = imageBase64 }
            });

            var totalSteps = CountSteps(program.Steps);
            // Clear any terminal state so the incoming Running update is not blocked by the guard
            _programManager.ResetToReady(program.Name, totalSteps);
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName       = program.Name,
                ProgramStatus     = global::ProgramStatus.Running,
                CurrentStepNumber = 0,
                MaxStepCount      = totalSteps,
                StepDescription   = "Starting…",
            });

            _frameStack.Clear();
            _frameStack.Push(new StepListFrame(program.Steps, 0));

            // Initialise variables from the program definition
            _variables.Clear();
            _listVariables.Clear();
            _pointVariables.Clear();
            _booleanVariables.Clear();
            _stringVariables.Clear();
            _globalVarNames.Clear();
            _persistentVarNames.Clear();
            _stopwatches.Clear();
            _waitingForBackground = null;

            InitializeVariables(program);

            _program = program;
            _stopRequested = false;
            _isPaused      = false;
            _awaitingMove    = false;
            _awaitingAuxMove = false;
            _pendingAuxStep  = null;
            _awaitingVision  = false;
            _visionProgramId = null;
            _activeLocal     = null;
            _running         = true;
        }

        public void Resume()
        {
            if (!_isPaused) return;
            _isPaused      = false;
            _stopRequested = false;
            _running       = true;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName       = _program!.Name,
                ProgramStatus     = global::ProgramStatus.Running,
                CurrentStepNumber = _globalStepIndex,
                StepDescription   = "Resuming…",
            });
        }

        public void Stop()
        {
            if (!_running) return;

            if (!_isBackground)
            {
                // Request a queue drain on the control loop thread — calling Clear() directly
                // here would race with RunCommands() which reads QueuedCommands on the loop thread.
                _controller.RequestQueueDrain();
                // Hard-stop any active motion profiler so IsMoving clears immediately.
                _controller.HardStop();
            }

            if (_awaitingVision && _visionProgramId != null)
                _controller.VisionManager.StopProgram(_visionProgramId);

            _awaitingMove    = false;
            _pendingStep     = null;
            _awaitingAuxMove = false;
            _pendingAuxStep  = null;
            _awaitingVision  = false;
            _visionProgramId = null;
            _waitingForBackground = null;

            // Emit Stopped status immediately rather than waiting for the next Update() tick.
            Finish(global::ProgramStatus.Stopped, "Stopped by user");
        }

        /// <summary>
        /// Immediately halts execution and clears all state — no status update is emitted.
        /// The caller is responsible for pushing a final status (e.g. Ready) to programManager.
        /// </summary>
        public void Reset()
        {
            _running              = false;
            _stopRequested        = false;
            _isPaused             = false;
            _awaitingMove         = false;
            _pendingStep          = null;
            _awaitingAuxMove      = false;
            _pendingAuxStep       = null;
            _awaitingVision       = false;
            _visionProgramId      = null;
            _activeLocal          = null;
            _globalStepIndex      = 0;
            _loopDepth            = 0;
            _jumpSubStep          = 0;
            _threadSubStep        = 0;
            _threadMoveQueue      = null;
            _waitingForBackground = null;
            _frameStack.Clear();
            _variables.Clear();
            _listVariables.Clear();
            _pointVariables.Clear();
            _stopwatches.Clear();
            _booleanVariables.Clear();
            _stringVariables.Clear();
            _globalVarNames.Clear();
            _persistentVarNames.Clear();
        }

        // ── Main update — called every control loop tick ──────────────────────

        public void Update()
        {
            if (!_running || _program is null) return;

            // Blocking on a background program finishing
            if (_waitingForBackground != null)
            {
                if (_backgroundManager == null || !_backgroundManager.IsRunning(_waitingForBackground))
                    _waitingForBackground = null; // done or never started — advance
                else
                    return;
            }

            // Refresh stopwatch variable values so expressions always see the current elapsed time
            if (_stopwatches.Count > 0)
            {
                var nowTick = System.Environment.TickCount64;
                foreach (var name in _stopwatches.Keys)
                {
                    var sw = _stopwatches[name];
                    _variables[name] = sw.Running ? sw.AccumMs + (nowTick - sw.StartTick) : sw.AccumMs;
                }
            }

            // If we dispatched a robot move, wait until the queue is clear and the robot is idle
            if (_awaitingMove)
            {
                if (_controller.QueuedCommands.Count == 0 && !_controller.IsMoving)
                {
                    _awaitingMove = false;
                    // Report the step as completed now that the move has finished
                    if (_pendingStep is not null)
                    {
                        ReportStepCompleted(_pendingStep);
                        _pendingStep = null;
                    }
                    // Report every step consumed by a blended run
                    if (_pendingSteps is not null)
                    {
                        foreach (var s in _pendingSteps) ReportStepCompleted(s);
                        _pendingSteps = null;
                    }
                }
                else
                    return;
            }

            // If we dispatched an aux indexed move with WaitForDone=true, block until it completes
            if (_awaitingAuxMove)
            {
                if (!_controller.IsAuxMoving)
                {
                    _awaitingAuxMove = false;
                    if (_pendingAuxStep is not null)
                    {
                        ReportStepCompleted(_pendingAuxStep);
                        _pendingAuxStep = null;
                    }
                }
                else
                    return;
            }

            // Nothing left to execute?
            if (_frameStack.Count == 0)
            {
                Finish(global::ProgramStatus.Complete, "Complete");
                return;
            }

            var frame = _frameStack.Peek();

            // Frame exhausted — pop and continue
            if (frame.Index >= frame.Steps.Count)
            {
                _frameStack.Pop();

                // If this was a loop frame, decrement and possibly re-push
                if (frame.IsLoop)
                {
                    if (frame.WhileCondition != null)
                    {
                        // While loop: re-push only if condition still holds
                        if (EvalWhileCondition(frame.WhileCondition))
                            _frameStack.Push(new StepListFrame(frame.Steps, 0, isLoop: true,
                                loopRemaining: int.MaxValue, whileCondition: frame.WhileCondition));
                        else
                            _loopDepth--;
                    }
                    else if (frame.IsForEach)
                    {
                        frame.ForEachCurrentIndex++;
                        if (frame.ForEachCurrentIndex >= frame.ForEachCount)
                        {
                            _loopDepth--;
                        }
                        else
                        {
                            var nextFrame = new StepListFrame(frame.Steps, 0, isLoop: true,
                                loopRemaining: 1, isForEach: true,
                                forEachCount: frame.ForEachCount,
                                forEachCurrentIndex: frame.ForEachCurrentIndex,
                                forEachSourceVar: frame.ForEachSourceVar,
                                forEachValueVar: frame.ForEachValueVar,
                                forEachIndexVar: frame.ForEachIndexVar);
                            _frameStack.Push(nextFrame);
                            InjectForEachVars(nextFrame);
                        }
                    }
                    else
                    {
                        frame.LoopRemaining--;
                        if (frame.LoopRemaining == 0)
                        {
                            // Loop finished; outer frame already advanced past the loop step
                            _loopDepth--;
                        }
                        else
                        {
                            // Increment count-loop index variable if configured
                            if (!string.IsNullOrEmpty(frame.ForEachIndexVar))
                                SetVariable(frame.ForEachIndexVar,
                                    (frame.ForEachCount - frame.LoopRemaining)); // iteration number (0-based)

                            _frameStack.Push(new StepListFrame(frame.Steps, 0, isLoop: true,
                                loopRemaining: frame.LoopRemaining,
                                forEachIndexVar: frame.ForEachIndexVar,
                                forEachCount: frame.ForEachCount));
                        }
                    }
                }
                return; // Re-enter next tick with updated stack
            }

            var step = frame.Steps[frame.Index];
            ExecuteStep(step, frame);
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
                Finish(global::ProgramStatus.Error, $"Unknown variable '${ex.VariableName}' in step: {StepDescription(step)}");
            }
        }

        private void ExecuteStepInner(ProgramStep step, StepListFrame frame)
        {
            // Background programs skip motion/tool/homing steps rather than error
            if (_isBackground && IsRestrictedInBackground(step.Type))
            {
                _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
                {
                    ProgramName     = _program!.Name,
                    StepDescription = $"[Skipped — background] {step.Type}",
                    ShouldLog       = true,
                });
                ReportStepCompleted(step);
                frame.Index++;
                return;
            }

            switch (step.Type)
            {
                case StepType.MoveL:
                case StepType.MoveJ:
                    ExecuteMove(step, frame);
                    break;

                case StepType.JumpL:
                case StepType.JumpJ:
                    ExecuteJump(step, frame);
                    break;

                case StepType.SetOutput:
                    ExecuteSetOutput(step, frame);
                    break;

                case StepType.Wait:
                    ExecuteWait(step, frame);
                    break;

                case StepType.Loop:
                    ExecuteLoop(step, frame);
                    break;

                case StepType.StatusUpdate:
                    ExecuteStatusUpdate(step, frame);
                    break;

                case StepType.CallRoutine:
                    ExecuteCallRoutine(step, frame);
                    break;

                case StepType.SetSpeedL:
                    ExecuteSetSpeedL(step, frame);
                    break;

                case StepType.SetSpeedJ:
                    ExecuteSetSpeedJ(step, frame);
                    break;

                case StepType.SetBlendRadius:
                    _defaultBlendRadius = Math.Max(0, EvalField(step, "blendRadius", step.BlendRadius ?? 0));
                    ReportStepCompleted(step);
                    frame.Index++;
                    break;

                case StepType.SetVariable:
                    ExecuteSetVariable(step, frame);
                    break;

                case StepType.PauseProgram:
                    ExecutePauseProgram(step, frame);
                    break;

                case StepType.Label:
                    ExecuteLabel(step, frame);
                    break;

                case StepType.GoToLabel:
                    ExecuteGoToLabel(step, frame);
                    break;

                case StepType.IfCondition:
                    ExecuteIfCondition(step, frame);
                    break;

                case StepType.SetTool:
                    ExecuteSetTool(step, frame);
                    break;

                case StepType.SetLocal:
                    ExecuteSetLocal(step, frame);
                    break;

                case StepType.ClearLocal:
                    ExecuteClearLocal(step, frame);
                    break;

                case StepType.RunHoming:
                    ExecuteRunHoming(step, frame);
                    break;

                case StepType.AuxMove:
                    ExecuteAuxMove(step, frame);
                    break;

                case StepType.AuxContinuous:
                    ExecuteAuxContinuous(step, frame);
                    break;

                case StepType.AuxStop:
                    ExecuteAuxStop(step, frame);
                    break;

                case StepType.AuxEnable:
                    ExecuteAuxEnable(step, frame);
                    break;

                case StepType.RunVision:
                    ExecuteRunVision(step, frame);
                    break;

                case StepType.StartBackground:
                    ExecuteStartBackground(step, frame);
                    break;

                case StepType.StopBackground:
                    ExecuteStopBackground(step, frame);
                    break;

                case StepType.WaitForBackground:
                    ExecuteWaitForBackground(step, frame);
                    break;

                case StepType.StopwatchControl:
                    ExecuteStopwatchControl(step, frame);
                    break;

                case StepType.SaveImage:
                    ExecuteSaveImage(step, frame);
                    break;

                case StepType.ThreadMove:
                    ExecuteThreadMove(step, frame);
                    break;

                case StepType.CncProgram:
                    ExecuteCncProgram(step, frame);
                    break;
            }
        }

        private void ExecuteMove(ProgramStep step, StepListFrame frame)
        {
            if (_awaitingMove) return;

            if (!ResolveMoveTarget(step, out Vector6 target)) return;

            bool hasToolOffset = HasToolOffset(step);

            // Blended MoveL run: gather consecutive blendable MoveL steps into one
            // continuous path so the robot rounds the corners instead of stopping.
            if (step.Type == StepType.MoveL && EffectivelyBlends(step) && !hasToolOffset)
            {
                if (TryDispatchBlendedRun(step, target, frame)) return;
            }

            var cmd = new RobotCommand
            {
                CommandType = step.Type == StepType.MoveL ? "MoveL" : "MoveJ",
                X  = target.X,
                Y  = target.Y,
                Z  = target.Z,
                RX = target.RX,
                RY = target.RY,
                RZ = target.RZ,
                // Optional local tool offset applied on top of the active tool
                TX  = hasToolOffset ? EvalField(step, "toolOffsetX",  step.ToolOffsetX  ?? 0) : null,
                TY  = hasToolOffset ? EvalField(step, "toolOffsetY",  step.ToolOffsetY  ?? 0) : null,
                TZ  = hasToolOffset ? EvalField(step, "toolOffsetZ",  step.ToolOffsetZ  ?? 0) : null,
                TRX = hasToolOffset ? EvalField(step, "toolOffsetRX", step.ToolOffsetRX ?? 0) : null,
                TRY = hasToolOffset ? EvalField(step, "toolOffsetRY", step.ToolOffsetRY ?? 0) : null,
                TRZ = hasToolOffset ? EvalField(step, "toolOffsetRZ", step.ToolOffsetRZ ?? 0) : null,
                // Raw speed — the global override is applied centrally in MoveL/MoveJ.
                Speed = (step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true) ? EvalField(step, "speed", step.Speed ?? 0) : (double?)null,
                Accel = (step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true) ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null,
                Decel = (step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true) ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null,
                ApplySpeedOverride = true,   // program move — subject to the speed override
            };

            _controller.QueuedCommands.Enqueue(cmd);
            _awaitingMove = true;
            _pendingStep  = step; // completion is reported in Update() once the move finishes

            // Announce the step is in progress without counting it yet
            ReportStepStarted(step);
            frame.Index++;
        }

        private static bool HasToolOffset(ProgramStep step) =>
            step.ToolOffsetX.HasValue || step.ToolOffsetY.HasValue || step.ToolOffsetZ.HasValue
            || step.ToolOffsetRX.HasValue || step.ToolOffsetRY.HasValue || step.ToolOffsetRZ.HasValue;

        // Effective blend radius for a move: its own override if set, else the program default.
        private double EffectiveBlendRadius(ProgramStep step) =>
            (step.BlendRadius.HasValue || step.Expressions?.ContainsKey("blendRadius") == true)
                ? Math.Max(0, EvalField(step, "blendRadius", step.BlendRadius ?? 0))
                : _defaultBlendRadius;

        // A move only actually blends when blending is on AND it has a non-zero radius.
        // Blend-on with a zero radius would round nothing yet still not stop — a jolt — so
        // it's treated as a normal (stopping) move instead.
        private bool EffectivelyBlends(ProgramStep step) =>
            (step.Blend ?? false) && EffectiveBlendRadius(step) > 0;

        // Gather a run of consecutive blendable MoveL steps and dispatch one continuous
        // (blended) path. Returns false when there is nothing to blend, so the caller
        // falls back to a normal single move.
        private bool TryDispatchBlendedRun(ProgramStep first, Vector6 firstTarget, StepListFrame frame)
        {
            var steps     = frame.Steps;
            var run       = new List<ProgramStep> { first };
            var waypoints = new List<Vector6> { firstTarget };

            bool prevBlend = EffectivelyBlends(first);
            int j = frame.Index + 1;
            while (prevBlend && j < steps.Count)
            {
                var nxt = steps[j];
                // Only plain MoveL steps without a per-step tool offset can join the path.
                if (nxt.Type != StepType.MoveL || HasToolOffset(nxt)) break;
                // Resolve relative to the previous waypoint so "current position" moves chain
                // off where the robot actually ends up, not where the blend started.
                if (!ResolveMoveTarget(nxt, out Vector6 t, waypoints[^1])) return true; // errored — program is finishing
                run.Add(nxt);
                waypoints.Add(t);
                // A blend-on move with a zero radius stops here (terminates the run).
                prevBlend = EffectivelyBlends(nxt);
                j++;
            }

            if (run.Count < 2) return false; // only one move — nothing to blend

            // Corner radius at waypoint k comes from the move arriving there; the final
            // waypoint is always an exact stop.
            var radii = new List<double>(run.Count);
            for (int k = 0; k < run.Count; k++)
                radii.Add(k < run.Count - 1 ? EffectiveBlendRadius(run[k]) : 0);

            double? speed = (first.Speed.HasValue || first.Expressions?.ContainsKey("speed") == true) ? EvalField(first, "speed", first.Speed ?? 0) : (double?)null;
            double? accel = (first.Accel.HasValue || first.Expressions?.ContainsKey("accel") == true) ? EvalField(first, "accel", first.Accel ?? 0) : (double?)null;
            double? decel = (first.Decel.HasValue || first.Expressions?.ContainsKey("decel") == true) ? EvalField(first, "decel", first.Decel ?? 0) : (double?)null;

            _controller.StartContinuousMove(waypoints, radii, speed, accel, decel, applyOverride: true);
            _awaitingMove = true;
            _pendingSteps = run;

            foreach (var s in run) ReportStepStarted(s);
            frame.Index += run.Count;
            return true;
        }

        // Resolve a move step's final Cartesian target (point/grid/stack/var + offsets +
        // overrides + active local). Returns false after calling Finish() on any error.
        // currentPos overrides the base used for "current position" moves — during a blended
        // run this is the previous waypoint (where the robot ends up), not its live position.
        private bool ResolveMoveTarget(ProgramStep step, out Vector6 target, Vector6? currentPos = null)
        {
            target = Vector6.Zero;

            Point point;
            if (step.GridPoint != null)
            {
                var gp   = step.GridPoint;
                var grid = _gridRepo.Get(gp.GridId);
                if (grid == null) { Finish(global::ProgramStatus.Error, $"Grid not found: {gp.GridId}"); return false; }

                var basePoint = _pointRepo.Get(grid.BasePointName);
                if (basePoint == null) { Finish(global::ProgramStatus.Error, $"Grid base point not found: {grid.BasePointName}"); return false; }

                int row, col;
                if (gp.UseGridIndex)
                {
                    if (!grid.ColCount.HasValue || grid.ColCount.Value <= 0)
                    {
                        Finish(global::ProgramStatus.Error, $"Grid '{grid.Name}' requires colCount to use grid index");
                        return false;
                    }
                    int idx = (int)Math.Round(EvalField(step, "gridGridIndex", gp.GridIndex ?? 0));
                    row = idx / grid.ColCount.Value;
                    col = idx % grid.ColCount.Value;
                }
                else
                {
                    row = (int)Math.Round(EvalField(step, "gridRowIndex", gp.RowIndex ?? 0));
                    col = (int)Math.Round(EvalField(step, "gridColIndex", gp.ColIndex ?? 0));
                }

                double rawX = row * grid.RowOffsetX + col * grid.ColOffsetX;
                double rawY = row * grid.RowOffsetY + col * grid.ColOffsetY;
                double rawZ = row * grid.RowOffsetZ + col * grid.ColOffsetZ;

                double theta = grid.Rotation * Math.PI / 180.0;
                double rotX  = rawX * Math.Cos(theta) - rawY * Math.Sin(theta);
                double rotY  = rawX * Math.Sin(theta) + rawY * Math.Cos(theta);

                point = new Point
                {
                    X  = basePoint.X  + rotX,
                    Y  = basePoint.Y  + rotY,
                    Z  = basePoint.Z  + rawZ,
                    RX = basePoint.RX,
                    RY = basePoint.RY,
                    RZ = basePoint.RZ,
                };
            }
            else if (step.StackPoint != null)
            {
                var sp    = step.StackPoint;
                var stack = _stackRepo.Get(sp.StackId);
                if (stack == null) { Finish(global::ProgramStatus.Error, $"Stack not found: {sp.StackId}"); return false; }

                var basePoint = _pointRepo.Get(stack.BasePointName);
                if (basePoint == null) { Finish(global::ProgramStatus.Error, $"Stack base point not found: {stack.BasePointName}"); return false; }

                int idx = (int)Math.Round(EvalField(step, "stackIndex", sp.Index ?? 0));
                if (stack.MaxCount.HasValue && stack.MaxCount.Value > 0)
                    idx = ((idx % stack.MaxCount.Value) + stack.MaxCount.Value) % stack.MaxCount.Value;

                point = new Point
                {
                    X  = basePoint.X  + idx * stack.OffsetX,
                    Y  = basePoint.Y  + idx * stack.OffsetY,
                    Z  = basePoint.Z  + idx * stack.OffsetZ,
                    RX = basePoint.RX,
                    RY = basePoint.RY,
                    RZ = basePoint.RZ,
                };
            }
            else if (!string.IsNullOrEmpty(step.VarPointName))
            {
                if (!_pointVariables.TryGetValue(step.VarPointName, out var ptList) || ptList.Count == 0)
                {
                    Finish(global::ProgramStatus.Error, $"Variable point '{step.VarPointName}' is empty or not set");
                    return false;
                }
                int ptIdx = 0;
                if (!string.IsNullOrEmpty(step.VarPointIndex))
                {
                    try { ptIdx = (int)Math.Round(ExpressionEvaluator.Evaluate(step.VarPointIndex, EvalVars(), _listVariables, _pointVariables)); }
                    catch (UnknownVariableException) { throw; }
                    catch { /* malformed expression — default 0 */ }
                }
                ptIdx = Math.Clamp(ptIdx, 0, ptList.Count - 1);
                var vp = ptList[ptIdx];
                point = new Point { X = vp.X, Y = vp.Y, Z = vp.Z, RX = vp.RX, RY = vp.RY, RZ = vp.RZ };
            }
            else if (string.IsNullOrEmpty(step.PointName))
            {
                // No point specified — use the base TCP position so offsets act as relative
                // displacements. In a blended run this is the previous waypoint (where the
                // robot will be), otherwise the robot's live current position.
                var pos = currentPos ?? _controller.GetCurrentPosition();
                point = new Point { X = pos.X, Y = pos.Y, Z = pos.Z, RX = pos.RX, RY = pos.RY, RZ = pos.RZ };
            }
            else
            {
                var found = _pointRepo.Get(step.PointName);
                if (found is null)
                {
                    Finish(global::ProgramStatus.Error, $"Point not found: {step.PointName}");
                    return false;
                }
                point = found;
            }

            // Base position + offsets
            double finalX  = point.X  + EvalField(step, "offsetX",  step.OffsetX  ?? 0);
            double finalY  = point.Y  + EvalField(step, "offsetY",  step.OffsetY  ?? 0);
            double finalZ  = point.Z  + EvalField(step, "offsetZ",  step.OffsetZ  ?? 0);
            double finalRX = point.RX + EvalField(step, "offsetRX", step.OffsetRX ?? 0);
            double finalRY = point.RY + EvalField(step, "offsetRY", step.OffsetRY ?? 0);
            double finalRZ = point.RZ + EvalField(step, "offsetRZ", step.OffsetRZ ?? 0);

            // Per-axis absolute overrides — replace calculated value when set
            if (step.OverrideX.HasValue  || step.Expressions?.ContainsKey("overrideX")  == true) finalX  = EvalField(step, "overrideX",  step.OverrideX  ?? 0);
            if (step.OverrideY.HasValue  || step.Expressions?.ContainsKey("overrideY")  == true) finalY  = EvalField(step, "overrideY",  step.OverrideY  ?? 0);
            if (step.OverrideZ.HasValue  || step.Expressions?.ContainsKey("overrideZ")  == true) finalZ  = EvalField(step, "overrideZ",  step.OverrideZ  ?? 0);
            if (step.OverrideRX.HasValue || step.Expressions?.ContainsKey("overrideRX") == true) finalRX = EvalField(step, "overrideRX", step.OverrideRX ?? 0);
            if (step.OverrideRY.HasValue || step.Expressions?.ContainsKey("overrideRY") == true) finalRY = EvalField(step, "overrideRY", step.OverrideRY ?? 0);
            if (step.OverrideRZ.HasValue || step.Expressions?.ContainsKey("overrideRZ") == true) finalRZ = EvalField(step, "overrideRZ", step.OverrideRZ ?? 0);

            // Apply active local offset — per-step localName overrides the program-level active local
            Vector6? effectiveLocal;
            if (!string.IsNullOrEmpty(step.LocalName))
            {
                var stepLocal = _localRepo.Get(step.LocalName);
                effectiveLocal = stepLocal != null ? new Vector6(stepLocal.X, stepLocal.Y, stepLocal.Z, stepLocal.RX, stepLocal.RY, stepLocal.RZ) : null;
            }
            else
            {
                effectiveLocal = _activeLocal;
            }
            if (effectiveLocal != null)
            {
                finalX  += effectiveLocal.X;
                finalY  += effectiveLocal.Y;
                finalZ  += effectiveLocal.Z;
                finalRX += effectiveLocal.RX;
                finalRY += effectiveLocal.RY;
                finalRZ += effectiveLocal.RZ;
            }

            target = new Vector6(finalX, finalY, finalZ, finalRX, finalRY, finalRZ);
            return true;
        }

        private void ExecuteThreadMove(ProgramStep step, StepListFrame frame)
        {
            if (_awaitingMove) return;

            if (_threadSubStep == 0)
            {
                var start    = _controller.GetCurrentPosition();
                double dist  = EvalField(step, "threadDistance",  step.ThreadDistance  ?? 0);
                double pitch = EvalField(step, "threadPitch",     step.ThreadPitch     ?? 1);
                bool   peck  = step.ThreadPeck ?? false;
                double peckD = step.ThreadPeckDepth ?? Math.Abs(dist);
                bool   rev   = step.ThreadReverseOut ?? true;

                if (Math.Abs(pitch) < 0.0001) pitch = 1.0;
                if (peckD <= 0) peckD = Math.Abs(dist);

                double? speed = step.Speed.HasValue ? EvalField(step, "speed", step.Speed ?? 0) : null; // override applied in MoveL
                double? accel = step.Accel;
                double? decel = step.Decel;
                double  sign  = dist >= 0 ? 1.0 : -1.0;
                double  absDist = Math.Abs(dist);

                RobotCommand Move(double dZ, double dRZ) => new RobotCommand
                {
                    CommandType = "MoveL",
                    X  = start.X, Y  = start.Y, Z  = start.Z  + dZ,
                    RX = start.RX, RY = start.RY, RZ = start.RZ + dRZ,
                    Speed = speed, Accel = accel, Decel = decel,
                    ApplySpeedOverride = true,
                };

                _threadMoveQueue = new Queue<RobotCommand>();

                if (peck && peckD > 0)
                {
                    // 2x down, 1x up: advance 2*peckD each cycle, retract 1*peckD between cycles
                    double accumulated = 0;
                    while (accumulated < absDist - 0.0001)
                    {
                        double nextDepth = Math.Min(accumulated + peckD * 2, absDist);
                        _threadMoveQueue.Enqueue(Move(sign * nextDepth, (sign * nextDepth / pitch) * 360.0));
                        if (nextDepth >= absDist - 0.0001) break;
                        double retractTo = Math.Max(nextDepth - peckD, 0);
                        _threadMoveQueue.Enqueue(Move(sign * retractTo, (sign * retractTo / pitch) * 360.0));
                        accumulated = retractTo;
                    }
                }
                else
                {
                    _threadMoveQueue.Enqueue(Move(dist, (dist / pitch) * 360.0));
                }

                if (rev)
                    _threadMoveQueue.Enqueue(Move(0, 0)); // reverse back to start

                _threadSubStep = 1;
                ReportStepStarted(step);
            }

            // Dispatch next queued move (or finish if queue empty)
            if (_threadMoveQueue == null || _threadMoveQueue.Count == 0)
            {
                _threadSubStep   = 0;
                _threadMoveQueue = null;
                frame.Index++;
                ReportStepCompleted(step);
                return;
            }

            _controller.QueuedCommands.Enqueue(_threadMoveQueue.Dequeue());
            _awaitingMove = true;
        }

        private void ExecuteJump(ProgramStep step, StepListFrame frame)
        {
            if (_awaitingMove) return;

            if (_jumpSubStep == 0)
            {
                // Resolve final target via the shared resolver — same as ExecuteMove.
                // Supports PointName, GridPoint, StackPoint, variable points, offsets,
                // per-axis overrides, and the active local. Finish(Error) on failure.
                if (!ResolveMoveTarget(step, out Vector6 resolvedTarget)) return;

                bool hasJumpZ      = step.JumpZ.HasValue      || step.Expressions?.ContainsKey("jumpZ")      == true;
                bool hasJumpZStart = step.JumpZStart.HasValue || step.Expressions?.ContainsKey("jumpZStart") == true;
                bool hasJumpZEnd   = step.JumpZEnd.HasValue   || step.Expressions?.ContainsKey("jumpZEnd")   == true;

                if (!hasJumpZ && !hasJumpZStart)
                {
                    Finish(global::ProgramStatus.Error, "Jump step: JumpZ must be set");
                    return;
                }

                var cur = _controller.GetCurrentPosition();
                _jumpStartPos = cur;
                _jumpTarget   = resolvedTarget;
                _jumpZStart   = hasJumpZStart ? EvalField(step, "jumpZStart", step.JumpZStart ?? 0) : EvalField(step, "jumpZ", step.JumpZ ?? 0);
                _jumpZEnd     = hasJumpZEnd   ? EvalField(step, "jumpZEnd",   step.JumpZEnd   ?? 0) : EvalField(step, "jumpZ", step.JumpZ ?? 0);
                _jumpCmdType  = step.Type == StepType.JumpJ ? "MoveJ" : "MoveL";
                _jumpSpeed    = (step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true) ? EvalField(step, "speed", step.Speed ?? 0) : (double?)null; // override applied in MoveL/MoveJ
                _jumpAccel    = (step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true) ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null;
                _jumpDecel    = (step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true) ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null;

                // Blended JumpL: run lift → traverse → lower as one continuous path,
                // rounding the two apex corners instead of stopping at each leg. (JumpJ's
                // traverse is a joint move, so it keeps the stepped behaviour for now.)
                if (step.Type == StepType.JumpL && EffectivelyBlends(step))
                {
                    double r = EffectiveBlendRadius(step);
                    var apexUp   = new Vector6(_jumpStartPos.X, _jumpStartPos.Y, _jumpZStart, _jumpStartPos.RX, _jumpStartPos.RY, _jumpStartPos.RZ);
                    var apexOver = new Vector6(_jumpTarget.X,   _jumpTarget.Y,   _jumpZEnd,   _jumpTarget.RX,   _jumpTarget.RY,   _jumpTarget.RZ);
                    _controller.StartContinuousMove(
                        new List<Vector6> { apexUp, apexOver, _jumpTarget },
                        new List<double> { r, r, 0 },   // round both apexes; land exactly on target
                        _jumpSpeed, _jumpAccel, _jumpDecel, applyOverride: true);
                    _jumpSubStep  = 0;
                    _awaitingMove = true;
                    _pendingStep  = step;
                    ReportStepStarted(step);
                    frame.Index++;
                    return;
                }

                _jumpSubStep  = 1;

                _controller.QueuedCommands.Enqueue(new RobotCommand
                {
                    CommandType = "MoveL",
                    X = _jumpStartPos.X, Y = _jumpStartPos.Y, Z = _jumpZStart,
                    RX = _jumpStartPos.RX, RY = _jumpStartPos.RY, RZ = _jumpStartPos.RZ,
                    Speed = _jumpSpeed, Accel = _jumpAccel, Decel = _jumpDecel,
                    ApplySpeedOverride = true,
                });
                _awaitingMove = true;
                ReportStepStarted(step);
                return;
            }

            if (_jumpSubStep == 1)
            {
                _controller.QueuedCommands.Enqueue(new RobotCommand
                {
                    CommandType = _jumpCmdType,
                    X = _jumpTarget.X, Y = _jumpTarget.Y, Z = _jumpZEnd,
                    RX = _jumpTarget.RX, RY = _jumpTarget.RY, RZ = _jumpTarget.RZ,
                    Speed = _jumpSpeed, Accel = _jumpAccel, Decel = _jumpDecel,
                    ApplySpeedOverride = true,
                });
                _jumpSubStep  = 2;
                _awaitingMove = true;
                return;
            }

            if (_jumpSubStep == 2)
            {
                _controller.QueuedCommands.Enqueue(new RobotCommand
                {
                    CommandType = "MoveL",
                    X = _jumpTarget.X, Y = _jumpTarget.Y, Z = _jumpTarget.Z,
                    RX = _jumpTarget.RX, RY = _jumpTarget.RY, RZ = _jumpTarget.RZ,
                    Speed = _jumpSpeed, Accel = _jumpAccel, Decel = _jumpDecel,
                    ApplySpeedOverride = true,
                });
                _jumpSubStep  = 3;
                _awaitingMove = true;
                return;
            }

            // _jumpSubStep == 3: all three legs complete
            _jumpSubStep = 0;
            frame.Index++;
            ReportStepCompleted(step);
        }

        private void ExecuteStatusUpdate(ProgramStep step, StepListFrame frame)
        {
            if (_loopDepth == 0) _globalStepIndex++;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName        = _program!.Name,
                ProgramStatus      = global::ProgramStatus.Running,
                CurrentStepNumber  = _globalStepIndex,
                StepDescription    = !string.IsNullOrEmpty(step.StatusMessage)
                    ? InterpolateVariables(step.StatusMessage)
                    : StepDescription(step),
                WarningDescription = !string.IsNullOrEmpty(step.StatusWarning)
                    ? InterpolateVariables(step.StatusWarning)
                    : null,
                ErrorDescription   = !string.IsNullOrEmpty(step.StatusError)
                    ? InterpolateVariables(step.StatusError)
                    : null,
                ShouldLog          = true,
            });
            frame.Index++;
        }

        private string InterpolateVariables(string template)
        {
            // Matches: $name, $name[expr], $name[expr].component
            var allVarsForTemplate = MergedVars();
            return Regex.Replace(template, @"\$(\w+)(?:\[([^\]]*)\](?:\.(\w+))?)?", m =>
            {
                var name     = m.Groups[1].Value;
                var hasIndex = m.Groups[2].Success;
                var idxExpr  = m.Groups[2].Value.Trim();
                var hasComp  = m.Groups[3].Success;
                var compName = m.Groups[3].Value.ToLower();

                if (!hasIndex)
                {
                    // Plain $name — scalar → value, list → count, points → "N points", string → value
                    if (allVarsForTemplate.TryGetValue(name, out var sv))
                        return _booleanVariables.Contains(name) ? (sv != 0 ? "True" : "False") : sv.ToString("G6");
                    if (_stringVariables.TryGetValue(name, out var strVal))
                        return strVal;
                    if (_listVariables.ContainsKey(name))
                        return $"{_listVariables[name].Count} items";
                    if (_pointVariables.ContainsKey(name))
                        return $"{_pointVariables[name].Count} points";
                    return m.Value;
                }

                // Evaluate index expression (literal int or variable expression)
                int idx = 0;
                if (!string.IsNullOrEmpty(idxExpr))
                {
                    try { idx = (int)Math.Round(ExpressionEvaluator.Evaluate(idxExpr, allVarsForTemplate, _listVariables, _pointVariables)); }
                    catch { idx = 0; }
                }

                // Points variable
                if (_pointVariables.TryGetValue(name, out var ptList))
                {
                    if (ptList.Count == 0) return "(empty)";
                    idx = Math.Clamp(idx, 0, ptList.Count - 1);
                    var pt = ptList[idx];

                    if (hasComp)
                        return pt.GetComponent(compName).ToString("G6");

                    return $"(x={pt.X:G6}, y={pt.Y:G6}, z={pt.Z:G6}, rx={pt.RX:G6}, ry={pt.RY:G6}, rz={pt.RZ:G6})";
                }

                // List variable indexed
                if (_listVariables.TryGetValue(name, out var list))
                {
                    if (list.Count == 0) return "(empty)";
                    idx = Math.Clamp(idx, 0, list.Count - 1);
                    return list[idx].ToString("G6");
                }

                return m.Value;
            });
        }

        private void ExecuteSetOutput(ProgramStep step, StepListFrame frame)
        {
            var card   = step.OutputCard ?? "stb";
            var number = step.OutputNumber ?? 1;
            var value  = step.OutputValue ?? false;
            var pulse  = step.PulseMs ?? 0;

            if (!frame.WaitStarted)
            {
                ApplyOutput(_controller, card, number, value, step.OutputNanoId);

                if (pulse > 0 && step.PulseBlocking == true)
                {
                    _waitStartMs      = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                    frame.WaitStarted = true;
                    ReportStepStarted(step);
                    return;
                }

                if (pulse > 0)
                {
                    var ctrl   = _controller;
                    var nanoId = step.OutputNanoId;
                    _ = Task.Run(async () =>
                    {
                        await Task.Delay(pulse);
                        ApplyOutput(ctrl, card, number, !value, nanoId);
                    });
                }

                ReportStepCompleted(step);
                frame.Index++;
                return;
            }

            // Blocking pulse — wait for pulse duration then flip
            var elapsed = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() - _waitStartMs;
            if (elapsed >= pulse)
            {
                ApplyOutput(_controller, card, number, !value, step.OutputNanoId);
                frame.WaitStarted = false;
                frame.Index++;
                ReportStepCompleted(step);
            }
        }

        /// <summary>Applies a single output state to the correct IO card.</summary>
        private static void ApplyOutput(RobotController ctrl, string card, int number, bool value, string? nanoId)
        {
            switch (card)
            {
                case "relay":
                    ctrl.RelayManager.SetRelay(number, value);
                    break;
                case "nano":
                    if (!string.IsNullOrEmpty(nanoId))
                        ctrl.NanoManager.SetOutput(nanoId, number, value);
                    break;
                default: // "stb"
                    ctrl.stb.SetOutput(number, value);
                    break;
            }
        }

        private void ExecuteWait(ProgramStep step, StepListFrame frame)
        {
            if (!frame.WaitStarted)
            {
                _waitStartMs      = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                frame.WaitStarted = true;
                ReportStepStarted(step);
                return;
            }

            var nowMs   = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            var elapsed = nowMs - _waitStartMs;

            if (step.WaitMode == "condition" && step.WaitCondition != null)
            {
                // Build combined variable dict (program vars + IO)
                var allVars = MergedVars();
                foreach (var kv in BuildIoVariables()) allVars[kv.Key] = kv.Value;

                bool condMet  = EvaluateConditionGroup(step.WaitCondition, allVars);
                int  timeout  = step.WaitTimeoutMs ?? 0;
                bool timedOut = timeout > 0 && elapsed >= timeout;

                if (condMet || timedOut)
                {
                    if (!string.IsNullOrEmpty(step.WaitTimeoutVariableName))
                        SetVariable(step.WaitTimeoutVariableName, timedOut ? 1 : 0);
                    frame.WaitStarted = false;
                    frame.Index++;
                    ReportStepCompleted(step);
                }
            }
            else
            {
                if (elapsed >= EvalField(step, "waitMs", step.WaitMs ?? 0))
                {
                    frame.WaitStarted = false;
                    frame.Index++;
                    ReportStepCompleted(step);
                }
            }
        }

        /// <summary>
        /// Resolves the runtime zone override for a RunVision step: a variable (1-based
        /// index into the program's zones) takes priority over a fixed zone id. Returns
        /// null when no override is set (each inspection uses its own configured zone).
        /// </summary>
        private string? ResolveVisionZoneOverride(ProgramStep step)
        {
            string? zoneId = step.VisionZoneId;
            if (!string.IsNullOrEmpty(step.VisionZoneVar) &&
                _variables.TryGetValue(step.VisionZoneVar, out var zoneIdxVal))
            {
                var vp = _controller.VisionManager.GetProgram(step.VisionProgramId!);
                if (vp != null)
                {
                    int zoneIdx = (int)zoneIdxVal - 1; // 1-based → 0-based
                    zoneId = (zoneIdx >= 0 && zoneIdx < vp.Zones.Count) ? vp.Zones[zoneIdx].Id : null;
                }
            }
            return zoneId;
        }

        private void ExecuteRunVision(ProgramStep step, StepListFrame frame)
        {
            var programId = step.VisionProgramId;
            if (string.IsNullOrEmpty(programId))
            {
                Finish(global::ProgramStatus.Error, "RunVision step has no vision program selected");
                return;
            }

            if (!_awaitingVision)
            {
                // First entry: start the processor (applying any zone override so the
                // analysis and debug frame run in the selected zone) and record the trigger.
                _controller.VisionManager.StartProgram(programId, ResolveVisionZoneOverride(step));
                _visionProgramId = programId;
                _visionStartMs   = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                _awaitingVision  = true;

                _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
                {
                    ProgramName       = _program!.Name,
                    ProgramStatus     = global::ProgramStatus.Running,
                    CurrentStepNumber = _globalStepIndex,
                    StepDescription   = $"Vision → {step.VisionProgramName ?? programId}",
                });
                return;
            }

            // Subsequent entries: poll for a fresh result
            var proc = _controller.VisionManager.GetProcessor(_visionProgramId!);
            if (proc == null)
            {
                // Processor was stopped externally — treat as error
                _awaitingVision  = false;
                _visionProgramId = null;
                Finish(global::ProgramStatus.Error, $"Vision processor lost for '{step.VisionProgramName}'");
                return;
            }

            var result = proc.GetLatestResult();
            if (result == null || result.TimestampMs <= _visionStartMs)
                return; // no fresh result yet — keep waiting

            // Got a fresh result — write output variables, stop processor, advance.
            // The zone override (if any) was applied to the processor at start, so every
            // inspection ran in the selected zone and all of its outputs are relevant.
            HashSet<string>? zoneInspIds = null;

            foreach (var output in step.VisionOutputs ?? [])
            {
                if (zoneInspIds != null && !zoneInspIds.Contains(output.InspectionId)) continue;
                var ir = result.Inspections.Find(i => i.InspectionId == output.InspectionId);
                if (ir == null) continue;

                if (!string.IsNullOrEmpty(output.CountVar))
                    _variables[output.CountVar] = ir.Blobs.Count;

                if (!string.IsNullOrEmpty(output.PointsVar))
                {
                    _pointVariables[output.PointsVar] = ir.Blobs
                        .Select(b => new Vector6Val { X = b.X, Y = b.Y })
                        .ToList();
                }

                if (!string.IsNullOrEmpty(output.DetectedVar))
                    _variables[output.DetectedVar] = ir.Blobs.Count > 0 ? 1 : 0;
            }

            foreach (var output in step.ColorOutputs ?? [])
            {
                if (zoneInspIds != null && !zoneInspIds.Contains(output.InspectionId)) continue;
                var cr = result.ColorResults.Find(r => r.InspectionId == output.InspectionId);
                if (cr == null) continue;

                if (!string.IsNullOrEmpty(output.CoverageVar))
                    _variables[output.CoverageVar] = cr.Coverage;

                if (!string.IsNullOrEmpty(output.PassedVar))
                    _variables[output.PassedVar] = cr.Passed ? 1 : 0;
            }

            foreach (var output in step.PolygonOutputs ?? [])
            {
                if (zoneInspIds != null && !zoneInspIds.Contains(output.InspectionId)) continue;
                var pr = result.PolygonResults.Find(r => r.InspectionId == output.InspectionId);
                if (pr == null) continue;

                if (!string.IsNullOrEmpty(output.CountVar))
                    _variables[output.CountVar] = pr.Count;

                if (!string.IsNullOrEmpty(output.FoundVar))
                    _variables[output.FoundVar] = pr.Found ? 1 : 0;

                if (!string.IsNullOrEmpty(output.AngleVar))
                    _variables[output.AngleVar] = pr.Angle;

                if (!string.IsNullOrEmpty(output.CenterXVar))
                    _variables[output.CenterXVar] = pr.CenterX;

                if (!string.IsNullOrEmpty(output.CenterYVar))
                    _variables[output.CenterYVar] = pr.CenterY;
            }

            foreach (var output in step.ArucoOutputs ?? [])
            {
                if (zoneInspIds != null && !zoneInspIds.Contains(output.InspectionId)) continue;
                var ar = result.ArucoResults.Find(r => r.InspectionId == output.InspectionId);
                if (ar == null) continue;

                if (!string.IsNullOrEmpty(output.CountVar))
                    _variables[output.CountVar] = ar.Count;

                if (!string.IsNullOrEmpty(output.FoundVar))
                    _variables[output.FoundVar] = ar.Found ? 1 : 0;

                var first = ar.Markers.FirstOrDefault();
                if (first != null)
                {
                    if (!string.IsNullOrEmpty(output.FirstIdVar))
                        _variables[output.FirstIdVar] = first.MarkerId;

                    if (!string.IsNullOrEmpty(output.FirstCenterXVar))
                        _variables[output.FirstCenterXVar] = first.CenterX;

                    if (!string.IsNullOrEmpty(output.FirstCenterYVar))
                        _variables[output.FirstCenterYVar] = first.CenterY;
                }
            }

            var snap = proc.GetLatestAnnotated();
            if (snap != null) _controller.SetProgramVisionSnapshot(_visionProgramId!, snap);
            _controller.SetProgramVisionResult(_visionProgramId!, result);

            _controller.VisionManager.StopProgram(_visionProgramId!);
            _awaitingVision  = false;
            _visionProgramId = null;
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteCallRoutine(ProgramStep step, StepListFrame frame)
        {
            // Resolve by id first (survives renames), falling back to name for legacy steps.
            var routine = !string.IsNullOrEmpty(step.RoutineId) ? _builtProgramRepo.GetById(step.RoutineId!) : null;
            routine ??= _builtProgramRepo.Get(step.RoutineName ?? "");
            if (routine is null)
            {
                Finish(global::ProgramStatus.Error, $"Routine not found: {step.RoutineName}");
                return;
            }
            if (routine.Steps.Count == 0) { frame.Index++; ReportStepCompleted(step); return; }

            // Register the routine's own variables so they can be used inside the routine.
            // Caller variables remain available (shared _variables); a routine variable with
            // the same name as a caller's takes the routine's declared default.
            InitializeVariables(routine);

            ReportStepCompleted(step); // the call step itself counts as done; routine steps count separately
            frame.Index++;

            // Push the routine's steps as a plain (non-loop) frame
            _frameStack.Push(new StepListFrame(routine.Steps, 0));
        }

        // Register a program's declared variables (points, lists, stopwatches, strings and
        // scalars, with global/persistent handling). Used for the main program on Start and
        // for each routine when it is entered, so routine-local variables exist at runtime.
        private void InitializeVariables(BuiltProgram program)
        {
            var savedPersistent = LoadPersistentVars();
            var persistPrefix = string.IsNullOrEmpty(program.Id) ? "" : program.Id + ":";

            foreach (var v in program.Variables ?? [])
            {
                bool isGlobal     = v.IsGlobal == true && _globalVars != null;
                bool isPersistent = v.IsPersistent == true;
                if (v.Points != null)
                    _pointVariables[v.Name] = new List<Vector6Val>(v.Points);
                else if (v.Values != null && v.Values.Count > 0)
                    _listVariables[v.Name] = v.Values;
                else if (v.IsStopwatch == true)
                {
                    _stopwatches[v.Name] = new StopwatchEntry { Running = false, AccumMs = 0, StartTick = 0 };
                    _variables[v.Name] = 0; // elapsed ms, updated each tick
                }
                else if (v.IsString == true)
                {
                    _stringVariables[v.Name] = v.StringValue ?? "";
                }
                else
                {
                    // Persistent: restore saved value if available (keyed by programId:varName), else use declared default
                    double initialValue = isPersistent && savedPersistent.TryGetValue(persistPrefix + v.Name, out var saved)
                        ? saved
                        : v.Value;

                    if (isGlobal)
                    {
                        _globalVarNames.Add(v.Name);
                        _globalVars!.InitIfAbsent(v.Name, initialValue);
                    }
                    else
                        _variables[v.Name] = initialValue;

                    if (isPersistent) _persistentVarNames.Add(v.Name);
                    if (v.IsBoolean == true) _booleanVariables.Add(v.Name);
                }
            }
        }

        private void ExecuteCncProgram(ProgramStep step, StepListFrame frame)
        {
            var innerSteps = step.CncProgramSteps ?? new();
            if (innerSteps.Count == 0) { frame.Index++; ReportStepCompleted(step); return; }

            ReportStepCompleted(step);
            frame.Index++;

            _frameStack.Push(new StepListFrame(innerSteps, 0));
        }

        private void ExecuteLoop(ProgramStep step, StepListFrame frame)
        {
            var innerSteps = step.LoopSteps ?? new();
            if (innerSteps.Count == 0) { frame.Index++; ReportStepCompleted(step); return; }

            ReportStepCompleted(step); // the loop header itself is done; body steps count separately
            frame.Index++;

            if (step.LoopMode == "while" && step.LoopWhileCondition != null)
            {
                // Pre-check: if condition is already false, skip the body entirely
                if (!EvalWhileCondition(step.LoopWhileCondition))
                {
                    _loopDepth++; _loopDepth--;
                    return;
                }
                _frameStack.Push(new StepListFrame(innerSteps, 0, isLoop: true, loopRemaining: int.MaxValue,
                    whileCondition: step.LoopWhileCondition));
            }
            else if (step.LoopMode == "forEach" && !string.IsNullOrEmpty(step.ForEachVariableName))
            {
                // Determine iteration count from the source collection
                int count = 0;
                if (_listVariables.TryGetValue(step.ForEachVariableName, out var lst))
                    count = lst.Count;
                else if (_pointVariables.TryGetValue(step.ForEachVariableName, out var pts))
                    count = pts.Count;

                if (count == 0) { _loopDepth++; _loopDepth--; return; } // empty — skip body

                var bodyFrame = new StepListFrame(innerSteps, 0, isLoop: true, loopRemaining: 1,
                    isForEach: true, forEachCount: count, forEachCurrentIndex: 0,
                    forEachSourceVar: step.ForEachVariableName,
                    forEachValueVar:  step.ForEachValueVariableName ?? "",
                    forEachIndexVar:  step.ForEachIndexVariableName ?? "");
                _frameStack.Push(bodyFrame);
                InjectForEachVars(bodyFrame);
            }
            else
            {
                int count     = (int)EvalField(step, "loopCount", step.LoopCount ?? 1);
                int remaining = count == 0 ? int.MaxValue : count; // 0 = infinite

                // Initialise the count-loop index variable to 0 on first entry
                string indexVar = step.ForEachIndexVariableName ?? "";
                if (!string.IsNullOrEmpty(indexVar))
                    SetVariable(indexVar, 0);

                _frameStack.Push(new StepListFrame(innerSteps, 0, isLoop: true, loopRemaining: remaining,
                    forEachIndexVar: indexVar, forEachCount: count == 0 ? int.MaxValue : count));
            }

            _loopDepth++;
        }

        private void ExecuteSetSpeedL(ProgramStep step, StepListFrame frame)
        {
            bool hasSpeed = step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true;
            bool hasAccel = step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true;
            bool hasDecel = step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true;
            if (hasSpeed)
                _controller.QueuedCommands.Enqueue(new RobotCommand { CommandType = "SpeedS", Speed = EvalField(step, "speed", step.Speed ?? 0) }); // raw; override applied in MoveL
            if (hasAccel || hasDecel)
                _controller.QueuedCommands.Enqueue(new RobotCommand { CommandType = "AccelS",
                    Accel = hasAccel ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null,
                    Decel = hasDecel ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null });
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteSetSpeedJ(ProgramStep step, StepListFrame frame)
        {
            bool hasSpeed = step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true;
            bool hasAccel = step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true;
            bool hasDecel = step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true;
            if (hasSpeed)
                _controller.QueuedCommands.Enqueue(new RobotCommand { CommandType = "SpeedJ", Speed = EvalField(step, "speed", step.Speed ?? 0) }); // raw; override applied in MoveJ
            if (hasAccel || hasDecel)
                _controller.QueuedCommands.Enqueue(new RobotCommand { CommandType = "AccelJ",
                    Accel = hasAccel ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null,
                    Decel = hasDecel ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null });
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecutePauseProgram(ProgramStep step, StepListFrame frame)
        {
            frame.Index++;
            ReportStepCompleted(step);
            // Pause without clearing the frame stack so Resume() can continue from the next step
            _running         = false;
            _isPaused        = true;
            _awaitingMove    = false;
            _pendingStep     = null;
            _awaitingAuxMove = false;
            _pendingAuxStep  = null;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName         = _program!.Name,
                ProgramStatus       = global::ProgramStatus.Stopped,
                CurrentStepNumber   = _globalStepIndex,
                StepDescription     = "Paused — press Continue to resume",
                CurrentPointName    = "",
                CurrentOffsetX      = null, CurrentOffsetY   = null, CurrentOffsetZ   = null,
                CurrentOffsetRX     = null, CurrentOffsetRY  = null, CurrentOffsetRZ  = null,
                CurrentToolOffsetX  = null, CurrentToolOffsetY  = null, CurrentToolOffsetZ  = null,
                CurrentToolOffsetRX = null, CurrentToolOffsetRY = null, CurrentToolOffsetRZ = null,
            });
        }

        private void ExecuteLabel(ProgramStep step, StepListFrame frame)
        {
            // Labels are no-ops at runtime — they are only markers for GoToLabel
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteGoToLabel(ProgramStep step, StepListFrame frame)
        {
            if (string.IsNullOrEmpty(step.LabelId))
            {
                Finish(global::ProgramStatus.Error, "GoToLabel: no label ID set");
                return;
            }

            // Search current frame first, then walk up the stack toward the top-level frame.
            // frames[0] = current (deepest), frames[last] = top-level.
            var frames = _frameStack.ToArray();
            for (int fi = 0; fi < frames.Length; fi++)
            {
                var target = frames[fi];
                for (int i = 0; i < target.Steps.Count; i++)
                {
                    if (target.Steps[i].Type == StepType.Label &&
                        target.Steps[i].LabelId == step.LabelId)
                    {
                        // Unwind any frames above the target; fix _loopDepth for abandoned loops.
                        for (int k = 0; k < fi; k++)
                        {
                            if (frames[k].IsLoop) _loopDepth--;
                            _frameStack.Pop();
                        }
                        ReportStepCompleted(step);
                        target.Index = i; // Label step advances past itself on the next tick
                        return;
                    }
                }
            }

            Finish(global::ProgramStatus.Error,
                $"GoToLabel: label '{step.LabelName ?? step.LabelId}' not found");
        }

        private Dictionary<string, double> BuildIoVariables()
        {
            var io = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);

            io["stb.in1"]  = _controller.stb.Input1  ? 1.0 : 0.0;
            io["stb.in2"]  = _controller.stb.Input2  ? 1.0 : 0.0;
            io["stb.in3"]  = _controller.stb.Input3  ? 1.0 : 0.0;
            io["stb.in4"]  = _controller.stb.Input4  ? 1.0 : 0.0;
            io["stb.out1"] = _controller.stb.Output1 ? 1.0 : 0.0;
            io["stb.out2"] = _controller.stb.Output2 ? 1.0 : 0.0;
            io["stb.out3"] = _controller.stb.Output3 ? 1.0 : 0.0;
            io["stb.out4"] = _controller.stb.Output4 ? 1.0 : 0.0;

            var relays = _controller.RelayManager.GetRelayStates();
            if (relays != null)
                for (int ri = 0; ri < relays.Length && ri < 4; ri++)
                    io[$"relay.{ri + 1}"] = relays[ri] ? 1.0 : 0.0;

            foreach (var nano in _controller.NanoManager.GetAllStates())
                if (!string.IsNullOrEmpty(nano.Name))
                    foreach (var pin in nano.Pins)
                        if (!string.IsNullOrEmpty(pin.Name))
                            io[$"nano.{nano.Name}.{pin.Name}"] = pin.Value ? 1.0 : 0.0;

            return io;
        }

        private bool EvaluateConditionGroup(ConditionGroup group, Dictionary<string, double> vars)
        {
            if (group.Items.Count == 0) return true;
            bool isAny = group.Combinator == "ANY";
            foreach (var item in group.Items)
            {
                bool result = EvaluateConditionItem(item, vars);
                if (isAny && result)  return true;
                if (!isAny && !result) return false;
            }
            return !isAny;
        }

        private bool EvaluateConditionItem(ConditionItem item, Dictionary<string, double> vars)
        {
            // String operator path — also used when left side is a string variable
            bool isStringOp = item.Operator is "contains" or "startsWith" or "endsWith";
            if (isStringOp || IsStringVarRef(item.Left))
            {
                string ls = ResolveStringValue(item.Left);
                string rs = ResolveStringValue(item.Right);
                return item.Operator switch
                {
                    "==" => string.Equals(ls, rs, StringComparison.Ordinal),
                    "!=" => !string.Equals(ls, rs, StringComparison.Ordinal),
                    "contains"   => ls.Contains(rs, StringComparison.Ordinal),
                    "startsWith" => ls.StartsWith(rs, StringComparison.Ordinal),
                    "endsWith"   => ls.EndsWith(rs, StringComparison.Ordinal),
                    _    => false,
                };
            }

            // Unknown variables propagate (and error the program) — a typo'd condition
            // silently comparing 0 could take the wrong branch on a machine that moves.
            double left, right;
            try { left  = ExpressionEvaluator.Evaluate(item.Left,  vars, _listVariables, _pointVariables); }
            catch (UnknownVariableException) { throw; }
            catch { left  = 0; }
            try { right = ExpressionEvaluator.Evaluate(item.Right, vars, _listVariables, _pointVariables); }
            catch (UnknownVariableException) { throw; }
            catch { right = 0; }
            const double eps = 1e-9;
            return item.Operator switch
            {
                "==" => Math.Abs(left - right) < eps,
                "!=" => Math.Abs(left - right) >= eps,
                ">"  => left > right,
                ">=" => left >= right,
                "<"  => left < right,
                "<=" => left <= right,
                _    => false,
            };
        }

        private bool IsStringVarRef(string expr) =>
            !string.IsNullOrEmpty(expr) && expr.StartsWith('$') &&
            _stringVariables.ContainsKey(expr.Substring(1));

        private string ResolveStringValue(string expr)
        {
            if (string.IsNullOrEmpty(expr)) return "";
            if (expr.StartsWith('$') && _stringVariables.TryGetValue(expr.Substring(1), out var sv))
                return sv;
            return InterpolateVariables(expr);
        }

        private void ExecuteIfCondition(ProgramStep step, StepListFrame frame)
        {
            frame.Index++;
            ReportStepCompleted(step);

            var allVars = MergedVars();
            foreach (var kv in BuildIoVariables()) allVars[kv.Key] = kv.Value;

            if (step.Condition != null && EvaluateConditionGroup(step.Condition, allVars))
            {
                var body = step.IfSteps ?? new();
                if (body.Count > 0) _frameStack.Push(new StepListFrame(body, 0));
                return;
            }

            foreach (var elif in step.ElseIfBranches ?? [])
            {
                if (EvaluateConditionGroup(elif.Condition, allVars))
                {
                    if (elif.Steps.Count > 0) _frameStack.Push(new StepListFrame(elif.Steps, 0));
                    return;
                }
            }

            var elseBody = step.ElseSteps ?? new();
            if (elseBody.Count > 0) _frameStack.Push(new StepListFrame(elseBody, 0));
        }

        private void ExecuteSetTool(ProgramStep step, StepListFrame frame)
        {
            _controller.ApplyTool(step.ToolName);
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteSetLocal(ProgramStep step, StepListFrame frame)
        {
            if (string.IsNullOrEmpty(step.LocalName))
            {
                _activeLocal = null;
            }
            else
            {
                var local = _localRepo.Get(step.LocalName);
                if (local is null) { Finish(global::ProgramStatus.Error, $"Local not found: {step.LocalName}"); return; }
                _activeLocal = new Vector6(local.X, local.Y, local.Z, local.RX, local.RY, local.RZ);
            }
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteClearLocal(ProgramStep step, StepListFrame frame)
        {
            _activeLocal = null;
            ReportStepCompleted(step);
            frame.Index++;
        }

        private bool _homingTriggered = false;
        private void ExecuteRunHoming(ProgramStep step, StepListFrame frame)
        {
            if (!_homingTriggered)
            {
                _controller.TriggerHoming();
                _homingTriggered = true;
                return;
            }
            // Wait for homing to start, then wait for it to finish
            if (_controller.HomingState == "WaitingForStart")
            {
                // Either homing completed or hasn't started yet — if we triggered it, it's done
                // Guard: only advance after we've seen the state leave WaitingForStart at least once
                // Use a separate flag set when state left WaitingForStart
                if (_homingStartedMoving)
                {
                    _homingTriggered    = false;
                    _homingStartedMoving = false;
                    ReportStepCompleted(step);
                    frame.Index++;
                }
                // else: still waiting for homing state machine to pick up the trigger
            }
            else
            {
                _homingStartedMoving = true;
            }
        }
        private bool _homingStartedMoving = false;

        // ── Aux axis steps ────────────────────────────────────────────────────

        private void ExecuteAuxMove(ProgramStep step, StepListFrame frame)
        {
            string deviceId  = step.AuxDeviceId ?? _controller.AuxAxisManager.GetFirstDevice()?.Id ?? "";
            int    axisIndex = step.AuxAxisIndex ?? 0;

            var axisCfg = _controller.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);
            double spu   = axisCfg?.StepsPerUnit() ?? 0;

            long   steps;
            double velocity, accel, decel;

            if (!string.IsNullOrEmpty(step.AuxUnit) && step.AuxDistance.HasValue && spu > 0)
            {
                // Physical-unit mode — convert distance and rates to steps
                double dist = EvalField(step, "auxDistance", step.AuxDistance.Value);
                steps    = (long)Math.Round(dist * spu);
                velocity = EvalField(step, "auxVelocity", step.AuxVelocity ?? 10) * spu;
                accel    = EvalField(step, "auxAccel",    step.AuxAccel    ?? 50)  * spu;
                decel    = EvalField(step, "auxDecel",    step.AuxDecel    ?? accel / spu) * spu;
            }
            else
            {
                steps    = (long)EvalField(step, "auxSteps",    step.AuxSteps    ?? 0);
                velocity = EvalField(step, "auxVelocity", step.AuxVelocity ?? 1600);
                accel    = EvalField(step, "auxAccel",    step.AuxAccel    ?? 3200);
                decel    = EvalField(step, "auxDecel",    step.AuxDecel    ?? accel);
            }

            if (step.AuxAbsolute == true)
            {
                long currentPos = _controller.AuxAxisManager.GetPosition(deviceId, axisIndex);
                steps = steps - currentPos;
            }

            if (steps == 0) { ReportStepCompleted(step); frame.Index++; return; }

            // StartAuxMove derives direction from the sign of steps (and applies the
            // axis InvertDirection), then moves by the absolute amount. Pass the SIGNED
            // value — pre-abs'ing it here made every move go the same direction
            // regardless of a positive or negative distance.
            _controller.StartAuxMove(deviceId, axisIndex, steps, velocity, accel, decel);

            bool wait = step.AuxWaitForDone ?? true;
            if (wait)
            {
                ReportStepStarted(step);
                frame.Index++;
                _awaitingAuxMove = true;
                _pendingAuxStep  = step;
            }
            else
            {
                frame.Index++;
                ReportStepCompleted(step);
            }
        }

        private void ExecuteAuxContinuous(ProgramStep step, StepListFrame frame)
        {
            string deviceId  = step.AuxDeviceId ?? _controller.AuxAxisManager.GetFirstDevice()?.Id ?? "";
            int    axisIndex = step.AuxAxisIndex ?? 0;

            var axisCfg = _controller.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);
            double spu   = axisCfg?.StepsPerUnit() ?? 0;

            double velocity, accel;
            if (!string.IsNullOrEmpty(step.AuxUnit) && spu > 0)
            {
                velocity = EvalField(step, "auxVelocity", step.AuxVelocity ?? 10) * spu;
                accel    = EvalField(step, "auxAccel",    step.AuxAccel    ?? 50)  * spu;
            }
            else
            {
                velocity = EvalField(step, "auxVelocity", step.AuxVelocity ?? 1600);
                accel    = EvalField(step, "auxAccel",    step.AuxAccel    ?? 3200);
            }

            // StartAuxContinuous derives direction from the sign of velocity (and
            // applies InvertDirection), then ramps by the absolute value. Pass the
            // SIGNED velocity — abs'ing it here made CW and CCW go the same way.
            _controller.StartAuxContinuous(deviceId, axisIndex, velocity, accel);

            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteAuxStop(ProgramStep step, StepListFrame frame)
        {
            double decel     = EvalField(step, "auxDecel", step.AuxDecel ?? 10000);
            bool   immediate = step.AuxImmediate ?? false;

            if (!string.IsNullOrEmpty(step.AuxDeviceId) && step.AuxAxisIndex.HasValue)
            {
                // Stop a specific axis on a specific device
                if (immediate)
                    _controller.AuxAxisManager.StopAll(step.AuxDeviceId);
                else
                    _controller.AuxAxisManager.StopSmooth(step.AuxDeviceId, step.AuxAxisIndex.Value, (int)Math.Max(1, decel));
            }
            else
            {
                // Stop all axes on all devices
                if (immediate)
                    _controller.AuxAxisManager.StopAllDevices();
                else
                    _controller.StopAux(decel, false);
            }

            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteAuxEnable(ProgramStep step, StepListFrame frame)
        {
            string deviceId = step.AuxDeviceId ?? _controller.AuxAxisManager.GetFirstDevice()?.Id ?? "";
            bool   enable   = step.AuxEnable ?? true;
            if (!string.IsNullOrEmpty(deviceId))
                _controller.AuxAxisManager.Enable(deviceId, enable);
            ReportStepCompleted(step);
            frame.Index++;
        }

        // ── Thread move sub-step state ────────────────────────────────────────
        private int                  _threadSubStep   = 0;
        private Queue<RobotCommand>? _threadMoveQueue = null;

        // ── Jump sub-step state ───────────────────────────────────────────────
        private int     _jumpSubStep  = 0;  // 0=idle, 1=lift dispatched, 2=transit dispatched, 3=lower dispatched
        private Vector6 _jumpTarget   = new();
        private Vector6 _jumpStartPos = new();
        private double  _jumpZStart   = 0;
        private double  _jumpZEnd     = 0;
        private string  _jumpCmdType  = "MoveL";
        private double? _jumpSpeed, _jumpAccel, _jumpDecel;

        private void ExecuteSetVariable(ProgramStep step, StepListFrame frame)
        {
            if (!string.IsNullOrEmpty(step.VariableName) && !string.IsNullOrEmpty(step.VariableExpr))
            {
                if (_stringVariables.ContainsKey(step.VariableName))
                {
                    // String variable — treat expr as a template and interpolate $var tokens
                    _stringVariables[step.VariableName] = InterpolateVariables(step.VariableExpr);
                }
                else
                {
                    try
                    {
                        double value = ExpressionEvaluator.Evaluate(step.VariableExpr, EvalVars(), _listVariables, _pointVariables);
                        if (_globalVars != null && _globalVarNames.Contains(step.VariableName))
                            _globalVars.Set(step.VariableName, value);
                        else
                            _variables[step.VariableName] = value;
                    }
                    catch (UnknownVariableException)
                    {
                        throw; // errors the program via the dispatch-level handler
                    }
                    catch
                    {
                        // Malformed expression — leave variable unchanged
                    }
                }
            }
            ReportStepCompleted(step);
            frame.Index++;
        }

        // ── Background program step handlers ─────────────────────────────────

        private BuiltProgram? ResolveBackgroundProgram(ProgramStep step)
        {
            if (!string.IsNullOrEmpty(step.BackgroundProgramId))
                return _builtProgramRepo.GetById(step.BackgroundProgramId);
            if (!string.IsNullOrEmpty(step.BackgroundProgramName))
                return _builtProgramRepo.Get(step.BackgroundProgramName);
            return null;
        }

        private void ExecuteStartBackground(ProgramStep step, StepListFrame frame)
        {
            if (_backgroundManager != null)
            {
                var prog = ResolveBackgroundProgram(step);
                if (prog != null) _backgroundManager.TryStart(prog);
            }
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteStopBackground(ProgramStep step, StepListFrame frame)
        {
            if (_backgroundManager != null)
            {
                var prog = ResolveBackgroundProgram(step);
                if (prog != null) _backgroundManager.Stop(prog.Id);
            }
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteWaitForBackground(ProgramStep step, StepListFrame frame)
        {
            if (_backgroundManager == null)
            {
                ReportStepCompleted(step);
                frame.Index++;
                return;
            }
            var prog = ResolveBackgroundProgram(step);
            if (prog == null || !_backgroundManager.IsRunning(prog.Id))
            {
                ReportStepCompleted(step);
                frame.Index++;
                return;
            }
            // Still running — set the wait flag and yield; Update() will clear it when done
            _waitingForBackground = prog.Id;
        }

        private void ExecuteStopwatchControl(ProgramStep step, StepListFrame frame)
        {
            var varName = step.StopwatchVariableName;
            if (!string.IsNullOrEmpty(varName))
            {
                if (!_stopwatches.TryGetValue(varName, out var sw))
                    sw = new StopwatchEntry { Running = false, AccumMs = 0, StartTick = 0 };

                var now = System.Environment.TickCount64;
                sw = step.StopwatchAction switch
                {
                    "Start" when !sw.Running => new StopwatchEntry { Running = true,  AccumMs = sw.AccumMs, StartTick = now },
                    "Stop"  when  sw.Running => new StopwatchEntry { Running = false, AccumMs = sw.AccumMs + (now - sw.StartTick), StartTick = 0 },
                    "Reset"                  => new StopwatchEntry { Running = false, AccumMs = 0, StartTick = 0 },
                    _                        => sw, // Start when already running / Stop when already stopped — no-op
                };
                _stopwatches[varName] = sw;
                _variables[varName]   = sw.Running ? sw.AccumMs + (now - sw.StartTick) : sw.AccumMs;
            }

            ReportStepStarted(step);
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteSaveImage(ProgramStep step, StepListFrame frame)
        {
            var pathTemplate = step.SaveImagePath ?? "";
            if (string.IsNullOrEmpty(pathTemplate))
            {
                ReportStepStarted(step);
                ReportStepCompleted(step);
                frame.Index++;
                return;
            }

            var resolvedPath = InterpolateVariables(pathTemplate);
            if (!System.IO.Path.IsPathRooted(resolvedPath))
                resolvedPath = System.IO.Path.Combine(AppContext.BaseDirectory, resolvedPath);

            var cameraId = step.SaveImageCameraId ?? "";
            var camera = string.IsNullOrEmpty(cameraId)
                ? _controller.CameraManager.GetFirstCamera()
                : _controller.CameraManager.GetCamera(cameraId);

            var frameBytes = camera?.GetLatestFrame();
            if (frameBytes == null || frameBytes.Length == 0)
            {
                Finish(global::ProgramStatus.Error,
                    $"SaveImage: no frame available from camera '{(string.IsNullOrEmpty(cameraId) ? "default" : cameraId)}'");
                return;
            }

            try
            {
                var dir = System.IO.Path.GetDirectoryName(resolvedPath);
                if (!string.IsNullOrEmpty(dir))
                    System.IO.Directory.CreateDirectory(dir);
                System.IO.File.WriteAllBytes(resolvedPath, frameBytes);
            }
            catch (Exception ex)
            {
                Finish(global::ProgramStatus.Error, $"SaveImage failed writing '{resolvedPath}': {ex.Message}");
                return;
            }

            ReportStepStarted(step);
            ReportStepCompleted(step);
            frame.Index++;
        }

        // ── Helpers ──────────────────────────────────────────────────────────

        /// <summary>Merged program/global variables plus live IO values — the full
        /// dictionary for expression evaluation (matches what conditions see).</summary>
        private Dictionary<string, double> EvalVars()
        {
            var vars = MergedVars();
            foreach (var kv in BuildIoVariables()) vars[kv.Key] = kv.Value;
            return vars;
        }

        /// <summary>
        /// Returns the evaluated value for a numeric field.
        /// If the step has an expression keyed by <paramref name="fieldName"/>, that expression is
        /// evaluated against the current variable dictionary; otherwise <paramref name="fallback"/> is returned.
        /// An unknown variable in the expression propagates (and errors the program) rather than
        /// silently falling back — a typo'd offset must never move the robot to the wrong place.
        /// </summary>
        private double EvalField(ProgramStep step, string fieldName, double fallback)
        {
            if (step.Expressions != null && step.Expressions.TryGetValue(fieldName, out var expr))
            {
                try { return ExpressionEvaluator.Evaluate(expr, EvalVars(), _listVariables, _pointVariables); }
                catch (UnknownVariableException) { throw; }
                catch { /* malformed expression — fall through to the literal */ }
            }
            return fallback;
        }

        private int _globalStepIndex = 0;
        private int _loopDepth       = 0;

        /// <summary>Emits a "step in progress" update — description shown but count not yet incremented.</summary>
        private void ReportStepStarted(ProgramStep step)
        {
            var isMove = step.Type == StepType.MoveL || step.Type == StepType.MoveJ
                      || step.Type == StepType.JumpL || step.Type == StepType.JumpJ;

            double? Off(string key, double? raw) =>
                isMove && (raw.HasValue || (step.Expressions?.ContainsKey(key) == true))
                    ? EvalField(step, key, raw ?? 0)
                    : (double?)null;

            var desc = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step);
            CurrentStepDescription = desc;

            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName          = _program!.Name,
                ProgramStatus        = global::ProgramStatus.Running,
                CurrentStepNumber    = _globalStepIndex,
                StepDescription      = desc,
                WarningDescription   = string.IsNullOrEmpty(step.StatusWarning) ? null : step.StatusWarning,
                ErrorDescription     = string.IsNullOrEmpty(step.StatusError)   ? null : step.StatusError,
                CurrentPointName     = isMove ? (step.PointName ?? "") : null,
                CurrentOffsetX       = Off("offsetX",       step.OffsetX),
                CurrentOffsetY       = Off("offsetY",       step.OffsetY),
                CurrentOffsetZ       = Off("offsetZ",       step.OffsetZ),
                CurrentOffsetRX      = Off("offsetRX",      step.OffsetRX),
                CurrentOffsetRY      = Off("offsetRY",      step.OffsetRY),
                CurrentOffsetRZ      = Off("offsetRZ",      step.OffsetRZ),
                CurrentToolOffsetX   = Off("toolOffsetX",   step.ToolOffsetX),
                CurrentToolOffsetY   = Off("toolOffsetY",   step.ToolOffsetY),
                CurrentToolOffsetZ   = Off("toolOffsetZ",   step.ToolOffsetZ),
                CurrentToolOffsetRX  = Off("toolOffsetRX",  step.ToolOffsetRX),
                CurrentToolOffsetRY  = Off("toolOffsetRY",  step.ToolOffsetRY),
                CurrentToolOffsetRZ  = Off("toolOffsetRZ",  step.ToolOffsetRZ),
            });
        }

        /// <summary>Increments the completed step count and emits the updated progress.</summary>
        private void ReportStepCompleted(ProgramStep step)
        {
            if (_loopDepth == 0) _globalStepIndex++;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName        = _program!.Name,
                ProgramStatus      = global::ProgramStatus.Running,
                CurrentStepNumber  = _globalStepIndex,
                StepDescription    = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step),
                WarningDescription = string.IsNullOrEmpty(step.StatusWarning) ? null : step.StatusWarning,
                ErrorDescription   = string.IsNullOrEmpty(step.StatusError)   ? null : step.StatusError,
                ShouldLog          = true,
            });
        }

        private void Finish(global::ProgramStatus status, string description)
        {
            SavePersistentVars();
            _running              = false;
            _stopRequested        = false;
            _awaitingMove         = false;
            _pendingStep          = null;
            _awaitingAuxMove      = false;
            _pendingAuxStep       = null;
            _awaitingVision       = false;
            _visionProgramId      = null;
            _waitingForBackground = null;
            _frameStack.Clear();

            // Main program finishing: optionally kill all background programs
            if (!_isBackground && (_program?.KillBackgroundOnStop ?? true))
                _backgroundManager?.StopAll();

            // Notify manager so it removes this executor from the running set (keyed by ID)
            _backgroundManager?.OnExecutorFinished(_program?.Id ?? "");

            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName          = _program!.Name,
                ProgramStatus        = status,
                CurrentStepNumber    = _globalStepIndex,
                StepDescription      = description,
                ErrorDescription     = status == global::ProgramStatus.Error ? description : null,
                CurrentPointName  = "",
                CurrentOffsetX   = null, CurrentOffsetY  = null, CurrentOffsetZ  = null,
                CurrentOffsetRX  = null, CurrentOffsetRY = null, CurrentOffsetRZ = null,
                CurrentToolOffsetX  = null, CurrentToolOffsetY  = null, CurrentToolOffsetZ  = null,
                CurrentToolOffsetRX = null, CurrentToolOffsetRY = null, CurrentToolOffsetRZ = null,
            });

            _globalStepIndex = 0;
            _loopDepth       = 0;
        }

        private static string StepDescription(ProgramStep step)
        {
            var type = step.Type switch
            {
                StepType.MoveL        => $"MoveL → {step.PointName}",
                StepType.MoveJ        => $"MoveJ → {step.PointName}",
                StepType.JumpL        => $"JumpL → {step.PointName}",
                StepType.JumpJ        => $"JumpJ → {step.PointName}",
                StepType.SetOutput    => BuildSetOutputDescription(step),
                StepType.Wait         => $"Wait {step.WaitMs} ms",
                StepType.Loop         => $"Loop ×{(step.LoopCount == 0 ? "∞" : step.LoopCount)}",
                StepType.StatusUpdate => step.StatusMessage ?? "Status update",
                StepType.CallRoutine  => $"Routine → {step.RoutineName}",
                StepType.SetSpeedL    => $"Set Linear Speed → {step.Speed} mm/s",
                StepType.SetSpeedJ    => $"Set Joint Speed → {step.Speed} mm/s",
                StepType.SetVariable  => $"${step.VariableName} = {step.VariableExpr}",
                StepType.PauseProgram => "Pause Program",
                StepType.Label        => $"Label: {step.LabelName ?? step.LabelId}",
                StepType.GoToLabel    => $"Go To: {step.LabelName ?? step.LabelId}",
                StepType.IfCondition  => step.Condition != null
                    ? $"If [{step.Condition.Combinator} · {step.Condition.Items.Count} condition(s)]"
                    : "If Condition",
                StepType.SetTool      => $"Set Tool → {(string.IsNullOrEmpty(step.ToolName) ? "None" : step.ToolName)}",
                StepType.SetLocal     => $"Set Local → {(string.IsNullOrEmpty(step.LocalName) ? "None" : step.LocalName)}",
                StepType.ClearLocal   => "Clear Local",
                StepType.RunHoming     => "Run Homing",
                StepType.AuxMove       => !string.IsNullOrEmpty(step.AuxUnit) && step.AuxDistance.HasValue
                    ? $"Aux Move · axis {step.AuxAxisIndex} · {step.AuxDistance} {step.AuxUnit}"
                    : $"Aux Move · axis {step.AuxAxisIndex} · {step.AuxSteps} steps",
                StepType.AuxContinuous => $"Aux Continuous · axis {step.AuxAxisIndex}",
                StepType.AuxStop       => $"Aux Stop · device {step.AuxDeviceId ?? "default"}",
                StepType.AuxEnable     => $"Aux Motors {(step.AuxEnable == true ? "ON" : "OFF")}",
                StepType.RunVision          => $"Vision → {step.VisionProgramName ?? step.VisionProgramId ?? "?"}",
                StepType.StartBackground    => $"Start Background → {step.BackgroundProgramName ?? "?"}",
                StepType.StopBackground     => $"Stop Background → {step.BackgroundProgramName ?? "?"}",
                StepType.WaitForBackground  => $"Wait for Background → {step.BackgroundProgramName ?? "?"}",
                StepType.StopwatchControl   => $"Stopwatch {step.StopwatchAction ?? "?"} → ${step.StopwatchVariableName ?? "?"}",
                StepType.SaveImage          => $"Save Image → {step.SaveImagePath ?? "?"}",
                _                           => step.Type.ToString(),
            };
            return string.IsNullOrEmpty(step.Name) ? type : $"{step.Name}  ({type})";
        }

        private static string BuildSetOutputDescription(ProgramStep step)
        {
            var state = step.OutputValue == true ? "ON" : "OFF";
            var base_ = step.OutputCard switch {
                "relay" => $"Relay {step.OutputNumber} → {state}",
                "nano"  => $"Nano Output {step.OutputNumber} → {state}",
                _       => $"STB Output {step.OutputNumber} → {state}",
            };
            var pulseSuffix = (step.PulseMs ?? 0) > 0
                ? $"  (pulse {step.PulseMs} ms{(step.PulseBlocking == true ? ", blocking" : "")})"
                : "";
            return $"{base_}{pulseSuffix}";
        }

        internal static int CountSteps(List<ProgramStep> steps, BuiltProgramRepository? repo = null)
        {
            int count = 0;
            foreach (var s in steps)
            {
                count++;
                if (s.Type == StepType.CallRoutine && repo != null)
                {
                    var routine = !string.IsNullOrEmpty(s.RoutineId) ? repo.GetById(s.RoutineId!) : null;
                    routine ??= repo.Get(s.RoutineName ?? "");
                    if (routine != null) count += CountSteps(routine.Steps, repo);
                }
                if (s.Type == StepType.IfCondition)
                {
                    if (s.IfSteps != null) count += CountSteps(s.IfSteps, repo);
                    foreach (var elif in s.ElseIfBranches ?? []) count += CountSteps(elif.Steps, repo);
                    if (s.ElseSteps != null) count += CountSteps(s.ElseSteps, repo);
                }
            }
            return count;
        }

        // ── Inner frame class ─────────────────────────────────────────────────
        private class StepListFrame
        {
            public List<ProgramStep> Steps             { get; }
            public int               Index             { get; set; }
            public bool              IsLoop            { get; }
            public int               LoopRemaining     { get; set; }
            public bool              WaitStarted       { get; set; }

            // ForEach loop support
            public bool   IsForEach          { get; }
            public int    ForEachCount        { get; }
            public int    ForEachCurrentIndex { get; set; }
            public string ForEachSourceVar    { get; }
            public string ForEachValueVar     { get; }
            public string ForEachIndexVar     { get; }

            // While loop support
            public ConditionGroup? WhileCondition { get; }

            public StepListFrame(
                List<ProgramStep> steps, int index,
                bool isLoop = false, int loopRemaining = 0,
                bool isForEach = false, int forEachCount = 0, int forEachCurrentIndex = 0,
                string forEachSourceVar = "", string forEachValueVar = "", string forEachIndexVar = "",
                ConditionGroup? whileCondition = null)
            {
                Steps               = steps;
                Index               = index;
                IsLoop              = isLoop;
                LoopRemaining       = loopRemaining;
                IsForEach           = isForEach;
                ForEachCount        = forEachCount;
                ForEachCurrentIndex = forEachCurrentIndex;
                ForEachSourceVar    = forEachSourceVar;
                ForEachValueVar     = forEachValueVar;
                ForEachIndexVar     = forEachIndexVar;
                WhileCondition      = whileCondition;
            }
        }
    }
}
