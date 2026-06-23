using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl
{
    /// <summary>
    /// Thread-safe store for scalar variables shared across all concurrently running programs.
    /// First writer wins on initialisation; subsequent writes update the shared value.
    /// </summary>
    internal class GlobalVariableStore
    {
        private readonly Dictionary<string, double> _vars = new(System.StringComparer.OrdinalIgnoreCase);
        private readonly object _lock = new();

        public bool TryGet(string name, out double value)
        {
            lock (_lock) return _vars.TryGetValue(name, out value);
        }

        public void Set(string name, double value)
        {
            lock (_lock) _vars[name] = value;
        }

        /// <summary>Sets the value only if the key does not already exist (first-program-wins on start).</summary>
        public void InitIfAbsent(string name, double value)
        {
            lock (_lock) { if (!_vars.ContainsKey(name)) _vars[name] = value; }
        }

        /// <summary>Returns a point-in-time copy safe to iterate without holding the lock.</summary>
        public Dictionary<string, double> Snapshot()
        {
            lock (_lock) return new Dictionary<string, double>(_vars, System.StringComparer.OrdinalIgnoreCase);
        }

        public void Clear()
        {
            lock (_lock) _vars.Clear();
        }
    }

    /// <summary>Status snapshot for one running background program.</summary>
    public class BackgroundProgramStatus
    {
        public string Name        { get; set; } = "";
        public string CurrentStep { get; set; } = "";
    }

    /// <summary>
    /// Manages a set of concurrently running background <see cref="ProgramExecutor"/> instances.
    /// Enforces the single-instance-per-program-name rule.
    /// </summary>
    internal class BackgroundProgramManager
    {
        // Dependencies forwarded to each background executor
        private readonly RobotController        _controller;
        private readonly ProgramCycleManager    _programManager;
        private readonly PointRepository        _pointRepo;
        private readonly ToolRepository         _toolRepo;
        private readonly LocalRepository        _localRepo;
        private readonly BuiltProgramRepository _builtProgramRepo;
        private readonly GridRepository         _gridRepo;
        private readonly StackRepository        _stackRepo;

        public readonly GlobalVariableStore GlobalVars = new();

        private readonly Dictionary<string, ProgramExecutor> _running = new();
        private readonly object _lock = new();

        public BackgroundProgramManager(
            RobotController        controller,
            ProgramCycleManager    programManager,
            PointRepository        pointRepo,
            ToolRepository         toolRepo,
            LocalRepository        localRepo,
            BuiltProgramRepository builtProgramRepo,
            GridRepository         gridRepo,
            StackRepository        stackRepo)
        {
            _controller       = controller;
            _programManager   = programManager;
            _pointRepo        = pointRepo;
            _toolRepo         = toolRepo;
            _localRepo        = localRepo;
            _builtProgramRepo = builtProgramRepo;
            _gridRepo         = gridRepo;
            _stackRepo        = stackRepo;
        }

        /// <summary>
        /// Attempts to start the given program as a background task.
        /// Returns false (no-op) if an instance with the same name is already running.
        /// </summary>
        public bool TryStart(BuiltProgram program, string? imageBase64 = null)
        {
            lock (_lock)
            {
                if (_running.ContainsKey(program.Name)) return false;

                var executor = new ProgramExecutor(
                    _controller, _programManager, _pointRepo, _toolRepo, _localRepo,
                    _builtProgramRepo, _gridRepo, _stackRepo,
                    isBackground: true, globalVars: GlobalVars, backgroundManager: this);

                _running[program.Name] = executor;
                executor.Start(program, imageBase64);
                return true;
            }
        }

        /// <summary>Stops the named background program. No-op if not running.</summary>
        public void Stop(string name)
        {
            ProgramExecutor? executor;
            lock (_lock) _running.TryGetValue(name, out executor);
            executor?.Stop();
        }

        /// <summary>Stops all running background programs (called when the main program finishes with KillBackgroundOnStop=true).</summary>
        public void StopAll()
        {
            List<ProgramExecutor> executors;
            lock (_lock) executors = _running.Values.ToList();
            foreach (var e in executors) e.Stop();
        }

        /// <summary>Called by a background executor when it finishes (any terminal state).</summary>
        public void OnExecutorFinished(string name)
        {
            lock (_lock) _running.Remove(name);
        }

        /// <returns>True if a background program with this name is currently running.</returns>
        public bool IsRunning(string name)
        {
            lock (_lock) return _running.TryGetValue(name, out var e) && e.IsRunning;
        }

        /// <summary>Advances all background executors by one tick. Must be called from the control-loop thread.</summary>
        public void Update()
        {
            List<ProgramExecutor> executors;
            lock (_lock) executors = _running.Values.ToList();
            foreach (var e in executors) e.Update();
        }

        /// <summary>Returns a status snapshot for the UI (names of currently running programs + current step).</summary>
        public List<BackgroundProgramStatus> GetStatuses()
        {
            lock (_lock)
                return _running.Select(kv => new BackgroundProgramStatus
                {
                    Name        = kv.Key,
                    CurrentStep = kv.Value.CurrentStepDescription,
                }).ToList();
        }

        /// <summary>Returns just the names of running background programs.</summary>
        public List<string> GetRunningNames()
        {
            lock (_lock) return _running.Keys.ToList();
        }

        /// <summary>Returns display variables for a named running background program.</summary>
        public IReadOnlyList<(string Name, double Value, bool IsBoolean)> GetDisplayVariables(string name)
        {
            ProgramExecutor? executor;
            lock (_lock) _running.TryGetValue(name, out executor);
            return executor?.GetDisplayVariables() ?? [];
        }

        /// <summary>Resets global variable state — called when no programs are running.</summary>
        public void ClearGlobals() => GlobalVars.Clear();
    }
}
