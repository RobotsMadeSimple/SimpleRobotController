using Controller.RobotControl.Alarms;
using Controller.RobotControl.Programs;
using Controller.RobotControl.Safety;

namespace Controller.RobotControl
{
    /// <summary>
    /// Hardwired e-stop monitoring (docs/estop.md) and the alarm watch: a separate "SafetyMonitor"
    /// thread reads the e-stop input from state the controller already has (the STB4100's published
    /// inputs, an edge device's pin state) — it adds no work to the STB loops or the motion loop
    /// (docs/stb-loop-timing.md). On a trip it runs the HardStop path, aborts every program with the
    /// uncatchable <c>emergencyStop</c> error, marks the robot unhomed, fires <c>robot.estop</c> and
    /// raises an alarm. The same thread turns device disconnects, joint-limit faults and plugin
    /// crashes into alarms.
    /// </summary>
    internal partial class RobotController
    {
        /// <summary>How often the safety thread reads the e-stop input. The STB publishes its inputs
        /// every ~20 ms; this samples comfortably faster without adding load anywhere else.</summary>
        internal const int SafetyTickMs = 5;
        /// <summary>How often the slower alarm watch (devices, faults, plugins) runs.</summary>
        private const long AlarmWatchPeriodMs = 100;

        public AlarmHistory Alarms { get; private set; } = null!;
        internal EstopMonitor Estop { get; } = new();

        // Motion inhibit, mirrored from the e-stop state each safety tick (and set the instant a trip
        // is detected). Read on the motion thread by RunCommands — one volatile read per tick.
        private volatile bool _estopInhibit;

        // A restore was applied and the controller is about to restart: only reads are served.
        private volatile bool _restorePending;

        /// <summary>Test seam: replaces the e-stop input read (there is no STB or Nano in a test).</summary>
        internal Func<EstopReading>? EstopReaderOverride { get; set; }

        /// <summary>Test seam: the clock the safety tick uses.</summary>
        internal Func<long> SafetyClock { get; set; } = () => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        private long _lastAlarmWatchMs;

        private void InitSafety()
        {
            Alarms = new AlarmHistory("alarms.json");
        }

        // ── Test seams ────────────────────────────────────────────────────────
        /// <summary>Tests: an alarm history in a temp folder instead of the working directory's.</summary>
        internal void UseAlarmHistoryForTest(AlarmHistory history) => Alarms = history;
        /// <summary>Tests: run what was posted to the motion thread (no motion loop runs in a test).</summary>
        internal void RunPostedMotionActionsForTest() => DrainControlThreadActions();
        internal bool HardStopPendingForTest => _hardStopRequested;

        private void StartSafetyThread() =>
            new Thread(SafetyLoop) { IsBackground = true, Name = "SafetyMonitor" }.Start();

        private void SafetyLoop()
        {
            while (true)
            {
                try { SafetyTick(); }
                catch (Exception ex) { Console.WriteLine($"[Safety] Tick failed: {ex}"); }
                Thread.Sleep(SafetyTickMs);
            }
        }

        /// <summary>One pass of the safety monitor: the e-stop, then (every 100 ms) the alarm watch.
        /// Runs on the SafetyMonitor thread; tests call it directly.</summary>
        internal void SafetyTick()
        {
            // The thread, GetEstopConfig and SetEstopConfig all tick; one at a time.
            lock (_safetyTickLock) SafetyTickCore();
        }

        private readonly object _safetyTickLock = new();

        private void SafetyTickCore()
        {
            long now = SafetyClock();
            var cfg = _config.Estop ?? new EstopConfig();
            var reading = ReadEstopInput(cfg);
            bool latchedNow = Estop.Evaluate(cfg, reading, now);
            var state = Estop.State;
            // React on a new latch, and also whenever the input becomes tripped without latching
            // (monitoring turned on while open, a second press before reset): the drivers may have
            // lost power either way, so the same stop / abort / unhome runs.
            // At boot (unhomed, idle, source not connected yet) there is nothing to react to and no
            // alarm is raised; motion is blocked by the inhibit below until the input reads healthy.
            if (latchedNow
                || (state.Active && !_estopWasActive
                    && (Homed || IsMoving || MotionBusy || programExecutor?.IsRunning == true
                        || backgroundProgramManager.GetRunningIds().Count > 0)))
                OnEstopTripped(cfg, reading);
            // An active state that never latched clears its alarm when it goes away; a latched
            // one clears on ResetEstop.
            if (_estopWasActive && !state.Active && !state.Latched)
                Alarms.ClearCondition("estop");
            _estopWasActive = state.Active;

            bool inhibit = Estop.Inhibits;
            _estopInhibit = inhibit;
            // Belt and braces: nothing may move while active or latched. A move that slipped in
            // between the trip and the program abort is stopped here on the next tick.
            if (inhibit && (IsMoving || MotionBusy || startHoming || _homing.IsActive))
                HardStop();
            // A program start that raced the trip past the command gate ends here.
            if (inhibit && (programExecutor?.IsRunning == true || backgroundProgramManager.GetRunningIds().Count > 0))
                AbortProgramsForEstop(state.Reason ?? "Emergency stop");

            if (now - _lastAlarmWatchMs >= AlarmWatchPeriodMs)
            {
                _lastAlarmWatchMs = now;
                AlarmWatchTick(now);
            }
        }

        /// <summary>The e-stop input as the controller already knows it — no device I/O here.</summary>
        internal EstopReading ReadEstopInput(EstopConfig cfg)
        {
            if (EstopReaderOverride is { } fake) return fake();

            if (cfg.Source == EstopConfig.SourceIo)
            {
                string id  = cfg.DeviceId ?? "";
                int    pin = cfg.Pin ?? -1;
                if (NanoManager.GetDevice(id) is { } nano)
                {
                    var state = nano.GetPinState(pin);
                    string label = $"{(string.IsNullOrEmpty(nano.Name) ? id : nano.Name)} pin {pin}";
                    return new EstopReading(nano.Connected && state != null, state?.Value, label);
                }
                if (AuxAxisManager.GetDevice(id) is { } aux)
                {
                    var state = aux.GetIoPinStates().Find(p => p.Pin == pin);
                    string label = $"{(string.IsNullOrEmpty(aux.Name) ? id : aux.Name)} pin {pin}";
                    return new EstopReading(aux.Connected && state != null, state?.Value, label);
                }
                return new EstopReading(false, null, $"device '{id}' pin {pin}");
            }

            int n = Math.Clamp(cfg.StbInput, 1, 4);
            bool raw = n switch { 1 => stb.Input1, 2 => stb.Input2, 3 => stb.Input3, _ => stb.Input4 };
            return new EstopReading(stb.connected, raw, $"STB input {n}");
        }

        /// <summary>
        /// The e-stop reaction. Same path as HardStop (motion, aux), then: abort the foreground and
        /// background programs with emergencyStop (uncatchable), mark every joint unreferenced (the
        /// drivers lost power, so the arm may have moved), fire robot.estop and raise the alarm.
        /// </summary>
        private void OnEstopTripped(EstopConfig cfg, EstopReading reading)
        {
            _estopInhibit = true;
            HardStop();
            RequestQueueDrain();
            PostToMotionThread(() =>
            {
                ClearAllReferenced();
                startHoming = false;
            });

            var state   = Estop.State;
            string why  = state.Reason ?? (state.SourceOnline ? $"E-stop input ({reading.Label}) tripped" : $"E-stop source offline ({reading.Label})");
            Console.WriteLine($"[Safety] EMERGENCY STOP — {why}");

            AbortProgramsForEstop(why);

            try { AuxAxisManager.StopAllDevices(); }
            catch (Exception ex) { Console.WriteLine($"[Safety] Aux stop failed: {ex.Message}"); }

            PublishEstopEvent(state);
            Alarms.Raise(ErrorCodes.EmergencyStop, $"{why}. Motion stopped and every program was aborted. The robot must be homed again.",
                new AlarmSource { DeviceId = cfg.Source == EstopConfig.SourceIo ? cfg.DeviceId : "stb" },
                conditionKey: "estop");
        }

        private bool _estopWasActive;
        private int  _estopAbortInFlight;

        /// <summary>Ends the foreground and every background program with emergencyStop — on the
        /// thread pool, since executor locks may wait on a step doing I/O.</summary>
        private void AbortProgramsForEstop(string why)
        {
            if (Interlocked.Exchange(ref _estopAbortInFlight, 1) == 1) return;
            var exec = programExecutor;
            var bg   = backgroundProgramManager;
            _ = Task.Run(() =>
            {
                string msg = $"{why}. The program was stopped; reset the e-stop and home the robot before running again.";
                try { exec?.AbortWithError(ErrorCodes.EmergencyStop, msg); }
                catch (Exception ex) { Console.WriteLine($"[Safety] Program abort failed: {ex.Message}"); }
                try { bg.AbortAll(ErrorCodes.EmergencyStop, msg); }
                catch (Exception ex) { Console.WriteLine($"[Safety] Background abort failed: {ex.Message}"); }
                finally { Interlocked.Exchange(ref _estopAbortInFlight, 0); }
            });
        }

        private void PublishEstopEvent(EstopMonitor.Snapshot s)
        {
            try
            {
                PluginManager?.PublishEvent(Plugins.PluginEvents.RobotEstop, new
                {
                    active  = s.Active,
                    latched = s.Latched,
                    reason  = s.Reason,
                    source  = s.Label,
                });
            }
            catch (Exception ex) { Console.WriteLine($"[Safety] robot.estop event failed: {ex.Message}"); }
        }

        /// <summary>ResetEstop: clears the latch once the input reads healthy. Null on success, else why not.</summary>
        internal string? ResetEstop()
        {
            var cfg = _config.Estop ?? new EstopConfig();
            bool wasLatched = Estop.State.Latched;
            var error = Estop.TryReset(cfg, ReadEstopInput(cfg));
            _estopInhibit = Estop.Inhibits;
            if (error != null) return error;
            if (wasLatched)
            {
                Alarms.ClearCondition("estop");
                PublishEstopEvent(Estop.State);
                Console.WriteLine("[Safety] E-stop reset — home the robot before moving it.");
            }
            return null;
        }

        /// <summary>Replaces the e-stop config, persists it with the robot config and re-evaluates now.</summary>
        internal void ApplyEstopConfig(EstopConfig cfg)
        {
            var c = _config;
            c.Estop = cfg;
            RobotConfigService.Save(c);
            SafetyTick();
        }

        // ── Command gate ──────────────────────────────────────────────────────

        // Program starts and anything that drives the arm or an aux axis. Names starting with
        // "Move" or "Jog" are gated too (MoveL, MoveJ, MoveAux, JogL, JogAux, …).
        private static readonly HashSet<string> EstopGatedCommands = new(StringComparer.Ordinal)
        {
            "ExecuteBuiltProgram", "StartBackgroundProgram", "StartProgram", "StepBuiltProgram",
            "RunPluginStep", "Home", "SetHomed", "SetJointPosition", "OffsetL", "StartContinuous", "SiloAction",
        };

        private static bool IsEstopGated(string name) =>
            EstopGatedCommands.Contains(name)
            || name.StartsWith("Move", StringComparison.Ordinal)
            || name.StartsWith("Jog", StringComparison.Ordinal);

        /// <summary>
        /// The rejection for <paramref name="command"/> while the e-stop is active or latched (or a
        /// restore is restarting the controller), or null when it may run.
        /// </summary>
        internal object? SafetyGate(string? command)
        {
            if (command is null) return null;
            if (_restorePending && !command.StartsWith("Get", StringComparison.Ordinal))
                return new { ok = false, error = "restoring", message = "A backup was restored and the controller is restarting." };
            if (!IsEstopGated(command)) return null;
            var s = Estop.State;
            if (s.Active)
                return new { ok = false, error = "estopActive",
                    message = $"Emergency stop is active ({s.Reason ?? s.Label}). Release the e-stop, reset it, then home the robot." };
            if (s.Latched)
                return new { ok = false, error = "estopLatched",
                    message = "The emergency stop has not been reset. Reset it, then home the robot." };
            return null;
        }

        /// <summary>Blocks every command but reads until the process restarts (a restore was applied).</summary>
        private readonly object _restoreLock = new();

        /// <summary>Marks a restore as started; false when one already is (a second upload is refused).</summary>
        internal bool TryMarkRestorePending()
        {
            lock (_restoreLock)
            {
                if (_restorePending) return false;
                _restorePending = true;
                return true;
            }
        }

        /// <summary>A restore failed: serve commands again (Apply rolls the data back itself).</summary>
        internal void ClearRestorePending() => _restorePending = false;

        /// <summary>True when a program is running (foreground or background) or the arm is moving/homing.</summary>
        internal bool IsBusyForRestore =>
            programExecutor?.IsRunning == true
            || backgroundProgramManager.GetRunningIds().Count > 0
            || IsMoving || MotionBusy || HomingRequestedOrActive || IsAuxMoving;

        // ── Alarm watch ───────────────────────────────────────────────────────

        private bool _watchStbSeen, _watchStbConnected;
        private bool _watchFaulted;
        private readonly Dictionary<string, bool> _watchDevices = new(StringComparer.Ordinal);
        private readonly Dictionary<string, Plugins.PluginState> _watchPlugins = new(StringComparer.OrdinalIgnoreCase);

        /// <summary>Device connect/disconnect, joint-limit fault and plugin crash transitions → alarms.</summary>
        private void AlarmWatchTick(long now)
        {
            // Motor driver (STB). Only after it has been connected once — a box with no driver
            // attached is not an alarm.
            bool stbNow = stb.connected;
            if (stbNow) _watchStbSeen = true;
            if (_watchStbSeen && stbNow != _watchStbConnected)
            {
                if (!stbNow)
                    Alarms.Raise(ErrorCodes.DriverDisconnected, "The motor driver board disconnected. Motion is unavailable until it reconnects; home the robot again afterwards.",
                        new AlarmSource { DeviceId = "stb" }, conditionKey: "device:stb");
                else
                    Alarms.ClearCondition("device:stb");
            }
            _watchStbConnected = stbNow;

            // Edge devices (IO boards and aux stepper boards), by stable id.
            void Device(string id, string name, bool connected)
            {
                if (_watchDevices.TryGetValue(id, out var was))
                {
                    if (was && !connected)
                        Alarms.Raise(AlarmCatalog.DeviceDisconnected, $"{(string.IsNullOrEmpty(name) ? id : name)} disconnected.",
                            new AlarmSource { DeviceId = id }, conditionKey: "device:" + id);
                    else if (!was && connected)
                        Alarms.ClearCondition("device:" + id);
                }
                // Track only devices that have connected at least once.
                if (connected || _watchDevices.ContainsKey(id)) _watchDevices[id] = connected;
            }
            foreach (var d in NanoManager.GetDevices()) Device(d.Id, d.Name, d.Connected);
            foreach (var d in AuxAxisManager.GetAllIoStates()) Device(d.Id, d.Name, d.Connected);

            // Joint soft-limit fault.
            var fault = FaultStatus;
            if (fault.Faulted != _watchFaulted)
            {
                if (fault.Faulted)
                    Alarms.Raise(ErrorCodes.JointLimit, string.IsNullOrEmpty(fault.Message) ? "A joint reached its soft limit." : fault.Message,
                        conditionKey: "jointLimit");
                else
                    Alarms.ClearCondition("jointLimit");
                _watchFaulted = fault.Faulted;
            }

            // Plugins: entering Crashed raises; leaving it (running again, stopped, disabled) clears.
            var pm = PluginManager;
            if (pm != null)
            {
                foreach (var h in pm.Plugins)
                {
                    var st = h.State;
                    _watchPlugins.TryGetValue(h.Id, out var prev);
                    if (st == Plugins.PluginState.Crashed && prev != Plugins.PluginState.Crashed)
                        Alarms.Raise(AlarmCatalog.PluginCrashed, $"Plugin '{h.Manifest?.Name ?? h.Id}' crashed: {h.Message ?? "exited unexpectedly"}.",
                            new AlarmSource { PluginId = h.Id }, conditionKey: "plugin:" + h.Id);
                    else if (st != Plugins.PluginState.Crashed && prev == Plugins.PluginState.Crashed
                             && st is Plugins.PluginState.Running or Plugins.PluginState.Degraded or Plugins.PluginState.Stopped or Plugins.PluginState.Disabled)
                        Alarms.ClearCondition("plugin:" + h.Id);
                    _watchPlugins[h.Id] = st;
                }
            }
        }
    }
}
