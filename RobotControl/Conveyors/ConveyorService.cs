using System.Collections.Concurrent;
using System.Diagnostics;

namespace Controller.RobotControl.Conveyors
{
    /// <summary>The part being tracked: which conveyor, which part, and the config it was started with.</summary>
    /// <param name="ZeroTravel">Travel at which the part sat at conveyor X = 0 (part X = travel − ZeroTravel).</param>
    /// <param name="Generation">Bumped per tracking session, so the motion thread sees a new session.</param>
    /// <param name="Owner">The program run that started it (its ExecutionContext); faults go back to it.</param>
    public sealed record ConveyorTrackingState(
        string ConveyorId, long PartId, double ZeroTravel, Conveyor Config, long Generation, object? Owner)
    {
        public double DirX => Config.Direction.X;
        public double DirY => Config.Direction.Y;
    }

    /// <summary>A tracking failure raised outside the program thread (motion thread, window check),
    /// waiting for the owning program to raise it as a catchable error.</summary>
    public sealed record ConveyorTrackingFault(string Code, string Message, object? Owner);

    /// <summary>
    /// Conveyor runtime (docs/conveyor-tracking.md): per-conveyor belt travel and speed, the part
    /// queue with sensor latching, window drops and counts, and the single tracked part the motion
    /// thread follows.
    /// </summary>
    /// <remarks>
    /// Thread model: <see cref="TravelAt"/> / <see cref="TrySample"/> are lock-free (volatile
    /// snapshots and the source's own thread-safe read) and are called from the motion thread every
    /// tick. Queue changes take the per-conveyor lock (never held across I/O). <see cref="Tick"/>
    /// polls sensors and drops parts past the window; <see cref="Start"/> runs it on its own thread,
    /// tests call it directly with a fake clock.
    /// </remarks>
    public sealed class ConveyorService : IDisposable
    {
        /// <summary>The aux position-stream period the controller asks for (docs: PS:20).</summary>
        public const int StreamPeriodMs = 20;
        /// <summary>Parts in a status entry are capped at this many.</summary>
        public const int StatusPartCap = 16;
        /// <summary>A belt counts as running above this speed (mm/s).</summary>
        public const double RunningThresholdMmS = 0.1;

        private static readonly double Freq = Stopwatch.Frequency;

        private readonly ConveyorRepository _repo;
        private readonly IConveyorTravelSource _src;
        private readonly Func<long> _clock;
        private readonly ConcurrentDictionary<string, Runtime> _rt = new(StringComparer.OrdinalIgnoreCase);

        private volatile ConveyorTrackingState? _tracking;
        private volatile ConveyorTrackingFault? _fault;
        private long _generation;

        private volatile bool _disposed;
        private int _started;

        // Status cache (the broadcast polls fast; the payload is rebuilt at most every 100 ms).
        private List<ConveyorStatus>? _statusCache;
        private long _statusTicks;
        private readonly object _statusLock = new();

        /// <param name="clock">Stopwatch ticks; defaults to <see cref="Stopwatch.GetTimestamp"/>.</param>
        public ConveyorService(ConveyorRepository repo, IConveyorTravelSource source, Func<long>? clock = null)
        {
            _repo  = repo;
            _src   = source;
            _clock = clock ?? Stopwatch.GetTimestamp;
            _src.InputLatched += OnAuxLatched;
            Reload();
        }

        public void Dispose()
        {
            _disposed = true;
            _src.InputLatched -= OnAuxLatched;
        }

        /// <summary>Starts the sensor / window thread. Subsequent calls are no-ops.</summary>
        public void Start()
        {
            if (Interlocked.Exchange(ref _started, 1) != 0) return;
            new Thread(Loop) { IsBackground = true, Name = "ConveyorLoop" }.Start();
        }

        private void Loop()
        {
            while (!_disposed)
            {
                try { Tick(); }
                catch (Exception ex) { Console.WriteLine($"[Conveyor] tick failed: {ex.Message}"); }
                Thread.Sleep(_rt.IsEmpty ? 50 : 2);
            }
        }

        public ConveyorRepository Repository => _repo;

        public long Now => _clock();

        // ── Runtime state ─────────────────────────────────────────────────────

        /// <summary>A travel model that is linear in time: travel = Travel + Speed · (t − Ticks).</summary>
        private sealed record Linear(double Travel, long Ticks, double Speed)
        {
            public double At(long ticks) => Travel + Speed * ((ticks - Ticks) / Freq);
        }

        private sealed class Runtime
        {
            public volatile Conveyor Cfg;
            public readonly object Sync = new();
            public readonly List<ConveyorPart> Queue = new();
            public long NextPartId = 1;
            public int Latched, Picked, Missed;
            public double? LastLatchTravel;
            public bool? LastSensorActive;

            /// <summary>timed / manual: the travel model (raw, before scale; manual ignores scale).</summary>
            public volatile Linear Lin;
            public bool TimedRunning;
            /// <summary>aux: last good raw travel (mm), the fallback while the device is silent. A plain
            /// double, so the motion thread's read allocates nothing.</summary>
            public double LastAuxTravel;
            /// <summary>aux: re-zero guard (see Continuous) — the last read and the folded-in offset.</summary>
            public readonly object AuxSync = new();
            public long   AuxPrevTicks;
            public double AuxPrevSteps, AuxPrevSps, AuxOffset;
            /// <summary>aux: belt mm per step (cached; refreshed by Tick).</summary>
            public double MmPerStep;
            public volatile bool SourceOk;
            public long LastStreamRequestTicks;

            public Runtime(Conveyor cfg, long now)
            {
                Cfg = cfg;
                Lin = new Linear(0, now, 0);
            }
        }

        /// <summary>Re-reads the repository: new conveyors get a runtime, changed ones a new config
        /// snapshot, deleted ones are dropped. Aux devices a conveyor uses are asked to stream.</summary>
        public void Reload()
        {
            long now = _clock();
            var all = _repo.GetAll();
            var ids = new HashSet<string>(all.Select(c => c.Id), StringComparer.OrdinalIgnoreCase);
            foreach (var c in all)
            {
                var cfg = c.Clone();
                if (_rt.TryGetValue(cfg.Id, out var rt))
                {
                    var old = rt.Cfg;
                    bool sourceChanged = !SameSource(old, cfg);
                    if (sourceChanged)
                    {
                        // Counted travel restarts on a new source / scale — queued parts were latched
                        // against the old one, so they cannot be placed any more.
                        double travel = TravelAt(rt, now, out _);
                        lock (rt.Sync) rt.Queue.Clear();
                        rt.Lin = new Linear(cfg.Source.Kind == ConveyorSourceKinds.Manual ? travel : 0, now, 0);
                        rt.LastAuxTravel = 0;
                        lock (rt.AuxSync) { rt.AuxPrevTicks = 0; rt.AuxPrevSteps = rt.AuxPrevSps = rt.AuxOffset = 0; }
                        rt.TimedRunning = false;
                        if (_tracking?.ConveyorId == cfg.Id) _tracking = null;
                    }
                    rt.Cfg = cfg;
                }
                else
                {
                    rt = new Runtime(cfg, now);
                    _rt[cfg.Id] = rt;
                }
                RefreshAux(rt, now, force: true);
            }
            foreach (var id in _rt.Keys.ToList())
                if (!ids.Contains(id))
                {
                    _rt.TryRemove(id, out _);
                    if (string.Equals(_tracking?.ConveyorId, id, StringComparison.OrdinalIgnoreCase)) _tracking = null;
                }
            InvalidateStatus();
        }

        private static bool SameSource(Conveyor a, Conveyor b) =>
            a.Source.Kind == b.Source.Kind && a.Source.DeviceId == b.Source.DeviceId && a.Source.Axis == b.Source.Axis
            && a.Source.MmPerStep == b.Source.MmPerStep && a.Scale == b.Scale;

        // aux: cache mm/step and (re)request streaming — at load, and every 2 s while the device is silent.
        private void RefreshAux(Runtime rt, long now, bool force)
        {
            var s = rt.Cfg.Source;
            if (s.Kind != ConveyorSourceKinds.Aux || string.IsNullOrEmpty(s.DeviceId)) return;
            double mmPerStep = s.MmPerStep is > 0 ? s.MmPerStep.Value : 0;
            if (mmPerStep <= 0)
            {
                double spm = SafeStepsPerMm(s.DeviceId, s.Axis);
                mmPerStep = spm > 0 ? 1.0 / spm : 0;
            }
            rt.MmPerStep = mmPerStep;
            if (force || (!rt.SourceOk && now - rt.LastStreamRequestTicks > 2 * Freq))
            {
                rt.LastStreamRequestTicks = now;
                try { _src.SetPositionStreaming(s.DeviceId, StreamPeriodMs); }
                catch (Exception ex) { Console.WriteLine($"[Conveyor] streaming request to '{s.DeviceId}' failed: {ex.Message}"); }
            }
        }

        private double SafeStepsPerMm(string deviceId, int axis)
        {
            try { return _src.StepsPerMm(deviceId, axis); } catch { return 0; }
        }

        // ── Config CRUD ───────────────────────────────────────────────────────

        public IReadOnlyList<Conveyor> GetAll() => _repo.GetAll().OrderBy(c => c.Name, StringComparer.OrdinalIgnoreCase).ToList();

        public Conveyor? Get(string? id) => _repo.Find(id);

        /// <summary>Validates and saves <paramref name="c"/> (empty id = new). Throws
        /// <see cref="ArgumentException"/> with a readable message on a bad config.</summary>
        public Conveyor Save(Conveyor c)
        {
            Validate(c);
            if (!string.IsNullOrWhiteSpace(c.Id) && _repo.Find(c.Id) is { } existing) c.Id = existing.Id;
            var saved = _repo.Save(c);
            Reload();
            return saved;
        }

        public bool Delete(string id)
        {
            var c = _repo.Find(id);
            if (c == null) return false;
            bool ok = _repo.Delete(c.Id);
            Reload();
            return ok;
        }

        internal static void Validate(Conveyor c)
        {
            if (!string.IsNullOrWhiteSpace(c.Id) && !ConveyorRepository.IsValidId(c.Id))
                throw new ArgumentException("Conveyor id may only contain letters, digits and '_'");
            if (string.IsNullOrWhiteSpace(c.Name)) throw new ArgumentException("Conveyor name is required");
            c.Source ??= new ConveyorSource();
            c.Frame  ??= new ConveyorFrame();
            c.Sensor ??= new ConveyorSensor();
            c.Window ??= new ConveyorWindow();
            if (!ConveyorSourceKinds.All.Contains(c.Source.Kind))
                throw new ArgumentException($"source.kind must be one of {string.Join(", ", ConveyorSourceKinds.All)}");
            if (!ConveyorSensorKinds.All.Contains(c.Sensor.Kind))
                throw new ArgumentException($"sensor.kind must be one of {string.Join(", ", ConveyorSensorKinds.All)}");
            if (c.Source.Kind == ConveyorSourceKinds.Aux && string.IsNullOrWhiteSpace(c.Source.DeviceId))
                throw new ArgumentException("An aux source needs source.deviceId");
            if (c.Source.Kind == ConveyorSourceKinds.Timed && c.Source.RunOutput == null)
                throw new ArgumentException("A timed source needs source.runOutput");
            if (c.Source.MmPerStep is { } mps && !(mps > 0))
                throw new ArgumentException("source.mmPerStep must be above 0 (or empty)");
            if (!double.IsFinite(c.Scale) || c.Scale == 0) throw new ArgumentException("scale must be a non-zero number");
            if (!(c.Window.EndMm > c.Window.StartMm)) throw new ArgumentException("window.endMm must be greater than window.startMm");
            if (c.Sensor.Kind is ConveyorSensorKinds.AuxPin or ConveyorSensorKinds.Io
                && (string.IsNullOrWhiteSpace(c.Sensor.DeviceId) || c.Sensor.Pin == null))
                throw new ArgumentException("This sensor needs sensor.deviceId and sensor.pin");
            if (c.Sensor.Kind == ConveyorSensorKinds.Stb && c.Sensor.Input is not (>= 1 and <= 4))
                throw new ArgumentException("An stb sensor needs sensor.input 1–4");
            if (c.Sensor.MinSpacingMm < 0) c.Sensor.MinSpacingMm = 0;
            if (c.MaxQueue < 1) c.MaxQueue = 1;
        }

        // ── Travel ────────────────────────────────────────────────────────────

        /// <summary>Belt travel (mm) of conveyor <paramref name="id"/> at Stopwatch time
        /// <paramref name="ticks"/> (0 for an unknown conveyor). Lock-free; safe on the motion thread.</summary>
        public double TravelAt(string id, long ticks) =>
            _rt.TryGetValue(id, out var rt) ? TravelAt(rt, ticks, out _) : 0;

        /// <summary>Travel and speed (mm/s) at <paramref name="ticks"/>; false for an unknown conveyor.</summary>
        public bool TrySample(string id, long ticks, out double travel, out double speed)
        {
            if (!_rt.TryGetValue(id, out var rt)) { travel = speed = 0; return false; }
            travel = TravelAt(rt, ticks, out speed);
            return true;
        }

        /// <summary>Belt speed now (mm/s).</summary>
        public double SpeedOf(string id)
        {
            TrySample(id, _clock(), out _, out var speed);
            return speed;
        }

        private double TravelAt(Runtime rt, long ticks, out double speed)
        {
            var cfg = rt.Cfg;
            switch (cfg.Source.Kind)
            {
                case ConveyorSourceKinds.Aux:
                {
                    double mmPerStep = rt.MmPerStep;
                    var dev = cfg.Source.DeviceId;
                    if (mmPerStep > 0 && !string.IsNullOrEmpty(dev)
                        && _src.TryGetLivePosition(dev, cfg.Source.Axis, out var steps, out var sps)
                        && double.IsFinite(steps) && double.IsFinite(sps))
                    {
                        long now = _clock();
                        steps = Continuous(rt, steps, sps, now);
                        double raw = (steps + sps * ((ticks - now) / Freq)) * mmPerStep;
                        rt.LastAuxTravel = steps * mmPerStep;
                        rt.SourceOk = true;
                        speed = sps * mmPerStep * cfg.Scale;
                        return raw * cfg.Scale;
                    }
                    rt.SourceOk = false;
                    speed = 0;
                    return rt.LastAuxTravel * cfg.Scale;
                }
                case ConveyorSourceKinds.Timed:
                {
                    var lin = rt.Lin;
                    rt.SourceOk = true;
                    speed = lin.Speed * cfg.Scale;
                    return lin.At(ticks) * cfg.Scale;
                }
                default: // manual: travel is what ConveyorSetTravel set (scale does not apply)
                    rt.SourceOk = true;
                    speed = 0;
                    return rt.Lin.Travel;
            }
        }

        /// <summary>A step jump bigger than this (plus what the speed explains) is a re-zero, not motion.</summary>
        private const double JumpToleranceSteps = 200;

        /// <summary>
        /// Keeps aux travel continuous across a re-zero of the board counter (SetPosition /
        /// ZeroPosition, or the stream resuming after dead reckoning): a jump the belt speed cannot
        /// explain is folded into a per-conveyor offset, so parts already latched keep their place.
        /// A tiny lock, never held across I/O.
        /// </summary>
        private static double Continuous(Runtime rt, double steps, double sps, long now)
        {
            lock (rt.AuxSync)
            {
                if (rt.AuxPrevTicks != 0)
                {
                    double dt = Math.Max(0, (now - rt.AuxPrevTicks) / Freq);
                    double expected = rt.AuxPrevSteps + rt.AuxPrevSps * dt;
                    double tol = JumpToleranceSteps + 2 * (Math.Abs(sps) + Math.Abs(rt.AuxPrevSps)) * dt;
                    double seen = steps + rt.AuxOffset;
                    if (Math.Abs(seen - expected) > tol) rt.AuxOffset += expected - seen;
                }
                double adjusted = steps + rt.AuxOffset;
                if (now >= rt.AuxPrevTicks)
                {
                    rt.AuxPrevTicks = now;
                    rt.AuxPrevSteps = adjusted;
                    rt.AuxPrevSps   = sps;
                }
                return adjusted;
            }
        }

        /// <summary>ConveyorSetTravel: manual sources only.</summary>
        public void SetTravel(string id, double travelMm)
        {
            var rt = RequireRuntime(id);
            if (rt.Cfg.Source.Kind != ConveyorSourceKinds.Manual)
                throw new InvalidOperationException("ConveyorSetTravel works on a manual conveyor only");
            if (!double.IsFinite(travelMm)) throw new ArgumentException("travelMm must be a number");
            rt.Lin = new Linear(travelMm, _clock(), 0);
            InvalidateStatus();
        }

        private Runtime RequireRuntime(string id) =>
            _rt.TryGetValue(id ?? "", out var rt) ? rt : throw new KeyNotFoundException($"Conveyor not found: {id}");

        // ── Parts ─────────────────────────────────────────────────────────────

        /// <summary>A software trigger: a part at conveyor X = <paramref name="offsetMm"/> now. Returns its id.</summary>
        public long AddPart(string id, double offsetMm)
        {
            var rt = RequireRuntime(id);
            if (!double.IsFinite(offsetMm)) throw new ArgumentException("offsetMm must be a number");
            double travel = TravelAt(rt, _clock(), out _);
            long pid;
            lock (rt.Sync)
            {
                pid = rt.NextPartId++;
                rt.Queue.Add(new ConveyorPart { Id = pid, TravelAtLatch = travel, ZeroTravel = travel - offsetMm });
                rt.Latched++;
                TrimQueue(rt);
            }
            InvalidateStatus();
            return pid;
        }

        public void ClearQueue(string id)
        {
            var rt = RequireRuntime(id);
            lock (rt.Sync) rt.Queue.Clear();
            if (_tracking is { } t && string.Equals(t.ConveyorId, rt.Cfg.Id, StringComparison.OrdinalIgnoreCase))
                _tracking = null; // the tracked part is gone: freeze where the robot is
            InvalidateStatus();
        }

        /// <summary>A sensor latched a part at belt travel <paramref name="travelAtLatch"/>. False when
        /// it is closer than minSpacingMm to the previous latch (a re-trigger).</summary>
        private bool Latch(Runtime rt, double travelAtLatch)
        {
            var cfg = rt.Cfg;
            lock (rt.Sync)
            {
                if (rt.LastLatchTravel is { } last && Math.Abs(travelAtLatch - last) < cfg.Sensor.MinSpacingMm)
                    return false;
                rt.LastLatchTravel = travelAtLatch;
                rt.Queue.Add(new ConveyorPart
                {
                    Id = rt.NextPartId++, TravelAtLatch = travelAtLatch, ZeroTravel = travelAtLatch - cfg.Sensor.OffsetMm,
                });
                rt.Latched++;
                TrimQueue(rt);
            }
            InvalidateStatus();
            return true;
        }

        // Over maxQueue: the oldest parts (not the tracked one) go as missed. Caller holds rt.Sync.
        private void TrimQueue(Runtime rt)
        {
            int max = Math.Max(1, rt.Cfg.MaxQueue);
            var tracked = _tracking;
            for (int i = 0; rt.Queue.Count > max && i < rt.Queue.Count;)
            {
                var p = rt.Queue[i];
                if (tracked != null && tracked.PartId == p.Id && string.Equals(tracked.ConveyorId, rt.Cfg.Id, StringComparison.OrdinalIgnoreCase)) { i++; continue; }
                rt.Queue.RemoveAt(i);
                if (!p.Released) rt.Missed++;
            }
        }

        private void OnAuxLatched(ConveyorLatch e)
        {
            try
            {
                foreach (var rt in _rt.Values)
                {
                    var cfg = rt.Cfg;
                    var s = cfg.Sensor;
                    if (s.Kind != ConveyorSensorKinds.AuxPin || s.Pin != e.Pin
                        || !string.Equals(s.DeviceId, e.DeviceId, StringComparison.OrdinalIgnoreCase)) continue;
                    bool active = (e.Value != 0) == s.ActiveHigh;
                    rt.LastSensorActive = active;
                    if (!active) continue;

                    // The board captured the step counts at the edge: exact when the belt is driven by
                    // an axis of that same board; otherwise interpolate at the edge time.
                    double travel;
                    var src = cfg.Source;
                    if (src.Kind == ConveyorSourceKinds.Aux && string.Equals(src.DeviceId, e.DeviceId, StringComparison.OrdinalIgnoreCase)
                        && e.AxisSteps is { } steps && src.Axis >= 0 && src.Axis < steps.Length && rt.MmPerStep > 0)
                    {
                        double offset;
                        lock (rt.AuxSync) offset = rt.AuxOffset; // same re-zero guard as the live read
                        travel = (steps[src.Axis] + offset) * rt.MmPerStep * cfg.Scale;
                    }
                    else
                        travel = TravelAt(rt, e.StopwatchTicks, out _);
                    Latch(rt, travel);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Conveyor] latch handling failed: {ex.Message}");
            }
        }

        // ── Tick: sensors, timed belts, window ───────────────────────────────

        /// <summary>
        /// One pass: integrate timed belts, poll io / stb sensors (an inactive → active edge latches a
        /// part at the belt travel now), and drop parts past the window end — as missed, or as the
        /// catchable <c>conveyorWindow</c> fault when it is the tracked part.
        /// </summary>
        public void Tick()
        {
            long now = _clock();
            foreach (var rt in _rt.Values)
            {
                var cfg = rt.Cfg;

                if (cfg.Source.Kind == ConveyorSourceKinds.Aux) RefreshAux(rt, now, force: false);

                if (cfg.Source.Kind == ConveyorSourceKinds.Timed && cfg.Source.RunOutput is { } run)
                {
                    bool running = SafeRead(() => _src.ReadOutput(run)) == true;
                    if (running != rt.TimedRunning || rt.Lin.Speed != (running ? cfg.Source.SpeedMmS : 0))
                    {
                        // Rebase the linear model at the switch so travel stays continuous.
                        double raw = rt.Lin.At(now);
                        rt.Lin = new Linear(raw, now, running ? cfg.Source.SpeedMmS : 0);
                        rt.TimedRunning = running;
                    }
                }

                if (cfg.Sensor.Kind is ConveyorSensorKinds.Io or ConveyorSensorKinds.Stb)
                {
                    bool? v = SafeRead(() => _src.ReadInput(cfg.Sensor));
                    if (v is { } raw)
                    {
                        bool active = raw == cfg.Sensor.ActiveHigh;
                        if (rt.LastSensorActive == false && active) Latch(rt, TravelAt(rt, now, out _));
                        rt.LastSensorActive = active;
                    }
                }

                DropPastWindow(rt, TravelAt(rt, now, out _));
            }
        }

        private static bool? SafeRead(Func<bool?> read)
        {
            try { return read(); } catch { return null; }
        }

        private void DropPastWindow(Runtime rt, double travel)
        {
            var cfg = rt.Cfg;
            bool changed = false;
            string? windowFault = null;
            lock (rt.Sync)
            {
                for (int i = 0; i < rt.Queue.Count;)
                {
                    var p = rt.Queue[i];
                    double x = p.XAt(travel);
                    if (x <= cfg.Window.EndMm) { i++; continue; }
                    rt.Queue.RemoveAt(i);
                    changed = true;
                    if (!p.Released) rt.Missed++;
                    if (_tracking is { } t && t.PartId == p.Id && string.Equals(t.ConveyorId, cfg.Id, StringComparison.OrdinalIgnoreCase))
                        windowFault = $"Conveyor '{cfg.Name}': part {p.Id} left the window (x {x:0.#} > {cfg.Window.EndMm:0.#} mm) while tracked";
                }
            }
            if (windowFault != null) RaiseFault(Programs.ErrorCodes.ConveyorWindow, windowFault);
            if (changed) InvalidateStatus();
        }

        // ── Tracking ──────────────────────────────────────────────────────────

        /// <summary>The part the motion thread follows, or null.</summary>
        public ConveyorTrackingState? Tracking => _tracking;

        /// <summary>The oldest part inside the window that is not released, or null.</summary>
        public ConveyorPart? OldestInWindow(string id)
        {
            if (!_rt.TryGetValue(id, out var rt)) return null;
            double travel = TravelAt(rt, _clock(), out _);
            var w = rt.Cfg.Window;
            lock (rt.Sync)
                return rt.Queue.FirstOrDefault(p => !p.Released && p.XAt(travel) >= w.StartMm && p.XAt(travel) <= w.EndMm);
        }

        /// <summary>
        /// Starts tracking part <paramref name="partId"/> of conveyor <paramref name="id"/> for
        /// <paramref name="owner"/>. A part already tracked is ended first (counted picked).
        /// False when the conveyor or part is unknown.
        /// </summary>
        public bool StartTracking(string id, long partId, object? owner)
        {
            if (!_rt.TryGetValue(id, out var rt)) return false;
            ConveyorPart? part;
            lock (rt.Sync) part = rt.Queue.FirstOrDefault(p => p.Id == partId);
            if (part == null) return false;
            if (_tracking != null) EndTracking(release: false);
            _fault = null;
            _tracking = new ConveyorTrackingState(rt.Cfg.Id, part.Id, part.ZeroTravel, rt.Cfg,
                                                  Interlocked.Increment(ref _generation), owner);
            InvalidateStatus();
            return true;
        }

        /// <summary>
        /// Ends tracking (the robot freezes where it is). The part counts as picked and leaves the
        /// queue, or with <paramref name="release"/> stays on the belt untracked (never re-tracked).
        /// <paramref name="conveyorId"/>: only when that conveyor is the tracked one (null = any).
        /// </summary>
        public bool EndTracking(bool release, string? conveyorId = null)
        {
            var t = _tracking;
            if (t == null) return false;
            if (!string.IsNullOrEmpty(conveyorId) && !string.Equals(conveyorId, t.ConveyorId, StringComparison.OrdinalIgnoreCase))
                return false;
            _tracking = null;
            if (_rt.TryGetValue(t.ConveyorId, out var rt))
            {
                lock (rt.Sync)
                {
                    int i = rt.Queue.FindIndex(p => p.Id == t.PartId);
                    if (i >= 0)
                    {
                        if (release) rt.Queue[i].Released = true;
                        else { rt.Queue.RemoveAt(i); rt.Picked++; }
                    }
                }
            }
            InvalidateStatus();
            return true;
        }

        /// <summary>
        /// A tracking failure (belt too fast, target out of reach, part past the window): tracking
        /// stops (the robot freezes), the part is released, and the fault waits for its owner
        /// (<see cref="ConsumeFault"/>). Any thread.
        /// </summary>
        public void RaiseFault(string code, string message)
        {
            var t = _tracking;
            if (t == null) return;
            EndTracking(release: true);
            _fault = new ConveyorTrackingFault(code, message, t.Owner);
            Console.WriteLine($"[Conveyor] tracking stopped — {code}: {message}");
        }

        /// <summary>Returns and clears a pending fault raised for <paramref name="owner"/>.</summary>
        public ConveyorTrackingFault? ConsumeFault(object? owner)
        {
            var f = _fault;
            if (f == null || !ReferenceEquals(f.Owner, owner)) return null;
            return Interlocked.CompareExchange(ref _fault, null, f) == f ? f : null;
        }

        /// <summary>A program run ended: its tracking stops (the robot freezes, the part rides on)
        /// and a fault still pending for it is dropped.</summary>
        public void ReleaseOwner(object owner)
        {
            if (_tracking is { } t && ReferenceEquals(t.Owner, owner))
            {
                _tracking = null;
                InvalidateStatus();
            }
            if (_fault is { } f && ReferenceEquals(f.Owner, owner)) _fault = null;
        }

        /// <summary>The tracked part's X in the conveyor frame now, or null when nothing is tracked.</summary>
        public double? TrackedPartX()
        {
            var t = _tracking;
            if (t == null) return null;
            return TravelAt(t.ConveyorId, _clock()) - t.ZeroTravel;
        }

        /// <summary>The part frame of the tracked part now and the belt travel it was computed at.</summary>
        public bool TryPartFrame(out Vector6 frame, out double travel)
        {
            var t = _tracking;
            if (t == null) { frame = Vector6.Zero; travel = 0; return false; }
            travel = TravelAt(t.ConveyorId, _clock());
            frame  = t.Config.PartFrame(travel - t.ZeroTravel);
            return true;
        }

        // ── Expressions ($conveyor.<id>.…) ───────────────────────────────────

        /// <summary>The property names a conveyor publishes (after <c>conveyor.&lt;id&gt;.</c>).</summary>
        public static readonly IReadOnlyList<(string Name, string Description, string Type)> PropertyNames =
        [
            ("travel",   "belt travel (mm)",                          "number"),
            ("speed",    "belt speed (mm/s)",                         "number"),
            ("count",    "queued parts",                              "number"),
            ("inWindow", "queued parts inside the pick window",       "number"),
            ("tracking", "1 while a part of this conveyor is tracked", "boolean"),
            ("part.x",   "tracked part X in the conveyor frame (mm)", "number"),
            ("part.id",  "tracked part id (0 when none)",             "number"),
            ("latched",  "parts latched since start",                 "number"),
            ("picked",   "parts picked since start",                  "number"),
            ("missed",   "parts missed (left the window untracked)",  "number"),
        ];

        /// <summary><c>$conveyor.&lt;id&gt;.&lt;prop&gt;</c>; false for an unknown conveyor or property.</summary>
        public bool TryGetProperty(string id, string prop, out double value)
        {
            value = 0;
            if (!_rt.TryGetValue(id, out var rt)) return false;
            var cfg = rt.Cfg;
            double travel = TravelAt(rt, _clock(), out var speed);
            var t = _tracking;
            bool tracked = t != null && string.Equals(t.ConveyorId, cfg.Id, StringComparison.OrdinalIgnoreCase);
            switch (prop.ToLowerInvariant())
            {
                case "travel":   value = travel; return true;
                case "speed":    value = speed;  return true;
                case "count":    lock (rt.Sync) value = rt.Queue.Count(p => !p.Released); return true;
                case "inwindow":
                    lock (rt.Sync)
                        value = rt.Queue.Count(p => !p.Released && p.XAt(travel) >= cfg.Window.StartMm && p.XAt(travel) <= cfg.Window.EndMm);
                    return true;
                case "tracking": value = tracked ? 1 : 0; return true;
                case "part.x":   value = tracked ? travel - t!.ZeroTravel : 0; return true;
                case "part.id":  value = tracked ? t!.PartId : 0; return true;
                case "latched":  value = rt.Latched; return true;
                case "picked":   value = rt.Picked;  return true;
                case "missed":   value = rt.Missed;  return true;
                default: return false;
            }
        }

        /// <summary>The configured conveyor ids (for the expression symbol list).</summary>
        public IEnumerable<(string Id, string Name)> Ids() => _rt.Values.Select(r => (r.Cfg.Id, r.Cfg.Name));

        // ── Status ────────────────────────────────────────────────────────────

        private void InvalidateStatus()
        {
            lock (_statusLock) _statusCache = null;
        }

        /// <summary>The status broadcast's <c>conveyors</c> list (rebuilt at most every 100 ms).</summary>
        public List<ConveyorStatus> Status()
        {
            long now = _clock();
            lock (_statusLock)
            {
                if (_statusCache != null && now - _statusTicks < Freq / 10) return _statusCache;
            }
            var list = new List<ConveyorStatus>();
            var t = _tracking;
            foreach (var rt in _rt.Values.OrderBy(r => r.Cfg.Name, StringComparer.OrdinalIgnoreCase))
            {
                var cfg = rt.Cfg;
                double travel = TravelAt(rt, now, out var speed);
                bool tracked = t != null && string.Equals(t.ConveyorId, cfg.Id, StringComparison.OrdinalIgnoreCase);
                List<ConveyorPartStatus> parts;
                int latched, picked, missed;
                double? lastLatch;
                lock (rt.Sync)
                {
                    parts = rt.Queue.Where(p => !p.Released).Take(StatusPartCap)
                        .Select(p => new ConveyorPartStatus { Id = p.Id, X = Round(p.XAt(travel)), TravelAtLatch = Round(p.TravelAtLatch) })
                        .ToList();
                    latched = rt.Latched; picked = rt.Picked; missed = rt.Missed;
                    lastLatch = rt.LastLatchTravel;
                }
                list.Add(new ConveyorStatus
                {
                    Id = cfg.Id, TravelMm = Round(travel), SpeedMmS = Round(speed),
                    Running = Math.Abs(speed) > RunningThresholdMmS,
                    Tracking = tracked, TrackedPartId = tracked ? t!.PartId : null,
                    Parts = parts, Latched = latched, Picked = picked, Missed = missed,
                    LastLatchTravelMm = lastLatch is { } l ? Round(l) : null,
                    SourceOk = rt.SourceOk,
                });
            }
            lock (_statusLock)
            {
                _statusCache = list;
                _statusTicks = now;
            }
            return list;
        }

        private static double Round(double v) => Math.Round(v, 3);
    }
}
