namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>
    /// One run of the calibration wizard: the detected grid of the last frame, the dots the
    /// user has taught, and a solved-but-maybe-unsaved result. Lock on <see cref="Sync"/>
    /// around every read-modify-write.
    /// </summary>
    public sealed class CalibrationSession
    {
        /// <summary>A taught dot is carried over a re-detect when a new dot lies within this many spacings of it.</summary>
        private const double RematchSpacings = 0.3;

        public string            Id          { get; }
        public string            CameraId    { get; }
        public object            Sync        { get; } = new();
        public double            DotPitchMm  { get; set; }
        public DotDetectorParams Params      { get; set; } = new();
        public byte[]?           FrameJpeg   { get; private set; }
        public DotGridResult?    Grid        { get; private set; }
        public byte[]?           AnnotatedJpeg { get; private set; }
        public List<TaughtDot>   Taught      { get; } = new();
        /// <summary>Set by a solve; cleared whenever the grid or the taught dots change.</summary>
        public CameraCalibration? Solved     { get; set; }

        public CalibrationSession(string id, string cameraId, double dotPitchMm, DotDetectorParams p)
        {
            Id         = id;
            CameraId   = cameraId;
            DotPitchMm = dotPitchMm;
            Params     = p;
        }

        /// <summary>
        /// Adopts a new frame's grid. Taught dots whose pixel position still has a dot
        /// nearby take that dot's (possibly new) index; the rest are dropped. Returns the
        /// warnings for dots that could not be carried over.
        /// </summary>
        public List<string> ApplyDetection(byte[] frameJpeg, DotGridResult grid)
        {
            var warnings = new List<string>();
            var kept = new List<TaughtDot>();
            double radius = RematchSpacings * (grid.MedianSpacingPx > 0 ? grid.MedianSpacingPx : 10);
            foreach (var t in Taught)
            {
                var match = grid.Dots.MinBy(d => (d.X - t.X) * (d.X - t.X) + (d.Y - t.Y) * (d.Y - t.Y));
                if (match != null && Math.Sqrt((match.X - t.X) * (match.X - t.X) + (match.Y - t.Y) * (match.Y - t.Y)) <= radius
                    && kept.All(k => k.DotIndex != match.Index))
                {
                    t.DotIndex = match.Index; t.I = match.I; t.J = match.J;
                    t.X = match.X; t.Y = match.Y; t.U = match.U; t.V = match.V;
                    kept.Add(t);
                }
                else warnings.Add($"Taught dot {t.DotIndex} is no longer found at the same place and was removed; teach it again");
            }
            Taught.Clear();
            Taught.AddRange(kept);

            FrameJpeg = frameJpeg;
            Grid      = grid;
            Solved    = null;
            Rerender();
            return warnings;
        }

        /// <summary>Shows a frame in which no grid was found (the wizard's image); an earlier grid is kept.</summary>
        public void ShowRawFrame(byte[] frameJpeg)
        {
            if (Grid == null) { FrameJpeg = frameJpeg; AnnotatedJpeg = frameJpeg; }
        }

        /// <summary>Records (or replaces) the teach of one dot.</summary>
        public TaughtDot Teach(int dotIndex, RobotXyz robot, string tool, long nowUnixMs)
        {
            var dot = Grid?.FindDot(dotIndex)
                ?? throw new CalibrationException(CalibrationErrors.UnknownDot, $"There is no dot {dotIndex} in the detected grid");
            Taught.RemoveAll(t => t.DotIndex == dotIndex);
            var taught = new TaughtDot
            {
                DotIndex = dot.Index, I = dot.I, J = dot.J, X = dot.X, Y = dot.Y, U = dot.U, V = dot.V,
                Robot = robot, Tool = tool, TaughtUnixMs = nowUnixMs,
            };
            Taught.Add(taught);
            Taught.Sort((a, b) => a.DotIndex.CompareTo(b.DotIndex));
            Solved = null;
            Rerender();
            return taught;
        }

        public void Unteach(int dotIndex)
        {
            if (Grid?.FindDot(dotIndex) == null && Taught.All(t => t.DotIndex != dotIndex))
                throw new CalibrationException(CalibrationErrors.UnknownDot, $"There is no dot {dotIndex} in the detected grid");
            if (Taught.RemoveAll(t => t.DotIndex == dotIndex) > 0)
            {
                Solved = null;
                Rerender();
            }
        }

        private void Rerender()
        {
            if (FrameJpeg == null || Grid == null) { AnnotatedJpeg = null; return; }
            try
            {
                AnnotatedJpeg = CalibrationImage.Render(FrameJpeg, Grid, Taught.Select(t => t.DotIndex).ToHashSet());
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Calibration] Failed to render annotated image: {ex.Message}");
                AnnotatedJpeg = null;
            }
        }
    }

    /// <summary>
    /// In-memory calibration sessions. A session expires <see cref="Ttl"/> after its last use;
    /// expired ones are swept on every access. Thread-safe.
    /// </summary>
    public sealed class CalibrationSessionManager
    {
        public static readonly TimeSpan DefaultTtl = TimeSpan.FromMinutes(30);

        private readonly TimeProvider _time;
        private readonly object _lock = new();
        private readonly Dictionary<string, (CalibrationSession Session, DateTimeOffset LastUsed)> _sessions = new(StringComparer.Ordinal);

        public TimeSpan Ttl { get; }

        public CalibrationSessionManager(TimeProvider? time = null, TimeSpan? ttl = null)
        {
            _time = time ?? TimeProvider.System;
            Ttl   = ttl ?? DefaultTtl;
        }

        public long NowUnixMs => _time.GetUtcNow().ToUnixTimeMilliseconds();

        public CalibrationSession Create(string cameraId, double dotPitchMm, DotDetectorParams p)
        {
            var s = new CalibrationSession(Guid.NewGuid().ToString("N")[..12], cameraId, dotPitchMm, p);
            lock (_lock)
            {
                Sweep();
                _sessions[s.Id] = (s, _time.GetUtcNow());
            }
            return s;
        }

        /// <summary>The live session with this id, marking it used; null when unknown or expired.</summary>
        public CalibrationSession? Get(string? id)
        {
            if (string.IsNullOrEmpty(id)) return null;
            lock (_lock)
            {
                Sweep();
                if (!_sessions.TryGetValue(id, out var e)) return null;
                _sessions[id] = (e.Session, _time.GetUtcNow());
                return e.Session;
            }
        }

        public bool Remove(string? id)
        {
            if (string.IsNullOrEmpty(id)) return false;
            lock (_lock) return _sessions.Remove(id);
        }

        public int Count
        {
            get { lock (_lock) { Sweep(); return _sessions.Count; } }
        }

        private void Sweep()
        {
            var now = _time.GetUtcNow();
            foreach (var id in _sessions.Where(kv => now - kv.Value.LastUsed > Ttl).Select(kv => kv.Key).ToList())
                _sessions.Remove(id);
        }
    }
}
