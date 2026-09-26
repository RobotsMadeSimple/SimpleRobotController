using System.Text.Json;
using Controller.RobotControl.Vision.Calibration;

namespace Controller.RobotControl.Commands;

/// <summary>What the calibration commands need to know about a camera's latest frame.</summary>
internal readonly record struct CalibrationFrame(bool CameraExists, bool Connected, byte[]? Jpeg);

/// <summary>
/// The camera-to-robot calibration wizard (docs/camera-calibration.md): sessions that detect
/// the dot grid, record taught TCP positions, solve and save, plus reading, deleting and
/// predicting from saved calibrations. A failure answers <c>{ ok: false, error: code, message }</c>.
/// </summary>
internal sealed class CalibrationCommands
{
    private readonly CameraCalibrationRepository   _repo;
    private readonly CalibrationSessionManager     _sessions;
    private readonly Func<string, CalibrationFrame> _frame;
    private readonly Func<RobotXyz>                _tcp;
    private readonly Func<string>                  _tool;

    public CalibrationCommands(RobotController robot)
        : this(robot.CalibrationRepo, robot.CalibrationSessions,
               id => FrameFrom(robot, id),
               () => { var p = robot.LivePosition; return new RobotXyz { X = p.X, Y = p.Y, Z = p.Z }; },
               () => robot.ActiveToolName)
    {
    }

    /// <summary>For tests: every hardware dependency supplied by the caller.</summary>
    internal CalibrationCommands(CameraCalibrationRepository repo, CalibrationSessionManager sessions,
                                 Func<string, CalibrationFrame> frame, Func<RobotXyz> tcp, Func<string> tool)
    {
        _repo     = repo;
        _sessions = sessions;
        _frame    = frame;
        _tcp      = tcp;
        _tool     = tool;
    }

    private static CalibrationFrame FrameFrom(RobotController robot, string cameraId)
    {
        var cam = robot.CameraManager.GetCamera(cameraId);
        if (cam == null) return new CalibrationFrame(false, false, null);
        return new CalibrationFrame(true, cam.Connected, cam.GetLatestFrame());
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("GetCameraCalibration",         m => Guard(m, GetCameraCalibration));
        d.Add("DeleteCameraCalibration",      m => Guard(m, DeleteCameraCalibration));
        d.AddAsync("CalibrationStart",        m => Task.Run(() => Guard(m, CalibrationStart)));
        d.AddAsync("CalibrationRedetect",     m => Task.Run(() => Guard(m, CalibrationRedetect)));
        d.Add("CalibrationTeachDot",          m => Guard(m, CalibrationTeachDot));
        d.Add("CalibrationUnteachDot",        m => Guard(m, CalibrationUnteachDot));
        d.Add("CalibrationSolve",             m => Guard(m, CalibrationSolve));
        d.Add("CalibrationPredict",           m => Guard(m, CalibrationPredict));
        d.Add("CalibrationDiscard",           m => Guard(m, CalibrationDiscard));
    }

    // ── Parameters ────────────────────────────────────────────────────────────

    internal sealed class CalibrationParams
    {
        public string? CameraId     { get; set; }
        public string? SessionId    { get; set; }
        public double? DotPitchMm   { get; set; }
        public double? MinDotAreaPx { get; set; }
        public double? MaxDotAreaPx { get; set; }
        public bool?   DarkDots     { get; set; }
        public int?    DotIndex     { get; set; }
        public bool?   Save         { get; set; }
        public double? U            { get; set; }
        public double? V            { get; set; }
    }

    private static readonly JsonSerializerOptions ParamOptions = new() { PropertyNameCaseInsensitive = true };

    private static CalibrationParams Params(CommandMessage msg)
    {
        if (msg.Params is not { ValueKind: JsonValueKind.Object } el) return new CalibrationParams();
        try
        {
            return el.Deserialize<CalibrationParams>(ParamOptions) ?? new CalibrationParams();
        }
        catch (JsonException ex)
        {
            throw new CalibrationException(CalibrationErrors.InvalidParams, $"Bad parameter: {ex.Message}");
        }
    }

    private static object? Guard(CommandMessage msg, Func<CalibrationParams, object?> handler)
    {
        try
        {
            return handler(Params(msg));
        }
        catch (CalibrationException ex)
        {
            return Error(ex.Code, ex.Message);
        }
    }

    private static Dictionary<string, object?> Error(string code, string message) =>
        new() { ["ok"] = false, ["error"] = code, ["message"] = message };

    private static string Require(string? value, string name) =>
        !string.IsNullOrWhiteSpace(value) ? value
            : throw new CalibrationException(CalibrationErrors.InvalidParams, $"'{name}' is required");

    private CalibrationSession Session(CalibrationParams p) =>
        _sessions.Get(Require(p.SessionId, "sessionId"))
            ?? throw new CalibrationException(CalibrationErrors.UnknownSession, $"Calibration session '{p.SessionId}' does not exist or has expired");

    private static DotDetectorParams DetectorParams(CalibrationParams p, DotDetectorParams? previous)
    {
        var d = previous?.Clone() ?? new DotDetectorParams();
        if (p.MinDotAreaPx is { } min) d.MinAreaPx = min;
        if (p.MaxDotAreaPx is { } max) d.MaxAreaPx = max;
        if (p.DarkDots is { } dark)    d.DarkDots  = dark;
        if (d.MinAreaPx < 0 || d.MaxAreaPx <= d.MinAreaPx)
            throw new CalibrationException(CalibrationErrors.InvalidParams, "minDotAreaPx must be ≥ 0 and below maxDotAreaPx");
        return d;
    }

    private byte[] GrabFrame(string cameraId)
    {
        var f = _frame(cameraId);
        if (!f.CameraExists)
            throw new CalibrationException(CalibrationErrors.UnknownCamera, $"Camera '{cameraId}' does not exist");
        if (!f.Connected || f.Jpeg == null || f.Jpeg.Length == 0)
            throw new CalibrationException(CalibrationErrors.CameraNotConnected, $"Camera '{cameraId}' is not delivering frames");
        return f.Jpeg;
    }

    // ── Saved calibrations ────────────────────────────────────────────────────

    private object? GetCameraCalibration(CalibrationParams p)
    {
        var cameraId = Require(p.CameraId, "cameraId");
        return new { calibration = _repo.Get(cameraId) };
    }

    private object? DeleteCameraCalibration(CalibrationParams p)
    {
        _repo.Delete(Require(p.CameraId, "cameraId"));
        return null;
    }

    // ── Sessions ──────────────────────────────────────────────────────────────

    private object? CalibrationStart(CalibrationParams p)
    {
        var cameraId = Require(p.CameraId, "cameraId");
        if (p.DotPitchMm is not { } pitch || !(pitch > 0) || !double.IsFinite(pitch))
            throw new CalibrationException(CalibrationErrors.InvalidParams, "'dotPitchMm' must be a positive number");
        var detector = DetectorParams(p, null);
        var jpeg = GrabFrame(cameraId);

        var session = _sessions.Create(cameraId, pitch, detector);
        lock (session.Sync)
        {
            try
            {
                var grid = DotGridDetector.DetectJpeg(jpeg, detector);
                var carried = session.ApplyDetection(jpeg, grid);
                return DetectionResponse(session, carried);
            }
            catch (CalibrationException ex)
            {
                // The session is kept (showing the raw frame) so the wizard can show what the
                // camera saw and re-detect with other parameters.
                session.ShowRawFrame(jpeg);
                var error = Error(ex.Code, ex.Message);
                error["sessionId"] = session.Id;
                error["imageUrl"]  = ImageUrl(session);
                return error;
            }
        }
    }

    private object? CalibrationRedetect(CalibrationParams p)
    {
        var session = Session(p);
        lock (session.Sync)
        {
            var detector = DetectorParams(p, session.Params);
            var jpeg = GrabFrame(session.CameraId);
            var grid = DotGridDetector.DetectJpeg(jpeg, detector);
            session.Params = detector;
            if (p.DotPitchMm is { } pitch && pitch > 0 && double.IsFinite(pitch)) session.DotPitchMm = pitch;
            var carried = session.ApplyDetection(jpeg, grid);
            return DetectionResponse(session, carried);
        }
    }

    private object? CalibrationTeachDot(CalibrationParams p)
    {
        var session = Session(p);
        if (p.DotIndex is not { } index)
            throw new CalibrationException(CalibrationErrors.InvalidParams, "'dotIndex' is required");
        lock (session.Sync)
        {
            session.Teach(index, _tcp(), _tool() ?? "", _sessions.NowUnixMs);
            return new { taught = TaughtList(session) };
        }
    }

    private object? CalibrationUnteachDot(CalibrationParams p)
    {
        var session = Session(p);
        if (p.DotIndex is not { } index)
            throw new CalibrationException(CalibrationErrors.InvalidParams, "'dotIndex' is required");
        lock (session.Sync)
        {
            session.Unteach(index);
            return new { taught = TaughtList(session) };
        }
    }

    private object? CalibrationSolve(CalibrationParams p)
    {
        var session = Session(p);
        lock (session.Sync)
        {
            if (session.Grid == null)
                throw new CalibrationException(CalibrationErrors.NotEnoughTaught, "No grid has been detected in this session yet");
            var result = CalibrationSolver.Solve(session.CameraId, session.Grid, session.DotPitchMm,
                                                 session.Taught, _sessions.NowUnixMs);
            var cal = result.Calibration;
            session.Solved = cal;
            bool save = p.Save ?? true;
            if (save) _repo.Save(cal);
            return new
            {
                calibration        = cal,
                taughtRmsMm        = cal.TaughtRmsMm,
                taughtMaxMm        = cal.TaughtMaxMm,
                pitchScaleEstimate = cal.PitchScaleEstimate,
                mirrored           = cal.Mirrored,
                residuals          = cal.TaughtDots.Select(t => new { dotIndex = t.DotIndex, i = t.I, j = t.J, errorMm = t.ErrorMm }),
                warnings           = result.Warnings,
                saved              = save,
            };
        }
    }

    private object? CalibrationPredict(CalibrationParams p)
    {
        if (p.U is not { } u || p.V is not { } v || !double.IsFinite(u) || !double.IsFinite(v))
            throw new CalibrationException(CalibrationErrors.InvalidParams, "'u' and 'v' (normalized 0–1) are required");

        CameraCalibration? cal;
        string source;
        if (!string.IsNullOrWhiteSpace(p.SessionId))
        {
            var session = Session(p);
            lock (session.Sync) cal = session.Solved;
            source = "session";
            if (cal == null)
                throw new CalibrationException(CalibrationErrors.NotCalibrated, "This session has not been solved yet");
        }
        else
        {
            var cameraId = Require(p.CameraId, "cameraId or sessionId");
            cal = _repo.Get(cameraId)
                ?? throw new CalibrationException(CalibrationErrors.NotCalibrated, $"Camera '{cameraId}' has no calibration");
            source = "saved";
        }

        var (x, y, z) = cal.PixelToRobot(u, v);
        return new { robot = new { x, y, z }, source };
    }

    private object? CalibrationDiscard(CalibrationParams p)
    {
        _sessions.Remove(Require(p.SessionId, "sessionId"));
        return null;
    }

    // ── Responses ─────────────────────────────────────────────────────────────

    internal static string ImageUrl(CalibrationSession s) => $"/calibration/{s.Id}/image";

    private static object DetectionResponse(CalibrationSession s, List<string> carriedWarnings)
    {
        var grid = s.Grid!;
        return new
        {
            sessionId   = s.Id,
            cameraId    = s.CameraId,
            dotPitchMm  = s.DotPitchMm,
            imageWidth  = grid.ImageWidth,
            imageHeight = grid.ImageHeight,
            dots        = grid.Dots.Select(d => new { index = d.Index, i = d.I, j = d.J, u = d.U, v = d.V, areaPx = d.AreaPx }),
            gridRows    = grid.Rows,
            gridCols    = grid.Cols,
            gridRmsPx   = grid.RmsPx,
            warnings    = grid.Warnings.Concat(carriedWarnings).ToList(),
            imageUrl    = ImageUrl(s),
            taught      = TaughtList(s),
        };
    }

    private static List<object> TaughtList(CalibrationSession s) =>
        s.Taught.Select(t => (object)new
        {
            dotIndex = t.DotIndex, i = t.I, j = t.J, u = t.U, v = t.V,
            robot = new { x = t.Robot.X, y = t.Robot.Y, z = t.Robot.Z },
            tool  = t.Tool,
        }).ToList();
}
