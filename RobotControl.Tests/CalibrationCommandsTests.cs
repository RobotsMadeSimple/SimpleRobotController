using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Vision.Calibration;
using OpenCvSharp;

namespace RobotControl.Tests;

/// <summary>
/// The calibration commands through the dispatcher, with a synthetic camera frame and a
/// scripted TCP instead of hardware: the whole wizard flow and the argument/error rules.
/// </summary>
public sealed class CalibrationCommandsTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "calib-cmd-" + Guid.NewGuid().ToString("N"));
    private readonly CameraCalibrationRepository _repo;
    private readonly CalibrationSessionManager _sessions = new();
    private readonly CommandDispatcher _d = new();
    private readonly byte[] _jpeg;
    private bool _connected = true;
    private RobotXyz _tcp = new();

    public CalibrationCommandsTests()
    {
        _repo = new CameraCalibrationRepository(_dir);
        using var img = CameraCalibrationTests.RenderSheet();
        _jpeg = img.ImEncode(".png"); // lossless, so the synthetic centroids stay exact
        new CalibrationCommands(_repo, _sessions,
            id => id == "CAM_0" ? new CalibrationFrame(true, _connected, _connected ? _jpeg : null) : new CalibrationFrame(false, false, null),
            () => _tcp, () => "Pen").Register(_d);
    }

    public void Dispose()
    {
        if (Directory.Exists(_dir)) Directory.Delete(_dir, true);
    }

    private JsonElement Send(string command, object? parameters = null)
    {
        Assert.True(_d.TryGet(command, out var handler), command);
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = parameters == null ? null : JsonSerializer.SerializeToElement(parameters),
        };
        var result = handler(msg).GetAwaiter().GetResult();
        return result == null ? JsonSerializer.SerializeToElement(new { }) : JsonSerializer.SerializeToElement(result);
    }

    private static string? Error(JsonElement r) =>
        r.TryGetProperty("ok", out var ok) && !ok.GetBoolean() ? r.GetProperty("error").GetString() : null;

    [Fact]
    public void FullWizardFlowSolvesSavesAndPredicts()
    {
        var start = Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = 20 });
        Assert.Null(Error(start));
        var sessionId = start.GetProperty("sessionId").GetString()!;
        Assert.Equal(48, start.GetProperty("dots").GetArrayLength());
        Assert.Equal(6, start.GetProperty("gridRows").GetInt32());
        Assert.Equal(8, start.GetProperty("gridCols").GetInt32());
        Assert.Equal($"/calibration/{sessionId}/image", start.GetProperty("imageUrl").GetString());
        Assert.NotNull(_sessions.Get(sessionId)!.AnnotatedJpeg);

        var dots = start.GetProperty("dots").EnumerateArray().ToDictionary(d => d.GetProperty("index").GetInt32());
        foreach (var index in new[] { 0, 7, 40 })
        {
            var d = dots[index];
            var (x, y) = CameraCalibrationTests.SheetToRobotTruth(d.GetProperty("i").GetInt32() * 20, d.GetProperty("j").GetInt32() * 20);
            _tcp = new RobotXyz { X = x, Y = y, Z = 12.5 };
            var taught = Send("CalibrationTeachDot", new { sessionId, dotIndex = index });
            Assert.Null(Error(taught));
        }

        Assert.Equal("notCalibrated", Error(Send("CalibrationPredict", new { sessionId, u = 0.5, v = 0.5 })));

        var solve = Send("CalibrationSolve", new { sessionId, save = false });
        Assert.Null(Error(solve));
        Assert.True(solve.GetProperty("taughtRmsMm").GetDouble() < 0.05);
        Assert.True(solve.GetProperty("mirrored").GetBoolean());
        Assert.Equal(3, solve.GetProperty("residuals").GetArrayLength());
        Assert.Equal("Pen", solve.GetProperty("calibration").GetProperty("activeTool").GetString());
        Assert.Null(_repo.Get("CAM_0"));

        // Predict from the solved session: the centre of dot 20 lands on its true robot position.
        var d20 = dots[20];
        var p = Send("CalibrationPredict", new { sessionId, u = d20.GetProperty("u").GetDouble(), v = d20.GetProperty("v").GetDouble() });
        Assert.Equal("session", p.GetProperty("source").GetString());
        var (ex, ey) = CameraCalibrationTests.SheetToRobotTruth(d20.GetProperty("i").GetInt32() * 20, d20.GetProperty("j").GetInt32() * 20);
        Assert.Equal(ex, p.GetProperty("robot").GetProperty("x").GetDouble(), 0.25);
        Assert.Equal(ey, p.GetProperty("robot").GetProperty("y").GetDouble(), 0.25);
        Assert.Equal(12.5, p.GetProperty("robot").GetProperty("z").GetDouble(), 9);

        Assert.Equal("notCalibrated", Error(Send("CalibrationPredict", new { cameraId = "CAM_0", u = 0.5, v = 0.5 })));
        Assert.Null(Error(Send("CalibrationSolve", new { sessionId })));
        Assert.NotNull(_repo.Get("CAM_0"));
        Assert.Equal("saved", Send("CalibrationPredict", new { cameraId = "CAM_0", u = 0.5, v = 0.5 }).GetProperty("source").GetString());

        var get = Send("GetCameraCalibration", new { cameraId = "CAM_0" });
        Assert.Equal("CAM_0", get.GetProperty("calibration").GetProperty("cameraId").GetString());
        Assert.Equal(3, get.GetProperty("calibration").GetProperty("pixelToRobot").GetArrayLength());

        // Redetect on the same frame keeps the taught dots.
        var re = Send("CalibrationRedetect", new { sessionId });
        Assert.Null(Error(re));
        Assert.Equal(3, re.GetProperty("taught").GetArrayLength());

        Assert.Equal(2, Send("CalibrationUnteachDot", new { sessionId, dotIndex = 40 }).GetProperty("taught").GetArrayLength());

        Send("DeleteCameraCalibration", new { cameraId = "CAM_0" });
        Assert.Equal(JsonValueKind.Null, Send("GetCameraCalibration", new { cameraId = "CAM_0" }).GetProperty("calibration").ValueKind);

        Send("CalibrationDiscard", new { sessionId });
        Assert.Equal("unknownSession", Error(Send("CalibrationTeachDot", new { sessionId, dotIndex = 0 })));
    }

    [Fact]
    public void ArgumentsAndErrorCodes()
    {
        Assert.Equal("invalidParams", Error(Send("CalibrationStart", new { cameraId = "CAM_0" })));
        Assert.Equal("invalidParams", Error(Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = -1 })));
        Assert.Equal("invalidParams", Error(Send("CalibrationStart", new { dotPitchMm = 20 })));
        Assert.Equal("invalidParams", Error(Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = 20, minDotAreaPx = 500, maxDotAreaPx = 100 })));
        Assert.Equal("invalidParams", Error(Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = "twenty" })));
        Assert.Equal("invalidParams", Error(Send("GetCameraCalibration")));
        Assert.Equal("unknownCamera", Error(Send("CalibrationStart", new { cameraId = "NOPE", dotPitchMm = 20 })));

        _connected = false;
        Assert.Equal("cameraNotConnected", Error(Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = 20 })));
        _connected = true;

        // Dots outside the area range: noDotsFound, but the session survives with the raw frame.
        var none = Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = 20, minDotAreaPx = 5000, maxDotAreaPx = 6000 });
        Assert.Equal("noDotsFound", Error(none));
        var failedSession = none.GetProperty("sessionId").GetString()!;
        Assert.NotNull(_sessions.Get(failedSession)!.AnnotatedJpeg);
        Assert.Null(Error(Send("CalibrationRedetect", new { sessionId = failedSession, minDotAreaPx = 30, maxDotAreaPx = 20000 })));

        var sessionId = Send("CalibrationStart", new { cameraId = "CAM_0", dotPitchMm = 20 }).GetProperty("sessionId").GetString()!;
        Assert.Equal("unknownSession", Error(Send("CalibrationTeachDot", new { sessionId = "nope", dotIndex = 0 })));
        Assert.Equal("unknownDot",     Error(Send("CalibrationTeachDot", new { sessionId, dotIndex = 99 })));
        Assert.Equal("invalidParams",  Error(Send("CalibrationTeachDot", new { sessionId })));
        Assert.Equal("unknownDot",     Error(Send("CalibrationUnteachDot", new { sessionId, dotIndex = -3 })));

        Send("CalibrationTeachDot", new { sessionId, dotIndex = 0 });
        Assert.Equal("notEnoughTaught", Error(Send("CalibrationSolve", new { sessionId })));
        _tcp = new RobotXyz { X = 20 };
        Send("CalibrationTeachDot", new { sessionId, dotIndex = 1 });
        _tcp = new RobotXyz { X = 40 };
        Send("CalibrationTeachDot", new { sessionId, dotIndex = 2 });
        Assert.Equal("taughtCollinear", Error(Send("CalibrationSolve", new { sessionId })));

        Assert.Equal("invalidParams", Error(Send("CalibrationPredict", new { cameraId = "CAM_0" })));
        Assert.Equal("notCalibrated", Error(Send("CalibrationPredict", new { cameraId = "CAM_0", u = 0.1, v = 0.1 })));
    }

    [Fact]
    public void DispatcherRegistersEveryCalibrationCommand()
    {
        var robot = new RobotController();
        var programs = new ProgramCycleManager();
        var background = new BackgroundProgramManager(robot, programs, robot.pointRepo, robot.toolRepo,
            robot.localRepo, robot.builtProgramRepo, robot.gridRepo, robot.stackRepo);
        var d = CommandDispatcher.Create(robot, programs, null, background);
        foreach (var name in new[] { "GetCameraCalibration", "DeleteCameraCalibration", "CalibrationStart", "CalibrationRedetect",
                                     "CalibrationTeachDot", "CalibrationUnteachDot", "CalibrationSolve", "CalibrationPredict", "CalibrationDiscard" })
            Assert.Contains(name, d.Names);
    }
}
