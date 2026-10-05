using Controller.RobotControl;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Validation;
using Controller.RobotControl.Vision.Calibration;

namespace RobotControl.Tests;

/// <summary>RunVision's outputFrame: the coordinate conversion, validation and the $camera properties.</summary>
public class CalibrationProgramTests
{
    // ── Coordinate conversion ─────────────────────────────────────────────────

    [Fact]
    public void OutputFrameNamesParse()
    {
        Assert.True(VisionOutputFrame.TryParse(null, out var k));   Assert.Equal(OutputFrameKind.Default, k);
        Assert.True(VisionOutputFrame.TryParse("", out k));         Assert.Equal(OutputFrameKind.Default, k);
        Assert.True(VisionOutputFrame.TryParse("pixel", out k));    Assert.Equal(OutputFrameKind.Pixel, k);
        Assert.True(VisionOutputFrame.TryParse("normalized", out k)); Assert.Equal(OutputFrameKind.Normalized, k);
        Assert.True(VisionOutputFrame.TryParse("robot", out k));    Assert.Equal(OutputFrameKind.Robot, k);
        Assert.False(VisionOutputFrame.TryParse("Robot", out _));
        Assert.False(VisionOutputFrame.TryParse("mm", out _));
    }

    [Fact]
    public void FramesConvertPixelAndNormalizedPositions()
    {
        var def = new VisionOutputFrame(OutputFrameKind.Default, 640, 480);
        Assert.Equal((320.0, 240.0, 0.0), def.FromPixel(320, 240));       // blobs stay pixels
        Assert.Equal((0.25, 0.5, 0.0), def.FromNormalized(0.25, 0.5));    // centers stay normalized

        var px = new VisionOutputFrame(OutputFrameKind.Pixel, 640, 480);
        Assert.Equal((160.0, 240.0, 0.0), px.FromNormalized(0.25, 0.5));

        var norm = new VisionOutputFrame(OutputFrameKind.Normalized, 640, 480);
        Assert.Equal((0.5, 0.5, 0.0), norm.FromPixel(320, 240));

        Assert.Throws<ArgumentException>(() => new VisionOutputFrame(OutputFrameKind.Robot, 640, 480));
    }

    [Fact]
    public void RobotFrameUsesTheCalibrationAndPlaneZ()
    {
        using var img = CameraCalibrationTests.RenderSheet();
        var grid = DotGridDetector.Detect(img, new DotDetectorParams());
        var cal = CalibrationSolver.Solve("CAM_0", grid, 20,
            [CameraCalibrationTests.Teach(grid, 0, 20), CameraCalibrationTests.Teach(grid, 7, 20), CameraCalibrationTests.Teach(grid, 40, 20)], 0).Calibration;
        var frame = new VisionOutputFrame(OutputFrameKind.Robot, cal.ImageWidth, cal.ImageHeight, cal);

        var dot = grid.FindDot(21)!;
        var (ex, ey) = CameraCalibrationTests.SheetToRobotTruth(dot.I * 20, dot.J * 20);

        var (x, y, z) = frame.FromPixel(dot.X, dot.Y);
        Assert.Equal(ex, x, 0.25);
        Assert.Equal(ey, y, 0.25);
        Assert.Equal(12.5, z);

        (x, y, _) = frame.FromNormalized(dot.U, dot.V);
        Assert.Equal(ex, x, 0.25);
        Assert.Equal(ey, y, 0.25);
    }

    // ── Validation ────────────────────────────────────────────────────────────

    private static BuiltProgram Prog(params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList() };

    private static ProgramStep Vision(string programId, string? frame) =>
        new() { Id = "v" + programId, Type = StepType.RunVision, VisionProgramId = programId, OutputFrame = frame };

    private static readonly ValidationContext Ctx = new()
    {
        VisionProgramExists = id => id is "calibrated" or "uncalibrated",
        VisionProgramCamera = id => id switch
        {
            "calibrated"   => ("CAM_0", true),
            "uncalibrated" => ("CAM_1", false),
            _              => null,
        },
    };

    [Fact]
    public void RobotFrameNeedsACalibratedCamera()
    {
        var problems = ProgramValidator.Validate(Prog(
            Vision("calibrated", "robot"),
            Vision("uncalibrated", "robot"),
            Vision("uncalibrated", "pixel"),
            Vision("uncalibrated", null)), Ctx);

        var p = Assert.Single(problems);
        Assert.Equal(ValidationCodes.CameraNotCalibrated, p.Code);
        Assert.Equal(ValidationSeverity.Error, p.Severity);
        Assert.Equal("outputFrame", p.Field);
        Assert.Equal("steps[1]", p.StepPath);
        Assert.Contains("CAM_1", p.Message);
    }

    [Fact]
    public void UnknownOutputFrameIsReported()
    {
        var problems = ProgramValidator.Validate(Prog(Vision("calibrated", "world")), Ctx);
        var p = Assert.Single(problems);
        Assert.Equal(ValidationCodes.BadOutputFrame, p.Code);
        Assert.Equal("outputFrame", p.Field);
    }

    [Fact]
    public void OfflineValidationCannotCheckCalibration()
    {
        Assert.Empty(ProgramValidator.Validate(Prog(Vision("anything", "robot")), ValidationContext.Offline()));
    }

    // ── Properties ────────────────────────────────────────────────────────────

    [Fact]
    public void CameraIsAPropertyRootAndUnknownCamerasAreNotProperties()
    {
        Assert.Contains("camera", RobotPropertySource.Roots);
        var props = new RobotPropertySource(new RobotController(), null);
        Assert.False(props.TryGet("camera.NO_SUCH_CAMERA.calibrated", out _));
        Assert.False(props.TryGet("camera.calibrated", out _));
        Assert.False(new RobotPropertySource(null, null).TryGet("camera.CAM_0.calibrated", out _));
    }
}
