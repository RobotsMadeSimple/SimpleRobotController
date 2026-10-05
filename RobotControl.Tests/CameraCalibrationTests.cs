using Controller.RobotControl.Vision.Calibration;
using OpenCvSharp;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

/// <summary>
/// Camera-to-robot calibration (docs/camera-calibration.md): dot-grid detection and lattice
/// fitting on synthetic sheets, the rigid sheet → robot fit, the composed pixel ↔ robot map,
/// persistence and session expiry.
/// </summary>
public class CameraCalibrationTests
{
    internal const int W = 640, H = 480;
    internal const int Rows = 6, Cols = 8;
    internal const double PitchPx = 40, OriginX = 170, OriginY = 130, DotRadius = 8;

    /// <summary>A mild perspective: the image corners pulled in by a few percent, unevenly.</summary>
    internal static readonly Point2f[] WarpSrc = [new(0, 0), new(W, 0), new(W, H), new(0, H)];
    internal static readonly Point2f[] WarpDst = [new(18, 10), new(W - 6, 22), new(W - 24, H - 8), new(4, H - 20)];

    internal static (double X, double Y) Ideal(double i, double j) => (OriginX + i * PitchPx, OriginY + j * PitchPx);

    internal static double[][] WarpMatrix()
    {
        using var m = Cv2.GetPerspectiveTransform(WarpSrc, WarpDst);
        return Homography.FromMat(m);
    }

    /// <summary>Where lattice point (i, j) lands in the warped image.</summary>
    internal static (double X, double Y) Warped(double i, double j)
    {
        var (x, y) = Ideal(i, j);
        return Homography.Apply(WarpMatrix(), x, y);
    }

    /// <summary>A white sheet with Rows × Cols dark dots (plus <paramref name="extra"/> lattice positions), perspective-warped.</summary>
    internal static Mat RenderSheet(IEnumerable<(double I, double J)>? extra = null, bool darkDots = true)
    {
        var bg = darkDots ? Scalar.White : Scalar.Black;
        var fg = darkDots ? Scalar.Black : Scalar.White;
        using var flat = new Mat(H, W, MatType.CV_8UC3, bg);
        var positions = new List<(double I, double J)>();
        for (int j = 0; j < Rows; j++)
            for (int i = 0; i < Cols; i++) positions.Add((i, j));
        if (extra != null) positions.AddRange(extra);

        const int shift = 4; // sub-pixel centres
        foreach (var (i, j) in positions)
        {
            var (x, y) = Ideal(i, j);
            Cv2.Circle(flat, new Point((int)Math.Round(x * 16), (int)Math.Round(y * 16)), (int)(DotRadius * 16), fg, -1, LineTypes.AntiAlias, shift);
        }

        var warped = new Mat();
        using var m = Cv2.GetPerspectiveTransform(WarpSrc, WarpDst);
        Cv2.WarpPerspective(flat, warped, m, new Size(W, H), InterpolationFlags.Linear, BorderTypes.Constant, bg);
        return warped;
    }

    // ── Detection ─────────────────────────────────────────────────────────────

    [Fact]
    public void DetectorRecoversAWarpedGrid()
    {
        using var img = RenderSheet();
        var grid = DotGridDetector.Detect(img, new DotDetectorParams());

        Assert.Equal(Rows * Cols, grid.Dots.Count);
        Assert.Equal(Rows, grid.Rows);
        Assert.Equal(Cols, grid.Cols);
        Assert.True(grid.RmsPx < 0.5, $"rms {grid.RmsPx}");
        Assert.Equal(0, grid.DroppedCount);

        // Dot 0 is the top-left one, i runs right and j down; every dot sits where the warp put it.
        foreach (var d in grid.Dots)
        {
            var (ex, ey) = Warped(d.I, d.J);
            Assert.True(Math.Abs(d.X - ex) < 0.5 && Math.Abs(d.Y - ey) < 0.5, $"dot {d.Index} ({d.I},{d.J}) at {d.X:0.00},{d.Y:0.00} expected {ex:0.00},{ey:0.00}");
            Assert.Equal(d.J * Cols + d.I, d.Index);
            Assert.InRange(d.U, 0, 1);
            Assert.Equal(d.X / W, d.U, 9);
        }

        // The lattice homography predicts even the positions between dots.
        var (px, py) = Homography.Apply(grid.LatticeToPixel, 3.5, 2.5);
        var (qx, qy) = Warped(3.5, 2.5);
        Assert.True(Math.Abs(px - qx) < 0.5 && Math.Abs(py - qy) < 0.5);
    }

    [Fact]
    public void DetectorFindsLightDotsOnADarkSheet()
    {
        using var img = RenderSheet(darkDots: false);
        var grid = DotGridDetector.Detect(img, new DotDetectorParams { DarkDots = false });
        Assert.Equal(Rows * Cols, grid.Dots.Count);
        Assert.True(grid.RmsPx < 0.5);
    }

    [Fact]
    public void AStrayDotOffTheLatticeIsDropped()
    {
        using var img = RenderSheet(extra: [(Cols + 0.25, 2.2)]);
        Assert.Equal(Rows * Cols + 1, DotGridDetector.DetectBlobs(img, new DotDetectorParams()).Count);

        var grid = DotGridDetector.Detect(img, new DotDetectorParams());
        Assert.Equal(Rows * Cols, grid.Dots.Count);
        Assert.Equal(Cols, grid.Cols);
        Assert.Equal(1, grid.DroppedCount);
        Assert.Contains(grid.Warnings, w => w.Contains("ignored"));
        Assert.True(grid.RmsPx < 0.5);
    }

    [Fact]
    public void OutliersBeyondThreeRmsAreDroppedFromNoisyCentroids()
    {
        var rnd = new Random(7);
        var blobs = new List<DotBlob>();
        for (int j = 0; j < Rows; j++)
            for (int i = 0; i < Cols; i++)
            {
                var (x, y) = Warped(i, j);
                blobs.Add(new DotBlob(x + (rnd.NextDouble() - 0.5) * 0.4, y + (rnd.NextDouble() - 0.5) * 0.4, 200));
            }
        var (sx, sy) = Warped(Cols + 0.2, 1.25);     // a stray that rounds onto an empty lattice position
        blobs.Add(new DotBlob(sx, sy, 200));
        // …and one that sits on a lattice point already owned by a real dot.
        var (dx, dy) = Warped(2.3, 3);
        blobs.Add(new DotBlob(dx, dy, 200));

        var grid = DotGridDetector.FitGrid(blobs, W, H);
        Assert.Equal(Rows * Cols, grid.Dots.Count);
        Assert.Equal(2, grid.DroppedCount);
        Assert.True(grid.RmsPx < 0.5);
    }

    [Fact]
    public void RandomDotsAreNotAGrid()
    {
        var rnd = new Random(1234);
        var blobs = Enumerable.Range(0, 40)
            .Select(_ => new DotBlob(20 + rnd.NextDouble() * 600, 20 + rnd.NextDouble() * 440, 150)).ToList();
        var ex = Assert.Throws<CalibrationException>(() => DotGridDetector.FitGrid(blobs, W, H));
        Assert.Equal(CalibrationErrors.GridNotFound, ex.Code);
    }

    [Fact]
    public void TooFewDotsIsGridNotFoundAndNoneIsNoDotsFound()
    {
        var ex = Assert.Throws<CalibrationException>(() =>
            DotGridDetector.FitGrid([new(10, 10, 100), new(50, 10, 100), new(10, 50, 100)], W, H));
        Assert.Equal(CalibrationErrors.GridNotFound, ex.Code);

        using var blank = new Mat(H, W, MatType.CV_8UC3, Scalar.White);
        ex = Assert.Throws<CalibrationException>(() => DotGridDetector.Detect(blank, new DotDetectorParams()));
        Assert.Equal(CalibrationErrors.NoDotsFound, ex.Code);
    }

    // ── Rigid fit ─────────────────────────────────────────────────────────────

    private static List<(double X, double Y)> Transform(IEnumerable<(double X, double Y)> pts, double deg, double tx, double ty, bool mirror)
    {
        double c = Math.Cos(deg * Math.PI / 180), s = Math.Sin(deg * Math.PI / 180);
        return pts.Select(p =>
        {
            double y = mirror ? -p.Y : p.Y;
            return (c * p.X - s * y + tx, s * p.X + c * y + ty);
        }).ToList();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RigidFitRecoversRotationTranslationAndMirroring(bool mirror)
    {
        List<(double X, double Y)> src = [(0, 0), (140, 0), (0, 100), (140, 100)];
        var dst = Transform(src, 32.5, 250, -80, mirror);

        var fit = RigidFit2D.Fit(src, dst);
        Assert.Equal(mirror, fit.Transform.Mirrored);
        Assert.Equal(32.5, fit.Transform.AngleDeg, 6);
        Assert.Equal(250, fit.Transform.Tx, 6);
        Assert.Equal(-80, fit.Transform.Ty, 6);
        Assert.True(fit.Rms < 1e-9 && fit.Max < 1e-9);
        Assert.Equal(1, fit.ScaleEstimate, 9);
        Assert.False(fit.HandednessAssumed);
    }

    [Fact]
    public void RigidFitReportsResidualsAndScale()
    {
        List<(double X, double Y)> src = [(0, 0), (100, 0), (0, 100)];
        var dst = Transform(src, -10, 5, 5, false);
        dst = dst.Select(p => (p.X * 1.01, p.Y * 1.01)).ToList(); // taught 1 % long
        var fit = RigidFit2D.Fit(src, dst);
        Assert.Equal(1.01, fit.ScaleEstimate, 6);
        Assert.True(fit.Rms > 0.1 && fit.Max >= fit.Rms);
        Assert.Equal(3, fit.Residuals.Length);
    }

    [Fact]
    public void TwoPointsUseThePreferredHandedness()
    {
        List<(double X, double Y)> src = [(0, 0), (100, 0)];
        var dst = Transform(src, 90, 10, 20, true);
        var fit = RigidFit2D.Fit(src, dst, preferMirrored: true);
        Assert.True(fit.HandednessAssumed);
        Assert.True(fit.Transform.Mirrored);
        Assert.True(fit.Rms < 1e-9);
    }

    // ── Solve and the composed map ────────────────────────────────────────────

    /// <summary>The robot pose of sheet point (sx, sy) mm for a camera looking down: mirrored, rotated 20°, offset.</summary>
    internal static (double X, double Y) SheetToRobotTruth(double sx, double sy) =>
        Transform([(sx, sy)], 20, 300, -40, mirror: true)[0];

    internal static TaughtDot Teach(DotGridResult grid, int index, double pitch, double z = 12.5, string tool = "Pen")
    {
        var d = grid.FindDot(index)!;
        var (x, y) = SheetToRobotTruth(d.I * pitch, d.J * pitch);
        return new TaughtDot { DotIndex = d.Index, I = d.I, J = d.J, X = d.X, Y = d.Y, U = d.U, V = d.V,
                               Robot = new RobotXyz { X = x, Y = y, Z = z }, Tool = tool };
    }

    [Fact]
    public void SolvedCalibrationMapsPixelsToRobotAndBack()
    {
        using var img = RenderSheet();
        var grid = DotGridDetector.Detect(img, new DotDetectorParams());
        const double pitch = 20;
        var taught = new List<TaughtDot> { Teach(grid, 0, pitch), Teach(grid, 7, pitch), Teach(grid, 40, pitch) };

        var r = CalibrationSolver.Solve("CAM_0", grid, pitch, taught, 1_790_000_000_000);
        var cal = r.Calibration;
        Assert.True(cal.Mirrored);
        Assert.True(cal.TaughtRmsMm < 0.05, $"taught rms {cal.TaughtRmsMm}");
        Assert.Equal(1, cal.PitchScaleEstimate, 2);
        Assert.Equal(12.5, cal.PlaneZ, 9);
        Assert.Equal(Rows * Cols, cal.DotCount);
        Assert.Equal("Pen", cal.ActiveTool);
        Assert.Empty(r.Warnings);

        // Every dot — taught or not — maps to its true robot position.
        foreach (var d in grid.Dots)
        {
            var (x, y, z) = cal.PixelToRobot(d.U, d.V);
            var (ex, ey) = SheetToRobotTruth(d.I * pitch, d.J * pitch);
            Assert.True(Math.Abs(x - ex) < 0.25 && Math.Abs(y - ey) < 0.25, $"dot {d.Index}: {x:0.000},{y:0.000} vs {ex:0.000},{ey:0.000}");
            Assert.Equal(12.5, z);

            var (u, v) = cal.RobotToPixel(x, y);
            Assert.Equal(d.U, u, 9);
            Assert.Equal(d.V, v, 9);
        }
    }

    [Fact]
    public void SolveRulesForTaughtDots()
    {
        using var img = RenderSheet();
        var grid = DotGridDetector.Detect(img, new DotDetectorParams());

        var ex = Assert.Throws<CalibrationException>(() => CalibrationSolver.Solve("C", grid, 20, [Teach(grid, 0, 20)], 0));
        Assert.Equal(CalibrationErrors.NotEnoughTaught, ex.Code);

        // Three dots along one row.
        ex = Assert.Throws<CalibrationException>(() =>
            CalibrationSolver.Solve("C", grid, 20, [Teach(grid, 0, 20), Teach(grid, 3, 20), Teach(grid, 7, 20)], 0));
        Assert.Equal(CalibrationErrors.TaughtCollinear, ex.Code);

        // Three along a diagonal are collinear too.
        ex = Assert.Throws<CalibrationException>(() =>
            CalibrationSolver.Solve("C", grid, 20, [Teach(grid, 0, 20), Teach(grid, 9, 20), Teach(grid, 18, 20)], 0));
        Assert.Equal(CalibrationErrors.TaughtCollinear, ex.Code);

        // Two is allowed, with a warning; the mirrored (camera-looking-down) handedness is assumed.
        var r = CalibrationSolver.Solve("C", grid, 20, [Teach(grid, 0, 20), Teach(grid, 47, 20)], 0);
        Assert.Contains(r.Warnings, w => w.Contains("Only 2"));
        Assert.True(r.Calibration.Mirrored);

        // A wrong pitch shows up as a scale mismatch warning.
        r = CalibrationSolver.Solve("C", grid, 19, [Teach(grid, 0, 20), Teach(grid, 7, 20), Teach(grid, 40, 20)], 0);
        Assert.True(r.Calibration.PitchScaleEstimate > 1.04);
        Assert.Contains(r.Warnings, w => w.Contains("pitch"));
    }

    // ── Persistence and sessions ──────────────────────────────────────────────

    [Fact]
    public void RepositorySavesLoadsAndDeletes()
    {
        var dir = Path.Combine(Path.GetTempPath(), "calib-test-" + Guid.NewGuid().ToString("N"));
        try
        {
            using var img = RenderSheet();
            var grid = DotGridDetector.Detect(img, new DotDetectorParams());
            var cal = CalibrationSolver.Solve("CAM_1", grid, 20,
                [Teach(grid, 0, 20), Teach(grid, 7, 20), Teach(grid, 40, 20)], 123).Calibration;

            var repo = new CameraCalibrationRepository(dir);
            Assert.False(repo.IsCalibrated("CAM_1"));
            repo.Save(cal);
            Assert.True(File.Exists(Path.Combine(dir, "CAM_1.json")));
            var json = File.ReadAllText(Path.Combine(dir, "CAM_1.json"));
            Assert.Contains("\"pixelToRobot\"", json);
            Assert.Contains("\"sheetToRobot\"", json);
            Assert.Contains("\"taughtRmsMm\"", json);

            var reloaded = new CameraCalibrationRepository(dir).Get("CAM_1");
            Assert.NotNull(reloaded);
            Assert.Equal(cal.PlaneZ, reloaded!.PlaneZ);
            Assert.Equal(3, reloaded.TaughtDots.Count);
            var a = cal.PixelToRobot(0.3, 0.6);
            var b = reloaded.PixelToRobot(0.3, 0.6);
            Assert.Equal(a.X, b.X, 9);
            Assert.Equal(a.Y, b.Y, 9);
            Assert.Single(repo.List());

            Assert.True(repo.Delete("CAM_1"));
            Assert.Null(repo.Get("CAM_1"));
            Assert.Null(new CameraCalibrationRepository(dir).Get("CAM_1"));
        }
        finally
        {
            if (Directory.Exists(dir)) Directory.Delete(dir, true);
        }
    }

    internal sealed class ManualTime : TimeProvider
    {
        public DateTimeOffset Now = new(2026, 1, 1, 0, 0, 0, TimeSpan.Zero);
        public override DateTimeOffset GetUtcNow() => Now;
    }

    [Fact]
    public void SessionsExpireThirtyMinutesAfterLastUse()
    {
        var time = new ManualTime();
        var mgr = new CalibrationSessionManager(time);
        var s = mgr.Create("CAM_0", 20, new DotDetectorParams());

        time.Now += TimeSpan.FromMinutes(20);
        Assert.Same(s, mgr.Get(s.Id));          // use resets the clock
        time.Now += TimeSpan.FromMinutes(29);
        Assert.Same(s, mgr.Get(s.Id));
        time.Now += TimeSpan.FromMinutes(31);
        Assert.Null(mgr.Get(s.Id));
        Assert.Equal(0, mgr.Count);
    }

    [Fact]
    public void SessionTeachingAndRedetectKeepMatchingDots()
    {
        using var img = RenderSheet();
        var jpeg = img.ImEncode(".jpg");
        var grid = DotGridDetector.Detect(img, new DotDetectorParams());
        var s = new CalibrationSession("s", "CAM_0", 20, new DotDetectorParams());
        Assert.Empty(s.ApplyDetection(jpeg, grid));
        Assert.NotNull(s.AnnotatedJpeg);

        s.Teach(5, new RobotXyz { X = 1, Y = 2, Z = 3 }, "None", 0);
        s.Teach(5, new RobotXyz { X = 4, Y = 5, Z = 6 }, "None", 0); // replaces
        s.Teach(40, new RobotXyz(), "None", 0);
        Assert.Equal(2, s.Taught.Count);
        Assert.Equal(4, s.Taught.Single(t => t.DotIndex == 5).Robot.X);

        var ex = Assert.Throws<CalibrationException>(() => s.Teach(999, new RobotXyz(), "None", 0));
        Assert.Equal(CalibrationErrors.UnknownDot, ex.Code);

        // Same frame again: both taught dots are carried over.
        Assert.Empty(s.ApplyDetection(jpeg, grid));
        Assert.Equal(2, s.Taught.Count);

        s.Unteach(40);
        Assert.Single(s.Taught);
    }
}
