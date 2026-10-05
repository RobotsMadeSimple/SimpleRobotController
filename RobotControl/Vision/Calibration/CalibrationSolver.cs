namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>A dot taught during a session: where it was in the image and where the TCP was.</summary>
    public sealed class TaughtDot
    {
        public int      DotIndex { get; set; }
        public int      I        { get; set; }
        public int      J        { get; set; }
        /// <summary>Pixel centroid of the dot in the frame it was taught on.</summary>
        public double   X        { get; set; }
        public double   Y        { get; set; }
        public double   U        { get; set; }
        public double   V        { get; set; }
        public RobotXyz Robot    { get; set; } = new();
        public string   Tool     { get; set; } = "";
        public long     TaughtUnixMs { get; set; }
    }

    public sealed class CalibrationSolveResult
    {
        public CameraCalibration Calibration { get; init; } = new();
        public List<string>      Warnings    { get; init; } = new();
    }

    /// <summary>
    /// Fits sheet → robot from the taught dots and composes the stored pixel → robot map
    /// (docs/camera-calibration.md, "Method" step 4).
    /// </summary>
    public static class CalibrationSolver
    {
        /// <summary>Pitch-scale mismatch above this fraction is warned about.</summary>
        public const double PitchScaleWarnFraction = 0.02;
        /// <summary>A taught-dot residual above this (mm) is warned about.</summary>
        public const double ResidualWarnMm = 1.0;
        /// <summary>A spread of taught Z above this (mm) is warned about.</summary>
        public const double ZSpreadWarnMm = 1.0;

        public static CalibrationSolveResult Solve(string cameraId, DotGridResult grid, double dotPitchMm,
                                                   IReadOnlyList<TaughtDot> taught, long nowUnixMs)
        {
            if (taught.Count < 2)
                throw new CalibrationException(CalibrationErrors.NotEnoughTaught,
                    $"Teach at least 2 dots (3 not in a line is best); {taught.Count} taught");

            var warnings = new List<string>();
            bool collinear = AllCollinear(taught);
            if (taught.Count >= 3 && collinear)
                throw new CalibrationException(CalibrationErrors.TaughtCollinear,
                    "All taught dots lie on one line of the grid; teach one dot off that line");
            if (taught.Count == 2)
                warnings.Add("Only 2 dots taught: the sheet's handedness was assumed for a camera looking down at it. " +
                             "Teach a third dot, not in line with the others, to confirm it");

            // The taught dots' exact sheet positions come from their lattice indices.
            var src = taught.Select(t => (t.I * dotPitchMm, t.J * dotPitchMm)).ToList();
            var dst = taught.Select(t => (t.Robot.X, t.Robot.Y)).ToList();
            // A camera looking down sees the robot's XY plane mirrored (image y runs down), so
            // that is the handedness assumed when the points cannot tell.
            var fit = RigidFit2D.Fit(src, dst, allowReflection: true, preferMirrored: true);

            var pixelToSheet = grid.PixelToSheet(dotPitchMm);
            var pixelToRobot = Homography.ComposeRigid(fit.Transform, pixelToSheet);

            double planeZ = taught.Average(t => t.Robot.Z);
            double zSpread = taught.Max(t => t.Robot.Z) - taught.Min(t => t.Robot.Z);
            if (zSpread > ZSpreadWarnMm)
                warnings.Add($"Taught Z varies by {zSpread:0.0} mm: the sheet may not be level or the tip height was not the same for every dot");
            if (Math.Abs(fit.ScaleEstimate - 1) > PitchScaleWarnFraction)
                warnings.Add($"Taught distances are {(fit.ScaleEstimate - 1) * 100:+0.0;-0.0}% off the {dotPitchMm} mm pitch: check the pitch value, the sheet's print scale and the tool offset");
            if (fit.Max > ResidualWarnMm)
                warnings.Add($"Largest taught-dot residual is {fit.Max:0.00} mm: re-teach the worst dot or check that the tip was centred");
            var tools = taught.Select(t => t.Tool).Distinct(StringComparer.Ordinal).ToList();
            if (tools.Count > 1)
                warnings.Add($"Dots were taught with different active tools ({string.Join(", ", tools)}); the TCP must be the same for every dot");

            var cal = new CameraCalibration
            {
                CameraId           = cameraId,
                ImageWidth         = grid.ImageWidth,
                ImageHeight        = grid.ImageHeight,
                DotPitchMm         = dotPitchMm,
                PixelToSheet       = pixelToSheet,
                SheetToRobot       = fit.Transform,
                PixelToRobotH      = pixelToRobot,
                PlaneZ             = planeZ,
                TaughtDots         = taught.Select((t, k) => new CalibrationTaughtDot
                {
                    DotIndex = t.DotIndex, I = t.I, J = t.J, U = t.U, V = t.V,
                    Robot    = new RobotXyz { X = t.Robot.X, Y = t.Robot.Y, Z = t.Robot.Z },
                    ErrorMm  = fit.Residuals[k],
                }).ToList(),
                GridRows           = grid.Rows,
                GridCols           = grid.Cols,
                DotCount           = grid.Dots.Count,
                GridRmsPx          = grid.RmsPx,
                TaughtRmsMm        = fit.Rms,
                TaughtMaxMm        = fit.Max,
                PitchScaleEstimate = fit.ScaleEstimate,
                Mirrored           = fit.Transform.Mirrored,
                ActiveTool         = taught[^1].Tool,
                CalibratedUnixMs   = nowUnixMs,
            };
            return new CalibrationSolveResult { Calibration = cal, Warnings = warnings };
        }

        /// <summary>True when every taught dot lies on one line of the lattice (exact, by index).</summary>
        public static bool AllCollinear(IReadOnlyList<TaughtDot> taught)
        {
            if (taught.Count < 3) return true;
            var o = taught[0];
            // The first dot at a different index sets the direction.
            var d = taught.FirstOrDefault(t => t.I != o.I || t.J != o.J);
            if (d == null) return true;
            long di = d.I - o.I, dj = d.J - o.J;
            return taught.All(t => di * (t.J - o.J) - dj * (t.I - o.I) == 0);
        }
    }
}
