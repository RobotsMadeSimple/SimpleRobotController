namespace Controller.RobotControl.Conveyors
{
    /// <summary>
    /// ConveyorCalibrate math (docs/conveyor-tracking.md, "Calibration"). The samples are the TCP
    /// jogged onto the same belt mark at two or more belt positions, each with the travel the
    /// controller reported then. A least-squares line of position against travel gives the belt
    /// direction (frame +X = the way the mark moved, first sample → last), the origin (the line at
    /// the first sample) and the travel scale (robot mm per reported mm, signed — a belt whose count
    /// runs backwards gets a negative factor). Pure; nothing is saved.
    /// </summary>
    public static class ConveyorCalibration
    {
        /// <summary>Samples closer than this along the belt cannot give a direction (mm).</summary>
        public const double MinSpanMm = 1.0;

        /// <param name="currentScale">The conveyor's scale when the samples' travel was reported.</param>
        /// <exception cref="ArgumentException">Fewer than two samples, or the belt / mark did not move.</exception>
        public static ConveyorCalibrationResult Compute(
            IReadOnlyList<ConveyorCalibrationSample> samples, ConveyorCalibrationSample? sensorSample, double currentScale = 1.0)
        {
            if (samples == null || samples.Count < 2)
                throw new ArgumentException("Calibration needs at least two samples of the same belt mark");
            if (samples.Any(s => !double.IsFinite(s.X) || !double.IsFinite(s.Y) || !double.IsFinite(s.Z) || !double.IsFinite(s.TravelMm)))
                throw new ArgumentException("Calibration samples must be numbers");
            if (!double.IsFinite(currentScale) || currentScale == 0) currentScale = 1.0;

            int n = samples.Count;
            double tm = samples.Average(s => s.TravelMm);
            double xm = samples.Average(s => s.X);
            double ym = samples.Average(s => s.Y);
            double zm = samples.Average(s => s.Z);
            double stt = 0, stx = 0, sty = 0;
            foreach (var s in samples)
            {
                double dt = s.TravelMm - tm;
                stt += dt * dt;
                stx += dt * (s.X - xm);
                sty += dt * (s.Y - ym);
            }
            double spanT = samples.Max(s => s.TravelMm) - samples.Min(s => s.TravelMm);
            if (spanT < MinSpanMm || stt <= 1e-12)
                throw new ArgumentException("The belt did not move between the samples — run it forward before the next sample");

            // Position = mean + v · (travel − mean travel); v in robot mm per reported mm.
            double vx = stx / stt, vy = sty / stt;
            double vlen = Math.Sqrt(vx * vx + vy * vy);
            var first = samples[0];
            var last  = samples[^1];
            double mx = last.X - first.X, my = last.Y - first.Y;
            if (vlen < 1e-9 || Math.Sqrt(mx * mx + my * my) < MinSpanMm)
                throw new ArgumentException("The mark did not move between the samples — jog onto the same mark after the belt ran");

            // +X is the direction the mark travelled (first → last sample).
            double dx = vx / vlen, dy = vy / vlen;
            if (dx * mx + dy * my < 0) { dx = -dx; dy = -dy; }
            double k = vx * dx + vy * dy; // signed robot mm per reported mm

            double ox = xm + vx * (first.TravelMm - tm);
            double oy = ym + vy * (first.TravelMm - tm);

            double residual = 0;
            foreach (var s in samples)
            {
                double fx = xm + vx * (s.TravelMm - tm), fy = ym + vy * (s.TravelMm - tm);
                double e = Math.Sqrt((s.X - fx) * (s.X - fx) + (s.Y - fy) * (s.Y - fy) + (s.Z - zm) * (s.Z - zm));
                residual = Math.Max(residual, e);
            }

            double? sensorOffset = null;
            if (sensorSample != null)
            {
                if (sensorSample.LatchTravelMm is not { } latch || !double.IsFinite(latch))
                    throw new ArgumentException("sensorSample needs latchTravelMm (the travel when the part was latched)");
                double partX = dx * (sensorSample.X - ox) + dy * (sensorSample.Y - oy);
                sensorOffset = partX - k * (sensorSample.TravelMm - latch);
            }

            return new ConveyorCalibrationResult
            {
                Frame = new ConveyorFrame
                {
                    X = Math.Round(ox, 3), Y = Math.Round(oy, 3), Z = Math.Round(zm, 3),
                    RZ = Math.Round(Math.Atan2(dy, dx) * 180.0 / Math.PI, 4),
                },
                Scale          = Math.Round(currentScale * k, 6),
                SensorOffsetMm = sensorOffset is { } o ? Math.Round(o, 3) : null,
                ResidualMm     = Math.Round(residual, 3),
            };
        }
    }
}
