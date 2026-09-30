using System;
using System.Collections.Generic;
using System.Globalization;

namespace Controller.RobotControl.Gcode
{
    /// <summary>Thrown for a line the interpreter cannot parse or a code it does not support.
    /// File/step mode skips-with-log; streaming returns it as <c>error:</c> to the sender.</summary>
    public sealed class GcodeException : Exception
    {
        public GcodeException(string message) : base(message) { }
    }

    /// <summary>Tuning shared by the file/step and streaming paths.</summary>
    public sealed class GcodeOptions
    {
        /// <summary>Rapid (G0) speed in mm/s. Linear (G1) uses the F word (÷60) instead.</summary>
        public double RapidSpeedMmPerSec { get; set; } = 100.0;
        /// <summary>Feed used when a G1 has never been given an F word (mm/s).</summary>
        public double DefaultFeedMmPerSec { get; set; } = 50.0;
        /// <summary>Max chord error (mm) when flattening G2/G3 arcs into line segments.</summary>
        public double ArcToleranceMm { get; set; } = 0.1;
    }

    public enum GcodeOpKind { Move, Dwell, Spindle, SetPosition, Pause, End, Home }

    /// <summary>
    /// One executable effect of a G-code line. A single line can yield several (an arc becomes
    /// many <see cref="GcodeOpKind.Move"/> ops). Move axis values are per-line "present" axes:
    /// null = the axis was not commanded (hold it). When <see cref="Relative"/> the values are
    /// deltas (G91); otherwise absolute base-frame targets (G90, units already converted to mm).
    /// </summary>
    public sealed class GcodeOp
    {
        public GcodeOpKind Kind { get; init; }

        // Move
        public bool    Rapid    { get; init; }
        public bool    Relative { get; init; }
        public double? X { get; init; }
        public double? Y { get; init; }
        public double? Z { get; init; }
        public double? A { get; init; } // rotary → RZ
        /// <summary>Feed in mm/s for a linear move; null for a rapid (uses the rapid speed).</summary>
        public double? FeedMmPerSec { get; init; }
        /// <summary>One segment of a flattened G2/G3 arc — blended into a continuous path.</summary>
        public bool ArcSegment { get; init; }

        // Dwell
        public double DwellMs { get; init; }

        // Spindle (M3/M4/M5)
        public bool    SpindleOn { get; init; }
        public double? SpindleSpeed { get; init; }

        public static GcodeOp Dwell(double ms)      => new() { Kind = GcodeOpKind.Dwell, DwellMs = ms };
        public static GcodeOp Spindle(bool on, double? s) => new() { Kind = GcodeOpKind.Spindle, SpindleOn = on, SpindleSpeed = s };
        public static GcodeOp Pause()               => new() { Kind = GcodeOpKind.Pause };
        public static GcodeOp End()                 => new() { Kind = GcodeOpKind.End };
        public static GcodeOp Home()                => new() { Kind = GcodeOpKind.Home };
        public static GcodeOp SetPosition()         => new() { Kind = GcodeOpKind.SetPosition };
    }

    /// <summary>
    /// A modal G-code interpreter for the common command set. Pure — it holds only parsing state
    /// (units, distance mode, feed, plane, work offset, modal position) and turns each line into
    /// zero or more <see cref="GcodeOp"/>. No robot references. Coordinates are base-frame mm.
    ///
    /// Supported: G0 G1 G2 G3 G4 G17 G20 G21 G28 G90 G91 G92 G94 · M0 M1 M2 M3 M4 M5 M30 ·
    /// words X Y Z A F S I J R P N · comments ';…' and '(…)'. A maps to the RZ rotary axis.
    /// </summary>
    public sealed class GcodeInterpreter
    {
        private readonly GcodeOptions _opt;

        private double _scale = 1.0;      // mm per unit (1 for G21, 25.4 for G20)
        private bool   _absolute = true;  // G90 (true) / G91 (false)
        private int    _motionMode = 0;   // active modal motion: 0,1,2,3
        private double _feedMmPerMin = 0; // 0 = never set → use DefaultFeed
        private int    _plane = 17;       // G17 XY (only plane supported for arcs)

        // Modal machine position (mm) and the G92 work offset (machine = work + offset).
        private double _mx, _my, _mz, _ma;
        private double _ox, _oy, _oz, _oa;

        public GcodeInterpreter(GcodeOptions? options = null, (double x, double y, double z, double a)? start = null)
        {
            _opt = options ?? new GcodeOptions();
            if (start is { } s) { _mx = s.x; _my = s.y; _mz = s.z; _ma = s.a; }
        }

        /// <summary>Interpret one line. Returns the ops it produces (possibly empty).</summary>
        public IReadOnlyList<GcodeOp> Feed(string rawLine)
        {
            var ops = new List<GcodeOp>();
            var words = Tokenize(rawLine);
            if (words.Count == 0) return ops;

            // Collect the coordinate/param words for this block.
            double? xw = null, yw = null, zw = null, aw = null, iw = null, jw = null, rw = null, pw = null, sw = null, fw = null;
            var gCodes = new List<int>();
            var mCodes = new List<int>();

            foreach (var (letter, value) in words)
            {
                switch (letter)
                {
                    case 'G': gCodes.Add((int)Math.Round(value)); break;
                    case 'M': mCodes.Add((int)Math.Round(value)); break;
                    case 'X': xw = value; break;
                    case 'Y': yw = value; break;
                    case 'Z': zw = value; break;
                    case 'A': aw = value; break;
                    case 'I': iw = value; break;
                    case 'J': jw = value; break;
                    case 'R': rw = value; break;
                    case 'P': pw = value; break;
                    case 'S': sw = value; break;
                    case 'F': fw = value; break;
                    case 'N': break; // line number — ignore
                    default:
                        throw new GcodeException($"Unsupported word '{letter}{Num(value)}'");
                }
            }

            // ── Modal settings (order-independent within a block) ──
            foreach (var g in gCodes)
            {
                switch (g)
                {
                    case 20: _scale = 25.4; break;   // inch
                    case 21: _scale = 1.0;  break;   // mm
                    case 90: _absolute = true;  break;
                    case 91: _absolute = false; break;
                    case 17: _plane = 17; break;
                    case 94: break;                  // feed per minute — the only mode we use
                    case 0: case 1: case 2: case 3: _motionMode = g; break;
                    case 4:   break;   // handled below (needs P)
                    case 28:  break;   // handled below
                    case 92:  break;   // handled below
                    default: throw new GcodeException($"Unsupported G-code 'G{g}'");
                }
            }
            if (fw is { } f) _feedMmPerMin = f * _scale;

            // ── G4 dwell ──
            if (gCodes.Contains(4))
            {
                double sec = pw ?? 0; // P in seconds (GRBL convention)
                ops.Add(GcodeOp.Dwell(Math.Max(0, sec) * 1000.0));
            }

            // ── G92 set position (define current work coordinates) ──
            if (gCodes.Contains(92))
            {
                // offset chosen so the current machine point reads as the given work coords.
                if (xw is { } vx) _ox = _mx - vx * _scale;
                if (yw is { } vy) _oy = _my - vy * _scale;
                if (zw is { } vz) _oz = _mz - vz * _scale;
                if (aw is { } va) _oa = _ma - va * _scale;
                ops.Add(GcodeOp.SetPosition());
                // G92 is not a motion block; if X/Y/Z were only present for the offset, stop here.
                xw = yw = zw = aw = null;
            }

            // ── G28 home ──
            if (gCodes.Contains(28)) ops.Add(GcodeOp.Home());

            // ── Spindle / program-flow M-codes ──
            foreach (var m in mCodes)
            {
                switch (m)
                {
                    case 3: case 4: ops.Add(GcodeOp.Spindle(true, sw)); break;
                    case 5:         ops.Add(GcodeOp.Spindle(false, sw)); break;
                    case 0: case 1: ops.Add(GcodeOp.Pause()); break;
                    case 2: case 30: ops.Add(GcodeOp.End()); break;
                    default: throw new GcodeException($"Unsupported M-code 'M{m}'");
                }
            }

            // ── Motion ──
            bool hasCoords = xw.HasValue || yw.HasValue || zw.HasValue || aw.HasValue;
            if (hasCoords && (_motionMode is 2 or 3))
            {
                EmitArc(ops, _motionMode == 2, xw, yw, zw, iw, jw, rw);
            }
            else if (hasCoords)
            {
                EmitLinear(ops, xw, yw, zw, aw);
            }

            return ops;
        }

        // ── Linear / rapid ──
        private void EmitLinear(List<GcodeOp> ops, double? xw, double? yw, double? zw, double? aw)
        {
            bool rapid = _motionMode == 0;
            double? fx = xw, fy = yw, fz = zw, fa = aw;
            if (_absolute)
            {
                // Absolute base-frame target (units + G92 offset applied); update modal pos.
                if (xw is { } vx) { fx = vx * _scale + _ox; _mx = fx.Value; }
                if (yw is { } vy) { fy = vy * _scale + _oy; _my = fy.Value; }
                if (zw is { } vz) { fz = vz * _scale + _oz; _mz = fz.Value; }
                if (aw is { } va) { fa = va * _scale + _oa; _ma = fa.Value; }
            }
            else
            {
                // Relative deltas; advance modal pos by them.
                if (xw is { } vx) { fx = vx * _scale; _mx += fx.Value; }
                if (yw is { } vy) { fy = vy * _scale; _my += fy.Value; }
                if (zw is { } vz) { fz = vz * _scale; _mz += fz.Value; }
                if (aw is { } va) { fa = va * _scale; _ma += fa.Value; }
            }
            ops.Add(new GcodeOp
            {
                Kind = GcodeOpKind.Move, Rapid = rapid, Relative = !_absolute,
                X = fx, Y = fy, Z = fz, A = fa,
                FeedMmPerSec = rapid ? null : EffectiveFeed(),
            });
        }

        private double EffectiveFeed() =>
            _feedMmPerMin > 0 ? _feedMmPerMin / 60.0 : _opt.DefaultFeedMmPerSec;

        // ── Arc (G2 CW / G3 CCW), XY plane, flattened to line segments ──
        private void EmitArc(List<GcodeOp> ops, bool clockwise, double? xw, double? yw, double? zw,
            double? iw, double? jw, double? rw)
        {
            if (_plane != 17) throw new GcodeException("Arcs are only supported in the G17 (XY) plane");
            if (!_absolute)   throw new GcodeException("Arcs require absolute mode (G90)");

            double sx = _mx, sy = _my, sz = _mz;
            double ex = xw is { } vx ? vx * _scale + _ox : sx;
            double ey = yw is { } vy ? vy * _scale + _oy : sy;
            double ez = zw is { } vz ? vz * _scale + _oz : sz;

            double cx, cy;
            if (rw is { } r)
            {
                // Radius form: find the center on the correct side of the chord.
                double rr = r * _scale;
                double dx = ex - sx, dy = ey - sy;
                double d = Math.Sqrt(dx * dx + dy * dy);
                if (d < 1e-9) throw new GcodeException("Arc with R needs distinct start/end points");
                double h2 = rr * rr - (d * d) / 4.0;
                if (h2 < -1e-6) throw new GcodeException("Arc radius too small for the endpoints");
                double h = Math.Sqrt(Math.Max(0, h2));
                double mx = (sx + ex) / 2.0, my = (sy + ey) / 2.0;
                // Perpendicular; sign picks minor/major arc per G-code R-sign convention.
                double ux = -dy / d, uy = dx / d;
                double sign = (rr < 0 ? -1 : 1) * (clockwise ? -1 : 1);
                cx = mx + sign * h * ux;
                cy = my + sign * h * uy;
            }
            else
            {
                // Center form (I/J are offsets from the start point).
                cx = sx + (iw ?? 0) * _scale;
                cy = sy + (jw ?? 0) * _scale;
            }

            double startAng = Math.Atan2(sy - cy, sx - cx);
            double endAng   = Math.Atan2(ey - cy, ex - cx);
            double radius   = Math.Sqrt((sx - cx) * (sx - cx) + (sy - cy) * (sy - cy));

            double sweep = endAng - startAng;
            if (clockwise) { if (sweep >= 0) sweep -= 2 * Math.PI; }
            else           { if (sweep <= 0) sweep += 2 * Math.PI; }
            if (Math.Abs(sweep) < 1e-9) sweep = clockwise ? -2 * Math.PI : 2 * Math.PI; // full circle

            // Segment count from chord tolerance: max angle per segment given the radius.
            double tol = Math.Max(1e-4, _opt.ArcToleranceMm);
            double maxAng = radius > tol ? 2 * Math.Acos(Math.Max(-1, 1 - tol / radius)) : Math.PI;
            int segs = Math.Max(1, (int)Math.Ceiling(Math.Abs(sweep) / Math.Max(1e-3, maxAng)));

            double feed = EffectiveFeed();
            for (int i = 1; i <= segs; i++)
            {
                double t = (double)i / segs;
                double ang = startAng + sweep * t;
                double px = cx + radius * Math.Cos(ang);
                double py = cy + radius * Math.Sin(ang);
                double pz = sz + (ez - sz) * t;
                ops.Add(new GcodeOp
                {
                    Kind = GcodeOpKind.Move, Rapid = false, Relative = false,
                    X = px, Y = py, Z = pz, FeedMmPerSec = feed, ArcSegment = true,
                });
            }
            _mx = ex; _my = ey; _mz = ez;
        }

        // ── Tokenizer ──
        // Splits a line into (letter, value) words after stripping comments. '(...)' inline
        // comments and ';...' trailing comments are removed; leading '%' (program markers) ignored.
        private static List<(char Letter, double Value)> Tokenize(string line)
        {
            var result = new List<(char, double)>();
            if (string.IsNullOrWhiteSpace(line)) return result;

            // Strip '(...)' comments.
            var sb = new System.Text.StringBuilder(line.Length);
            int depth = 0;
            foreach (char c in line)
            {
                if (c == '(') { depth++; continue; }
                if (c == ')') { if (depth > 0) depth--; continue; }
                if (depth == 0) sb.Append(c);
            }
            var s = sb.ToString();
            int semi = s.IndexOf(';');
            if (semi >= 0) s = s.Substring(0, semi);
            s = s.Trim();
            if (s.Length == 0 || s[0] == '%') return result;

            int i = 0;
            while (i < s.Length)
            {
                char c = s[i];
                if (char.IsWhiteSpace(c)) { i++; continue; }
                char letter = char.ToUpperInvariant(c);
                if (letter < 'A' || letter > 'Z')
                    throw new GcodeException($"Unexpected character '{c}'");
                i++;
                int start = i;
                if (i < s.Length && (s[i] == '+' || s[i] == '-')) i++;
                while (i < s.Length && (char.IsDigit(s[i]) || s[i] == '.')) i++;
                var numStr = s.Substring(start, i - start);
                if (numStr.Length == 0 || numStr == "+" || numStr == "-" || numStr == ".")
                    throw new GcodeException($"Missing number after '{letter}'");
                if (!double.TryParse(numStr, NumberStyles.Float, CultureInfo.InvariantCulture, out double val))
                    throw new GcodeException($"Bad number '{letter}{numStr}'");
                result.Add((letter, val));
            }
            return result;
        }

        private static string Num(double v) => v.ToString(CultureInfo.InvariantCulture);
    }
}
