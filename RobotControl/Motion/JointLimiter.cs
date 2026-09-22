using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

/// <summary>
/// Pure joint soft-limit clamping. Operates on the four joint-space components
/// of the target vector — index 0 = X (ASTRO J1 / CNC X), 1 = Y (ASTRO radial /
/// CNC Y), 2 = Z (ASTRO vertical / CNC Z), 3 = RZ (ASTRO J4 / CNC RZ).
///
/// The rule is "never move a joint further outside its window than it already
/// is": each component is clamped to <c>[min(lo, before), max(hi, before)]</c>.
/// If <c>before</c> is inside the window this blocks any crossing outright; if a
/// joint is already outside (limits changed, or homed out of range) it still
/// permits corrective motion back toward the window. A clamp that actually
/// changed the commanded value is reported as a violation, along with which
/// joint and the offending direction (+1 past max, -1 past min).
/// </summary>
public static class JointLimiter
{
    public readonly struct Result
    {
        public readonly bool Violated;
        public readonly int  Joint;      // 0..3, or -1
        public readonly int  Direction;  // +1 past max, -1 past min, 0 none
        public readonly Vector6 Clamped;
        public Result(bool violated, int joint, int direction, Vector6 clamped)
        {
            Violated = violated; Joint = joint; Direction = direction; Clamped = clamped;
        }
    }

    private static double Comp(Vector6 v, int i) => i switch { 0 => v.X, 1 => v.Y, 2 => v.Z, _ => v.RZ };
    private static void   Set (Vector6 v, int i, double x)
    {
        switch (i) { case 0: v.X = x; break; case 1: v.Y = x; break; case 2: v.Z = x; break; default: v.RZ = x; break; }
    }

    /// <param name="lims">Four (lo, hi) windows for joints 0..3.</param>
    public static Result Clamp(Vector6 target, Vector6 before, (double lo, double hi)[] lims)
    {
        var outv = new Vector6(target.X, target.Y, target.Z, target.RX, target.RY, target.RZ);
        bool violated = false; int joint = -1, dir = 0;

        for (int i = 0; i < 4 && i < lims.Length; i++)
        {
            double v  = Comp(target, i);
            double b  = Comp(before, i);
            double lo = Math.Min(lims[i].lo, lims[i].hi);
            double hi = Math.Max(lims[i].lo, lims[i].hi);

            double hiCap = Math.Max(hi, b);
            double loCap = Math.Min(lo, b);
            double c = Math.Clamp(v, loCap, hiCap);

            if (Math.Abs(c - v) > 1e-9 && !violated)
            {
                violated = true;
                joint = i;
                dir = v > hiCap ? 1 : -1;
            }
            Set(outv, i, c);
        }

        return new Result(violated, joint, dir, outv);
    }
}

