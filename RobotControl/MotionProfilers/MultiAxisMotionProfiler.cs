using System;
using System.Diagnostics;

namespace Controller.RobotControl.MotionProfilers
{
    /// <summary>
    /// Simple synchronized multi-axis motion profiler (X/Y/Z only).
    /// Each axis runs its own AxisMotionProfiler, but all are time-scaled
    /// so they finish at the same time (syncTime).
    /// </summary>
    public class MultiAxisMotionProfiler
    {
        private readonly AxisMotionProfiler x = new();
        private readonly AxisMotionProfiler y = new();
        private readonly AxisMotionProfiler z = new();

        private readonly Stopwatch sw = new();
        private double lastTime;
        private double syncTime;

        private Vector6 startPose;
        private Vector6 targetPose;

        public Vector6 CurrentPose =>
            new(x.Position, y.Position, z.Position, 0, 0, 0);

        public Vector6 CurrentVelocity =>
            new(x.Velocity, y.Velocity, z.Velocity, 0, 0, 0);

        public bool IsFinished => sw.Elapsed.TotalSeconds >= syncTime;

        // -------------------------------------------------
        // Setup
        // -------------------------------------------------
        public MultiAxisMotionProfiler(
            Vector6 current,
            Vector6 target,
            Vector6 maxAxisSpeeds,
            Vector6 accel,
            Vector6 decel
        )
        {
            startPose = current;
            targetPose = target;

            // We’re doing a fresh plan from rest on each axis.
            // If you want to honor existing velocity, pass it in here instead.
            Vector6 startVel = new Vector6(0, 0, 0, 0, 0, 0);

            // Clamp inputs
            double ax = Math.Max(Math.Abs(accel.X), 1e-9);
            double ay = Math.Max(Math.Abs(accel.Y), 1e-9);
            double az = Math.Max(Math.Abs(accel.Z), 1e-9);

            double dx = Math.Max(Math.Abs(decel.X), 1e-9);
            double dy = Math.Max(Math.Abs(decel.Y), 1e-9);
            double dz = Math.Max(Math.Abs(decel.Z), 1e-9);

            double vxMax = Math.Max(Math.Abs(maxAxisSpeeds.X), 1e-9);
            double vyMax = Math.Max(Math.Abs(maxAxisSpeeds.Y), 1e-9);
            double vzMax = Math.Max(Math.Abs(maxAxisSpeeds.Z), 1e-9);

            // -------------------------------------------------
            // 1) Compute each axis’s minimum achievable time
            //    given its own limits.
            // -------------------------------------------------
            double Tx = AxisMinTime(current.X, startVel.X, target.X, ax, dx, vxMax);
            double Ty = AxisMinTime(current.Y, startVel.Y, target.Y, ay, dy, vyMax);
            double Tz = AxisMinTime(current.Z, startVel.Z, target.Z, az, dz, vzMax);

            syncTime = Math.Max(Tx, Math.Max(Ty, Tz));

            // Degenerate: no motion
            if (syncTime <= 1e-9)
            {
                syncTime = 0.0;
                x.Setup(current.X, 0, target.X, ax, dx, 0, 0);
                y.Setup(current.Y, 0, target.Y, ay, dy, 0, 0);
                z.Setup(current.Z, 0, target.Z, az, dz, 0, 0);
                x.ForceTarget();
                y.ForceTarget();
                z.ForceTarget();
                sw.Reset();
                lastTime = 0;
                return;
            }

            // -------------------------------------------------
            // 2) Time-scale each axis to finish in syncTime.
            //    We do this by choosing an effective vmax per axis
            //    that makes its min time == syncTime.
            //
            //    We keep accel/decel unchanged (per-axis), and search
            //    for the needed vmax within [0, vmaxLimit].
            // -------------------------------------------------
            double vxEff = SolveVmaxForTime(current.X, startVel.X, target.X, ax, dx, vxMax, syncTime);
            double vyEff = SolveVmaxForTime(current.Y, startVel.Y, target.Y, ay, dy, vyMax, syncTime);
            double vzEff = SolveVmaxForTime(current.Z, startVel.Z, target.Z, az, dz, vzMax, syncTime);

            // Apply direction signs based on motion direction
            vxEff *= Math.Sign(target.X - current.X);
            vyEff *= Math.Sign(target.Y - current.Y);
            vzEff *= Math.Sign(target.Z - current.Z);

            // -------------------------------------------------
            // 3) Build per-axis profiles with shared syncTime
            // -------------------------------------------------
            x.Setup(current.X, startVel.X, target.X, ax, dx, vxEff, syncTime);
            y.Setup(current.Y, startVel.Y, target.Y, ay, dy, vyEff, syncTime);
            z.Setup(current.Z, startVel.Z, target.Z, az, dz, vzEff, syncTime);

            sw.Restart();
            lastTime = 0;
        }

        // -------------------------------------------------
        // Update
        // -------------------------------------------------
        public Vector6 Update()
        {
            if (syncTime <= 1e-9)
                return CurrentPose;

            double t = sw.Elapsed.TotalSeconds;
            double dt = t - lastTime;
            lastTime = t;

            if (t >= syncTime)
            {
                x.ForceTarget();
                y.ForceTarget();
                z.ForceTarget();
                return CurrentPose;
            }

            x.Update(dt);
            y.Update(dt);
            z.Update(dt);

            return CurrentPose;
        }

        // -------------------------------------------------
        // Helpers
        // -------------------------------------------------

        /// <summary>
        /// Minimum time for a single axis move with trapezoid/triangle profile,
        /// given limits accel, decel, vmax and initial velocity v0.
        /// </summary>
        private static double AxisMinTime(
            double pos,
            double vel,
            double target,
            double accel,
            double decel,
            double vmax
        )
        {
            accel = Math.Max(accel, 1e-9);
            decel = Math.Max(decel, 1e-9);
            vmax = Math.Max(vmax, 1e-9);

            double dist = Math.Abs(target - pos);
            double v0 = Math.Abs(vel);

            if (dist <= 1e-12)
                return 0.0;

            // Distance to accelerate from v0 to vmax
            double da = (vmax * vmax - v0 * v0) / (2 * accel);
            da = Math.Max(da, 0.0);

            // Distance to decelerate from vmax to 0
            double dd = vmax * vmax / (2 * decel);

            if (da + dd >= dist)
            {
                // Triangular: peak velocity vp < vmax
                double vp = Math.Sqrt(
                    (2 * dist * accel * decel + v0 * v0 * decel) /
                    (accel + decel)
                );

                return (vp - v0) / accel + vp / decel;
            }
            else
            {
                // Trapezoidal: hit vmax, cruise
                double dc = dist - da - dd;

                return
                    (vmax - v0) / accel +
                    dc / vmax +
                    vmax / decel;
            }
        }

        /// <summary>
        /// Find an effective vmax (<= vmaxLimit) such that the axis profile duration == targetTime.
        /// Uses monotonic bisection on vmax because AxisMinTime decreases as vmax increases.
        /// </summary>
        private static double SolveVmaxForTime(
            double pos,
            double vel,
            double target,
            double accel,
            double decel,
            double vmaxLimit,
            double targetTime
        )
        {
            double dist = Math.Abs(target - pos);
            if (dist <= 1e-12)
                return 0.0;

            // If even at vmaxLimit we are slower than targetTime, we can't meet it (shouldn't happen
            // because syncTime is max of min times), so just return vmaxLimit.
            double tAtLimit = AxisMinTime(pos, vel, target, accel, decel, vmaxLimit);
            if (tAtLimit >= targetTime - 1e-12)
                return vmaxLimit;

            // We need to SLOW DOWN to take longer -> choose smaller vmax
            double lo = 0.0;
            double hi = vmaxLimit;

            // Bisection
            for (int i = 0; i < 60; i++)
            {
                double mid = 0.5 * (lo + hi);
                double tMid = AxisMinTime(pos, vel, target, accel, decel, Math.Max(mid, 1e-12));

                // If mid makes it too slow (time too big), we can increase vmax
                if (tMid > targetTime)
                    lo = mid;
                else
                    hi = mid;

                if (Math.Abs(tMid - targetTime) < 1e-9)
                    break;
            }

            return hi;
        }
    }
}
