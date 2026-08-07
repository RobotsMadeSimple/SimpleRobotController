using System;
using System.Diagnostics;

namespace Controller.RobotControl
{
    /// <summary>
    /// Opt-in diagnostic instrumentation for the control-loop / program-execution
    /// pipeline. Everything writes to stdout (captured by journald) with a `[diag]`
    /// prefix so it can be grepped and correlated with journald timestamps.
    ///
    /// DISABLED by default — when <see cref="Enabled"/> is false the methods return
    /// immediately, and callers guard the hot paths (which build interpolated
    /// strings) with `if (Diag.Enabled)` so there is zero allocation or timing cost
    /// in normal operation. Turn it on for a debugging session by setting the
    /// environment variable RMS_DIAG=1 before launching, or flip Diag.Enabled at
    /// runtime.
    /// </summary>
    internal static class Diag
    {
        /// <summary>Master switch. Off unless RMS_DIAG=1 (or set at runtime).</summary>
        public static volatile bool Enabled =
            Environment.GetEnvironmentVariable("RMS_DIAG") is "1" or "true" or "TRUE";

        // Log a control-loop tick only when it runs longer than this. Normal
        // unthrottled ticks are sub-millisecond; anything above this is a stall.
        public const double SlowTickMs = 15.0;

        // Log a single ExecuteStep call only when it blocks the program thread
        // longer than this.
        public const double SlowStepMs = 40.0;

        // Single reusable stopwatch for motion-loop phase timing (one thread only).
        public static readonly Stopwatch LoopSw = new();

        private static long _stepStartTs;
        private static long _auxStartTs;
        private static int    _lastIdx = int.MinValue;
        private static string? _lastType;

        public static void Log(string msg)
        {
            if (!Enabled) return;
            Console.WriteLine($"[diag] {msg}");
        }

        // Pure timing helpers — no side effects, safe to call unguarded.
        private static double MsSince(long ts) =>
            ts == 0 ? 0 : (Stopwatch.GetTimestamp() - ts) * 1000.0 / Stopwatch.Frequency;
        public static long Now() => Stopwatch.GetTimestamp();
        public static double MsBetween(long startTs) =>
            (Stopwatch.GetTimestamp() - startTs) * 1000.0 / Stopwatch.Frequency;

        // ── Continuous heartbeat ──────────────────────────────────────────────
        // Tracks the worst tick and GC cadence and emits one line every ~5s, so
        // steady-state smoothness and gen-2 GC frequency are visible even when
        // nothing crosses the stall threshold.
        private static double _hbMaxTickMs;
        private static long   _hbTs;
        private static int    _hbGc0, _hbGc2;
        private static string _hbMotion = "";

        public static void Tick(double tickMs) => Tick(tickMs, "");

        public static void Tick(double tickMs, string motionState)
        {
            if (!Enabled) return;
            if (tickMs > _hbMaxTickMs) _hbMaxTickMs = tickMs;
            _hbMotion = motionState;
            if (_hbTs == 0) { _hbTs = Stopwatch.GetTimestamp(); _hbGc0 = GC.CollectionCount(0); _hbGc2 = GC.CollectionCount(2); return; }
            if (MsBetween(_hbTs) < 5000) return;

            int g0 = GC.CollectionCount(0), g2 = GC.CollectionCount(2);
            Console.WriteLine($"[diag] hb maxTick={_hbMaxTickMs:F1}ms/5s | gen0GCs={g0 - _hbGc0} gen2GCs={g2 - _hbGc2} | {_hbMotion}");
            _hbMaxTickMs = 0; _hbTs = Stopwatch.GetTimestamp(); _hbGc0 = g0; _hbGc2 = g2;
        }

        // ── Program step trace ────────────────────────────────────────────────
        // Logs only on transition INTO a step; a polling step (e.g. Wait) re-enters
        // ExecuteStep every program tick and would otherwise flood the log.
        public static void StepStart(int index, object stepType)
        {
            if (!Enabled) return;
            string t = stepType.ToString() ?? "";
            if (index == _lastIdx && t == _lastType) return;
            _lastIdx = index; _lastType = t;
            _stepStartTs = Stopwatch.GetTimestamp();
            Console.WriteLine($"[diag] step start idx={index} {t}");
        }

        public static void StepDone(object stepType)
        {
            if (!Enabled) return;
            Console.WriteLine($"[diag] step done  {stepType} ({MsSince(_stepStartTs):F1}ms since last start)");
            _lastIdx = int.MinValue; _lastType = null;
        }

        // Times one synchronous ExecuteStep call; logs only if it blocked the
        // program thread past SlowStepMs.
        public static void StepExec(object stepType, int index, long startTs)
        {
            if (!Enabled) return;
            double ms = MsBetween(startTs);
            if (ms > SlowStepMs)
                Console.WriteLine($"[diag] SLOW step exec {stepType} idx={index} {ms:F1}ms");
        }

        // ── Aux move dispatch → wait-complete ─────────────────────────────────
        public static void AuxDispatch(bool isAuxMovingNow)
        {
            if (!Enabled) return;
            _auxStartTs = Stopwatch.GetTimestamp();
            Console.WriteLine($"[diag] aux dispatch — IsAuxMoving now={isAuxMovingNow}");
        }

        public static void AuxWaitDone()
        {
            if (!Enabled) return;
            Console.WriteLine($"[diag] aux wait done ({MsSince(_auxStartTs):F0}ms)");
        }
    }
}
