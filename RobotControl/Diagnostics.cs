using System;
using System.Diagnostics;

namespace Controller.RobotControl
{
    /// <summary>
    /// Temporary diagnostic instrumentation for the program-execution / control-loop
    /// stutter investigation. Everything here writes to stdout (captured by
    /// journald) with a `[diag]` prefix so it can be grepped and correlated with
    /// journald's own timestamps. Cheap on the hot path: timing uses
    /// Stopwatch.GetTimestamp() (allocation-free) and cycle logging only fires when
    /// a tick exceeds SlowTickMs. Remove before merging the fix to main.
    /// </summary>
    internal static class Diag
    {
        // Log a control-loop tick only when it runs longer than this. Normal
        // unthrottled ticks are sub-millisecond; anything above this is a stall.
        public const double SlowTickMs = 15.0;

        // Log a single ExecuteStep call only when it blocks the shared control-loop
        // thread longer than this (which stalls RunMotion in the same tick).
        public const double SlowStepMs = 40.0;

        // Single reusable stopwatch for control-loop phase timing. The control loop
        // is single-threaded (one thread calls Loop()), so a shared instance is safe.
        public static readonly Stopwatch LoopSw = new();

        private static long _stepStartTs;
        private static long _auxStartTs;
        private static int    _lastIdx = int.MinValue;
        private static string? _lastType;

        public static void Log(string msg) => Console.WriteLine($"[diag] {msg}");

        private static double MsSince(long ts) =>
            ts == 0 ? 0 : (Stopwatch.GetTimestamp() - ts) * 1000.0 / Stopwatch.Frequency;

        public static long Now() => Stopwatch.GetTimestamp();
        public static double MsBetween(long startTs) =>
            (Stopwatch.GetTimestamp() - startTs) * 1000.0 / Stopwatch.Frequency;

        // ── Continuous heartbeat ──────────────────────────────────────────────
        // Called every control-loop tick. Tracks the worst tick and GC cadence and
        // emits one line every ~30s, so steady-state smoothness and gen-2 GC
        // frequency are visible even when nothing crosses the stall threshold.
        private static double _hbMaxTickMs;
        private static long   _hbTs;
        private static int    _hbGc0, _hbGc2;

        public static void Tick(double tickMs)
        {
            if (tickMs > _hbMaxTickMs) _hbMaxTickMs = tickMs;
            if (_hbTs == 0) { _hbTs = Stopwatch.GetTimestamp(); _hbGc0 = GC.CollectionCount(0); _hbGc2 = GC.CollectionCount(2); return; }
            if (MsBetween(_hbTs) < 30000) return;

            int g0 = GC.CollectionCount(0), g2 = GC.CollectionCount(2);
            Console.WriteLine($"[diag] hb maxTick={_hbMaxTickMs:F1}ms/30s | gen0GCs={g0 - _hbGc0} gen2GCs={g2 - _hbGc2}");
            _hbMaxTickMs = 0; _hbTs = Stopwatch.GetTimestamp(); _hbGc0 = g0; _hbGc2 = g2;
        }

        // ── Program step trace ────────────────────────────────────────────────
        // Log only on transition INTO a step. A polling step (e.g. Wait) re-enters
        // ExecuteStep every unthrottled tick; without this it floods the log.
        public static void StepStart(int index, object stepType)
        {
            string t = stepType.ToString() ?? "";
            if (index == _lastIdx && t == _lastType) return;
            _lastIdx = index; _lastType = t;
            _stepStartTs = Stopwatch.GetTimestamp();
            Console.WriteLine($"[diag] step start idx={index} {t}");
        }

        public static void StepDone(object stepType)
        {
            Console.WriteLine($"[diag] step done  {stepType} ({MsSince(_stepStartTs):F1}ms since last start)");
            _lastIdx = int.MinValue; _lastType = null; // let the next step log even if same idx
        }

        // Times one synchronous ExecuteStep call. Logs only if it blocked the shared
        // control-loop thread past SlowStepMs — i.e. froze RunMotion in the same tick.
        public static void StepExec(object stepType, int index, long startTs)
        {
            double ms = MsBetween(startTs);
            if (ms > SlowStepMs)
                Console.WriteLine($"[diag] SLOW step exec {stepType} idx={index} {ms:F1}ms — blocked the control loop (motion frozen this long)");
        }

        // ── Aux move dispatch → wait-complete ─────────────────────────────────
        public static void AuxDispatch(bool isAuxMovingNow)
        {
            _auxStartTs = Stopwatch.GetTimestamp();
            Console.WriteLine($"[diag] aux dispatch — IsAuxMoving now={isAuxMovingNow}");
        }

        public static void AuxWaitDone() =>
            Console.WriteLine($"[diag] aux wait done ({MsSince(_auxStartTs):F0}ms)");
    }
}
