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

        // Single reusable stopwatch for control-loop phase timing. The control loop
        // is single-threaded (one thread calls Loop()), so a shared instance is safe.
        public static readonly Stopwatch LoopSw = new();

        private static long _stepStartTs;
        private static long _auxStartTs;

        public static void Log(string msg) => Console.WriteLine($"[diag] {msg}");

        private static double MsSince(long ts) =>
            ts == 0 ? 0 : (Stopwatch.GetTimestamp() - ts) * 1000.0 / Stopwatch.Frequency;

        // ── Program step trace ────────────────────────────────────────────────
        public static void StepStart(int index, object stepType)
        {
            _stepStartTs = Stopwatch.GetTimestamp();
            Console.WriteLine($"[diag] step start idx={index} {stepType}");
        }

        public static void StepDone(object stepType) =>
            Console.WriteLine($"[diag] step done  {stepType} ({MsSince(_stepStartTs):F1}ms since last start)");

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
