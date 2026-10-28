namespace Controller.RobotControl.Execution
{
    /// <summary>What happens the first time a move step's stop condition is true.</summary>
    internal enum MoveStopMode
    {
        /// <summary>The move ends where it is; the program continues with the next step.</summary>
        Stop,
        /// <summary>The move runs on to its target; only the position is captured.</summary>
        Record,
    }

    /// <summary>
    /// A move step's stop condition while that move is in flight: evaluated every executor tick
    /// until it fires, then completed (flag variable written) when the move ends. One per
    /// <see cref="MotionDispatcher"/> — a robot move and an aux move are never awaited at once.
    /// </summary>
    /// <param name="Step">The move step the watch belongs to.</param>
    /// <param name="Condition">The condition evaluated each tick (fresh IO every tick).</param>
    /// <param name="PositionVar">Points list (robot) / numeric (aux) variable for the position at the first true evaluation.</param>
    /// <param name="FlagVar">Numeric / boolean variable set to 1 / 0 on completion.</param>
    /// <param name="IsAux">An aux-axis move (AuxMove / GripperAction) rather than a robot move.</param>
    /// <param name="AuxDeviceId">Aux moves: the device, for the position read and the smooth stop.</param>
    /// <param name="AuxAxis">Aux moves: the axis index.</param>
    /// <param name="AuxDecelHz">Aux moves: the deceleration (steps/s²) used to stop smoothly.</param>
    internal sealed record MoveWatch(
        ProgramStep    Step,
        ConditionGroup Condition,
        MoveStopMode   Mode,
        string?        PositionVar,
        string?        FlagVar,
        bool           IsAux,
        string?        AuxDeviceId = null,
        int            AuxAxis     = 0,
        int            AuxDecelHz  = 10000)
    {
        /// <summary>True once the condition has been seen true during this move.</summary>
        public bool Fired { get; set; }
    }

    /// <summary>The step-field side of a move stop condition, shared by the handlers and the validator.</summary>
    internal static class MoveStopConditions
    {
        /// <summary>The step carries a condition with at least one row — the feature is on.</summary>
        public static bool HasCondition(ProgramStep step) =>
            step.MoveStopCondition is { Items.Count: > 0 };

        /// <summary>"stop" / "record" (case-insensitive); null or empty reads as stop.</summary>
        public static bool TryParseMode(string? text, out MoveStopMode mode)
        {
            if (string.IsNullOrWhiteSpace(text)) { mode = MoveStopMode.Stop; return true; }
            if (text.Trim().Equals("stop",   StringComparison.OrdinalIgnoreCase)) { mode = MoveStopMode.Stop;   return true; }
            if (text.Trim().Equals("record", StringComparison.OrdinalIgnoreCase)) { mode = MoveStopMode.Record; return true; }
            mode = MoveStopMode.Stop;
            return false;
        }

        /// <summary>The watch for <paramref name="step"/>, or null when it has no condition. An
        /// unknown mode falls back to stop — the validator reports it before a run.</summary>
        public static MoveWatch? Create(ProgramStep step, bool isAux,
            string? auxDeviceId = null, int auxAxis = 0, int auxDecelHz = 10000)
        {
            if (!HasCondition(step)) return null;
            TryParseMode(step.MoveStopMode, out var mode);
            return new MoveWatch(step, step.MoveStopCondition!, mode,
                Trim(step.MoveStopPositionVariableName), Trim(step.MoveStopFlagVariableName),
                isAux, auxDeviceId, auxAxis, auxDecelHz);
        }

        // Variable names arrive as typed ("$pts" or "pts"); the scope wants the bare name.
        private static string? Trim(string? name)
        {
            if (string.IsNullOrWhiteSpace(name)) return null;
            var t = name.Trim().TrimStart('$');
            return t.Length == 0 ? null : t;
        }
    }
}
