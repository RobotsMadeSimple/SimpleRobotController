using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Io;

namespace Controller.RobotControl;

/// <summary>
/// In-place upgrades of saved program steps whose block no longer exists in the editor.
/// Runs over every stored program at startup and over every program an app saves, so a
/// program authored before a block was removed keeps working and opens in the editor as
/// its modern equivalent. Steps that cannot be converted are left untouched — their legacy
/// handler still runs them — and reported through the notes.
/// </summary>
public static class ProgramMigrations
{
    /// <summary>Looks a step's load cell up by device id + pin: the nano name and pin name an
    /// expression reads it by, or null when the pin is not a named load cell on a named nano.</summary>
    public delegate (string Nano, string Pin)? LoadCellResolver(string deviceId, int pin);

    /// <summary>A resolver over a snapshot of the configured IO devices.</summary>
    public static LoadCellResolver Resolver(IEnumerable<IoDeviceState> devices)
    {
        var list = devices.ToList();
        return (deviceId, pin) =>
        {
            var dev = list.FirstOrDefault(d => d.Id == deviceId);
            if (dev == null || string.IsNullOrEmpty(dev.Name)) return null;
            var p = dev.Pins.FirstOrDefault(x => x.Pin == pin && x.Type == PinType.LoadCell);
            return p == null || string.IsNullOrEmpty(p.Name) ? null : (dev.Name, p.Name);
        };
    }

    /// <summary>
    /// Upgrades every legacy step in <paramref name="steps"/> (recursing into loop, if / else-if /
    /// else and CNC bodies). Returns how many steps changed; <paramref name="notes"/> receives one
    /// line per step that was left as is or lost something in the conversion.
    /// </summary>
    public static int MigrateSteps(List<ProgramStep>? steps, LoadCellResolver resolveLoadCell, List<string> notes)
    {
        if (steps == null) return 0;
        int changed = 0;
        foreach (var s in steps)
        {
            if (s.Type == StepType.MoveWithForce && ForceMoveToMoveL(s, resolveLoadCell, notes)) changed++;
            changed += MigrateSteps(s.LoopSteps, resolveLoadCell, notes);
            changed += MigrateSteps(s.IfSteps, resolveLoadCell, notes);
            changed += MigrateSteps(s.ElseSteps, resolveLoadCell, notes);
            changed += MigrateSteps(s.CncProgramSteps, resolveLoadCell, notes);
            if (s.ElseIfBranches != null)
                foreach (var b in s.ElseIfBranches)
                    changed += MigrateSteps(b.Steps, resolveLoadCell, notes);
        }
        return changed;
    }

    /// <summary>
    /// "Move with Force" (press along an axis until a load cell reads a target, then hold) becomes
    /// the move it was shorthand for: a MoveL from the current position, offset along that axis by
    /// the old max travel at the old approach speed, with a stop condition
    /// <c>$io.&lt;nano&gt;.&lt;cell&gt; &gt;= &lt;grams&gt;</c>. The id, name and comment stay. The hold
    /// phase (keep correcting position to maintain the force) has no equivalent and is dropped —
    /// noted when it was in use. Needs a named load cell on a named nano and a positive target;
    /// otherwise the step is left as a legacy MoveWithForce, which the executor still runs.
    /// </summary>
    public static bool ForceMoveToMoveL(ProgramStep s, LoadCellResolver resolveLoadCell, List<string> notes)
    {
        if (s.Type != StepType.MoveWithForce) return false;
        string label = string.IsNullOrWhiteSpace(s.Name) ? "Move with Force" : s.Name!;
        double target = s.ForceTargetGrams ?? 0;
        var cell = !string.IsNullOrEmpty(s.LoadCellDeviceId) && s.LoadCellPin is int pin
            ? resolveLoadCell(s.LoadCellDeviceId, pin) : null;
        if (cell == null || target <= 0)
        {
            notes.Add($"'{label}' kept as Move with Force: " + (target <= 0
                ? "it has no target force"
                : "its load cell is not a named load cell on a named nano, so no expression can read it"));
            return false;
        }

        string axis = (s.ForceAxis ?? "Z").ToUpperInvariant();
        int dir = Math.Sign(s.ForceDirection ?? -1); if (dir == 0) dir = -1;
        double travel = dir * (s.ForceMaxTravelMm ?? 10);
        int holdMs = s.ForceHoldMs ?? 0;

        s.Type      = StepType.MoveL;
        s.PointName = null;   // from the current position: the offset is a relative move
        s.PointId   = null;
        s.Speed     = s.ForceApproachSpeed ?? 5;
        switch (axis)
        {
            case "X":  s.OffsetX  = travel; break;
            case "Y":  s.OffsetY  = travel; break;
            case "RX": s.OffsetRX = travel; break;
            case "RY": s.OffsetRY = travel; break;
            case "RZ": s.OffsetRZ = travel; break;
            default:   s.OffsetZ  = travel; break;
        }
        var (nano, pinName) = cell.Value;
        string symbol = IoVariableNames.Alias(nano, pinName) ?? IoVariableNames.Key(nano, pinName);
        s.MoveStopCondition = new ConditionGroup
        {
            Combinator = "ALL",
            Items = { new ConditionItem { Id = Guid.NewGuid().ToString("N")[..8],
                                          Expr = $"${symbol} >= {target.ToString(CultureInfo.InvariantCulture)}" } },
        };
        s.MoveStopMode = "stop";

        s.ForceAxis = null; s.ForceDirection = null; s.ForceTargetGrams = null; s.ForceToleranceGrams = null;
        s.ForceMaxTravelMm = null; s.ForceApproachSpeed = null; s.ForceHoldMs = null; s.ForceTimeoutMs = null;
        s.LoadCellDeviceId = null; s.LoadCellPin = null; s.LoadCellUnit = null;

        if (holdMs > 0)
            notes.Add($"'{label}' converted to a MoveL with a stop condition; its {holdMs} ms force hold was dropped (a stop-on-condition move stops and does not keep correcting)");
        return true;
    }
}
