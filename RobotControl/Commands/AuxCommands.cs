using System.Text.Json;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Serial;

namespace Controller.RobotControl.Commands;

/// <summary>Auxiliary stepper axes (conveyors, rotary tables, …).</summary>
internal sealed class AuxCommands
{
    private readonly RobotController _robot;

    public AuxCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetAuxState",      GetAuxState);
        d.Add("GetAuxConfig",     GetAuxConfig);
        d.Add("MoveAux",          MoveAux);
        d.Add("JogAux",           JogAux);
        d.Add("StopAux",          StopAux);
        d.Add("SetAuxAxisConfig", SetAuxAxisConfig);
        d.Add("SetAuxPosition",   SetAuxPosition);
        d.Add("EnableAux",        EnableAux);
        d.Add("SiloAction",       SiloAction);
    }

    private object? GetAuxState(CommandMessage msg)
    {
        var auxStates = _robot.AuxAxisManager.GetState();
        var auxJson   = JsonSerializer.Serialize(auxStates, CommandJson.CamelCase);
        return new { state = auxJson };
    }

    private object? GetAuxConfig(CommandMessage msg)
    {
        var auxCfg     = _robot.AuxAxisManager.GetConfig();
        var auxCfgJson = JsonSerializer.Serialize(auxCfg, CommandJson.CamelCase);
        return new { config = auxCfgJson };
    }

    /// <summary>Refuse a move on a board whose firmware reports no stepper capability (sanity check —
    /// every AuxStepperDriver build reports it; a legacy board is never refused).</summary>
    private void RequireStepper(string deviceId) =>
        _robot.AuxAxisManager.GetDevice(deviceId)?.RequireCapability(FirmwareInfo.CapStepper, "Stepper motion");

    private void MoveAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<MoveAuxParams>(msg);
        RequireStepper(p.DeviceId);
        _robot.StartAuxMove(p.DeviceId, p.Axis, p.Steps, p.Velocity, p.Accel, p.Decel);
    }

    private void JogAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<JogAuxParams>(msg);
        if (p.Velocity == 0)
            _robot.StopAux(p.Decel);
        else
        {
            RequireStepper(p.DeviceId);
            _robot.StartAuxContinuous(p.DeviceId, p.Axis, p.Velocity, p.Accel);
        }
    }

    private void StopAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StopAuxParams>(msg);
        // Stop a specific axis when one is named (e.g. a Conveyor Stop button); otherwise fall
        // back to stopping the last-active aux axis.
        if (p.Axis.HasValue && !string.IsNullOrEmpty(p.DeviceId))
        {
            if (p.Immediate) _robot.AuxAxisManager.StopAll(p.DeviceId);
            else             _robot.AuxAxisManager.StopSmooth(p.DeviceId, p.Axis.Value, (int)System.Math.Max(1, p.Decel));
        }
        else
        {
            _robot.StopAux(p.Decel, p.Immediate);
        }
    }

    private void SetAuxAxisConfig(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetAuxAxisConfigParams>(msg);
        var cur = _robot.AuxAxisManager.GetAxisConfig(p.DeviceId, p.AxisIndex);
        _robot.AuxAxisManager.UpdateAxisConfig(p.DeviceId, p.AxisIndex, new AuxAxisChannelConfig
        {
            AxisIndex       = p.AxisIndex,
            Name            = p.Name,
            StepsPerRev     = p.StepsPerRev,
            InvertDirection = p.InvertDirection,
            AxisType        = p.AxisType,
            GearRatio       = p.GearRatio,
            MmPerRev        = p.MmPerRev,
            Role            = p.Role,
            ActionDistance  = p.ActionDistance,
            ActionVelocity  = p.ActionVelocity,
            GripperOpenMm   = p.GripperOpenMm,
            GripperClosedMm = p.GripperClosedMm,
            GripperOpenTimeSec  = p.GripperOpenTimeSec,
            GripperCloseTimeSec = p.GripperCloseTimeSec,
            GripperAccel        = p.GripperAccel,
            GripperOverTravelMm = p.GripperOverTravelMm,
            // Left out = keep (a client that predates the Silo role never wipes it).
            SiloPitchMm          = p.SiloPitchMm          ?? cur?.SiloPitchMm          ?? 0,
            SiloCapacity         = p.SiloCapacity         ?? cur?.SiloCapacity         ?? 0,
            SiloClearanceMm      = p.SiloClearanceMm      ?? cur?.SiloClearanceMm      ?? 0,
            SiloHomeOverTravelMm = p.SiloHomeOverTravelMm ?? cur?.SiloHomeOverTravelMm ?? 0,
            SiloHomeSpeed        = p.SiloHomeSpeed        ?? cur?.SiloHomeSpeed        ?? 0,
        });
    }

    /// <summary>Longest the background finish of a SiloAction waits for its lift move.</summary>
    internal static TimeSpan SiloMoveTimeout = TimeSpan.FromSeconds(300);

    /// <summary>
    /// SiloAction (docs/aux-silo.md): Home / Present / Refill / SetCount on a Silo-role axis.
    /// Acks AT ONCE, like MoveAux / Home — a socket handles its commands one at a time, so waiting
    /// for the lift would hold back a Stop sent meanwhile. The ack carries { siloCount, siloHomed,
    /// moving }; when moving, the finish (home zero + homed, or Present's count − 1) runs once the
    /// axis stops, and GetAuxState's siloCount / siloHomed show it. A move stopped part-way is not
    /// finished (the silo stays un-homed / the count unchanged). Refusals answer
    /// { ok:false, error, message } with error siloNotHomed / siloEmpty / siloConfig.
    /// </summary>
    private object? SiloAction(CommandMessage msg)
    {
        var p   = CommandJson.LoadParams<SiloActionParams>(msg);
        string act = Execution.SiloActions.Normalize(p.Action) ?? "";
        if (act != Execution.SiloActions.SetCount) RequireStepper(p.DeviceId);

        Execution.SiloPending? pending;
        try
        {
            pending = Execution.SiloActions.Begin(_robot, p.DeviceId, p.Axis, p.Action, p.Count);
        }
        catch (Execution.SiloException ex)
        {
            return new { ok = false, error = ex.Code, message = ex.Message };
        }

        if (pending != null) _ = FinishSiloAsync(pending);
        return SiloStatus(p.DeviceId, p.Axis, moving: pending != null);
    }

    /// <summary>Waits for the silo's lift move to stop, then runs the action's finish.</summary>
    internal Task FinishSiloAsync(Execution.SiloPending pending) => Task.Run(async () =>
    {
        try
        {
            var deadline = DateTime.UtcNow + SiloMoveTimeout;
            while (_robot.AuxAxisManager.IsDeviceMoving(pending.DeviceId))
            {
                if (DateTime.UtcNow > deadline) return;
                await Task.Delay(20);
            }
            if (!Execution.SiloActions.Complete(_robot, pending))
                Console.WriteLine($"[Aux] Silo {pending.Action} on {pending.DeviceId} axis {pending.Axis} was stopped before it finished");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[Aux] Silo {pending.Action} finish failed: {ex.Message}");
        }
    });
    private object SiloStatus(string deviceId, int axis, bool moving) => new
    {
        siloCount = _robot.AuxAxisManager.GetAxisConfig(deviceId, axis)?.SiloCount ?? 0,
        siloHomed = _robot.AuxAxisManager.IsSiloHomed(deviceId, axis),
        moving,
    };

    private void SetAuxPosition(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetAuxPositionParams>(msg);
        _robot.AuxAxisManager.SetPosition(p.DeviceId, p.Axis, p.Steps);
    }

    private void EnableAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<EnableAuxParams>(msg);
        _robot.AuxAxisManager.Enable(p.DeviceId, p.Enable);
    }
}
