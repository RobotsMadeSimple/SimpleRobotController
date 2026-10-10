using System.Text.Json;
using Controller.RobotControl.AuxAxis;

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

    private void MoveAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<MoveAuxParams>(msg);
        _robot.StartAuxMove(p.DeviceId, p.Axis, p.Steps, p.Velocity, p.Accel, p.Decel);
    }

    private void JogAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<JogAuxParams>(msg);
        if (p.Velocity == 0)
            _robot.StopAux(p.Decel);
        else
            _robot.StartAuxContinuous(p.DeviceId, p.Axis, p.Velocity, p.Accel);
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
        });
    }

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
