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
        _robot.StopAux(p.Decel, p.Immediate);
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
        });
    }

    private void EnableAux(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<EnableAuxParams>(msg);
        _robot.AuxAxisManager.Enable(p.DeviceId, p.Enable);
    }
}
