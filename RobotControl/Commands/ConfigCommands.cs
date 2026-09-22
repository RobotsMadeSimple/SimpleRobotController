using System.Text.Json;

namespace Controller.RobotControl.Commands;

/// <summary>Robot configuration, speed override and joint-limit bypass.</summary>
internal sealed class ConfigCommands
{
    private readonly RobotController _robot;

    public ConfigCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetRobotConfig",   GetRobotConfig);
        d.Add("SetRobotConfig",   SetRobotConfig);
        d.Add("SetSpeedOverride", SetSpeedOverride);
        d.Add("SetLimitBypass",   SetLimitBypass);
    }

    private object? GetRobotConfig(CommandMessage msg)
    {
        var c = _robot.Config;
        return new
        {
            robotType                 = c.RobotType,
            homingSpeed               = c.HomingSpeed,
            homingSlowSpeed           = c.HomingSlowSpeed,
            homingBackoffMm           = c.HomingBackoffMm,
            j1HomeOffsetDeg           = c.J1HomeOffsetDeg,
            verticalHomePosition      = c.VerticalHomePosition,
            horizontalHomePosition    = c.HorizontalHomePosition,
            verticalHomingDirection   = c.VerticalHomingDirection,
            horizontalHomingDirection = c.HorizontalHomingDirection,
            j1HomingDirection         = c.J1HomingDirection,
            j4HomeOffsetDeg           = c.J4HomeOffsetDeg,
            m1Direction               = c.M1Direction,
            m2Direction               = c.M2Direction,
            m3Direction               = c.M3Direction,
            m4Direction               = c.M4Direction,
            enableNanoCards           = c.EnableNanoCards,
            enableRelayCard           = c.EnableRelayCard,
            enableAuxAxis             = c.EnableAuxAxis,
            enableCameras             = c.EnableCameras,
            jogSlowSpeed              = c.JogSlowSpeed,
            jogNormalSpeed            = c.JogNormalSpeed,
            jogFastSpeed              = c.JogFastSpeed,
            cncStepsPerRevX           = c.CncStepsPerRevX,
            cncStepsPerRevY           = c.CncStepsPerRevY,
            cncStepsPerRevZ           = c.CncStepsPerRevZ,
            cncStepsPerRevRZ          = c.CncStepsPerRevRZ,
            cncMmPerRevX              = c.CncMmPerRevX,
            cncMmPerRevY              = c.CncMmPerRevY,
            cncMmPerRevZ              = c.CncMmPerRevZ,
            cncDegPerRevRZ            = c.CncDegPerRevRZ,
            cncXHomePosition          = c.CncXHomePosition,
            cncYHomePosition          = c.CncYHomePosition,
            cncZHomePosition          = c.CncZHomePosition,
            cncRzHomePosition         = c.CncRzHomePosition,
            cncXHomingDirection       = c.CncXHomingDirection,
            cncYHomingDirection       = c.CncYHomingDirection,
            cncZHomingDirection       = c.CncZHomingDirection,
            jointLimitsEnabled        = c.JointLimitsEnabled,
            joint1Min                 = c.Joint1Min,
            joint1Max                 = c.Joint1Max,
            joint2Min                 = c.Joint2Min,
            joint2Max                 = c.Joint2Max,
            joint3Min                 = c.Joint3Min,
            joint3Max                 = c.Joint3Max,
            joint4Min                 = c.Joint4Min,
            joint4Max                 = c.Joint4Max,
        };
    }

    private object? SetRobotConfig(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetRobotConfigParams>(msg);
        var c = _robot.Config;

        // Swapping the kinematics model mid-motion or mid-homing would
        // reinterpret live joint targets under a different model.
        if (p.RobotType != null && p.RobotType != c.RobotType
            && (_robot.IsMoving || _robot.MotionBusy || _robot.HomingRequestedOrActive))
        {
            return new { ok = false, error = "Cannot change robot type while the robot is moving or homing." };
        }

        // Kinematics / motor-direction reinit runs on the motion thread (it
        // swaps the kinematics and reconfigures the STB motors it drives).
        bool motorDirectionsChanged = false;
        bool kinematicsChanged      = false;
        if (p.HomingSpeed.HasValue)              c.HomingSpeed               = p.HomingSpeed.Value;
        if (p.HomingSlowSpeed.HasValue)           c.HomingSlowSpeed           = p.HomingSlowSpeed.Value;
        if (p.HomingBackoffMm.HasValue)           c.HomingBackoffMm           = p.HomingBackoffMm.Value;
        if (p.J1HomeOffsetDeg.HasValue)           c.J1HomeOffsetDeg           = p.J1HomeOffsetDeg.Value;
        if (p.VerticalHomePosition.HasValue)      c.VerticalHomePosition      = p.VerticalHomePosition.Value;
        if (p.HorizontalHomePosition.HasValue)    c.HorizontalHomePosition    = p.HorizontalHomePosition.Value;
        if (p.VerticalHomingDirection.HasValue)   c.VerticalHomingDirection   = p.VerticalHomingDirection.Value;
        if (p.HorizontalHomingDirection.HasValue) c.HorizontalHomingDirection = p.HorizontalHomingDirection.Value;
        if (p.J1HomingDirection.HasValue)         c.J1HomingDirection         = p.J1HomingDirection.Value;
        if (p.J4HomeOffsetDeg.HasValue)           c.J4HomeOffsetDeg           = p.J4HomeOffsetDeg.Value;
        if (p.M1Direction.HasValue)               { c.M1Direction             = p.M1Direction.Value;   motorDirectionsChanged = true; }
        if (p.M2Direction.HasValue)               { c.M2Direction             = p.M2Direction.Value;   motorDirectionsChanged = true; }
        if (p.M3Direction.HasValue)               { c.M3Direction             = p.M3Direction.Value;   motorDirectionsChanged = true; }
        if (p.M4Direction.HasValue)               { c.M4Direction             = p.M4Direction.Value;   motorDirectionsChanged = true; }
        if (p.EnableNanoCards.HasValue)           c.EnableNanoCards           = p.EnableNanoCards.Value;
        if (p.EnableRelayCard.HasValue)           c.EnableRelayCard           = p.EnableRelayCard.Value;
        if (p.EnableAuxAxis.HasValue)             c.EnableAuxAxis             = p.EnableAuxAxis.Value;
        if (p.EnableCameras.HasValue)             c.EnableCameras             = p.EnableCameras.Value;
        if (p.JogSlowSpeed.HasValue)              c.JogSlowSpeed              = p.JogSlowSpeed.Value;
        if (p.JogNormalSpeed.HasValue)            c.JogNormalSpeed            = p.JogNormalSpeed.Value;
        if (p.JogFastSpeed.HasValue)              c.JogFastSpeed              = p.JogFastSpeed.Value;
        if (p.RobotType != null)                  { c.RobotType               = p.RobotType;             kinematicsChanged = true; }
        bool cncMotorConfigChanged = false;
        if (p.CncStepsPerRevX.HasValue)  { c.CncStepsPerRevX  = p.CncStepsPerRevX.Value;  cncMotorConfigChanged = true; }
        if (p.CncStepsPerRevY.HasValue)  { c.CncStepsPerRevY  = p.CncStepsPerRevY.Value;  cncMotorConfigChanged = true; }
        if (p.CncStepsPerRevZ.HasValue)  { c.CncStepsPerRevZ  = p.CncStepsPerRevZ.Value;  cncMotorConfigChanged = true; }
        if (p.CncStepsPerRevRZ.HasValue) { c.CncStepsPerRevRZ = p.CncStepsPerRevRZ.Value; cncMotorConfigChanged = true; }
        if (p.CncMmPerRevX.HasValue)     { c.CncMmPerRevX     = p.CncMmPerRevX.Value;     cncMotorConfigChanged = true; }
        if (p.CncMmPerRevY.HasValue)     { c.CncMmPerRevY     = p.CncMmPerRevY.Value;     cncMotorConfigChanged = true; }
        if (p.CncMmPerRevZ.HasValue)     { c.CncMmPerRevZ     = p.CncMmPerRevZ.Value;     cncMotorConfigChanged = true; }
        if (p.CncDegPerRevRZ.HasValue)   { c.CncDegPerRevRZ   = p.CncDegPerRevRZ.Value;   cncMotorConfigChanged = true; }
        if (cncMotorConfigChanged)        kinematicsChanged = true;
        if (p.CncXHomePosition.HasValue)          c.CncXHomePosition          = p.CncXHomePosition.Value;
        if (p.CncYHomePosition.HasValue)          c.CncYHomePosition          = p.CncYHomePosition.Value;
        if (p.CncZHomePosition.HasValue)          c.CncZHomePosition          = p.CncZHomePosition.Value;
        if (p.CncRzHomePosition.HasValue)         c.CncRzHomePosition         = p.CncRzHomePosition.Value;
        if (p.CncXHomingDirection.HasValue)       c.CncXHomingDirection       = p.CncXHomingDirection.Value;
        if (p.CncYHomingDirection.HasValue)       c.CncYHomingDirection       = p.CncYHomingDirection.Value;
        if (p.CncZHomingDirection.HasValue)       c.CncZHomingDirection       = p.CncZHomingDirection.Value;
        if (p.JointLimitsEnabled.HasValue)        c.JointLimitsEnabled        = p.JointLimitsEnabled.Value;
        // Joint-limit bounds: a property present in the patch is
        // authoritative, INCLUDING an explicit null which clears the
        // bound (so it is no longer enforced). Absent means unchanged —
        // so we read the raw params rather than the HasValue pattern,
        // which cannot tell "sent null" from "not sent".
        if (msg.Params is { } rawCfg)
        {
            void ApplyLimit(string name, Action<double?> set)
            {
                if (rawCfg.TryGetProperty(name, out var el))
                    set(el.ValueKind == JsonValueKind.Null ? (double?)null : el.GetDouble());
            }
            ApplyLimit("joint1Min", v => c.Joint1Min = v);
            ApplyLimit("joint1Max", v => c.Joint1Max = v);
            ApplyLimit("joint2Min", v => c.Joint2Min = v);
            ApplyLimit("joint2Max", v => c.Joint2Max = v);
            ApplyLimit("joint3Min", v => c.Joint3Min = v);
            ApplyLimit("joint3Max", v => c.Joint3Max = v);
            ApplyLimit("joint4Min", v => c.Joint4Min = v);
            ApplyLimit("joint4Max", v => c.Joint4Max = v);
        }
        if (motorDirectionsChanged || kinematicsChanged)
            _robot.ReapplyConfigOnMotionThread(motorDirectionsChanged, kinematicsChanged);
        RobotConfigService.Save(c);
        return null;
    }

    private void SetSpeedOverride(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetSpeedOverrideParams>(msg);
        _robot.SpeedOverrideFactor = Math.Clamp(p.Percent / 100.0,
            RobotController.MinSpeedOverrideFactor, RobotController.MaxSpeedOverrideFactor);
    }

    private void SetLimitBypass(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetLimitBypassParams>(msg);
        _robot.SetLimitBypass(p.Enable);
    }
}
