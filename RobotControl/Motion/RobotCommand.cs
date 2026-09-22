using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

public class RobotCommand
{
    public string? CommandType { get; set; }

    /// <summary>
    /// Server-set jog epoch, stamped when the command is enqueued. A StopJog bumps
    /// the controller's jog generation; a queued jog whose stamp is stale (a stop
    /// arrived after it was enqueued) is dropped instead of re-starting the jog —
    /// this prevents a trailing jog command from re-enabling motion right after a
    /// release. JsonIgnore so app payloads can't spoof it.
    /// </summary>
    [JsonIgnore]
    public int JogGeneration { get; set; }

    [JsonPropertyName("name")]
    public string? Name { get; set; }

    public double? X { get; set; }
    public double? Y { get; set; }
    public double? Z { get; set; }
    public double? RX { get; set; }
    public double? RY { get; set; }
    public double? RZ { get; set; }
    public double? TX { get; set; }
    public double? TY { get; set; }
    public double? TZ { get; set; }
    public double? TRX { get; set; }
    public double? TRY { get; set; }
    public double? TRZ { get; set; }


    public double? Speed { get; set; }
    public double? Accel { get; set; }
    public double? Decel { get; set; }
    public double? Time { get; set; }

    /// <summary>
    /// Set by the program executor so the global speed override scales program
    /// moves only. Manual moves (points page) and jogging leave it false and run
    /// at their commanded speed. JsonIgnore-d so app payloads can't set it.
    /// </summary>
    [JsonIgnore]
    public bool ApplySpeedOverride { get; set; } = false;

    /// <summary>
    /// Optional status update attached to this command.
    /// When the command is dequeued and starts executing the update is applied
    /// to the named program so the mobile app sees live progress.
    /// </summary>
    [JsonPropertyName("statusUpdate")]
    public ProgramCycleUpdate? StatusUpdate { get; set; }

    /// <summary>
    /// Waypoints for a "StartContinuous" (blended) path. Carried on the command so
    /// the blended move is started on the motion thread inside RunCommands, rather
    /// than the program-executor thread mutating motion state directly.
    /// </summary>
    [JsonIgnore] public List<Vector6>? Waypoints  { get; set; }
    [JsonIgnore] public List<double>?  BlendRadii { get; set; }

    public Vector6 Vector6 => new(X ?? 0, Y ?? 0, Z ?? 0, RX ?? 0, RY ?? 0, RZ ?? 0);
    public Vector6 ToolOffsetVector6 => new(TX ?? 0, TY ?? 0, TZ ?? 0, TRX ?? 0, TRY ?? 0, TRZ ?? 0);
}



