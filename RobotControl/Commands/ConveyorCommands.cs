using Controller.RobotControl.Conveyors;

namespace Controller.RobotControl.Commands;

/// <summary>Conveyor tracking (docs/conveyor-tracking.md): conveyor config, the part queue, manual
/// travel, and the calibration wizard's sample / calibrate pair.</summary>
internal sealed class ConveyorCommands
{
    private readonly RobotController _robot;

    public ConveyorCommands(RobotController robot) => _robot = robot;

    private ConveyorService Svc => _robot.Conveyors;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetConveyors",       GetConveyors);
        d.Add("SaveConveyor",       SaveConveyor);
        d.Add("DeleteConveyor",     DeleteConveyor);
        d.Add("ConveyorAddPart",    AddPart);
        d.Add("ConveyorClearQueue", ClearQueue);
        d.Add("ConveyorSetTravel",  SetTravel);
        d.Add("ConveyorSample",     Sample);
        d.Add("ConveyorCalibrate",  Calibrate);
    }

    private static object Bad(string error) => new { ok = false, error };

    private object? GetConveyors(CommandMessage msg) => new { conveyors = Svc.GetAll() };

    private object? SaveConveyor(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveConveyorParams>(msg);
        if (p.Conveyor == null) return Bad("conveyor is required");
        try
        {
            return new { conveyor = Svc.Save(p.Conveyor) };
        }
        catch (ArgumentException ex)
        {
            return Bad(ex.Message);
        }
    }

    private object? DeleteConveyor(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConveyorIdParams>(msg);
        return Svc.Delete(p.Id) ? null : Bad($"Conveyor not found: {p.Id}");
    }

    private object? AddPart(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConveyorAddPartParams>(msg);
        if (Svc.Get(p.Id) is not { } c) return Bad($"Conveyor not found: {p.Id}");
        return new { partId = Svc.AddPart(c.Id, p.OffsetMm) };
    }

    private object? ClearQueue(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConveyorIdParams>(msg);
        if (Svc.Get(p.Id) is not { } c) return Bad($"Conveyor not found: {p.Id}");
        Svc.ClearQueue(c.Id);
        return null;
    }

    private object? SetTravel(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConveyorSetTravelParams>(msg);
        if (Svc.Get(p.Id) is not { } c) return Bad($"Conveyor not found: {p.Id}");
        try
        {
            Svc.SetTravel(c.Id, p.TravelMm);
            return null;
        }
        catch (Exception ex) when (ex is InvalidOperationException or ArgumentException)
        {
            return Bad(ex.Message);
        }
    }

    /// <summary>The live TCP and the belt travel now — one calibration sample.</summary>
    private object? Sample(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConveyorIdParams>(msg);
        if (Svc.Get(p.Id) is not { } c) return Bad($"Conveyor not found: {p.Id}");
        var pos = _robot.GetCurrentPosition();
        return new
        {
            x = Math.Round(pos.X, 3), y = Math.Round(pos.Y, 3), z = Math.Round(pos.Z, 3), rz = Math.Round(pos.RZ, 3),
            travelMm = Math.Round(Svc.TravelAt(c.Id, Svc.Now), 3),
        };
    }

    /// <summary>A frame / scale / sensor offset proposal from the samples (not saved).</summary>
    private object? Calibrate(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConveyorCalibrateParams>(msg);
        if (Svc.Get(p.Id) is not { } c) return Bad($"Conveyor not found: {p.Id}");
        try
        {
            var r = ConveyorCalibration.Compute(p.Samples, p.SensorSample, c.Scale);
            return new { frame = r.Frame, scale = r.Scale, sensorOffsetMm = r.SensorOffsetMm, residualMm = r.ResidualMm };
        }
        catch (ArgumentException ex)
        {
            return Bad(ex.Message);
        }
    }
}
