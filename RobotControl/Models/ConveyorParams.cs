using System.Text.Json.Serialization;
using Controller.RobotControl.Conveyors;

namespace Controller.RobotControl
{
    // WebSocket params of the conveyor commands (docs/conveyor-tracking.md, docs/websocket-api.md).

    public sealed class SaveConveyorParams
    {
        /// <summary>The whole conveyor; an empty id creates a new one.</summary>
        [JsonPropertyName("conveyor")] public Conveyor? Conveyor { get; set; }
    }

    public sealed class ConveyorIdParams
    {
        [JsonPropertyName("id")] public string Id { get; set; } = "";
    }

    public sealed class ConveyorAddPartParams
    {
        [JsonPropertyName("id")]       public string Id       { get; set; } = "";
        /// <summary>The part's X in the conveyor frame now (mm).</summary>
        [JsonPropertyName("offsetMm")] public double OffsetMm { get; set; }
    }

    public sealed class ConveyorSetTravelParams
    {
        [JsonPropertyName("id")]       public string Id       { get; set; } = "";
        [JsonPropertyName("travelMm")] public double TravelMm { get; set; }
    }

    public sealed class ConveyorCalibrateParams
    {
        [JsonPropertyName("id")]           public string Id { get; set; } = "";
        /// <summary>Two or more samples of the same belt mark, in the order they were taken.</summary>
        [JsonPropertyName("samples")]      public List<ConveyorCalibrationSample> Samples { get; set; } = new();
        /// <summary>Optional: the TCP on a latched part, with the travel at the latch.</summary>
        [JsonPropertyName("sensorSample")] public ConveyorCalibrationSample? SensorSample { get; set; }
    }
}
