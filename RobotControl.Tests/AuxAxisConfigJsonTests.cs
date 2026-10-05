using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Persistence;

/// <summary>
/// aux_config.json on real robots stores an unconfigured axis as "axisType": "".
/// The file is read with the shared JsonDefaults.File options, whose
/// JsonStringEnumConverter outranks a type-level [JsonConverter]; the property-level
/// converter must win so "" still loads (a regression here quarantined a live config).
/// </summary>
public class AuxAxisConfigJsonTests
{
    private const string Sample = """
        {
          "devices": [
            {
              "id": "AUX_STEPPER_001",
              "name": "Aux Stepper",
              "axes": [
                { "axisIndex": 0, "name": "X", "stepsPerRev": 1600, "invertDirection": false, "axisType": "",       "gearRatio": 1, "mmPerRev": 0 },
                { "axisIndex": 1, "name": "Y", "stepsPerRev": 1600, "invertDirection": false, "axisType": "Linear", "gearRatio": 1, "mmPerRev": 5 },
                { "axisIndex": 2, "name": "Z", "stepsPerRev": 1600, "invertDirection": false, "axisType": "Rotary", "gearRatio": 2, "mmPerRev": 0 }
              ]
            }
          ]
        }
        """;

    [Fact]
    public void EmptyStringAxisTypeLoadsAsUnconfigured_WithSharedFileOptions()
    {
        var cfg = JsonSerializer.Deserialize<AuxAxisManagerConfig>(Sample, JsonDefaults.File)!;
        var axes = cfg.Devices[0].Axes;
        Assert.Equal(AuxAxisType.Unconfigured, axes[0].AxisType);
        Assert.Equal(AuxAxisType.Linear,       axes[1].AxisType);
        Assert.Equal(AuxAxisType.Rotary,       axes[2].AxisType);
    }

    [Fact]
    public void AxisTypeRoundTripsToTheSameStrings()
    {
        var cfg  = JsonSerializer.Deserialize<AuxAxisManagerConfig>(Sample, JsonDefaults.File)!;
        var json = JsonSerializer.Serialize(cfg, JsonDefaults.File);
        Assert.Contains("\"axisType\": \"\"", json);
        Assert.Contains("\"axisType\": \"Linear\"", json);
        Assert.Contains("\"axisType\": \"Rotary\"", json);
    }

    [Fact]
    public void StateAndParamsUseTheSameEncoding()
    {
        var state = JsonSerializer.Deserialize<AuxAxisChannelState>("""{ "axisIndex": 0, "axisType": "" }""", JsonDefaults.File)!;
        Assert.Equal(AuxAxisType.Unconfigured, state.AxisType);

        var p = JsonSerializer.Deserialize<SetAuxAxisConfigParams>("""{ "deviceId": "d", "axisIndex": 0, "axisType": "Linear" }""", JsonDefaults.File)!;
        Assert.Equal(AuxAxisType.Linear, p.AxisType);
    }
}
