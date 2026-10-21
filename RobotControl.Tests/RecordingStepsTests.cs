using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>ExportData's CSV/JSON building from program variables (StartRecording/StopRecording
/// fill a record list; ExportData serializes it). Driven straight off a VariableScope.</summary>
public sealed class RecordingStepsTests
{
    private static VariableScope ScopeWithForceData()
    {
        var scope = new VariableScope();
        scope.SetList("data", ListVar.OfRecords(new[]
        {
            new ObjectRecord { ["t"] = 1000, ["v"] = 5,   ["r"] = 100 },
            new ObjectRecord { ["t"] = 1001, ["v"] = 6.5, ["r"] = 101 },
            new ObjectRecord { ["t"] = 1002, ["v"] = 4,   ["r"] = 99  },
        }));
        return scope;
    }

    [Fact]
    public void Csv_of_a_single_record_list_uses_field_names_as_headers()
    {
        var cols = new List<JsonKeyValue> { new() { Key = "data", ListVar = "data" } };
        var lines = RecordingSteps.BuildCsv(cols, ScopeWithForceData()).Split('\n');

        Assert.Equal("t,v,r", lines[0]);     // single record list → bare field names
        Assert.Equal(4, lines.Length);        // header + 3 rows
        Assert.Equal("1000,5,100", lines[1]);
        Assert.Equal("1001,6.5,101", lines[2]);
    }

    [Fact]
    public void Csv_with_multiple_sources_prefixes_list_columns_and_repeats_scalars()
    {
        var cols = new List<JsonKeyValue>
        {
            new() { Key = "speed", Expr = "42" },
            new() { Key = "data",  ListVar = "data" },
        };
        var lines = RecordingSteps.BuildCsv(cols, ScopeWithForceData()).Split('\n');

        Assert.Equal("speed,data.t,data.v,data.r", lines[0]); // mixed → prefixed list columns
        Assert.Equal(4, lines.Length);
        Assert.StartsWith("42,1000,5,100", lines[1]);
        Assert.StartsWith("42,1001,6.5,101", lines[2]);
    }

    [Fact]
    public void Json_serializes_a_record_list_as_an_array_of_objects()
    {
        var cols = new List<JsonKeyValue>
        {
            new() { Key = "label", Expr = "7" },
            new() { Key = "samples", ListVar = "data" },
        };
        using var doc = JsonDocument.Parse(RecordingSteps.BuildJson(cols, ScopeWithForceData()));
        var root = doc.RootElement;

        Assert.Equal(7, root.GetProperty("label").GetDouble());
        var arr = root.GetProperty("samples");
        Assert.Equal(3, arr.GetArrayLength());
        Assert.Equal(1000, arr[0].GetProperty("t").GetDouble());
        Assert.Equal(6.5, arr[1].GetProperty("v").GetDouble());
    }

    [Fact]
    public void Csv_with_no_list_sources_is_a_single_data_row()
    {
        var cols = new List<JsonKeyValue> { new() { Key = "a", Expr = "1" }, new() { Key = "b", Expr = "2" } };
        var lines = RecordingSteps.BuildCsv(cols, new VariableScope()).Split('\n');

        Assert.Equal("a,b", lines[0]);
        Assert.Equal(2, lines.Length);
        Assert.Equal("1,2", lines[1]);
    }
}
