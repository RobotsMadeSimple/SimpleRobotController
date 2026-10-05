using Controller.RobotControl;
using System.Text.Json;

namespace RobotControl.Tests;

/// <summary>
/// Wire behaviour of list variables. ObjectRecord is a Dictionary subclass rather
/// than a plain model, so the two things that can break silently are whether a record
/// survives the round-trip at all and whether it keeps its case-insensitive comparer —
/// lose the latter and <c>$cells[0].Coverage</c> starts reading 0 instead of the value,
/// which looks like a vision problem rather than a serialization one.
///
/// The <c>objects</c> tests here cover the legacy shape, which is still read so that
/// programs saved before the list types were unified keep working. <c>items</c> is the
/// current shape.
/// </summary>
public class ObjectVariableTests
{
    private static readonly JsonSerializerOptions CaseInsensitive = new() { PropertyNameCaseInsensitive = true };

    [Fact]
    public void ObjectListVariableRoundTrips()
    {
        const string json = """
        {
          "id": "v4", "name": "cells", "value": 0,
          "objects": [
            { "row": 0, "col": 1, "index": 1, "coverage": 42.5, "passed": 1 },
            { "row": 1, "col": 0, "index": 2, "coverage": 0,    "passed": 0 }
          ]
        }
        """;

        var v = JsonSerializer.Deserialize<ProgramVariable>(json, CaseInsensitive)!;

        Assert.NotNull(v.Objects);
        Assert.Equal(2, v.Objects!.Count);
        Assert.Equal(42.5, v.Objects[0]["coverage"]);
        Assert.Equal(0, v.Objects[1]["passed"]);

        // Back out again — this is the direction that drops fields when a model is stale.
        var back = JsonSerializer.Deserialize<ProgramVariable>(
            JsonSerializer.Serialize(v), CaseInsensitive)!;

        Assert.Equal(2, back.Objects!.Count);
        Assert.Equal(42.5, back.Objects[0]["coverage"]);
    }

    [Fact]
    public void DeserializedRecordsKeepTheirCaseInsensitiveComparer()
    {
        var v = JsonSerializer.Deserialize<ProgramVariable>(
            """{ "id": "v", "name": "cells", "objects": [ { "coverage": 42.5 } ] }""",
            CaseInsensitive)!;

        Assert.Equal(42.5, v.Objects![0]["Coverage"]);
        Assert.Equal(42.5, v.Objects[0]["COVERAGE"]);
    }

    [Fact]
    public void AVariableWithNoObjectsStaysNull()
    {
        // Null is what distinguishes "not an object list" from "an empty one", and the
        // app keys the variable's type off exactly that.
        var v = JsonSerializer.Deserialize<ProgramVariable>(
            """{ "id": "v", "name": "count", "value": 3 }""", CaseInsensitive)!;

        Assert.Null(v.Objects);
    }

    [Fact]
    public void UnifiedItemsRoundTripWithTheirElementType()
    {
        const string json = """
        {
          "id": "v5", "name": "pts", "elementType": "Point",
          "items": [ { "x": 1, "y": 2, "z": 3, "rx": 4, "ry": 5, "rz": 6 } ]
        }
        """;

        var v = JsonSerializer.Deserialize<ProgramVariable>(json, CaseInsensitive)!;
        Assert.Equal(ListElementType.Point, v.ElementType);
        Assert.Equal(2, v.Items![0]["y"]);

        // Element type is a string on the wire, not the enum's ordinal — a numeric
        // encoding here would silently reshuffle if the enum ever gains a member.
        string back = JsonSerializer.Serialize(v);
        Assert.Contains("\"elementType\":\"Point\"", back);

        var round = JsonSerializer.Deserialize<ProgramVariable>(back, CaseInsensitive)!;
        Assert.Equal(ListElementType.Point, round.ElementType);
        Assert.Equal(6, round.Items![0]["rz"]);
    }

    [Fact]
    public void AnEmptyItemsArrayIsStillAList()
    {
        // The app no longer seeds a new list with a throwaway element, so this is what a
        // freshly created list variable looks like on the wire. It has to stay a list:
        // absent Items means scalar, but present-and-empty means a list with nothing in
        // it yet — which is the normal state for one that gets filled in at runtime.
        var v = JsonSerializer.Deserialize<ProgramVariable>(
            """{ "id": "v", "name": "flags", "elementType": "Boolean", "items": [] }""",
            CaseInsensitive)!;

        var lv = v.ToListVar();
        Assert.NotNull(lv);
        Assert.Empty(lv!.Items);
        Assert.Equal(ListElementType.Boolean, lv.ElementType);

        // The element type has to survive a round trip through the empty list, or the
        // variable would come back as a Record list and stop answering $flags[0].
        var back = JsonSerializer.Deserialize<ProgramVariable>(
            JsonSerializer.Serialize(v), CaseInsensitive)!;
        Assert.Equal(ListElementType.Boolean, back.ToListVar()!.ElementType);
    }

    [Fact]
    public void ItemRecordsKeepTheirCaseInsensitiveComparer()
    {
        var v = JsonSerializer.Deserialize<ProgramVariable>(
            """{ "id": "v", "name": "cells", "items": [ { "coverage": 42.5 } ] }""",
            CaseInsensitive)!;

        Assert.Equal(42.5, v.Items![0]["COVERAGE"]);
    }

    [Fact]
    public void NumberItemsUseTheReservedScalarKey()
    {
        var lv = ListVar.OfNumbers([5, 7]);
        var v  = new ProgramVariable { Name = "vals", Items = lv.Items, ElementType = lv.ElementType };

        // A number list is stored as records too, so it lands on the wire as
        // [{"value":5}, ...] rather than [5, ...]. Bulkier, but it means one shape
        // for every element type and no polymorphic converter to get wrong.
        string json = JsonSerializer.Serialize(v);
        Assert.Contains("\"items\":[{\"value\":5},{\"value\":7}]", json);

        var back = JsonSerializer.Deserialize<ProgramVariable>(json, CaseInsensitive)!;
        Assert.Equal(7, back.ToListVar()!.Items[1].Scalar);
    }

    [Fact]
    public void BooleanItemsShareTheNumberStorageAndDifferOnlyByElementType()
    {
        var lv = ListVar.OfBooleans([true, false]);
        var v  = new ProgramVariable { Name = "flags", Items = lv.Items, ElementType = lv.ElementType };

        // A boolean is a number on the wire — the element type is the only thing telling
        // it apart from a Number list, which is why it has to survive the round-trip.
        string json = JsonSerializer.Serialize(v);
        Assert.Contains("\"items\":[{\"value\":1},{\"value\":0}]", json);
        Assert.Contains("\"elementType\":\"Boolean\"", json);

        var back = JsonSerializer.Deserialize<ProgramVariable>(json, CaseInsensitive)!.ToListVar()!;
        Assert.Equal(ListElementType.Boolean, back.ElementType);
        Assert.Equal(1, back.Items[0].Scalar);
        Assert.Equal(0, back.Items[1].Scalar);
    }

    [Fact]
    public void BooleanAndNumberListsBothHaveScalarElements()
    {
        // The one behavioural rule the two share, and the one Point/Record must not join:
        // an element that is itself a value, so "$v[0]" needs no accessor.
        Assert.True(ListVar.OfBooleans([true]).HasScalarElements);
        Assert.True(ListVar.OfNumbers([1]).HasScalarElements);
        Assert.False(ListVar.OfPoints([new Vector6Val()]).HasScalarElements);
        Assert.False(ListVar.OfRecords([new ObjectRecord()]).HasScalarElements);
    }

    [Fact]
    public void GridColorOutputsSurviveDeserialization()
    {
        var step = JsonSerializer.Deserialize<ProgramStep>("""
        {
          "id": "s1", "type": "RunVision",
          "colorOutputs": [ { "inspectionId": "i1", "coverageVar": "cov", "passedVar": "ok",
                              "cellsVar": "cells", "cellsPassedVar": "nOk" } ]
        }
        """, CaseInsensitive)!;

        var output = Assert.Single(step.ColorOutputs!);
        Assert.Equal("cells", output.CellsVar);
        Assert.Equal("nOk", output.CellsPassedVar);
    }
}
