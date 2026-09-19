using System.Text.Json;
using Controller.RobotControl;

namespace RobotControl.Tests;

/// <summary>
/// List variables over the HttpRequest block, both directions.
///
/// Before this, every outbound row evaluated to one double and every inbound value was
/// coerced to one double, so a 32-square board had to be sent as 32 separate keys. These
/// tests pin the two halves that replace that: the shape a list takes on the wire, and the
/// rule that the *declared* element type survives whatever the server actually sent.
///
/// That second rule is the one that can break silently. A Boolean list that comes back as
/// a Number list still evaluates, still compares, still passes every condition — it just
/// renders as 1/0 instead of True/False and offers a keypad instead of toggles, which
/// looks like an editor bug rather than a deserialization one.
/// </summary>
public class JsonListTransferTests
{
    private static JsonElement Parse(string json) => JsonDocument.Parse(json).RootElement;

    private static string Serialize(object o) => JsonSerializer.Serialize(o);

    // ── Outbound ─────────────────────────────────────────────────────────────

    [Fact]
    public void BooleanListGoesOutAsJsonBooleans()
    {
        var lv = ListVar.OfBooleans([true, false, true]);

        // Not [1,0,1]: a boolean list reads as True/False everywhere else, and a server
        // written against "white": [true, ...] should not have to accept both.
        Assert.Equal("[true,false,true]", Serialize(ProgramExecutor.ListToJson(lv)));
    }

    [Fact]
    public void NumberListGoesOutAsJsonNumbers()
    {
        var lv = ListVar.OfNumbers([1, 2.5, -3]);
        Assert.Equal("[1,2.5,-3]", Serialize(ProgramExecutor.ListToJson(lv)));
    }

    [Fact]
    public void PointListGoesOutAsObjectsWithNamedAxes()
    {
        var lv = ListVar.OfPoints([new Vector6Val { X = 1, Y = 2, Z = 3, RX = 4, RY = 5, RZ = 6 }]);

        var back = Parse(Serialize(ProgramExecutor.ListToJson(lv)));
        Assert.Equal(JsonValueKind.Array, back.ValueKind);
        var p = back[0];
        Assert.Equal(1, p.GetProperty("x").GetDouble());
        Assert.Equal(6, p.GetProperty("rz").GetDouble());
    }

    [Fact]
    public void AnEmptyListGoesOutAsAnEmptyArray()
    {
        Assert.Equal("[]", Serialize(ProgramExecutor.ListToJson(ListVar.OfBooleans([]))));
    }

    // ── Inbound ──────────────────────────────────────────────────────────────

    [Fact]
    public void InboundBooleansLandInABooleanList()
    {
        var lv = ProgramExecutor.ListFromJson(Parse("[true, false, true]"), ListElementType.Boolean);

        Assert.Equal(ListElementType.Boolean, lv.ElementType);
        Assert.Equal(3, lv.Count);
        Assert.Equal(1, lv.Items[0].Scalar);
        Assert.Equal(0, lv.Items[1].Scalar);
    }

    [Fact]
    public void TheDeclaredElementTypeWinsOverWhatArrived()
    {
        // A server sending 1/0 into a Boolean list must not silently convert it to a
        // Number list — the program was written against True/False.
        var lv = ProgramExecutor.ListFromJson(Parse("[1, 0, 1]"), ListElementType.Boolean);

        Assert.Equal(ListElementType.Boolean, lv.ElementType);
        Assert.Equal(1, lv.Items[0].Scalar);
        Assert.Equal(0, lv.Items[1].Scalar);
    }

    [Fact]
    public void InboundObjectsLandInARecordListByFieldName()
    {
        var lv = ProgramExecutor.ListFromJson(
            Parse("""[{ "from": 10, "to": 17 }, { "from": 17, "to": 24 }]"""),
            ListElementType.Record);

        Assert.Equal(2, lv.Count);
        Assert.Equal(10, lv.Items[0]["from"]);
        Assert.Equal(24, lv.Items[1]["to"]);
    }

    [Fact]
    public void InboundRecordFieldsStayCaseInsensitive()
    {
        // ObjectRecord's comparer is what makes $moves[0].From work when the server sent
        // "from". Building the record field-by-field must not lose it.
        var lv = ProgramExecutor.ListFromJson(
            Parse("""[{ "from": 10 }]"""), ListElementType.Record);

        Assert.Equal(10, lv.Items[0]["FROM"]);
        Assert.Equal(10, lv.Items[0]["From"]);
    }

    [Fact]
    public void TheListIsReplacedWholeSoLengthFollowsTheResponse()
    {
        // $moves.length is how a program knows how many hops came back, so a two-element
        // response into a longer list must shorten it rather than leave stale elements.
        var lv = ProgramExecutor.ListFromJson(Parse("[5, 6]"), ListElementType.Number);
        Assert.Equal(2, lv.Count);

        Assert.Empty(ProgramExecutor.ListFromJson(Parse("[]"), ListElementType.Number).Items);
    }

    [Fact]
    public void AMalformedElementBecomesZeroRatherThanShiftingTheList()
    {
        // Position matters — $remove[2] must still be the third capture even if the
        // second element arrived as junk.
        var lv = ProgramExecutor.ListFromJson(Parse("""[5, "oops", 7]"""), ListElementType.Number);

        Assert.Equal(3, lv.Count);
        Assert.Equal(0, lv.Items[1].Scalar);
        Assert.Equal(7, lv.Items[2].Scalar);
    }

    [Fact]
    public void ANonObjectWhereARecordWasExpectedBecomesAnEmptyRecord()
    {
        var lv = ProgramExecutor.ListFromJson(Parse("[42]"), ListElementType.Record);

        Assert.Single(lv.Items);
        Assert.Empty(lv.Items[0]);
        Assert.Equal(0, lv.Items[0].Scalar); // reads as 0, not a throw
    }

    // ── Scalar coercion, shared by both ──────────────────────────────────────

    [Theory]
    [InlineData("42", 42)]
    [InlineData("-1", -1)]
    [InlineData("true", 1)]
    [InlineData("false", 0)]
    [InlineData("\"12.5\"", 12.5)]
    [InlineData("\"abc\"", 0)]
    [InlineData("null", 0)]
    [InlineData("{}", 0)]
    public void ScalarCoercionIsTheSameEverywhere(string json, double expected)
    {
        Assert.Equal(expected, ProgramExecutor.ScalarFromJson(Parse(json)));
    }

    [Fact]
    public void ANumberTooLargeForADoubleYieldsZeroRatherThanInfinity()
    {
        // Not a hypothetical: a 400-digit number does not fail to parse, it parses as ∞.
        // Left alone that lands in a program variable and silently poisons every
        // expression downstream, still comparing and evaluating like a real value.
        var huge = new string('9', 400);
        Assert.Equal(0, ProgramExecutor.ScalarFromJson(Parse(huge)));
        Assert.Equal(0, ProgramExecutor.ScalarFromJson(Parse("-" + huge)));
        Assert.Equal(0, ProgramExecutor.ScalarFromJson(Parse("\"1e999\"")));
    }

    // ── Round trip ───────────────────────────────────────────────────────────

    [Fact]
    public void ABoardSurvivesTheRoundTrip()
    {
        // The case this was built for: send the board, get it back, compare.
        var board = Enumerable.Range(0, 32).Select(i => i % 3 == 0).ToList();

        var wire = Serialize(ProgramExecutor.ListToJson(ListVar.OfBooleans(board)));
        var back = ProgramExecutor.ListFromJson(Parse(wire), ListElementType.Boolean);

        Assert.Equal(32, back.Count);
        Assert.Equal(board, back.Items.ConvertAll(r => r.Scalar != 0));
    }
}
