using Controller.RobotControl;

namespace RobotControl.Tests;

public class ExpressionEvaluatorTests
{
    // Matches VariableScope.MergedVars() - case-insensitive keys.
    private static Dictionary<string, double> Vars(params (string k, double v)[] entries)
    {
        var d = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        foreach (var (k, v) in entries) d[k] = v;
        return d;
    }

    // Every list kind reaches the evaluator in one ListVar dictionary. They stay separate
    // parameters here because the element type is what decides which index form applies,
    // so a test still has to say which kind it means.
    private static double Eval(string expr,
        Dictionary<string, double>? vars = null,
        Dictionary<string, List<double>>? lists = null,
        Dictionary<string, List<bool>>? bools = null,
        Dictionary<string, List<Vector6Val>>? pts = null,
        Dictionary<string, List<ObjectRecord>>? objs = null)
    {
        Dictionary<string, ListVar>? all = null;
        void Add(string k, ListVar v) => (all ??= new())[k] = v;
        if (lists != null) foreach (var kv in lists) Add(kv.Key, ListVar.OfNumbers(kv.Value));
        if (bools != null) foreach (var kv in bools) Add(kv.Key, ListVar.OfBooleans(kv.Value));
        if (pts   != null) foreach (var kv in pts)   Add(kv.Key, ListVar.OfPoints(kv.Value));
        if (objs  != null) foreach (var kv in objs)  Add(kv.Key, ListVar.OfRecords(kv.Value));
        return ExpressionEvaluator.Evaluate(expr, vars ?? Vars(), all);
    }

    // -- Arithmetic ------------------------------------------------------------

    [Theory]
    [InlineData("42", 42)]
    [InlineData("-5", -5)]
    [InlineData("2 + 3 * 4", 14)]        // precedence
    [InlineData("(2 + 3) * 4", 20)]      // grouping
    [InlineData("10 - 4 - 3", 3)]        // left assoc
    [InlineData("7 / 2", 3.5)]
    [InlineData("5 / 0", 0)]             // div-by-zero -> 0 (intentional)
    [InlineData("true", 1)]
    [InlineData("FALSE", 0)]
    public void Arithmetic(string expr, double expected)
        => Assert.Equal(expected, Eval(expr), 9);

    // -- Scalar variables --------------------------------------------------------

    [Fact]
    public void ScalarVariableResolves()
        => Assert.Equal(12.5, Eval("$x + 2.5", Vars(("x", 10))), 9);

    [Fact]
    public void VariableLookupIsCaseInsensitive()
        => Assert.Equal(10, Eval("$ApproachZ", Vars(("approachz", 10))), 9);

    // A bare name is NOT a variable reference — it tokenizes as a Word and yields 0,
    // so "x + 2.5" is 2.5, not 12.5. Pinned because UI hint text and docs have
    // claimed otherwise; the $ sigil is required.
    [Fact]
    public void BareNameDoesNotResolveAsVariable()
        => Assert.Equal(2.5, Eval("x + 2.5", Vars(("x", 10))), 9);

    // Braces are optional in an expression — the tokenizer skips them. The shared
    // TemplateInput lets users type the braced form in numeric fields too, so this
    // has to keep holding.
    [Theory]
    [InlineData("{$x + 2}")]
    [InlineData("$x + 2")]
    public void BracesAroundAnExpressionAreIgnored(string expr)
        => Assert.Equal(12, Eval(expr, Vars(("x", 10))), 9);

    // -- Lists and points --------------------------------------------------------

    [Fact]
    public void ListIndexAndLength()
    {
        var lists = new Dictionary<string, List<double>> { ["vals"] = [5, 7, 9] };
        Assert.Equal(7, Eval("$vals[1]", lists: lists), 9);
        Assert.Equal(3, Eval("$vals.length", lists: lists), 9);
        Assert.Equal(3, Eval("$vals.count", lists: lists), 9);
        Assert.Equal(0, Eval("$vals[99]", lists: lists), 9);   // out of range -> 0
        Assert.Equal(0, Eval("$vals", lists: lists), 9);       // bare list name -> 0, no throw
    }

    private static Dictionary<string, List<Vector6Val>> Blobs() => new()
    {
        ["blobs"] = [new Vector6Val { X = 1, Y = 2, Z = 3, RX = 4, RY = 5, RZ = 6 }],
    };

    [Fact]
    public void PointComponents()
    {
        var pts = Blobs();
        Assert.Equal(1, Eval("$blobs[0].x", pts: pts), 9);
        Assert.Equal(3, Eval("$blobs[0][2]", pts: pts), 9);
        Assert.Equal(1, Eval("$blobs.length", pts: pts), 9);
        Assert.Equal(0, Eval("$blobs", pts: pts), 9);          // bare points name -> 0, no throw
    }

    // -- Object lists ------------------------------------------------------------

    private static Dictionary<string, List<ObjectRecord>> Cells() => new()
    {
        ["cells"] =
        [
            new ObjectRecord { ["row"] = 0, ["col"] = 0, ["coverage"] = 12.5, ["passed"] = 0 },
            new ObjectRecord { ["row"] = 0, ["col"] = 1, ["coverage"] = 80.0, ["passed"] = 1 },
        ],
    };

    [Fact]
    public void ObjectFieldsAndLength()
    {
        var objs = Cells();
        Assert.Equal(80,   Eval("$cells[1].coverage", objs: objs), 9);
        Assert.Equal(1,    Eval("$cells[1].passed",   objs: objs), 9);
        Assert.Equal(2,    Eval("$cells.length",      objs: objs), 9);
        Assert.Equal(2,    Eval("$cells.count",       objs: objs), 9);
        Assert.Equal(92.5, Eval("$cells[0].coverage + $cells[1].coverage", objs: objs), 9);
    }

    [Fact]
    public void ObjectFieldsAreCaseInsensitive()
        => Assert.Equal(12.5, Eval("$cells[0].Coverage", objs: Cells()), 9);

    [Fact]
    public void ObjectIndexIsAnExpression()
    {
        var objs = Cells();
        Assert.Equal(80, Eval("$cells[$i + 1].coverage", Vars(("i", 0)), objs: objs), 9);
    }

    [Fact]
    public void ObjectMissesAreZeroNotThrows()
    {
        var objs = Cells();
        Assert.Equal(0, Eval("$cells[99].coverage", objs: objs), 9);  // index out of range
        Assert.Equal(0, Eval("$cells[0].nosuch",    objs: objs), 9);  // unknown field
        Assert.Equal(0, Eval("$cells",              objs: objs), 9);  // bare name, no throw
    }

    // -- Element type decides the syntax -------------------------------------------

    // The three list kinds share one storage shape, so these pin the differences that
    // are still real: what an element answers to, and what it refuses.

    [Fact]
    public void NumberElementConsumesNoAccessor()
    {
        // "$vals[0]" is already the value, so a trailing field is not part of the
        // expression — "$vals[0].foo" parses as "$vals[0]" and then a stray word.
        var lists = new Dictionary<string, List<double>> { ["vals"] = [5, 7, 9] };
        Assert.Equal(5, Eval("$vals[0].foo", lists: lists), 9);
        Assert.Equal(9, Eval("$vals[0] + 4", lists: lists), 9);
    }

    [Fact]
    public void PositionalComponentIsPointOnly()
    {
        // [n] needs a defined axis order, which only a point list has.
        Assert.Equal(3, Eval("$blobs[0][2]", pts: Blobs()), 9);
        Assert.Equal(0, Eval("$cells[0][2]", objs: Cells()), 9);
    }

    [Fact]
    public void PointAxesAreAlsoNamedFields()
    {
        // A point is stored as named doubles, so .x and a record's .coverage are the
        // same lookup — and an axis a point does not carry is 0, not a throw.
        Assert.Equal(6, Eval("$blobs[0].rz",    pts: Blobs()), 9);
        Assert.Equal(0, Eval("$blobs[0].nosuch", pts: Blobs()), 9);
    }

    [Fact]
    public void IndexedWithoutAnAccessorIsZero()
    {
        Assert.Equal(0, Eval("$blobs[0]", pts:  Blobs()), 9);
        Assert.Equal(0, Eval("$cells[0]", objs: Cells()), 9);
    }

    // -- Boolean elements ------------------------------------------------------------

    // A boolean list is stored exactly like a number list — 0/1 under the scalar key — so
    // expressions need no special case. These pin that it really does behave like one,
    // because the whole point of the separate element type is that it changes nothing here.

    private static Dictionary<string, List<bool>> Flags() =>
        new() { ["flags"] = [true, false, true] };

    [Fact]
    public void BooleanElementsEvaluateAsOneAndZero()
    {
        Assert.Equal(1, Eval("$flags[0]", bools: Flags()), 9);
        Assert.Equal(0, Eval("$flags[1]", bools: Flags()), 9);
    }

    [Fact]
    public void BooleanElementsAreArithmeticLikeAnyOtherNumber()
    {
        // The evaluator has no comparison operators — a condition evaluates each side and
        // compares them in ConditionGroup. So the 0/1 storage is the whole mechanism by
        // which a boolean list works in a condition: it arrives as a number either way.
        Assert.Equal(2, Eval("$flags[0] + $flags[2]", bools: Flags()), 9);
        Assert.Equal(1, Eval("$flags[0] * 1",         bools: Flags()), 9);
    }

    [Fact]
    public void BooleanElementConsumesNoAccessor()
    {
        // Same rule as a number element: "$flags[0]" is already the value, so a trailing
        // accessor is not part of the expression. Both forms below therefore read the
        // element itself rather than 0 — a point or record would read 0 here instead.
        Assert.Equal(1, Eval("$flags[0].foo", bools: Flags()), 9);
        Assert.Equal(1, Eval("$flags[0][2]",  bools: Flags()), 9);

        // Pinned against a number list so the two cannot drift apart.
        var nums = new Dictionary<string, List<double>> { ["vals"] = [1, 0, 1] };
        Assert.Equal(1, Eval("$vals[0][2]", lists: nums), 9);
    }

    [Fact]
    public void BooleanListCountsAndMissesMatchNumberLists()
    {
        Assert.Equal(3, Eval("$flags.length", bools: Flags()), 9);
        Assert.Equal(3, Eval("$flags.count",  bools: Flags()), 9);
        Assert.Equal(0, Eval("$flags[99]",    bools: Flags()), 9);  // out of range
    }

    // -- Legacy programs still load ------------------------------------------------

    // Programs saved before the list types were unified carry values/points/objects
    // instead of items. ToListVar folds them in, and these pin that the folded form
    // evaluates the same as it did.

    private static ListVar? Legacy(string json)
        => System.Text.Json.JsonSerializer.Deserialize<ProgramVariable>(json)!.ToListVar();

    [Fact]
    public void LegacyNumberListLoadsAsNumberElements()
    {
        var lv = Legacy("""{"name":"vals","values":[5,7,9]}""")!;
        Assert.Equal(ListElementType.Number, lv.ElementType);
        Assert.Equal(3, lv.Count);
        Assert.Equal(7, lv.Items[1].Scalar, 9);
    }

    [Fact]
    public void LegacyPointListLoadsAsPointElements()
    {
        var lv = Legacy("""{"name":"blobs","points":[{"x":1,"y":2,"z":3,"rx":4,"ry":5,"rz":6}]}""")!;
        Assert.Equal(ListElementType.Point, lv.ElementType);
        Assert.Equal(1, lv.Items[0]["x"], 9);
        Assert.Equal(6, lv.Items[0]["rz"], 9);
    }

    [Fact]
    public void LegacyObjectListLoadsAsRecordElements()
    {
        var lv = Legacy("""{"name":"cells","objects":[{"coverage":12.5}]}""")!;
        Assert.Equal(ListElementType.Record, lv.ElementType);
        Assert.Equal(12.5, lv.Items[0]["coverage"], 9);
    }

    // An empty legacy number list was indistinguishable from a scalar and the app
    // treated it as one. Kept deliberately — flipping it would turn saved scalars
    // into lists. Empty points/objects lists were always distinguishable, so they
    // stay lists.
    [Fact]
    public void EmptyLegacyListsKeepTheirOldClassification()
    {
        Assert.Null(Legacy("""{"name":"v","values":[]}"""));
        Assert.Equal(0, Legacy("""{"name":"v","points":[]}""")!.Count);
        Assert.Equal(0, Legacy("""{"name":"v","objects":[]}""")!.Count);
        Assert.Null(Legacy("""{"name":"v","value":3}"""));
    }

    // A program written by a current build and then edited by an older one can carry
    // both shapes. The current field wins, so it does not silently revert.
    [Fact]
    public void ItemsWinOverLegacyFields()
    {
        var lv = Legacy("""
            {"name":"v","elementType":"Number","items":[{"value":42}],"values":[1,2,3]}
            """)!;
        Assert.Equal(ListElementType.Number, lv.ElementType);
        Assert.Single(lv.Items);
        Assert.Equal(42, lv.Items[0].Scalar, 9);
    }

    // Absent elementType reads as Record — the open-ended shape, which cannot
    // misinterpret fields the way assuming Point or Number would.
    [Fact]
    public void ItemsWithoutElementTypeAreRecords()
    {
        var lv = Legacy("""{"name":"v","items":[{"coverage":5}]}""")!;
        Assert.Equal(ListElementType.Record, lv.ElementType);
    }

    [Fact]
    public void LegacyListsEvaluateThroughTheEvaluator()
    {
        var lists = new Dictionary<string, ListVar>
        {
            ["vals"]  = Legacy("""{"name":"vals","values":[5,7,9]}""")!,
            ["blobs"] = Legacy("""{"name":"blobs","points":[{"x":1,"y":2,"z":3}]}""")!,
            ["cells"] = Legacy("""{"name":"cells","objects":[{"coverage":12.5}]}""")!,
        };
        Assert.Equal(7,    ExpressionEvaluator.Evaluate("$vals[1]",         Vars(), lists), 9);
        Assert.Equal(3,    ExpressionEvaluator.Evaluate("$vals.length",     Vars(), lists), 9);
        Assert.Equal(3,    ExpressionEvaluator.Evaluate("$blobs[0][2]",     Vars(), lists), 9);
        Assert.Equal(2,    ExpressionEvaluator.Evaluate("$blobs[0].y",      Vars(), lists), 9);
        Assert.Equal(12.5, ExpressionEvaluator.Evaluate("$cells[0].coverage", Vars(), lists), 9);
    }

    // -- Dotted IO variables -----------------------------------------------------

    [Theory]
    [InlineData("$stb.in1", "stb.in1", 1)]
    [InlineData("$relay.1", "relay.1", 1)]
    [InlineData("$nano.Board.pin1", "nano.Board.pin1", 1)]
    public void DottedIoNamesResolve(string expr, string key, double value)
        => Assert.Equal(value, Eval(expr, Vars((key, value))), 9);

    [Fact]
    public void DottedLookupPrefersLongestMatch()
        => Assert.Equal(7, Eval("$stb.in1", Vars(("stb", 99), ("stb.in1", 7))), 9);

    [Fact]
    public void DottedIoNamesWorkInsideExpressions()
        => Assert.Equal(2, Eval("$stb.in1 + $stb.in2", Vars(("stb.in1", 1), ("stb.in2", 1))), 9);

    // -- Unknown variables throw ---------------------------------------------------

    [Fact]
    public void UnknownVariableThrows()
    {
        var ex = Assert.Throws<UnknownVariableException>(() => Eval("$typo + 1", Vars(("real", 5))));
        Assert.Equal("typo", ex.VariableName);
    }

    [Fact]
    public void UnknownVariableWithIndexThrows()
        => Assert.Throws<UnknownVariableException>(() => Eval("$typo[0]"));

    [Fact]
    public void UnknownDottedVariableThrows()
        => Assert.Throws<UnknownVariableException>(() => Eval("$stb.in9", Vars(("stb.in1", 1))));

    // -- Comparison and logic ------------------------------------------------------

    // 1 and 0 are not an arbitrary encoding: that is exactly how a boolean variable is
    // stored, so a comparison can be assigned to one with no conversion step.
    [Theory]
    [InlineData("5 > 3", 1)]
    [InlineData("3 > 5", 0)]
    [InlineData("5 >= 5", 1)]
    [InlineData("5 < 3", 0)]
    [InlineData("3 <= 3", 1)]
    [InlineData("2 == 2", 1)]
    [InlineData("2 != 2", 0)]
    [InlineData("2 != 3", 1)]
    public void ComparisonsYieldOneOrZero(string expr, double expected)
        => Assert.Equal(expected, Eval(expr), 9);

    // The same 1e-9 tolerance ConditionGroup rows use — and for the same reason: these
    // are doubles that have been through arithmetic, and 0.1 + 0.2 is not exactly 0.3.
    [Fact]
    public void EqualityIsWithinEpsilon()
    {
        Assert.Equal(1, Eval("0.1 + 0.2 == 0.3"), 9);
        Assert.Equal(0, Eval("0.1 + 0.2 != 0.3"), 9);
    }

    [Theory]
    [InlineData("1 and 1", 1)]
    [InlineData("1 and 0", 0)]
    [InlineData("0 or 1", 1)]
    [InlineData("0 or 0", 0)]
    [InlineData("not 0", 1)]
    [InlineData("not 1", 0)]
    [InlineData("not 5", 0)]        // any non-zero is true
    [InlineData("true and not false", 1)]
    public void LogicOperators(string expr, double expected)
        => Assert.Equal(expected, Eval(expr), 9);

    // The C-style spellings fold into the word forms in the tokenizer, so both have to
    // give the same answer for every operator that has two spellings.
    [Theory]
    [InlineData("1 && 0", "1 and 0")]
    [InlineData("0 || 1", "0 or 1")]
    [InlineData("!0", "not 0")]
    [InlineData("$a = 5", "$a == 5")]   // a lone '=' means '=='
    public void AlternateSpellingsAgree(string symbolic, string worded)
        => Assert.Equal(Eval(worded, Vars(("a", 5))), Eval(symbolic, Vars(("a", 5))), 9);

    // Arithmetic binds tighter than comparison, comparison tighter than and/or, so the
    // expression a user would actually write needs no parentheses.
    [Theory]
    [InlineData("2 + 3 > 4", 1)]                  // (2+3) > 4
    [InlineData("$count > 5 and $count < 10", 1)]
    [InlineData("$count > 5 and $count < 7", 0)]
    [InlineData("$count > 50 or $count == 8", 1)]
    [InlineData("not $count > 5", 0)]             // not (count > 5)
    [InlineData("($count > 5) and ($count < 10)", 1)]
    public void Precedence(string expr, double expected)
        => Assert.Equal(expected, Eval(expr, Vars(("count", 8))), 9);

    // Left-associative like C and JavaScript, not chained like maths: "1 < 2 < 3" is
    // "(1 < 2) < 3" = "1 < 3" = 1, and "3 > 2 > 1" is "(3 > 2) > 1" = "1 > 1" = 0.
    [Theory]
    [InlineData("1 < 2 < 3", 1)]
    [InlineData("3 > 2 > 1", 0)]
    public void ChainedComparisonsAreLeftAssociative(string expr, double expected)
        => Assert.Equal(expected, Eval(expr), 9);

    // Booleans reach expressions as the 0/1 they are stored as, so a boolean list element
    // compares and combines with no special case anywhere.
    [Fact]
    public void BooleanListElementsCombine()
    {
        var bools = new Dictionary<string, List<bool>> { ["flags"] = [true, false] };
        Assert.Equal(1, Eval("$flags[0] and not $flags[1]", bools: bools), 9);
        Assert.Equal(1, Eval("$flags[0] != $flags[1]",      bools: bools), 9);
    }

    // A record's fields are named by whatever wrote it, so a field can collide with an
    // operator word. After a dot it is always a field name.
    [Fact]
    public void FieldNamedLikeAnOperatorStillResolves()
    {
        var objs = new Dictionary<string, List<ObjectRecord>>
        {
            ["r"] = [new ObjectRecord { ["not"] = 3, ["and"] = 4 }],
        };
        Assert.Equal(3, Eval("$r[0].not", objs: objs), 9);
        Assert.Equal(7, Eval("$r[0].not + $r[0].and", objs: objs), 9);
    }
}
