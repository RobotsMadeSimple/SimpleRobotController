using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Inspections;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

/// <summary>
/// The vision pipeline pieces that sit between the geometry and the processing thread:
/// inspection ordering, the shared per-frame cache, label layout, and each strategy run on
/// a synthetic frame whose correct answer is known.
/// </summary>
public class VisionPipelineTests
{
    private const int W = 640, H = 480;

    // -- Ordering ----------------------------------------------------------------

    [Fact]
    public void ListedIdsRunInStoredOrderAndUnlistedOnesFollowInTypeOrder()
    {
        var typeOrder = new[] { "blob1", "color1", "poly1", "line1", "code1" };
        var order     = new List<string> { "line1", "blob1", "missing", "poly1" };

        var result = InspectionPlan.Order(typeOrder, s => s, order);

        Assert.Equal(new[] { "line1", "blob1", "poly1", "color1", "code1" }, result);
    }

    [Fact]
    public void NoStoredOrderMeansTypeOrder()
    {
        var typeOrder = new[] { "a", "b", "c" };
        Assert.Equal(typeOrder, InspectionPlan.Order(typeOrder, s => s, null));
        Assert.Equal(typeOrder, InspectionPlan.Order(typeOrder, s => s, new List<string>()));
    }

    [Fact]
    public void ADuplicatedIdUsesItsFirstPosition()
    {
        var result = InspectionPlan.Order(new[] { "a", "b" }, s => s, new List<string> { "b", "a", "b" });
        Assert.Equal(new[] { "b", "a" }, result);
    }

    [Fact]
    public void PlanHonoursInspectionOrderAndSkipsDisabled()
    {
        var prog = new VisionProgram
        {
            Id                 = "p",
            Inspections        = { new BlobInspection    { Id = "blob" } },
            ColorInspections   = { new ColorCoverageInspection { Id = "color", Enabled = false } },
            PolygonInspections = { new PolygonInspection { Id = "poly" } },
            LineInspections    = { new LineInspection    { Id = "line" } },
            BarcodeInspections = { new BarcodeInspection { Id = "code" } },
            InspectionOrder    = { "code", "color", "line" },
        };

        var plan = InspectionPlan.Build(prog, new ArucoStrategy());

        Assert.Equal(new[] { "code", "line", "blob", "poly" }, plan.Steps.Select(s => s.Id));
    }

    [Fact]
    public void PlanOutlinesOnlyZonesUsedByEnabledInspections()
    {
        var prog = new VisionProgram
        {
            Zones =
            {
                new VisionZone { Id = "z1", Name = "one" },
                new VisionZone { Id = "z2", Name = "two" },
                new VisionZone { Id = "z3", Name = "three" },
            },
            Inspections      = { new BlobInspection { Id = "b", ZoneId = "z3" } },
            LineInspections  = { new LineInspection { Id = "l", ZoneId = "z1" } },
            ArucoInspections = { new ArucoInspection { Id = "a", ZoneId = "z2", Enabled = false } },
        };

        var plan = InspectionPlan.Build(prog, new ArucoStrategy());

        // Zone-list order, not inspection order.
        Assert.Equal(new[] { "z1", "z3" }, plan.BorderZones.Select(z => z.Id));
    }

    [Fact]
    public void AllInspectionsCoversEveryListInTypeOrder()
    {
        var prog = new VisionProgram
        {
            BarcodeInspections = { new BarcodeInspection { Id = "6" } },
            LineInspections    = { new LineInspection    { Id = "5" } },
            ArucoInspections   = { new ArucoInspection   { Id = "4" } },
            PolygonInspections = { new PolygonInspection { Id = "3" } },
            ColorInspections   = { new ColorCoverageInspection { Id = "2" } },
            Inspections        = { new BlobInspection    { Id = "1" } },
        };

        Assert.Equal(new[] { "1", "2", "3", "4", "5", "6" }, prog.AllInspections().Select(i => i.Id));

        // Setting a zone through the shared interface reaches the typed object.
        foreach (var i in prog.AllInspections()) i.ZoneId = "z";
        Assert.Equal("z", prog.BarcodeInspections[0].ZoneId);
        Assert.Equal("z", prog.Inspections[0].ZoneId);
    }

    [Fact]
    public void AllInspectionsDoesNotChangeTheSerializedShape()
    {
        var json = System.Text.Json.JsonSerializer.Serialize(new VisionProgram());
        Assert.DoesNotContain("allInspections", json, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void AFailingBlobInspectionStillReportsAnEmptyResult()
    {
        var insp = new BlobInspection { Id = "b", Name = "Blob" };
        var prog = new VisionProgram { Inspections = { insp } };
        var step = InspectionPlan.Build(prog, new ArucoStrategy()).Steps.Single();

        var result = new VisionResult();
        step.RecordFailure(result);

        var ir = Assert.Single(result.Inspections);
        Assert.Equal("b", ir.InspectionId);
        Assert.Empty(ir.Blobs);
    }

    // -- Zone containment and masks ---------------------------------------------

    [Theory]
    [InlineData(320, 240, true)]
    [InlineData(100, 100, false)]
    [InlineData(479, 359, true)]
    [InlineData(481, 240, false)]
    public void PointInUprightRectangle(float x, float y, bool inside)
    {
        var geom = new VisionZoneGeometry { X = 0.25, Y = 0.25, Width = 0.5, Height = 0.5 };
        Assert.Equal(inside, ZoneGeometry.IsInsideZone(geom, x, y, W, H));
    }

    [Fact]
    public void PointInTiltedRectangleFollowsTheTilt()
    {
        // A long thin bar turned 90° becomes a tall thin bar about the same center.
        var geom = new VisionZoneGeometry { X = 0.25, Y = 0.45, Width = 0.5, Height = 0.1, Rotation = 90 };
        Assert.True (ZoneGeometry.IsInsideZone(geom, 320, 400, W, H));   // along the new long axis
        Assert.False(ZoneGeometry.IsInsideZone(geom, 450, 240, W, H));   // along the old long axis
    }

    [Fact]
    public void PointInCircleAndPolygon()
    {
        var circle = new VisionZoneGeometry { Shape = VisionZoneShape.Circle, Cx = 0.5, Cy = 0.5, Radius = 0.25 };
        Assert.True (ZoneGeometry.IsInsideZone(circle, 320 + 119, 240, W, H));
        Assert.False(ZoneGeometry.IsInsideZone(circle, 320 + 100, 240 + 100, W, H));

        var triangle = new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Polygon, Points = [[0, 0], [1, 0], [0, 1]],
        };
        Assert.True (ZoneGeometry.IsInsideZone(triangle, 100, 100, W, H));
        Assert.False(ZoneGeometry.IsInsideZone(triangle, 600, 450, W, H));
    }

    [Fact]
    public void ZoneMaskCoversTheZoneArea()
    {
        using var mask = new Mat(H, W, MatType.CV_8UC1, Scalar.Black);
        ZoneGeometry.FillZoneMask(mask, new VisionZoneGeometry { X = 0.25, Y = 0.25, Width = 0.5, Height = 0.5 }, W, H);
        Assert.Equal(320 * 240, Cv2.CountNonZero(mask));
    }

    [Fact]
    public void TiltedZoneMaskKeepsItsArea()
    {
        using var mask = new Mat(H, W, MatType.CV_8UC1, Scalar.Black);
        ZoneGeometry.FillZoneMask(mask, new VisionZoneGeometry
        {
            X = 0.25, Y = 0.25, Width = 0.5, Height = 0.5, Rotation = 30,
        }, W, H);
        Assert.InRange(Cv2.CountNonZero(mask), 320 * 240 * 0.98, 320 * 240 * 1.02);
    }

    [Fact]
    public void ClampRectKeepsTheRectInsideTheFrame()
    {
        Assert.Equal(new Rect(0, 0, 50, 40), ZoneGeometry.ClampRect(-10, -5, 50, 40, W, H));
        Assert.Equal(new Rect(639, 479, 1, 1), ZoneGeometry.ClampRect(700, 500, 50, 40, W, H));
        Assert.Equal(new Rect(600, 0, 40, 1), ZoneGeometry.ClampRect(600, 0, 100, 0, W, H));
    }

    // -- FrameContext ------------------------------------------------------------

    [Fact]
    public void FrameContextBuildsEachIntermediateOnce()
    {
        using var src = new Mat(H, W, MatType.CV_8UC3, new Scalar(10, 20, 30));
        using var ctx = new FrameContext(src);

        Assert.Same(ctx.Gray, ctx.Gray);
        Assert.Same(ctx.Blurred, ctx.Blurred);
        Assert.Same(ctx.Canny(50, 150), ctx.Canny(50, 150));
        Assert.NotSame(ctx.Canny(50, 150), ctx.Canny(10, 20));

        var zone = new VisionZone { Id = "z", Geometry = new VisionZoneGeometry { Width = 0.5, Height = 0.5 } };
        Assert.Same(ctx.ZoneMask(zone), ctx.ZoneMask(zone));
        Assert.Equal(W * H / 4, Cv2.CountNonZero(ctx.ZoneMask(zone)));
        Assert.Equal(W * H, Cv2.CountNonZero(ctx.ZoneMask(null)));
        Assert.Equal(MatType.CV_8UC1, ctx.Gray.Type());
    }

    [Fact]
    public void FrameContextDisposesWhatItBuiltButNotTheSource()
    {
        using var src = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        var ctx   = new FrameContext(src);
        var gray  = ctx.Gray;
        var edges = ctx.Canny(1, 2);
        var mask  = ctx.ZoneMask(new VisionZone { Id = "z" });

        ctx.Dispose();

        Assert.True(gray.IsDisposed);
        Assert.True(edges.IsDisposed);
        Assert.True(mask.IsDisposed);
        Assert.False(src.IsDisposed);
        Assert.Throws<ObjectDisposedException>(() => ctx.Gray);
    }

    // -- LabelStack --------------------------------------------------------------

    [Fact]
    public void LabelsStackDownThenWrapIntoANewColumn()
    {
        // 70px tall: baselines at 20, 42 and 64 fit (64 <= 70 - 4); the fourth would not.
        using var img = new Mat(70, 400, MatType.CV_8UC3, Scalar.All(0));
        var labels = new LabelStack(img.Height);

        var positions = new List<Point>();
        for (int i = 0; i < 4; i++)
        {
            positions.Add(labels.Next);
            labels.DrawStatusLabel(img, $"Inspection {i}", passed: i % 2 == 0);
        }

        Assert.Equal(new Point(6, 20), positions[0]);
        Assert.Equal(new Point(6, 42), positions[1]);
        Assert.Equal(new Point(6, 64), positions[2]);
        Assert.Equal(LabelStack.TopY, positions[3].Y);
        Assert.True(positions[3].X > 6 + 50, "second column starts right of the first column's text");
    }

    [Fact]
    public void AFrameTooShortForOneLabelDoesNotWrapForever()
    {
        using var img = new Mat(10, 200, MatType.CV_8UC3, Scalar.All(0));
        var labels = new LabelStack(img.Height);
        Assert.Equal(new Point(6, LabelStack.TopY), labels.Next);
    }

    // -- Barcode / ArUco configuration ------------------------------------------

    [Fact]
    public void BarcodeFormatsParseOnce()
    {
        Assert.Null(BarcodeStrategy.ParseFormats(Array.Empty<string>()));
        Assert.Equal(new[] { ZXing.BarcodeFormat.QR_CODE, ZXing.BarcodeFormat.CODE_128 },
                     BarcodeStrategy.ParseFormats(new[] { "QR_CODE", "nonsense", "CODE_128" }));
        // Only unknown names: an empty (not null) restriction, as before.
        Assert.Empty(BarcodeStrategy.ParseFormats(new[] { "nonsense" })!);
    }

    [Fact]
    public void AllDictionariesCoversTheClassicArucoSet()
    {
        var ids = ArucoStrategy.AllDictionaryTypes.Select(t => (int)t).ToHashSet();
        for (int i = 0; i <= 16; i++)
            Assert.Contains(i, ids);
        Assert.Equal(ids.Count, ArucoStrategy.AllDictionaryTypes.Length);
    }

    // -- Strategies on synthetic frames -----------------------------------------

    [Fact]
    public void ColorCoverageMeasuresTheMatchedFraction()
    {
        // Left half pure red, right half black.
        using var src = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        using (var left = new Mat(src, new Rect(0, 0, W / 2, H))) left.SetTo(new Scalar(0, 0, 255));
        using var ctx = new FrameContext(src);

        var insp = new ColorCoverageInspection
        {
            Id = "c", Colors = { new ColorEntry { R = 255, G = 0, B = 0, Tolerance = 10 } }, MinCoverage = 40,
        };
        using var d = InspectionPlan.Color.Run(ctx, insp, zone: null);

        Assert.Equal(50.0, d.Result.Coverage);
        Assert.True(d.Result.Passed);
        Assert.Null(d.Result.Cells);
    }

    [Fact]
    public void ColorCoverageGridFailsWhenAnyCellFails()
    {
        using var src = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        using (var left = new Mat(src, new Rect(0, 0, W / 2, H))) left.SetTo(new Scalar(0, 0, 255));
        using var ctx = new FrameContext(src);

        var zone = new VisionZone { Id = "z", Name = "Tray", Grid = new VisionZoneGrid { Rows = 1, Cols = 2 } };
        var insp = new ColorCoverageInspection
        {
            Id = "c", Name = "Fill", ZoneId = "z", Colors = { new ColorEntry { R = 255, Tolerance = 10 } }, MinCoverage = 40,
        };
        using var d = InspectionPlan.Color.Run(ctx, insp, zone);

        Assert.Equal(new[] { 100.0, 0.0 }, d.Result.Cells!.Select(c => c.Coverage));
        Assert.Equal(1, d.Result.CellsPassed);
        Assert.False(d.Result.Passed);

        using var annotated = src.Clone();
        InspectionPlan.Color.Annotate(annotated, d, insp, zone, new LabelStack(H));
    }

    [Fact]
    public void PolygonFindsASquareAndRespectsTheZone()
    {
        using var src = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        Cv2.Rectangle(src, new Rect(100, 100, 120, 120), Scalar.All(255), -1);
        using var ctx = new FrameContext(src);

        var insp = new PolygonInspection { Id = "p", Sides = 4, MinThreshold = 128, MaxThreshold = 255 };
        var found = InspectionPlan.Polygon.Run(ctx, insp, zone: null);

        Assert.Equal(1, found.Result.Count);
        Assert.Equal(160.0 / W, found.Result.CenterX, 2);
        Assert.Equal(160.0 / H, found.Result.CenterY, 2);

        var elsewhere = new VisionZone { Id = "z", Geometry = new VisionZoneGeometry { X = 0.5, Y = 0.5, Width = 0.5, Height = 0.5 } };
        Assert.False(InspectionPlan.Polygon.Run(ctx, insp, elsewhere).Result.Found);

        var debug = InspectionPlan.Polygon.RenderDebug(ctx, insp, null);
        Assert.NotNull(debug);
        Assert.True(debug!.Length > 0);
    }

    [Fact]
    public void LineFindsAHorizontalLineAndFiltersByAngle()
    {
        using var src = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        Cv2.Line(src, new Point(100, 240), new Point(500, 240), Scalar.All(255), 3);
        using var ctx = new FrameContext(src);

        var insp = new LineInspection { Id = "l", MinLineLength = 100 };
        var d    = InspectionPlan.Line.Run(ctx, insp, zone: null);
        Assert.True(d.Result.Found);
        Assert.All(d.Result.Lines, l => Assert.True(l.Angle < 5 || l.Angle > 175));
        Assert.Equal(d.Result.Lines.Count, d.Segments.Count);

        var vertical = new LineInspection { Id = "v", MinLineLength = 100, FilterByAngle = true, MinAngle = 80, MaxAngle = 100 };
        Assert.False(InspectionPlan.Line.Run(ctx, vertical, zone: null).Result.Found);

        // The shared Canny output must survive a zone-masked run untouched.
        int edgesBefore = Cv2.CountNonZero(ctx.Canny(insp.CannyThreshold1, insp.CannyThreshold2));
        var zone = new VisionZone { Id = "z", Geometry = new VisionZoneGeometry { Width = 0.1, Height = 0.1 } };
        Assert.False(InspectionPlan.Line.Run(ctx, insp, zone).Result.Found);
        Assert.Equal(edgesBefore, Cv2.CountNonZero(ctx.Canny(insp.CannyThreshold1, insp.CannyThreshold2)));

        Assert.NotNull(InspectionPlan.Line.RenderDebug(ctx, insp, zone));
    }

    [Fact]
    public void ArucoFindsADrawnMarker()
    {
        using var src = new Mat(H, W, MatType.CV_8UC3, Scalar.All(255));
        using (var dict = CvAruco.GetPredefinedDictionary(PredefinedDictionaryType.Dict4X4_100))
        using (var marker = new Mat())
        {
            dict.GenerateImageMarker(7, 120, marker, 1);
            using var bgr = new Mat();
            Cv2.CvtColor(marker, bgr, ColorConversionCodes.GRAY2BGR);
            using var roi = new Mat(src, new Rect(200, 150, 120, 120));
            bgr.CopyTo(roi);
        }
        using var ctx = new FrameContext(src);

        var aruco = new ArucoStrategy();
        try
        {
            var d = aruco.Run(ctx, new ArucoInspection { Id = "a", DictionaryId = (int)PredefinedDictionaryType.Dict4X4_100 }, null);
            var m = Assert.Single(d.Result.Markers);
            Assert.Equal(7, m.MarkerId);
            Assert.Equal(260.0 / W, m.CenterX, 2);
        }
        finally
        {
            aruco.ReleaseDetectors();
        }
    }
}
