using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;

namespace RobotControl.Tests;

/// <summary>Image layers: every filter, chaining, the per-frame cache and the upTo preview.</summary>
public class VisionLayerTests
{
    private static Mat Apply(Mat src, string type, object? p = null) => VisionFilters.Apply(src, Filter(type, p));

    private static double At(Mat m, int x, int y) => m.Channels() == 1 ? m.At<byte>(y, x) : m.At<Vec3b>(y, x).Item0;

    /// <summary>Left half pure red, right half pure blue (BGR).</summary>
    private static Mat RedBlue()
    {
        var img = new Mat(100, 200, MatType.CV_8UC3, new Scalar(0, 0, 255));
        Cv2.Rectangle(img, new Rect(100, 0, 100, 100), new Scalar(255, 0, 0), -1);
        return img;
    }

    [Fact]
    public void Channel_picks_one_plane()
    {
        using var img = RedBlue();
        using var r = Apply(img, "channel", new { channel = "r" });
        using var b = Apply(img, "channel", new { channel = "b" });
        using var g = Apply(img, "channel", new { channel = "gray" });
        using var s = Apply(img, "channel", new { channel = "s" });
        using var h = Apply(img, "channel", new { channel = "h" });
        Assert.Equal(1, r.Channels());
        Assert.Equal(255, At(r, 10, 50)); Assert.Equal(0, At(r, 150, 50));
        Assert.Equal(0, At(b, 10, 50));   Assert.Equal(255, At(b, 150, 50));
        Assert.InRange(At(g, 10, 50), 70, 80);          // 0.299 × 255
        Assert.Equal(255, At(s, 10, 50));
        Assert.Equal(0, At(h, 10, 50));                   // red
        Assert.InRange(At(h, 150, 50), 118, 122);         // blue = 120
    }

    [Fact]
    public void ColorMask_selects_an_hsv_range_including_a_wrapped_hue()
    {
        using var img = RedBlue();
        using var blue = Apply(img, "colorMask", new { hMin = 110, hMax = 130, sMin = 100, sMax = 255, vMin = 50, vMax = 255 });
        Assert.Equal(0, At(blue, 10, 50)); Assert.Equal(255, At(blue, 150, 50));
        // Reds sit either side of 0: hMin > hMax wraps round.
        using var red = Apply(img, "colorMask", new { hMin = 170, hMax = 10, sMin = 100, sMax = 255, vMin = 50, vMax = 255 });
        Assert.Equal(255, At(red, 10, 50)); Assert.Equal(0, At(red, 150, 50));
    }

    [Fact]
    public void Blur_and_median_smooth()
    {
        using var img = new Mat(60, 60, MatType.CV_8UC1, Scalar.All(100));
        img.Set(30, 30, (byte)255);   // one hot pixel
        using var blur = Apply(img, "blur", new { size = 5 });
        using var med = Apply(img, "median", new { size = 3 });
        Assert.InRange(At(blur, 30, 30), 101, 140);
        Assert.Equal(100, At(med, 30, 30));
        // Even sizes are made odd rather than rejected.
        using var blur4 = Apply(img, "blur", new { size = 4 });
        Assert.True(At(blur4, 30, 30) < 255);
    }

    [Fact]
    public void Threshold_fixed_otsu_adaptive_and_invert()
    {
        using var img = new Mat(50, 100, MatType.CV_8UC1, Scalar.All(60));
        Cv2.Rectangle(img, new Rect(50, 0, 50, 50), Scalar.All(200), -1);
        using var band = Apply(img, "threshold", new { mode = "fixed", low = 150, high = 255 });
        Assert.Equal(0, At(band, 10, 10)); Assert.Equal(255, At(band, 80, 10));
        using var otsu = Apply(img, "threshold", new { mode = "otsu" });
        Assert.Equal(0, At(otsu, 10, 10)); Assert.Equal(255, At(otsu, 80, 10));
        using var inv = Apply(img, "threshold", new { mode = "otsu", invert = true });
        Assert.Equal(255, At(inv, 10, 10)); Assert.Equal(0, At(inv, 80, 10));
        using var adaptive = Apply(img, "threshold", new { mode = "adaptive", blockSize = 15, c = 5 });
        Assert.Equal(255, At(adaptive, 52, 25));   // just past the step: brighter than its neighbourhood
        Assert.Equal(0, At(adaptive, 47, 25));
    }

    [Fact]
    public void Morph_open_removes_specks_and_close_fills_holes()
    {
        using var img = new Mat(80, 80, MatType.CV_8UC1, Scalar.Black);
        Cv2.Rectangle(img, new Rect(20, 20, 40, 40), Scalar.White, -1);
        img.Set(5, 5, (byte)255);          // speck
        img.Set(40, 40, (byte)0);          // hole
        using var open = Apply(img, "morph", new { op = "open", size = 3 });
        using var close = Apply(img, "morph", new { op = "close", size = 3 });
        using var erode = Apply(img, "morph", new { op = "erode", size = 3, iterations = 2 });
        using var dilate = Apply(img, "morph", new { op = "dilate", size = 3 });
        Assert.Equal(0, At(open, 5, 5));
        Assert.Equal(255, At(close, 40, 40));
        Assert.True(Cv2.CountNonZero(erode) < Cv2.CountNonZero(img));
        Assert.True(Cv2.CountNonZero(dilate) > Cv2.CountNonZero(img));
    }

    [Fact]
    public void Contrast_linear_equalize_and_clahe()
    {
        using var img = new Mat(50, 100, MatType.CV_8UC1, Scalar.All(100));
        Cv2.Rectangle(img, new Rect(50, 0, 50, 50), Scalar.All(120), -1);
        using var lin = Apply(img, "contrast", new { mode = "linear", contrast = 2, brightness = -100 });
        Assert.Equal(100, At(lin, 10, 10)); Assert.Equal(140, At(lin, 80, 10));
        using var eq = Apply(img, "contrast", new { mode = "equalize" });
        Assert.True(At(eq, 80, 10) - At(eq, 10, 10) > 100);
        using var clahe = Apply(img, "contrast", new { mode = "clahe", clipLimit = 4, tileSize = 2 });
        Assert.Equal(1, clahe.Channels());
        Assert.True(At(clahe, 80, 10) - At(clahe, 10, 10) >= 15, $"clahe {At(clahe, 10, 10)} → {At(clahe, 80, 10)}");
    }

    [Fact]
    public void Edges_sobel_and_canny_mark_the_step()
    {
        using var img = new Mat(50, 100, MatType.CV_8UC1, Scalar.All(30));
        Cv2.Rectangle(img, new Rect(50, 0, 50, 50), Scalar.All(220), -1);
        using var sobel = Apply(img, "edges", new { mode = "sobel" });
        using var canny = Apply(img, "edges", new { mode = "canny", low = 50, high = 150 });
        Assert.True(At(sobel, 50, 25) > 100); Assert.Equal(0, At(sobel, 20, 25));
        Assert.InRange(Cv2.CountNonZero(canny), 40, 120);   // one or two edge columns
    }

    [Fact]
    public void BackgroundRemove_flattens_shading_and_keeps_detail()
    {
        using var img = new Mat(100, 200, MatType.CV_8UC1);
        for (int x = 0; x < 200; x++) Cv2.Line(img, new Point(x, 0), new Point(x, 99), Scalar.All(40 + x), 1);
        Cv2.Circle(img, new Point(150, 50), 6, Scalar.All(20), 2);
        using var flat = Apply(img, "backgroundRemove", new { size = 15 });
        Cv2.MeanStdDev(new Mat(flat, new Rect(10, 10, 60, 80)), out var mean, out var std);
        Assert.InRange(mean.Val0, 124, 132);       // re-centred on 128, the slope is gone
        Assert.True(std.Val0 < 2, $"shading left {std.Val0}");
        Assert.True(At(flat, 150, 44) < 100, "the ring stays dark against the flattened background");
    }

    [Fact]
    public void Invert_flips()
    {
        using var img = new Mat(10, 10, MatType.CV_8UC1, Scalar.All(30));
        using var inv = Apply(img, "invert");
        Assert.Equal(225, At(inv, 5, 5));
    }

    [Fact]
    public void Unknown_filter_throws()
    {
        using var img = new Mat(10, 10, MatType.CV_8UC1, Scalar.All(30));
        Assert.Throws<InvalidOperationException>(() => Apply(img, "sharpen"));
    }

    [Fact]
    public void Layers_chain_cache_skip_disabled_filters_and_preview_up_to_a_filter()
    {
        using var img = Canvas(100);
        Cv2.Rectangle(img, new Rect(200, 0, 200, 300), Scalar.All(200), -1);
        var gray = new ImageLayer { Id = "gray", Input = "camera", Filters = { Filter("channel", new { channel = "gray" }) } };
        var bin = new ImageLayer
        {
            Id = "bin", Input = "gray",
            Filters = { Filter("threshold", new { mode = "fixed", low = 150, high = 255 }), Filter("invert"), Filter("blur", new { size = 31 }) },
        };
        bin.Filters[2].Enabled = false;
        using var cache = new LayerCache(img, new[] { gray, bin });

        var b = cache.Get("bin");
        Assert.Same(b, cache.Get("bin"));                 // cached for the frame
        Assert.Equal(255, b.At<byte>(10, 10));            // dark side, inverted
        Assert.Equal(0, b.At<byte>(10, 300));
        Assert.Equal(new[] { "gray", "bin" }, cache.Computed);

        using var p0 = cache.Preview("bin", 0);           // the threshold only
        Assert.Equal(0, p0.At<byte>(10, 10));
        using var pNone = cache.Preview("bin", -1);       // the input as it is
        Assert.Equal(100, pNone.At<byte>(10, 10));
        using var pAll = cache.Preview("bin", null);
        Assert.Equal(255, pAll.At<byte>(10, 10));

        Assert.Throws<InvalidOperationException>(() => cache.Get("nope"));
    }

    [Fact]
    public void A_layer_that_reads_itself_is_an_error_not_a_hang()
    {
        using var img = Canvas();
        var a = new ImageLayer { Id = "a", Input = "b" };
        var b = new ImageLayer { Id = "b", Input = "a" };
        using var cache = new LayerCache(img, new[] { a, b });
        Assert.Throws<InvalidOperationException>(() => cache.Get("a"));
    }

    [Fact]
    public void A_tool_reads_its_input_layer()
    {
        // Gray 100 on the left, 200 on the right; an inverted layer swaps them.
        using var img = Canvas(100);
        Cv2.Rectangle(img, new Rect(200, 0, 200, 300), Scalar.All(200), -1);
        var prog = Program(Tool("i", "intensity", Rect(0, 0, 100, 300), new { statistic = "mean" }, input: "inv"));
        prog.Layers.Add(new ImageLayer { Id = "inv", Input = "camera", Filters = { Filter("invert") } });
        var r = Run(prog, img);
        Assert.InRange(Num(r.Tool("i")!, "mean"), 154, 156);
        Assert.Contains("inv", r.Layers);
        Assert.True(r.Timings.ContainsKey("layer:inv"));

        prog.Tools[0].Input = "missing";
        var bad = Run(prog, img).Tool("i")!;
        Assert.Equal(ToolStatus.Error, bad.Status);
        Assert.Contains("missing", bad.Message);
    }
}
