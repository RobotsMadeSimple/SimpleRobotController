using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;

namespace RobotControl.Tests;

// The layer filters' pixel output on small known images, plus disabled filters, chaining and the
// upTo preview.

internal static class FilterKit
{
    public static Mat Apply(Mat src, string type, object? p = null) => VisionFilters.Apply(src, Filter(type, p));

    public static byte G(Mat m, int x, int y) => m.At<byte>(y, x);
    public static Vec3b C(Mat m, int x, int y) => m.At<Vec3b>(y, x);

    public static Mat Gray(int w, int h, int v) => new(h, w, MatType.CV_8UC1, Scalar.All(v));
    public static Mat Bgr(int w, int h, Scalar c) => new(h, w, MatType.CV_8UC3, c);

    public static void AssertSame(Mat a, Mat b)
    {
        Assert.Equal(a.Size(), b.Size());
        Assert.Equal(a.Type(), b.Type());
        using var diff = new Mat();
        Cv2.Absdiff(a, b, diff);
        using var flat = diff.Reshape(1);
        int n = Cv2.CountNonZero(flat);
        if (n == 0) return;
        Cv2.MinMaxLoc(flat, out _, out double max, out _, out Point at);
        string Rows(Mat m) => string.Join(" / ", Enumerable.Range(0, Math.Min(4, m.Rows)).Select(y => string.Join(",", Enumerable.Range(0, Math.Min(15, m.Cols)).Select(x => m.Get<byte>(y, x)))));
        Assert.Fail($"{n} values differ, the most by {max} at ({at.X}, {at.Y}); a: {Rows(a)}; b: {Rows(b)}");
    }
}

public class ChannelFilterTests
{
    // One BGR pixel (30, 90, 200): gray 0.299·200 + 0.587·90 + 0.114·30 = 116; HSV (OpenCV 8-bit)
    // H = 60°·(90 − 30)/170 / 2 = 11, S = 255·170/200 = 217, V = 200.
    [Theory]
    [InlineData("gray", 116)]
    [InlineData("b", 30)]
    [InlineData("g", 90)]
    [InlineData("r", 200)]
    [InlineData("h", 11)]
    [InlineData("s", 217)]
    [InlineData("v", 200)]
    [InlineData("R", 200)]      // case does not matter
    public void Each_channel_is_one_plane(string channel, int want)
    {
        using var img = FilterKit.Bgr(4, 3, new Scalar(30, 90, 200));
        using var o = FilterKit.Apply(img, "channel", new { channel });
        Assert.Equal(MatType.CV_8UC1, o.Type());
        Assert.Equal(img.Size(), o.Size());
        Assert.Equal(want, FilterKit.G(o, 2, 1));
    }

    [Fact]
    public void Gray_is_the_default_and_a_gray_input_passes_through()
    {
        using var img = FilterKit.Bgr(4, 3, new Scalar(30, 90, 200));
        using var dflt = FilterKit.Apply(img, "channel");
        Assert.Equal(116, FilterKit.G(dflt, 0, 0));
        using var gray = FilterKit.Gray(4, 3, 77);
        using var same = FilterKit.Apply(gray, "channel", new { channel = "r" });
        FilterKit.AssertSame(gray, same);
    }
}

public class ColorMaskFilterTests
{
    private static int Mask(Scalar bgr, object p)
    {
        using var img = FilterKit.Bgr(3, 3, bgr);
        using var o = FilterKit.Apply(img, "colorMask", p);
        Assert.Equal(MatType.CV_8UC1, o.Type());
        return FilterKit.G(o, 1, 1);
    }

    private static readonly object Greens = new { hMin = 50, hMax = 70, sMin = 100, sMax = 255, vMin = 100, vMax = 255 };

    [Theory]
    [InlineData(0, 255, 0, 255)]       // pure green: H 60
    [InlineData(0, 255, 255, 0)]       // yellow: H 30
    [InlineData(0, 100, 0, 255)]       // V 100: the bound is inclusive
    [InlineData(0, 99, 0, 0)]          // V 99
    [InlineData(155, 255, 155, 255)]   // S 255·100/255 = 100: inclusive
    [InlineData(156, 255, 156, 0)]     // S 99
    [InlineData(255, 0, 0, 0)]         // blue: H 120
    public void Keeps_pixels_inside_the_hsv_box(int b, int g, int r, int want) =>
        Assert.Equal(want, Mask(new Scalar(b, g, r), Greens));

    [Theory]
    [InlineData(0, 0, 255, 255)]      // red: H 0
    [InlineData(60, 0, 255, 255)]     // H 173 (past 170)
    [InlineData(0, 40, 255, 255)]     // H 5
    [InlineData(0, 128, 255, 0)]      // orange: H 15
    [InlineData(255, 0, 0, 0)]        // blue
    public void A_hue_range_with_min_above_max_wraps_round_red(int b, int g, int r, int want) =>
        Assert.Equal(want, Mask(new Scalar(b, g, r), new { hMin = 170, hMax = 10, sMin = 100, vMin = 100 }));

    [Fact]
    public void The_defaults_take_everything_and_a_gray_input_has_no_saturation()
    {
        Assert.Equal(255, Mask(new Scalar(12, 200, 99), new { }));
        using var gray = FilterKit.Gray(3, 3, 150);
        using var all = FilterKit.Apply(gray, "colorMask", new { });
        Assert.Equal(255, FilterKit.G(all, 1, 1));
        using var saturated = FilterKit.Apply(gray, "colorMask", new { sMin = 1 });
        Assert.Equal(0, FilterKit.G(saturated, 1, 1));
    }
}

public class BlurFilterTests
{
    [Fact]
    public void A_3px_blur_spreads_an_impulse_by_the_binomial_kernel()
    {
        // (OpenCV's Gaussian blur can read past the edge of images under ~32 px, so the
        // blur tests keep their images bigger than that.)
        using var img = FilterKit.Gray(40, 40, 0);
        img.Set(20, 20, (byte)255);
        using var o = FilterKit.Apply(img, "blur", new { size = 3 });
        // [¼ ½ ¼] ⊗ [¼ ½ ¼] × 255.
        Assert.InRange(FilterKit.G(o, 20, 20), 63, 64);
        Assert.InRange(FilterKit.G(o, 19, 20), 31, 32);
        Assert.InRange(FilterKit.G(o, 20, 21), 31, 32);
        Assert.InRange(FilterKit.G(o, 19, 19), 15, 16);
        Assert.Equal(0, FilterKit.G(o, 18, 20));
        Assert.InRange(Cv2.Sum(o).Val0, 250, 260);   // the light is kept, just spread
    }

    [Fact]
    public void Even_sizes_are_made_odd_and_size_1_does_nothing()
    {
        using var img = FilterKit.Gray(40, 40, 0);
        img.Set(20, 20, (byte)255);
        using var four = FilterKit.Apply(img, "blur", new { size = 4 });
        using var five = FilterKit.Apply(img, "blur", new { size = 5 });
        FilterKit.AssertSame(five, four);
        Assert.True(FilterKit.G(five, 18, 20) > 0);  // a 5 px kernel reaches 2 px out
        Assert.Equal(0, FilterKit.G(five, 17, 20));
        using var one = FilterKit.Apply(img, "blur", new { size = 1 });
        FilterKit.AssertSame(img, one);
        using var dflt = FilterKit.Apply(img, "blur");
        FilterKit.AssertSame(five, dflt);             // 5 by default
    }

    [Fact]
    public void Colour_is_kept_and_flat_areas_are_unchanged()
    {
        using var img = FilterKit.Bgr(40, 40, new Scalar(10, 100, 200));
        using var o = FilterKit.Apply(img, "blur", new { size = 7 });
        Assert.Equal(MatType.CV_8UC3, o.Type());
        Assert.Equal(new Vec3b(10, 100, 200), FilterKit.C(o, 20, 20));
    }
}

public class MedianFilterTests
{
    [Fact]
    public void Removes_specks_smaller_than_half_the_window()
    {
        using var img = FilterKit.Gray(20, 20, 100);
        img.Set(3, 3, (byte)255);                                    // a single hot pixel
        Cv2.Rectangle(img, new Rect(10, 10, 3, 3), Scalar.All(255), -1);   // a 3 × 3 block
        using var m3 = FilterKit.Apply(img, "median", new { size = 3 });
        Assert.Equal(100, FilterKit.G(m3, 3, 3));
        Assert.Equal(255, FilterKit.G(m3, 11, 11));                  // 9 of 9 in a 3 × 3 window
        using var m5 = FilterKit.Apply(img, "median", new { size = 5 });
        Assert.Equal(100, FilterKit.G(m5, 11, 11));                  // 9 of 25
        using var dflt = FilterKit.Apply(img, "median");
        FilterKit.AssertSame(m5, dflt);
    }

    [Fact]
    public void Sizes_below_3_become_3_and_colour_is_kept()
    {
        using var img = FilterKit.Gray(10, 10, 50);
        img.Set(5, 5, (byte)200);
        using var one = FilterKit.Apply(img, "median", new { size = 1 });
        Assert.Equal(50, FilterKit.G(one, 5, 5));
        using var col = FilterKit.Bgr(10, 10, new Scalar(10, 20, 30));
        col.Set(5, 5, new Vec3b(255, 255, 255));
        using var o = FilterKit.Apply(col, "median", new { size = 3 });
        Assert.Equal(MatType.CV_8UC3, o.Type());
        Assert.Equal(new Vec3b(10, 20, 30), FilterKit.C(o, 5, 5));
    }
}

public class ThresholdFilterTests
{
    /// <summary>Four columns of gray 99, 100, 200, 201.</summary>
    private static Mat Levels()
    {
        var img = FilterKit.Gray(4, 2, 0);
        int[] v = { 99, 100, 200, 201 };
        for (int x = 0; x < 4; x++) { img.Set(0, x, (byte)v[x]); img.Set(1, x, (byte)v[x]); }
        return img;
    }

    private static byte[] Row(Mat m) => Enumerable.Range(0, m.Cols).Select(x => FilterKit.G(m, x, 0)).ToArray();

    [Theory]
    [InlineData(100, 200, false, new byte[] { 0, 255, 255, 0 })]
    [InlineData(100, 200, true, new byte[] { 255, 0, 0, 255 })]
    [InlineData(150, 255, false, new byte[] { 0, 0, 255, 255 })]
    [InlineData(0, 99, false, new byte[] { 255, 0, 0, 0 })]
    public void Fixed_keeps_low_to_high_inclusive(int low, int high, bool invert, byte[] want)
    {
        using var img = Levels();
        using var o = FilterKit.Apply(img, "threshold", new { mode = "fixed", low, high, invert });
        Assert.Equal(want, Row(o));
    }

    [Fact]
    public void Fixed_with_defaults_keeps_128_and_up()
    {
        using var img = FilterKit.Gray(2, 1, 0);
        img.Set(0, 0, (byte)127); img.Set(0, 1, (byte)128);
        using var o = FilterKit.Apply(img, "threshold");
        Assert.Equal(new byte[] { 0, 255 }, Row(o));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Otsu_splits_two_levels(bool invert)
    {
        using var img = FilterKit.Gray(20, 10, 60);
        Cv2.Rectangle(img, new Rect(10, 0, 10, 10), Scalar.All(170), -1);
        using var o = FilterKit.Apply(img, "threshold", new { mode = "otsu", invert });
        Assert.Equal(invert ? 255 : 0, FilterKit.G(o, 2, 5));
        Assert.Equal(invert ? 0 : 255, FilterKit.G(o, 15, 5));
        Assert.Equal(100, Cv2.CountNonZero(o));
    }

    [Fact]
    public void Adaptive_marks_pixels_brighter_than_their_neighbourhood_by_c()
    {
        // A horizontal ramp (a ramp is its own local mean), a bright dot and a dark dot.
        // A pixel is kept when it is above its 15 px neighbourhood mean − c.
        using var img = new Mat(40, 80, MatType.CV_8UC1);
        for (int x = 0; x < 80; x++) Cv2.Line(img, new Point(x, 0), new Point(x, 39), Scalar.All(50 + x));
        Cv2.Circle(img, new Point(40, 20), 2, Scalar.All(200), -1);
        Cv2.Circle(img, new Point(60, 20), 2, Scalar.All(0), -1);
        using var o = FilterKit.Apply(img, "threshold", new { mode = "adaptive", blockSize = 15, c = 5 });
        Assert.Equal(255, FilterKit.G(o, 20, 30));     // the ramp: v > v − 5
        Assert.Equal(255, FilterKit.G(o, 40, 20));
        Assert.Equal(0, FilterKit.G(o, 60, 20));       // the dark dot is well below its mean
        using var inv = FilterKit.Apply(img, "threshold", new { mode = "adaptive", blockSize = 15, c = 5, invert = true });
        Assert.Equal(0, FilterKit.G(inv, 20, 30));
        Assert.Equal(255, FilterKit.G(inv, 60, 20));
        // A negative c keeps only what stands out above its neighbourhood.
        using var neg = FilterKit.Apply(img, "threshold", new { mode = "adaptive", blockSize = 15, c = -5 });
        Assert.Equal(0, FilterKit.G(neg, 20, 30));
        Assert.Equal(255, FilterKit.G(neg, 40, 20));
        Assert.Equal(0, FilterKit.G(neg, 60, 20));
    }

    [Fact]
    public void A_colour_input_is_thresholded_on_its_gray()
    {
        using var img = FilterKit.Bgr(2, 1, new Scalar(30, 90, 200));   // gray 116
        using var lo = FilterKit.Apply(img, "threshold", new { low = 116, high = 255 });
        using var hi = FilterKit.Apply(img, "threshold", new { low = 117, high = 255 });
        Assert.Equal(MatType.CV_8UC1, lo.Type());
        Assert.Equal(255, FilterKit.G(lo, 0, 0));
        Assert.Equal(0, FilterKit.G(hi, 0, 0));
    }
}

public class MorphFilterTests
{
    // The 3 px elliptical kernel is a cross: a pixel and its four neighbours.
    private static Mat Square()
    {
        var img = FilterKit.Gray(30, 30, 0);
        Cv2.Rectangle(img, new Rect(10, 10, 10, 10), Scalar.All(255), -1);
        return img;
    }

    [Theory]
    [InlineData("erode", 1, 64)]      // 8 × 8
    [InlineData("erode", 2, 36)]      // 6 × 6
    [InlineData("dilate", 1, 140)]    // 10 × 10 plus a row on each side
    [InlineData("open", 1, 96)]       // eroded to 8 × 8, grown back without its corners
    [InlineData("close", 1, 100)]     // a convex square is closed already
    public void Each_op_on_a_square(string op, int iterations, int count)
    {
        using var img = Square();
        using var o = FilterKit.Apply(img, "morph", new { op, size = 3, iterations });
        Assert.Equal(count, Cv2.CountNonZero(o));
    }

    [Fact]
    public void Open_removes_specks_and_close_fills_holes()
    {
        using var img = Square();
        img.Set(3, 3, (byte)255);          // a speck
        img.Set(15, 15, (byte)0);          // a hole
        using var open = FilterKit.Apply(img, "morph", new { op = "open", size = 3 });
        using var close = FilterKit.Apply(img, "morph", new { op = "close", size = 3 });
        Assert.Equal(0, FilterKit.G(open, 3, 3));
        Assert.Equal(255, FilterKit.G(close, 15, 15));
        Assert.Equal(100 + 1, Cv2.CountNonZero(close));   // the square, its hole filled, and the lone speck
        using var dflt = FilterKit.Apply(img, "morph");
        FilterKit.AssertSame(open, dflt);                // open, size 3 by default
    }

    [Fact]
    public void A_bigger_kernel_reaches_further_and_colour_is_kept()
    {
        using var img = Square();
        using var d5 = FilterKit.Apply(img, "morph", new { op = "dilate", size = 5 });
        Assert.Equal(255, FilterKit.G(d5, 8, 15));        // 2 px out
        Assert.Equal(0, FilterKit.G(d5, 7, 15));
        using var col = FilterKit.Bgr(30, 30, Scalar.All(0));
        Cv2.Rectangle(col, new Rect(10, 10, 10, 10), new Scalar(0, 0, 255), -1);
        using var o = FilterKit.Apply(col, "morph", new { op = "erode", size = 3 });
        Assert.Equal(MatType.CV_8UC3, o.Type());
        Assert.Equal(new Vec3b(0, 0, 255), FilterKit.C(o, 15, 15));
        Assert.Equal(new Vec3b(0, 0, 0), FilterKit.C(o, 10, 15));
    }
}

public class ContrastFilterTests
{
    [Theory]
    [InlineData(100, 2, -100, 100)]
    [InlineData(120, 2, -100, 140)]
    [InlineData(200, 2, -100, 255)]     // saturates
    [InlineData(30, 2, -100, 0)]
    [InlineData(80, 0.5, 10, 50)]
    public void Linear_is_contrast_times_v_plus_brightness(int v, double contrast, double brightness, int want)
    {
        using var img = FilterKit.Gray(2, 2, v);
        using var o = FilterKit.Apply(img, "contrast", new { mode = "linear", contrast, brightness });
        Assert.Equal(want, FilterKit.G(o, 1, 1));
    }

    [Fact]
    public void Linear_is_the_default_keeps_colour_and_defaults_to_no_change()
    {
        using var img = FilterKit.Bgr(2, 2, new Scalar(10, 100, 200));
        using var same = FilterKit.Apply(img, "contrast");
        FilterKit.AssertSame(img, same);
        using var o = FilterKit.Apply(img, "contrast", new { contrast = 1.5 });
        Assert.Equal(new Vec3b(15, 150, 255), FilterKit.C(o, 0, 0));
    }

    [Fact]
    public void Equalize_stretches_the_histogram_over_0_to_255()
    {
        using var img = FilterKit.Gray(20, 10, 100);
        Cv2.Rectangle(img, new Rect(10, 0, 10, 10), Scalar.All(120), -1);
        using var o = FilterKit.Apply(img, "contrast", new { mode = "equalize" });
        Assert.Equal(0, FilterKit.G(o, 2, 2));
        Assert.Equal(255, FilterKit.G(o, 15, 2));
        using var col = FilterKit.Bgr(4, 4, new Scalar(30, 90, 200));
        using var g = FilterKit.Apply(col, "contrast", new { mode = "equalize" });
        Assert.Equal(MatType.CV_8UC1, g.Type());          // a colour input is equalized on its gray
    }

    [Fact]
    public void Clahe_raises_local_contrast_on_a_gray_output()
    {
        // tileSize is the tile grid: 1 = one tile, i.e. equalization clipped at clipLimit.
        using var img = FilterKit.Gray(64, 64, 100);
        Cv2.Rectangle(img, new Rect(32, 0, 32, 64), Scalar.All(110), -1);
        using var o = FilterKit.Apply(img, "contrast", new { mode = "clahe", clipLimit = 40, tileSize = 1 });
        Assert.Equal(MatType.CV_8UC1, o.Type());
        int before = 10, after = FilterKit.G(o, 48, 32) - FilterKit.G(o, 16, 32);
        Assert.True(after > 2 * before, $"clahe step {after}");
        using var low = FilterKit.Apply(img, "contrast", new { mode = "clahe", clipLimit = 1, tileSize = 1 });
        Assert.True(FilterKit.G(low, 48, 32) - FilterKit.G(low, 16, 32) < after, "a lower clip limit stretches less");
    }
}

public class EdgesFilterTests
{
    private static Mat Step(int a = 30, int b = 220)
    {
        var img = FilterKit.Gray(20, 10, a);
        Cv2.Rectangle(img, new Rect(10, 0, 10, 10), Scalar.All(b), -1);
        return img;
    }

    [Fact]
    public void Sobel_is_a_quarter_of_the_gradient_magnitude()
    {
        using var img = Step();
        using var o = FilterKit.Apply(img, "edges", new { mode = "sobel" });
        // At the step's two columns the 3 × 3 Sobel sees 4 × 190 = 760; × ¼ = 190.
        Assert.Equal(190, FilterKit.G(o, 9, 5));
        Assert.Equal(190, FilterKit.G(o, 10, 5));
        Assert.Equal(0, FilterKit.G(o, 5, 5));
        Assert.Equal(0, FilterKit.G(o, 15, 5));
        using var dflt = FilterKit.Apply(img, "edges");
        FilterKit.AssertSame(o, dflt);                     // sobel by default
        // A horizontal step reads the same through gy.
        using var rot = new Mat();
        Cv2.Transpose(img, rot);
        using var oy = FilterKit.Apply(rot, "edges");
        Assert.Equal(190, FilterKit.G(oy, 5, 9));
    }

    [Fact]
    public void Canny_draws_a_thin_line_and_honours_its_thresholds()
    {
        using var img = Step();
        using var o = FilterKit.Apply(img, "edges", new { mode = "canny", low = 50, high = 150 });
        Assert.Equal(10, Cv2.CountNonZero(o));            // one pixel per row
        Assert.All(Enumerable.Range(0, 10), y => Assert.True(FilterKit.G(o, 9, y) == 255 || FilterKit.G(o, 10, y) == 255));
        using var faint = Step(30, 50);                     // gradient 20·4 = 80
        using var none = FilterKit.Apply(faint, "edges", new { mode = "canny", low = 50, high = 150 });
        Assert.Equal(0, Cv2.CountNonZero(none));
        using var some = FilterKit.Apply(faint, "edges", new { mode = "canny", low = 20, high = 60 });
        Assert.Equal(10, Cv2.CountNonZero(some));
    }

    [Fact]
    public void A_colour_input_gives_a_gray_edge_map()
    {
        using var img = FilterKit.Bgr(20, 10, Scalar.All(30));
        Cv2.Rectangle(img, new Rect(10, 0, 10, 10), Scalar.All(220), -1);
        using var o = FilterKit.Apply(img, "edges");
        Assert.Equal(MatType.CV_8UC1, o.Type());
        Assert.Equal(190, FilterKit.G(o, 10, 5));
    }
}

public class BackgroundRemoveFilterTests
{
    [Fact]
    public void A_flat_image_becomes_128()
    {
        using var img = FilterKit.Gray(40, 40, 77);
        using var o = FilterKit.Apply(img, "backgroundRemove", new { size = 15 });
        Assert.Equal(MatType.CV_8UC1, o.Type());
        Assert.Equal(40 * 40, Cv2.CountNonZero(o));
        Cv2.MinMaxLoc(o, out double min, out double max);
        Assert.Equal(128, min);
        Assert.Equal(128, max);
    }

    [Fact]
    public void Shading_is_flattened_and_small_detail_kept()
    {
        using var img = new Mat(100, 200, MatType.CV_8UC1);
        for (int x = 0; x < 200; x++) Cv2.Line(img, new Point(x, 0), new Point(x, 99), Scalar.All(40 + x / 2));
        Cv2.Circle(img, new Point(100, 50), 3, Scalar.All(250), -1);
        Cv2.Circle(img, new Point(150, 50), 3, Scalar.All(10), -1);
        using var o = FilterKit.Apply(img, "backgroundRemove", new { size = 31 });
        using var band = new Mat(o, new Rect(20, 10, 50, 80));
        Cv2.MinMaxLoc(band, out double lo, out double hi);
        Assert.InRange(lo, 126, 130);                      // a linear ramp is its own blur: nothing left
        Assert.InRange(hi, 126, 130);
        Assert.True(FilterKit.G(o, 100, 50) > 200, "a bright speck stays bright");
        Assert.True(FilterKit.G(o, 150, 50) < 60, "a dark speck stays dark");
    }

    [Fact]
    public void Size_below_3_is_held_at_3_and_colour_is_converted()
    {
        using var img = FilterKit.Bgr(40, 40, Scalar.All(90));
        using var o = FilterKit.Apply(img, "backgroundRemove", new { size = 1 });
        Assert.Equal(MatType.CV_8UC1, o.Type());
        Assert.Equal(128, FilterKit.G(o, 20, 20));
        using var spot = FilterKit.Gray(40, 40, 100);
        spot.Set(20, 20, (byte)200);
        using var s = FilterKit.Apply(spot, "backgroundRemove", new { size = 1 });
        Assert.InRange(FilterKit.G(s, 20, 20), 202, 204);   // 128 + 200 − (a 3 px blur of the spot: 125)
    }
}

public class InvertFilterTests
{
    [Fact]
    public void Gray_and_colour_are_flipped()
    {
        using var g = FilterKit.Gray(3, 3, 30);
        using var og = FilterKit.Apply(g, "invert");
        Assert.Equal(225, FilterKit.G(og, 1, 1));
        using var c = FilterKit.Bgr(3, 3, new Scalar(10, 20, 30));
        using var oc = FilterKit.Apply(c, "invert");
        Assert.Equal(new Vec3b(245, 235, 225), FilterKit.C(oc, 1, 1));
        using var twice = FilterKit.Apply(oc, "invert");
        FilterKit.AssertSame(c, twice);
    }
}

/// <summary>Filter stacks: disabled filters, unknown types, chaining layers and the upTo preview.</summary>
public class LayerPipelineTests
{
    private static readonly (string Type, object? P)[] Every =
    {
        ("channel", new { channel = "r" }), ("colorMask", new { hMin = 0, hMax = 10 }), ("blur", new { size = 9 }),
        ("median", new { size = 5 }), ("threshold", new { mode = "otsu" }), ("morph", new { op = "dilate", size = 5 }),
        ("contrast", new { mode = "equalize" }), ("edges", new { mode = "canny" }), ("backgroundRemove", null), ("invert", null),
        ("sharpen", null),   // even an unknown type is skipped when disabled
    };

    public static IEnumerable<object[]> Types() => Every.Select((e, i) => new object[] { i });

    [Theory]
    [MemberData(nameof(Types))]
    public void A_disabled_filter_is_skipped(int index)
    {
        var (type, p) = Every[index];
        using var img = Canvas(30, 60, 40);
        Cv2.Circle(img, new Point(20, 15), 8, new Scalar(10, 50, 230), -1);
        var f = Filter(type, p);
        f.Enabled = false;
        using var o = VisionFilters.ApplyAll(img, new[] { f }, 1);
        FilterKit.AssertSame(img, o);
        Assert.False(ReferenceEquals(img, o));            // always a copy
        if (type != "sharpen")
        {
            f.Enabled = true;
            using var on = VisionFilters.ApplyAll(img, new[] { f }, 1);
            using var a = on.Channels() == 1 ? on.Clone() : on.CvtColor(ColorConversionCodes.BGR2GRAY);
            using var b = img.CvtColor(ColorConversionCodes.BGR2GRAY);
            using var diff = new Mat();
            Cv2.Absdiff(a, b, diff);
            Assert.True(Cv2.CountNonZero(diff) > 0, $"{type} changed nothing when enabled");
        }
    }

    [Fact]
    public void An_unknown_enabled_filter_is_an_error_for_the_tools_that_read_it()
    {
        using var img = Canvas(100);
        var prog = Program(Tool("i", "intensity", null, null, input: "bad"), Tool("ok", "intensity"));
        prog.Layers.Add(new ImageLayer { Id = "bad", Filters = { Filter("sharpen") } });
        var r = Run(prog, img);
        Assert.Equal(ToolStatus.Error, r.Tool("i")!.Status);
        Assert.Contains("sharpen", r.Tool("i")!.Message);
        Assert.Contains(r.Errors, e => e.StartsWith("i:"));
        Assert.Equal(ToolStatus.Ok, r.Tool("ok")!.Status);
        Assert.False(r.Passed);
    }

    /// <summary>camera → "gray" (channel) → "bin" (threshold, disabled blur, invert) → "thick" (dilate).</summary>
    private static ImageLayer[] Chain()
    {
        var gray = new ImageLayer { Id = "gray", Input = "camera", Filters = { Filter("channel", new { channel = "gray" }) } };
        var bin = new ImageLayer
        {
            Id = "bin", Input = "gray",
            Filters = { Filter("threshold", new { mode = "fixed", low = 150, high = 255 }), Filter("blur", new { size = 31 }), Filter("invert") },
        };
        bin.Filters[1].Enabled = false;
        var thick = new ImageLayer { Id = "thick", Input = "bin", Filters = { Filter("morph", new { op = "dilate", size = 3 }) } };
        return new[] { gray, bin, thick };
    }

    /// <summary>100 everywhere, a 200 square over x 100..149, y 100..149.</summary>
    private static Mat Scene()
    {
        var img = Canvas(100);
        Box(img, 100, 100, 50, 50, Scalar.All(200));
        return img;
    }

    [Fact]
    public void Layers_chain_each_built_once_in_dependency_order()
    {
        using var img = Scene();
        using var cache = new LayerCache(img, Chain());
        var thick = cache.Get("thick");
        Assert.Equal(new[] { "gray", "bin", "thick" }, cache.Computed);
        Assert.Same(thick, cache.Get("thick"));
        Assert.Same(cache.Get("bin"), cache.Get("bin"));
        Assert.Equal(3, cache.Computed.Count);                     // nothing rebuilt
        Assert.Equal(3, cache.Timings.Count);
        // bin: the square (≥ 150) is 255, inverted → 0; the blur was skipped so the edge stays sharp.
        var bin = cache.Get("bin");
        Assert.Equal(255, FilterKit.G(bin, 50, 50));
        Assert.Equal(0, FilterKit.G(bin, 120, 120));
        Assert.Equal(400 * 300 - 2500, Cv2.CountNonZero(bin));
        // thick: the white background grown one pixel into the dark square, leaving its 48 × 48 inside.
        Assert.Equal(400 * 300 - 48 * 48, Cv2.CountNonZero(thick));
        Assert.Same(cache.Source, cache.Get("camera"));
        Assert.Same(cache.Source, cache.Get(""));
    }

    [Fact]
    public void A_tool_reads_the_end_of_a_chain()
    {
        using var img = Scene();
        var prog = Program(
            Tool("cam", "intensity", Rect(100, 100, 50, 50)),
            Tool("bin", "intensity", Rect(100, 100, 50, 50), input: "bin"),
            Tool("thick", "pixelCount", Rect(90, 90, 70, 70), new { mode = "gray", grayMin = 255, grayMax = 255 }, input: "thick"));
        prog.Layers.AddRange(Chain());
        var r = Run(prog, img);
        Assert.Equal(200, Num(r.Tool("cam")!, "mean"));
        Assert.Equal(0, Num(r.Tool("bin")!, "mean"));
        Assert.Equal((70.0 * 70 - 48 * 48) / (70 * 70) * 100, Num(r.Tool("thick")!, "percent"), 2);
        Assert.Equal(new[] { "gray", "bin", "thick" }, r.Layers);
        Assert.All(new[] { "gray", "bin", "thick" }, id => Assert.True(r.Timings.ContainsKey($"layer:{id}")));
    }

    [Fact]
    public void A_layer_no_tool_reads_is_not_built()
    {
        using var img = Scene();
        var prog = Program(Tool("bin", "intensity", null, input: "bin"));
        prog.Layers.AddRange(Chain());
        var r = Run(prog, img);
        Assert.Equal(new[] { "gray", "bin" }, r.Layers);
        Assert.False(r.Timings.ContainsKey("layer:thick"));
    }

    [Theory]
    [InlineData(-1, 100)]      // the input (gray) as it is
    [InlineData(0, 0)]         // threshold only: the background is below 150
    [InlineData(1, 0)]         // + the disabled blur: no change
    [InlineData(2, 255)]       // + invert
    [InlineData(99, 255)]      // past the end: every filter
    [InlineData(null, 255)]
    public void Preview_stops_after_filter_upTo(int? upTo, int background)
    {
        using var img = Scene();
        using var cache = new LayerCache(img, Chain());
        using var p = cache.Preview("bin", upTo);
        Assert.Equal(background, FilterKit.G(p, 50, 50));
        Assert.Equal(MatType.CV_8UC1, p.Type());
        Assert.DoesNotContain("bin", cache.Computed);            // a preview is not the cached layer
        Assert.Throws<InvalidOperationException>(() => cache.Preview("nope", 0));
    }

    [Fact]
    public void Preview_through_the_controller_endpoint()
    {
        using var img = Scene();
        Cv2.ImEncode(".png", img, out var png);
        var prog = Program();
        prog.Layers.AddRange(Chain());
        using var input = Cv2.ImDecode(VisionProcessor.RenderLayer(prog, png, "bin", -1)!, ImreadModes.Grayscale);
        using var first = Cv2.ImDecode(VisionProcessor.RenderLayer(prog, png, "bin", 0)!, ImreadModes.Grayscale);
        using var all = Cv2.ImDecode(VisionProcessor.RenderLayer(prog, png, "bin", null)!, ImreadModes.Grayscale);
        using var cam = Cv2.ImDecode(VisionProcessor.RenderLayer(prog, png, "camera", null)!, ImreadModes.Grayscale);
        Assert.InRange((int)FilterKit.G(input, 50, 50), 95, 105);   // JPEG: near enough
        Assert.InRange((int)FilterKit.G(first, 50, 50), 0, 8);
        Assert.InRange((int)FilterKit.G(all, 50, 50), 247, 255);
        Assert.InRange((int)FilterKit.G(cam, 120, 120), 195, 205);
        Assert.Null(VisionProcessor.RenderLayer(prog, null, "bin", 0));
    }
}
