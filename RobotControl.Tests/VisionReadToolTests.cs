using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;

namespace RobotControl.Tests;

/// <summary>barcode: 1-D and 2-D codes read with ZXing.</summary>
public class BarcodeToolTests
{
    /// <summary>A code rendered by ZXing as a BGR image (black modules on white).</summary>
    internal static Mat Code(ZXing.BarcodeFormat format, string text, int w, int h, int margin = 2)
    {
        var writer = new ZXing.BarcodeWriterPixelData
        {
            Format = format,
            Options = new ZXing.Common.EncodingOptions { Width = w, Height = h, Margin = margin },
        };
        var px = writer.Write(text);
        using var bgra = new Mat(px.Height, px.Width, MatType.CV_8UC4);
        System.Runtime.InteropServices.Marshal.Copy(px.Pixels, 0, bgra.Data, px.Pixels.Length);
        var bgr = new Mat();
        Cv2.CvtColor(bgra, bgr, ColorConversionCodes.BGRA2BGR);
        return bgr;
    }

    /// <summary>A white canvas with the code's top-left at (x, y); returns its pixel box.</summary>
    private static Mat Scene(ZXing.BarcodeFormat format, string text, int w, int h, int x, int y, out Rect box, int margin = 2)
    {
        var img = Canvas(255);
        using var code = Code(format, text, w, h, margin);
        Paste(img, code, x, y);
        box = new Rect(x, y, code.Width, code.Height);
        return img;
    }

    [Theory]
    [InlineData(ZXing.BarcodeFormat.QR_CODE, "HELLO-42", 160, 160, 2)]
    [InlineData(ZXing.BarcodeFormat.CODE_128, "RMS-0042", 260, 80, 2)]
    [InlineData(ZXing.BarcodeFormat.CODE_39, "ABC123", 300, 80, 2)]
    [InlineData(ZXing.BarcodeFormat.EAN_13, "5901234123457", 260, 90, 2)]
    [InlineData(ZXing.BarcodeFormat.DATA_MATRIX, "ASTRO", 200, 200, 4)]   // ZXing's Data Matrix detector wants a big, quiet code
    public void Reads_each_format_with_its_value_and_position(ZXing.BarcodeFormat format, string text, int w, int h, int margin)
    {
        using var img = Scene(format, text, w, h, 70, 60, out var box, margin);
        var r = RunOne(img, Tool("q", "barcode"));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        Assert.Equal(text, r.Values["value"]);
        Assert.Equal(format.ToString(), r.Values["format"]);
        var it = Assert.Single(r.Items);
        Assert.Equal(text, it["value"]);
        Assert.Equal(format.ToString(), it["format"]);
        // x/y: the mean of ZXing's result points — inside the code.
        Assert.InRange(Num(r, "x"), box.X, box.Right);
        Assert.InRange(Num(r, "y"), box.Y, box.Bottom);
        Assert.Equal(D(it["x"]), Num(r, "x"), 2);
        Assert.NotEmpty(r.Lines);
        Assert.Null(r.Pose);
    }

    [Fact]
    public void A_1D_codes_position_is_its_middle()
    {
        using var img = Scene(ZXing.BarcodeFormat.CODE_128, "RMS-0042", 260, 80, 70, 100, out var box);
        var r = RunOne(img, Tool("q", "barcode", null, new { formats = new[] { "CODE_128" } }));
        Assert.InRange(Num(r, "x"), box.X + box.Width / 2.0 - 15, box.X + box.Width / 2.0 + 15);
    }

    [Theory]
    [InlineData(new[] { "QR_CODE" }, true)]
    [InlineData(new[] { "CODE_128", "QR_CODE" }, true)]
    [InlineData(new[] { "CODE_128" }, false)]
    [InlineData(new[] { "DATA_MATRIX", "EAN_13" }, false)]
    [InlineData(new string[0], true)]                 // empty = every format
    [InlineData(new[] { "NOT_A_FORMAT", "QR_CODE" }, true)]   // unknown names are ignored
    public void Formats_limit_what_is_read(string[] formats, bool read)
    {
        using var img = Scene(ZXing.BarcodeFormat.QR_CODE, "HELLO-42", 160, 160, 120, 70, out _);
        var r = RunOne(img, Tool("q", "barcode", null, new { formats }));
        Assert.Equal(read ? ToolStatus.Ok : ToolStatus.Fail, r.Status);
        if (read) Assert.Equal("HELLO-42", r.Values["value"]);
    }

    [Fact]
    public void Several_codes_are_all_listed()
    {
        using var img = Canvas(255);
        using (var a = Code(ZXing.BarcodeFormat.QR_CODE, "LEFT", 140, 140)) Paste(img, a, 20, 80);
        using (var b = Code(ZXing.BarcodeFormat.QR_CODE, "RIGHT", 140, 140)) Paste(img, b, 230, 80);
        var r = RunOne(img, Tool("q", "barcode", null, new { formats = new[] { "QR_CODE" } }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(2, Num(r, "count"));
        Assert.Equal(new[] { "LEFT", "RIGHT" }, r.Items.Select(i => (string)i["value"]).OrderBy(v => v));
        var left = r.Items.Single(i => (string)i["value"] == "LEFT");
        Assert.InRange(D(left["x"]), 20, 160);
        var right = r.Items.Single(i => (string)i["value"] == "RIGHT");
        Assert.InRange(D(right["x"]), 230, 370);
        Assert.Equal((string)r.Items[0]["value"], r.Values["value"]);
    }

    [Theory]
    [InlineData(90)]
    [InlineData(180)]
    [InlineData(30)]
    public void A_turned_qr_code_is_read(double deg)
    {
        using var img = Canvas(255);
        using (var code = Code(ZXing.BarcodeFormat.QR_CODE, "TURNED", 150, 150, 4))
        using (var turned = Rotated(code, deg, 255))
            Paste(img, turned, 125, 75);
        var r = RunOne(img, Tool("q", "barcode", null, new { formats = new[] { "QR_CODE" } }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal("TURNED", r.Values["value"]);
    }

    [Fact]
    public void An_inverted_code_is_read()
    {
        using var img = Scene(ZXing.BarcodeFormat.QR_CODE, "WHITE-ON-BLACK", 160, 160, 120, 70, out _);
        Cv2.BitwiseNot(img, img);
        var r = RunOne(img, Tool("q", "barcode", null, new { formats = new[] { "QR_CODE" } }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal("WHITE-ON-BLACK", r.Values["value"]);
    }

    [Fact]
    public void Only_the_regions_bounding_box_is_read()
    {
        using var img = Scene(ZXing.BarcodeFormat.QR_CODE, "HELLO-42", 160, 160, 220, 70, out var box);
        var miss = RunOne(img, Tool("q", "barcode", Rect(0, 0, 200, 300)));
        Assert.Equal(ToolStatus.Fail, miss.Status);
        Assert.Equal("no code read", miss.Message);
        var hit = RunOne(img, Tool("q", "barcode", Rect(box.X - 10, box.Y - 10, box.Width + 20, box.Height + 20)));
        Assert.True(hit.Ok, hit.Message);
        Assert.InRange(Num(hit, "x"), box.X, box.Right);       // still in image pixels
        var tiny = RunOne(img, Tool("q", "barcode", Rect(10, 10, 5, 5)));
        Assert.Equal(ToolStatus.Fail, tiny.Status);
        Assert.Contains("outside the image", tiny.Message);
    }

    [Fact]
    public void Nothing_to_read_fails_with_empty_values()
    {
        using var blank = Canvas(255);
        var r = RunOne(blank, Tool("q", "barcode"));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(0, Num(r, "count"));
        Assert.Equal("", r.Values["value"]);
        Assert.Empty(r.Items);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_count()
    {
        using var img = Scene(ZXing.BarcodeFormat.QR_CODE, "HELLO-42", 160, 160, 120, 70, out _);
        Assert.Equal(1, AssertLimitBoundaries(img, "q", "count", Single(() => Tool("q", "barcode", null, new { formats = new[] { "QR_CODE" } }))));
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(255);
        using (var code = Code(ZXing.BarcodeFormat.QR_CODE, "FOLLOW", 120, 120)) Paste(teach, code, 140, 110);
        Dots(teach);
        var (r0, r1) = TeachThenMoved(FixturedProgram(
            Tool("q", "barcode", Rect(140, 110, 120, 120), new { formats = new[] { "QR_CODE" } }, fixture: "fx"),
            Tool("still", "barcode", Rect(140, 110, 120, 120), new { formats = new[] { "QR_CODE" } })), teach, 255);
        Assert.Equal("FOLLOW", r0.Tool("still")!.Values["value"]);
        var q = r1.Tool("q")!;
        Assert.True(q.Ok, q.Message);
        Assert.Equal("FOLLOW", q.Values["value"]);
        // The region turned and shifted with the part; the unfixtured one stayed where drawn.
        Near(q.Region.Average(p => p.X), q.Region.Average(p => p.Y), Moved(200, 170), 0.5, "region centre");
        Near(r1.Tool("still")!.Region.Average(p => p.X), r1.Tool("still")!.Region.Average(p => p.Y), (200, 170), 0.01, "unfixtured region centre");
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("q", "barcode", Rect(135, 105, 130, 130)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        // Red modules on gray 76: one flat gray to the camera, a (white-on-black) code on the red layer.
        using var img = Canvas(RedGray);
        using (var code = Code(ZXing.BarcodeFormat.QR_CODE, "RED", 160, 160))
        {
            int ch = code.Height, cw = code.Width;
            for (int y = 0; y < ch; y++)
                for (int x = 0; x < cw; x++)
                    if (code.At<Vec3b>(y, x).Item0 < 128) img.Set(70 + y, 120 + x, new Vec3b(0, 0, 255));
        }
        var tool = Tool("q", "barcode", null, new { formats = new[] { "QR_CODE" } });
        Assert.Equal(ToolStatus.Fail, RunOne(img, tool).Status);
        tool.Input = "red";
        var prog = Program(tool);
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("q")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal("RED", r.Values["value"]);
    }
}
