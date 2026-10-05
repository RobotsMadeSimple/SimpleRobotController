using System.Text.Json.Serialization;

namespace Controller.RobotControl.Vision.Calibration
{
    public sealed class RobotXyz
    {
        [JsonPropertyName("x")] public double X { get; set; }
        [JsonPropertyName("y")] public double Y { get; set; }
        [JsonPropertyName("z")] public double Z { get; set; }
    }

    /// <summary>A dot the robot tip was taught on, as stored with the calibration.</summary>
    public sealed class CalibrationTaughtDot
    {
        [JsonPropertyName("dotIndex")] public int      DotIndex { get; set; }
        [JsonPropertyName("i")]        public int      I        { get; set; }
        [JsonPropertyName("j")]        public int      J        { get; set; }
        /// <summary>Normalized image position (0–1) of the dot when it was taught.</summary>
        [JsonPropertyName("u")]        public double   U        { get; set; }
        [JsonPropertyName("v")]        public double   V        { get; set; }
        [JsonPropertyName("robot")]    public RobotXyz Robot    { get; set; } = new();
        /// <summary>Residual of this dot after the rigid fit, mm.</summary>
        [JsonPropertyName("errorMm")]  public double   ErrorMm  { get; set; }
    }

    /// <summary>
    /// A camera's pixel → robot mapping on the calibration plane — the persisted model of
    /// docs/camera-calibration.md (<c>cameraCalibrations/&lt;cameraId&gt;.json</c>).
    /// </summary>
    public sealed class CameraCalibration
    {
        [JsonPropertyName("cameraId")]           public string     CameraId           { get; set; } = "";
        [JsonPropertyName("imageWidth")]         public int        ImageWidth         { get; set; }
        [JsonPropertyName("imageHeight")]        public int        ImageHeight        { get; set; }
        [JsonPropertyName("dotPitchMm")]         public double     DotPitchMm         { get; set; }
        /// <summary>Homography pixel (px) → sheet mm.</summary>
        [JsonPropertyName("pixelToSheet")]       public double[][] PixelToSheet       { get; set; } = Homography.Identity();
        /// <summary>Rigid 2-D sheet mm → robot mm.</summary>
        [JsonPropertyName("sheetToRobot")]       public RigidTransform2D SheetToRobot { get; set; } = new();
        /// <summary>Composed homography pixel (px) → robot mm.</summary>
        [JsonPropertyName("pixelToRobot")]       public double[][] PixelToRobotH      { get; set; } = Homography.Identity();
        [JsonPropertyName("planeZ")]             public double     PlaneZ             { get; set; }
        [JsonPropertyName("taughtDots")]         public List<CalibrationTaughtDot> TaughtDots { get; set; } = new();
        [JsonPropertyName("gridRows")]           public int        GridRows           { get; set; }
        [JsonPropertyName("gridCols")]           public int        GridCols           { get; set; }
        [JsonPropertyName("dotCount")]           public int        DotCount           { get; set; }
        [JsonPropertyName("gridRmsPx")]          public double     GridRmsPx          { get; set; }
        [JsonPropertyName("taughtRmsMm")]        public double     TaughtRmsMm        { get; set; }
        [JsonPropertyName("taughtMaxMm")]        public double     TaughtMaxMm        { get; set; }
        [JsonPropertyName("pitchScaleEstimate")] public double     PitchScaleEstimate { get; set; } = 1;
        /// <summary>The sheet → robot fit used a reflection.</summary>
        [JsonPropertyName("mirrored")]           public bool       Mirrored           { get; set; }
        [JsonPropertyName("activeTool")]         public string     ActiveTool         { get; set; } = "";
        [JsonPropertyName("calibratedUnixMs")]   public long       CalibratedUnixMs   { get; set; }

        private double[][]? _robotToPixel;

        /// <summary>Normalized image position (0–1) → robot (x, y) on the plane, z = <see cref="PlaneZ"/>.</summary>
        public (double X, double Y, double Z) PixelToRobot(double u, double v)
        {
            var (x, y) = Homography.Apply(PixelToRobotH, u * ImageWidth, v * ImageHeight);
            return (x, y, PlaneZ);
        }

        /// <summary>Pixel position (px) → robot (x, y, z).</summary>
        public (double X, double Y, double Z) PixelPxToRobot(double px, double py)
        {
            var (x, y) = Homography.Apply(PixelToRobotH, px, py);
            return (x, y, PlaneZ);
        }

        /// <summary>Robot (x, y) on the plane → normalized image position (0–1).</summary>
        public (double U, double V) RobotToPixel(double x, double y)
        {
            _robotToPixel ??= Homography.Invert(PixelToRobotH);
            var (px, py) = Homography.Apply(_robotToPixel, x, y);
            return (ImageWidth > 0 ? px / ImageWidth : px, ImageHeight > 0 ? py / ImageHeight : py);
        }
    }
}
