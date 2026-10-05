namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>The coordinate frame a RunVision step writes positions in (<c>ProgramStep.OutputFrame</c>).</summary>
    public enum OutputFrameKind
    {
        /// <summary>No outputFrame: blob points in pixels, polygon/ArUco centers normalized (the original behaviour).</summary>
        Default,
        Pixel,
        Normalized,
        Robot,
    }

    /// <summary>
    /// Converts the vision results' positions into a RunVision step's output frame. Blob
    /// centers arrive in pixels, polygon and ArUco centers normalized (0–1).
    /// </summary>
    public sealed class VisionOutputFrame
    {
        public const string Pixel      = "pixel";
        public const string Normalized = "normalized";
        public const string Robot      = "robot";

        public OutputFrameKind   Kind        { get; }
        public int               Width       { get; }
        public int               Height      { get; }
        public CameraCalibration? Calibration { get; }

        public VisionOutputFrame(OutputFrameKind kind, int width, int height, CameraCalibration? calibration = null)
        {
            if (kind == OutputFrameKind.Robot && calibration == null)
                throw new ArgumentException("The robot frame needs a calibration", nameof(calibration));
            Kind        = kind;
            Width       = width;
            Height      = height;
            Calibration = calibration;
        }

        /// <summary>Parses a step's outputFrame; null/empty is <see cref="OutputFrameKind.Default"/>. Case-sensitive, as on the wire.</summary>
        public static bool TryParse(string? value, out OutputFrameKind kind)
        {
            kind = value switch
            {
                null or ""  => OutputFrameKind.Default,
                Pixel       => OutputFrameKind.Pixel,
                Normalized  => OutputFrameKind.Normalized,
                Robot       => OutputFrameKind.Robot,
                _           => (OutputFrameKind)(-1),
            };
            return Enum.IsDefined(kind);
        }

        /// <summary>A position given in pixels (blob centers). Z is the calibration plane in the robot frame, else 0.</summary>
        public (double X, double Y, double Z) FromPixel(double px, double py) => Kind switch
        {
            OutputFrameKind.Normalized => (Width > 0 ? px / Width : px, Height > 0 ? py / Height : py, 0),
            OutputFrameKind.Robot      => Calibration!.PixelPxToRobot(px, py),
            _                          => (px, py, 0),
        };

        /// <summary>A position given normalized 0–1 (polygon and ArUco centers).</summary>
        public (double X, double Y, double Z) FromNormalized(double u, double v) => Kind switch
        {
            OutputFrameKind.Pixel => (u * Width, v * Height, 0),
            OutputFrameKind.Robot => Calibration!.PixelPxToRobot(u * Width, v * Height),
            _                     => (u, v, 0),
        };
    }
}
