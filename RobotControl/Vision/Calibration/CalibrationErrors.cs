namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>The error codes of the calibration commands (docs/camera-calibration.md).</summary>
    public static class CalibrationErrors
    {
        public const string UnknownCamera      = "unknownCamera";
        public const string CameraNotConnected = "cameraNotConnected";
        public const string NoDotsFound        = "noDotsFound";
        public const string GridNotFound       = "gridNotFound";
        public const string UnknownSession     = "unknownSession";
        public const string UnknownDot         = "unknownDot";
        public const string NotEnoughTaught    = "notEnoughTaught";
        public const string TaughtCollinear    = "taughtCollinear";
        public const string NotCalibrated      = "notCalibrated";
        /// <summary>Beyond the contract: a missing or out-of-range parameter (e.g. dotPitchMm ≤ 0).</summary>
        public const string InvalidParams      = "invalidParams";
    }

    /// <summary>A calibration failure carrying one of the <see cref="CalibrationErrors"/> codes.</summary>
    public sealed class CalibrationException : Exception
    {
        public string Code { get; }

        public CalibrationException(string code, string message) : base(message) => Code = code;
    }
}
