using System;
using System.Text.Json.Serialization;

namespace Controller.RobotControl
{
    public class RobotConfig
    {
        /// <summary>Selects the kinematics model: "ASTRO" (default) or "CNC4Axis".</summary>
        [JsonPropertyName("robotType")]
        public string RobotType { get; set; } = "ASTRO";

        /// <summary>Speed used for all axes during the fast homing approach (mm/s or deg/s).</summary>
        [JsonPropertyName("homingSpeed")]
        public double HomingSpeed { get; set; } = 20;

        /// <summary>Speed used for the slow second-pass homing approach (mm/s or deg/s).</summary>
        [JsonPropertyName("homingSlowSpeed")]
        public double HomingSlowSpeed { get; set; } = 5;

        /// <summary>Distance to back off from the sensor before the slow approach (mm, or deg for J1).</summary>
        [JsonPropertyName("homingBackoffMm")]
        public double HomingBackoffMm { get; set; } = 10;

        /// <summary>J1 angle (degrees) applied once the limit switch is hit.</summary>
        [JsonPropertyName("j1HomeOffsetDeg")]
        public double J1HomeOffsetDeg { get; set; } = 107;

        /// <summary>Z height (mm) applied once the vertical limit switch is hit.</summary>
        [JsonPropertyName("verticalHomePosition")]
        public double VerticalHomePosition { get; set; } = 378;

        /// <summary>Horizontal distance (mm) applied once the horizontal limit switch is hit.</summary>
        [JsonPropertyName("horizontalHomePosition")]
        public double HorizontalHomePosition { get; set; } = 435;

        /// <summary>Direction to jog during vertical homing (1 = positive Z, -1 = negative Z).</summary>
        [JsonPropertyName("verticalHomingDirection")]
        public int VerticalHomingDirection { get; set; } = 1;

        /// <summary>Direction to jog during horizontal homing (1 = positive Y, -1 = negative Y).</summary>
        [JsonPropertyName("horizontalHomingDirection")]
        public int HorizontalHomingDirection { get; set; } = 1;

        /// <summary>Direction to jog during J1 homing (1 = positive, -1 = negative).</summary>
        [JsonPropertyName("j1HomingDirection")]
        public int J1HomingDirection { get; set; } = 1;

        /// <summary>J4 angle (degrees) applied once J4 has driven to its mechanical zero and stopped.</summary>
        [JsonPropertyName("j4HomeOffsetDeg")]
        public double J4HomeOffsetDeg { get; set; } = 0;

        // ── Motor directions ─────────────────────────────────────────────────────

        /// <summary>Physical inversion for M1 (J1 rotation motor). 1 = normal, -1 = inverted.</summary>
        [JsonPropertyName("m1Direction")]
        public int M1Direction { get; set; } = 1;

        /// <summary>Physical inversion for M2 (CoreXY motor A). 1 = normal, -1 = inverted.</summary>
        [JsonPropertyName("m2Direction")]
        public int M2Direction { get; set; } = 1;

        /// <summary>Physical inversion for M3 (CoreXY motor B). 1 = normal, -1 = inverted.</summary>
        [JsonPropertyName("m3Direction")]
        public int M3Direction { get; set; } = -1;

        /// <summary>Physical inversion for M4 (J4 rotation motor). 1 = normal, -1 = inverted.</summary>
        [JsonPropertyName("m4Direction")]
        public int M4Direction { get; set; } = -1;

        // ── Network discovery ─────────────────────────────────────────────────

        /// <summary>Advertise this robot on the network via mDNS. Requires a restart to take effect.</summary>
        [JsonPropertyName("enableMdns")]
        public bool EnableMdns { get; set; } = true;

        // ── IO card visibility ────────────────────────────────────────────────

        /// <summary>Show Arduino Nano IO cards in the app.</summary>
        [JsonPropertyName("enableNanoCards")]
        public bool EnableNanoCards { get; set; } = true;

        /// <summary>Show the USB relay board card in the app.</summary>
        [JsonPropertyName("enableRelayCard")]
        public bool EnableRelayCard { get; set; } = false;

        /// <summary>Show the Aux Axis stepper card in the app.</summary>
        [JsonPropertyName("enableAuxAxis")]
        public bool EnableAuxAxis { get; set; } = false;

        /// <summary>Show USB camera cards in the app.</summary>
        [JsonPropertyName("enableCameras")]
        public bool EnableCameras { get; set; } = false;

        // ── Jog speeds ────────────────────────────────────────────────────────────────

        /// <summary>Slow jog speed (mm/s or deg/s).</summary>
        [JsonPropertyName("jogSlowSpeed")]
        public double JogSlowSpeed { get; set; } = 10;

        /// <summary>Normal jog speed (mm/s or deg/s).</summary>
        [JsonPropertyName("jogNormalSpeed")]
        public double JogNormalSpeed { get; set; } = 100;

        /// <summary>Fast jog speed (mm/s or deg/s).</summary>
        [JsonPropertyName("jogFastSpeed")]
        public double JogFastSpeed { get; set; } = 300;

        // ── ASTRO motor configuration ─────────────────────────────────────────────────
        // Microstep resolution and gear ratio (motor turns per output turn) per motor.
        // Defaults mirror the values STB4100 has always constructed its motors with, so an
        // existing robot behaves identically until these are deliberately changed.
        // M1 = J1 base rotation, M2/M3 = CoreXY pair, M4 = J4 EOAT rotation.

        /// <summary>Steps per revolution for M1 (J1 base rotation).</summary>
        [JsonPropertyName("astroStepsPerRevM1")] public int AstroStepsPerRevM1 { get; set; } = 1600;
        /// <summary>Steps per revolution for M2 (CoreXY motor A).</summary>
        [JsonPropertyName("astroStepsPerRevM2")] public int AstroStepsPerRevM2 { get; set; } = 1600;
        /// <summary>Steps per revolution for M3 (CoreXY motor B).</summary>
        [JsonPropertyName("astroStepsPerRevM3")] public int AstroStepsPerRevM3 { get; set; } = 1600;
        /// <summary>Steps per revolution for M4 (J4 EOAT rotation).</summary>
        [JsonPropertyName("astroStepsPerRevM4")] public int AstroStepsPerRevM4 { get; set; } = 400;

        /// <summary>Gear ratio (motor turns per output turn) for M1.</summary>
        [JsonPropertyName("astroGearRatioM1")] public double AstroGearRatioM1 { get; set; } = 1.0;
        /// <summary>Gear ratio for M2.</summary>
        [JsonPropertyName("astroGearRatioM2")] public double AstroGearRatioM2 { get; set; } = 1.0;
        /// <summary>Gear ratio for M3.</summary>
        [JsonPropertyName("astroGearRatioM3")] public double AstroGearRatioM3 { get; set; } = 1.0;
        /// <summary>Gear ratio for M4.</summary>
        [JsonPropertyName("astroGearRatioM4")] public double AstroGearRatioM4 { get; set; } = 1.0;

        // ── ASTRO joint gearing ───────────────────────────────────────────────────────
        // The drivetrain reduction the kinematics uses to turn joint motion into motor
        // motion, separate from the per-motor gear ratio above. Defaults mirror the values
        // ASTROKinematics has always constructed its joints with.

        /// <summary>J1 base rotation gear ratio (motor turns per joint turn). Default 120t/30t.</summary>
        [JsonPropertyName("astroJoint1GearRatio")] public double AstroJoint1GearRatio { get; set; } = 120.0 / 30.0;
        /// <summary>J4 EOAT rotation gear ratio (motor turns per joint turn).</summary>
        [JsonPropertyName("astroJoint4GearRatio")] public double AstroJoint4GearRatio { get; set; } = 10.0;
        /// <summary>CoreXY belt pulley pitch diameter (mm). Linear travel per motor rev = π·PCD.</summary>
        [JsonPropertyName("astroCoreXyPulleyPcdMm")] public double AstroCoreXyPulleyPcdMm { get; set; } = 19.099;

        // ── CNC4Axis motor configuration ──────────────────────────────────────────────

        /// <summary>Motor steps per revolution for the X axis (microstepping setting).</summary>
        [JsonPropertyName("cncStepsPerRevX")]
        public int CncStepsPerRevX { get; set; } = 1600;

        /// <summary>Motor steps per revolution for the Y axis.</summary>
        [JsonPropertyName("cncStepsPerRevY")]
        public int CncStepsPerRevY { get; set; } = 1600;

        /// <summary>Motor steps per revolution for the Z axis.</summary>
        [JsonPropertyName("cncStepsPerRevZ")]
        public int CncStepsPerRevZ { get; set; } = 1600;

        /// <summary>Motor steps per revolution for the RZ spindle axis.</summary>
        [JsonPropertyName("cncStepsPerRevRZ")]
        public int CncStepsPerRevRZ { get; set; } = 1600;

        /// <summary>Leadscrew travel in mm per motor revolution for the X axis.</summary>
        [JsonPropertyName("cncMmPerRevX")]
        public double CncMmPerRevX { get; set; } = 5.0;

        /// <summary>Leadscrew travel in mm per motor revolution for the Y axis.</summary>
        [JsonPropertyName("cncMmPerRevY")]
        public double CncMmPerRevY { get; set; } = 5.0;

        /// <summary>Leadscrew travel in mm per motor revolution for the Z axis.</summary>
        [JsonPropertyName("cncMmPerRevZ")]
        public double CncMmPerRevZ { get; set; } = 5.0;

        /// <summary>Degrees of RZ output rotation per motor revolution (gear ratio).</summary>
        [JsonPropertyName("cncDegPerRevRZ")]
        public double CncDegPerRevRZ { get; set; } = 360.0;

        // ── CNC4Axis home positions ───────────────────────────────────────────────

        /// <summary>X coordinate (mm) set when the X axis limit switch is hit during homing.</summary>
        [JsonPropertyName("cncXHomePosition")]
        public double CncXHomePosition { get; set; } = 0;

        /// <summary>Y coordinate (mm) set when the Y axis limit switch is hit during homing.</summary>
        [JsonPropertyName("cncYHomePosition")]
        public double CncYHomePosition { get; set; } = 0;

        /// <summary>Z coordinate (mm) set when the Z axis limit switch is hit during homing.</summary>
        [JsonPropertyName("cncZHomePosition")]
        public double CncZHomePosition { get; set; } = 0;

        /// <summary>RZ angle (deg) applied at the end of CNC homing (no limit switch — just zeroed).</summary>
        [JsonPropertyName("cncRzHomePosition")]
        public double CncRzHomePosition { get; set; } = 0;

        // ── CNC4Axis homing directions ────────────────────────────────────────────

        /// <summary>Direction to jog during X homing (1 = positive, -1 = negative).</summary>
        [JsonPropertyName("cncXHomingDirection")]
        public int CncXHomingDirection { get; set; } = -1;

        /// <summary>Direction to jog during Y homing (1 = positive, -1 = negative).</summary>
        [JsonPropertyName("cncYHomingDirection")]
        public int CncYHomingDirection { get; set; } = -1;

        /// <summary>Direction to jog during Z homing (1 = positive, -1 = negative).</summary>
        [JsonPropertyName("cncZHomingDirection")]
        public int CncZHomingDirection { get; set; } = 1;

        // ── Joint soft limits ───────────────────────────────────────────────────
        // Applied to the joint-space target vector: joint1 = X component
        // (ASTRO J1 base rotation °, CNC X mm), joint2 = Y (ASTRO radial reach mm,
        // CNC Y mm), joint3 = Z (ASTRO vertical mm, CNC Z mm), joint4 = RZ
        // (ASTRO J4 EOAT °, CNC RZ °). When enabled, a commanded move that would
        // cross a set limit faults the robot; recovery requires operator bypass.
        //
        // Each bound is nullable and defaults to unset (null) — an unset bound is
        // simply not enforced, so nothing is limited until the operator dials one in.

        /// <summary>Master switch for joint soft limits. Off by default.</summary>
        [JsonPropertyName("jointLimitsEnabled")]
        public bool JointLimitsEnabled { get; set; } = false;

        [JsonPropertyName("joint1Min")] public double? Joint1Min { get; set; }
        [JsonPropertyName("joint1Max")] public double? Joint1Max { get; set; }
        [JsonPropertyName("joint2Min")] public double? Joint2Min { get; set; }
        [JsonPropertyName("joint2Max")] public double? Joint2Max { get; set; }
        [JsonPropertyName("joint3Min")] public double? Joint3Min { get; set; }
        [JsonPropertyName("joint3Max")] public double? Joint3Max { get; set; }
        [JsonPropertyName("joint4Min")] public double? Joint4Min { get; set; }
        [JsonPropertyName("joint4Max")] public double? Joint4Max { get; set; }
    }

    public static class RobotConfigService
    {
        private static readonly string ConfigFilePath = "robot-config.json";

        public static RobotConfig Load()
        {
            var config = Persistence.JsonFiles.Load<RobotConfig>(ConfigFilePath, logTag: "Config");
            if (config != null)
            {
                Console.WriteLine("[Config] Loaded robot-config.json");
                return config;
            }

            var defaults = new RobotConfig();
            Save(defaults);
            Console.WriteLine("[Config] Created robot-config.json with default values.");
            return defaults;
        }

        public static void Save(RobotConfig config)
        {
            try
            {
                Persistence.JsonFiles.Save(ConfigFilePath, config);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Config] Failed to write robot-config.json: {ex.Message}");
            }
        }
    }
}
