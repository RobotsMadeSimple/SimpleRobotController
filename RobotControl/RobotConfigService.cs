using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl
{
    public class RobotConfig
    {
        /// <summary>Speed used for all axes during the homing sequence (mm/s or deg/s).</summary>
        [JsonPropertyName("homingSpeed")]
        public double HomingSpeed { get; set; } = 20;

        /// <summary>J1 angle (degrees) applied once the limit switch is hit.</summary>
        [JsonPropertyName("j1HomeOffsetDeg")]
        public double J1HomeOffsetDeg { get; set; } = -17;

        /// <summary>Z height (mm) applied once the vertical limit switch is hit.</summary>
        [JsonPropertyName("verticalHomePosition")]
        public double VerticalHomePosition { get; set; } = 445;

        /// <summary>Horizontal distance (mm) applied once the horizontal limit switch is hit.</summary>
        [JsonPropertyName("horizontalHomePosition")]
        public double HorizontalHomePosition { get; set; } = 413;

        /// <summary>Direction to jog during vertical homing (1 = positive Z, -1 = negative Z).</summary>
        [JsonPropertyName("verticalHomingDirection")]
        public int VerticalHomingDirection { get; set; } = 1;

        /// <summary>Direction to jog during horizontal homing (1 = positive Y, -1 = negative Y).</summary>
        [JsonPropertyName("horizontalHomingDirection")]
        public int HorizontalHomingDirection { get; set; } = 1;

        /// <summary>Direction to jog during J1 homing (1 = positive, -1 = negative).</summary>
        [JsonPropertyName("j1HomingDirection")]
        public int J1HomingDirection { get; set; } = -1;

        /// <summary>J4 angle (degrees) applied once J4 has driven to its mechanical zero and stopped.</summary>
        [JsonPropertyName("j4HomeOffsetDeg")]
        public double J4HomeOffsetDeg { get; set; } = 0;
    }

    public static class RobotConfigService
    {
        private static readonly string ConfigFilePath =
            Path.Combine(AppContext.BaseDirectory, "robot-config.json");

        private static readonly JsonSerializerOptions JsonOptions = new()
        {
            WriteIndented = true
        };

        public static RobotConfig Load()
        {
            if (File.Exists(ConfigFilePath))
            {
                try
                {
                    string json = File.ReadAllText(ConfigFilePath);
                    var config = JsonSerializer.Deserialize<RobotConfig>(json, JsonOptions);
                    if (config != null)
                    {
                        Console.WriteLine("[Config] Loaded robot-config.json");
                        return config;
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Config] Failed to read robot-config.json: {ex.Message}. Using defaults.");
                }
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
                string json = JsonSerializer.Serialize(config, JsonOptions);
                File.WriteAllText(ConfigFilePath, json);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Config] Failed to write robot-config.json: {ex.Message}");
            }
        }
    }
}
