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
