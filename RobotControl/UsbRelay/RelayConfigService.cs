using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.UsbRelay
{
    public class RelayConfig
    {
        /// <summary>Display names for each relay channel. Index 0 = relay 1.</summary>
        [JsonPropertyName("relayNames")]
        public List<string> RelayNames { get; set; } = new()
        {
            "Relay 1", "Relay 2", "Relay 3", "Relay 4"
        };
    }

    public static class RelayConfigService
    {
        private static readonly string ConfigFilePath = "relay-config.json";

        private static readonly JsonSerializerOptions JsonOptions = new()
        {
            WriteIndented = true
        };

        public static RelayConfig Load()
        {
            if (File.Exists(ConfigFilePath))
            {
                try
                {
                    string json = File.ReadAllText(ConfigFilePath);
                    var cfg = JsonSerializer.Deserialize<RelayConfig>(json, JsonOptions);
                    if (cfg != null)
                    {
                        // Ensure exactly 4 names (pad or truncate if file was hand-edited)
                        while (cfg.RelayNames.Count < 4)
                            cfg.RelayNames.Add($"Relay {cfg.RelayNames.Count + 1}");
                        cfg.RelayNames = cfg.RelayNames.Take(4).ToList();
                        return cfg;
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[RelayConfig] Failed to read: {ex.Message}. Using defaults.");
                }
            }

            var defaults = new RelayConfig();
            Save(defaults);
            return defaults;
        }

        public static void Save(RelayConfig config)
        {
            try
            {
                string json = JsonSerializer.Serialize(config, JsonOptions);
                File.WriteAllText(ConfigFilePath, json);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[RelayConfig] Failed to write: {ex.Message}");
            }
        }
    }
}
