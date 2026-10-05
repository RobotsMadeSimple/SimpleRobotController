using System.Text.Json.Serialization;

namespace Controller.RobotControl.UsbRelay
{
    public class RelayConfig
    {
        /// <summary>Display names for each relay channel. Index 0 = relay 1.</summary>
        [JsonPropertyName("relayNames")]
        public List<string> RelayNames { get; set; } =
            Enumerable.Range(1, UsbRelayDevice.RelayCount).Select(i => $"Relay {i}").ToList();
    }

    public static class RelayConfigService
    {
        private static readonly string ConfigFilePath = "relay-config.json";

        public static RelayConfig Load()
        {
            var cfg = Persistence.JsonFiles.Load<RelayConfig>(ConfigFilePath, logTag: "RelayConfig");
            if (cfg != null)
            {
                // Ensure exactly RelayCount names (pad or truncate if file was hand-edited)
                while (cfg.RelayNames.Count < UsbRelayDevice.RelayCount)
                    cfg.RelayNames.Add($"Relay {cfg.RelayNames.Count + 1}");
                cfg.RelayNames = cfg.RelayNames.Take(UsbRelayDevice.RelayCount).ToList();
                return cfg;
            }

            var defaults = new RelayConfig();
            Save(defaults);
            return defaults;
        }

        public static void Save(RelayConfig config)
        {
            try
            {
                Persistence.JsonFiles.Save(ConfigFilePath, config);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[RelayConfig] Failed to write: {ex.Message}");
            }
        }
    }
}
