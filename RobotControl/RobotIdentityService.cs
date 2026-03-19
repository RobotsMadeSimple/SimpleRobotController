using System;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl
{
    public class RobotIdentity
    {
        [JsonPropertyName("serialNumber")]
        public string SerialNumber { get; set; } = string.Empty;

        [JsonPropertyName("robotType")]
        public string RobotType { get; set; } = string.Empty;

        [JsonPropertyName("robotName")]
        public string RobotName { get; set; } = string.Empty;
    }

    public static class RobotIdentityService
    {
        private static readonly string IdentityFilePath =
            Path.Combine(AppContext.BaseDirectory, "identity.json");

        private static readonly JsonSerializerOptions JsonOptions = new()
        {
            WriteIndented = true
        };

        public static RobotIdentity Load()
        {
            if (File.Exists(IdentityFilePath))
            {
                try
                {
                    string json = File.ReadAllText(IdentityFilePath);
                    var identity = JsonSerializer.Deserialize<RobotIdentity>(json, JsonOptions);
                    if (identity != null)
                        return identity;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Identity] Failed to read identity.json: {ex.Message}. Regenerating.");
                }
            }

            var newIdentity = new RobotIdentity
            {
                SerialNumber = GenerateSerialNumber(),
                RobotType    = string.Empty,
                RobotName    = string.Empty
            };

            Save(newIdentity);
            Console.WriteLine($"[Identity] Generated new serial number: {newIdentity.SerialNumber}");
            return newIdentity;
        }

        private static void Save(RobotIdentity identity)
        {
            try
            {
                string json = JsonSerializer.Serialize(identity, JsonOptions);
                File.WriteAllText(IdentityFilePath, json);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Identity] Failed to write identity.json: {ex.Message}");
            }
        }

        private static string GenerateSerialNumber()
        {
            const string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
            var random = new Random();
            var serial = new char[10];
            for (int i = 0; i < serial.Length; i++)
                serial[i] = chars[random.Next(chars.Length)];
            return new string(serial);
        }
    }
}
