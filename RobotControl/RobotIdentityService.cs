using System;
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
        private static readonly string IdentityFilePath = "identity.json";

        public static RobotIdentity Load()
        {
            var identity = Persistence.JsonFiles.Load<RobotIdentity>(IdentityFilePath, logTag: "Identity");
            if (identity != null)
                return identity;

            var newIdentity = new RobotIdentity
            {
                SerialNumber = GenerateSerialNumber(),
                RobotType    = "ASTRO",
                RobotName    = "ASTRO"
            };

            Save(newIdentity);
            Console.WriteLine($"[Identity] Generated new serial number: {newIdentity.SerialNumber}");
            return newIdentity;
        }

        public static void Save(RobotIdentity identity)
        {
            try
            {
                Persistence.JsonFiles.Save(IdentityFilePath, identity);
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
