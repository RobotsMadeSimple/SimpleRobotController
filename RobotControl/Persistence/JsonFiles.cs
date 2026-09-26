using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Persistence
{
    /// <summary>Shared serializer settings so every on-disk file uses the same conventions.</summary>
    public static class JsonDefaults
    {
        /// <summary>Indented, case-insensitive on read, enums as strings. Use for every data/config file.</summary>
        public static readonly JsonSerializerOptions File = new()
        {
            WriteIndented               = true,
            PropertyNameCaseInsensitive = true,
            Converters                  = { new JsonStringEnumConverter() },
        };
    }

    /// <summary>
    /// Load/save helpers for JSON-backed data files. A corrupt file is never silently
    /// overwritten: it is moved aside as <c>name.corrupt-timestamp</c> so the data can
    /// be recovered by hand, and the caller starts from its default.
    /// </summary>
    public static class JsonFiles
    {
        /// <summary>Returns the deserialized file, or null if it does not exist or is unreadable.</summary>
        public static T? Load<T>(string path, JsonSerializerOptions? options = null, string? logTag = null) where T : class
        {
            if (!System.IO.File.Exists(path)) return null;
            try
            {
                var json = System.IO.File.ReadAllText(path);
                return JsonSerializer.Deserialize<T>(json, options ?? JsonDefaults.File);
            }
            catch (Exception ex)
            {
                QuarantineCorrupt(path, ex, logTag);
                return null;
            }
        }

        /// <summary>Serializes and writes the value atomically.</summary>
        public static void Save<T>(string path, T value, JsonSerializerOptions? options = null)
        {
            var json = JsonSerializer.Serialize(value, options ?? JsonDefaults.File);
            AtomicFile.WriteAllText(path, json);
        }

        /// <summary>Moves an unreadable file aside so it is preserved instead of overwritten.</summary>
        public static void QuarantineCorrupt(string path, Exception ex, string? logTag = null)
        {
            var stamp = DateTimeOffset.UtcNow.ToString("yyyyMMdd-HHmmss");
            var aside = $"{path}.corrupt-{stamp}";
            try { System.IO.File.Move(path, aside, overwrite: true); } catch { /* best effort */ }
            Console.WriteLine($"[{logTag ?? "JsonFiles"}] Could not read {Path.GetFileName(path)} ({ex.Message}); moved to {Path.GetFileName(aside)} and starting from defaults.");
        }
    }
}
