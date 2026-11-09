using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace Controller.RobotControl.Vision
{
    public class VisionProgramRepository
    {
        private readonly string _dir;
        private readonly object _lock = new();
        private Dictionary<string, VisionProgram> _programs = new();

        private static readonly JsonSerializerOptions _opts = new()
        {
            WriteIndented           = true,
            PropertyNameCaseInsensitive = true,
        };

        public long LastUpdatedUnixMs { get; private set; }

        public VisionProgramRepository(string dir = "vision_programs")
        {
            _dir = dir;
            Load();
        }

        private void Load()
        {
            if (!Directory.Exists(_dir)) return;
            foreach (var file in Directory.GetFiles(_dir, "*.json"))
            {
                try
                {
                    var text = File.ReadAllText(file);
                    if (IsOldFormat(text))
                    {
                        // Zone + typed-inspection programs are not converted (docs/vision-tools.md):
                        // left on disk untouched, but not loaded. Saving a program with the same id
                        // overwrites the file.
                        Console.WriteLine($"[Vision] Skipping {Path.GetFileName(file)}: old zone/inspection format — rebuild it with tools");
                        continue;
                    }
                    var prog = JsonSerializer.Deserialize<VisionProgram>(text, _opts);
                    if (prog != null)
                        _programs[prog.Id] = prog;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Vision] Failed to load {file}: {ex.Message}");
                }
            }
        }

        /// <summary>A program saved by the zone + inspection model: no "tools", but zones or inspections.</summary>
        internal static bool IsOldFormat(string json)
        {
            try
            {
                using var doc = JsonDocument.Parse(json);
                var root = doc.RootElement;
                if (root.ValueKind != JsonValueKind.Object || root.TryGetProperty("tools", out _)) return false;
                foreach (var p in root.EnumerateObject())
                    if (p.Name == "zones" || p.Name.EndsWith("nspections", StringComparison.Ordinal)) return true;
                return false;
            }
            catch (JsonException) { return false; }
        }

        private string FilePath(string id)
        {
            var safe = string.Concat(id.Split(Path.GetInvalidFileNameChars()));
            return Path.Combine(_dir, safe + ".json");
        }

        public void Save(VisionProgram program)
        {
            lock (_lock)
            {
                program.LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                _programs[program.Id] = program;
                Directory.CreateDirectory(_dir);
                Persistence.AtomicFile.WriteAllText(FilePath(program.Id), JsonSerializer.Serialize(program, _opts));
                LastUpdatedUnixMs = program.LastUpdatedUnixMs;
            }
        }

        public bool Delete(string id)
        {
            lock (_lock)
            {
                if (!_programs.Remove(id)) return false;
                var path = FilePath(id);
                if (File.Exists(path)) File.Delete(path);
                LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                return true;
            }
        }

        public VisionProgram? Get(string id)
        {
            lock (_lock) { _programs.TryGetValue(id, out var p); return p; }
        }

        public List<VisionProgram> GetAll()
        {
            lock (_lock) { return _programs.Values.ToList(); }
        }
    }
}
