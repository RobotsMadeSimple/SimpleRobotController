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
                File.WriteAllText(FilePath(program.Id), JsonSerializer.Serialize(program, _opts));
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
