using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    public class VisionManager
    {
        private readonly Camera.CameraManager          _cameraManager;
        private readonly VisionProgramRepository       _repo;
        private readonly Dictionary<string, VisionProcessor> _processors = new();
        private readonly object _lock = new();

        public VisionManager(Camera.CameraManager cameraManager, VisionProgramRepository repo)
        {
            _cameraManager = cameraManager;
            _repo          = repo;
        }

        public void StartProgram(string programId, string? zoneOverrideId = null)
        {
            lock (_lock)
            {
                if (_processors.ContainsKey(programId)) return;

                var prog = _repo.Get(programId);
                if (prog == null) { Console.WriteLine($"[Vision] Program '{programId}' not found"); return; }

                var camera = _cameraManager.GetCamera(prog.CameraId);
                if (camera == null) { Console.WriteLine($"[Vision] Camera '{prog.CameraId}' not found"); return; }

                // Runtime zone override: point every inspection at the chosen zone so the
                // analysis — and the annotated debug frame — run in that zone instead of
                // each inspection's saved default. Clone first so the repo copy is untouched.
                if (!string.IsNullOrEmpty(zoneOverrideId))
                    prog = WithZoneOverride(prog, zoneOverrideId);

                var proc = new VisionProcessor(prog, camera);
                _processors[programId] = proc;
                proc.Start();
                Console.WriteLine($"[Vision] Started processor for '{programId}'"
                    + (string.IsNullOrEmpty(zoneOverrideId) ? "" : $" (zone override {zoneOverrideId})"));
            }
        }

        /// <summary>Deep-clones a program and re-points every inspection at one zone.</summary>
        private static VisionProgram WithZoneOverride(VisionProgram prog, string zoneId)
        {
            var clone = System.Text.Json.JsonSerializer.Deserialize<VisionProgram>(
                System.Text.Json.JsonSerializer.Serialize(prog))!;
            foreach (var i in clone.Inspections)        i.ZoneId = zoneId;
            foreach (var i in clone.ColorInspections)   i.ZoneId = zoneId;
            foreach (var i in clone.PolygonInspections) i.ZoneId = zoneId;
            foreach (var i in clone.ArucoInspections)   i.ZoneId = zoneId;
            foreach (var i in clone.LineInspections)    i.ZoneId = zoneId;
            foreach (var i in clone.BarcodeInspections) i.ZoneId = zoneId;
            return clone;
        }

        public void StopProgram(string programId)
        {
            lock (_lock)
            {
                if (!_processors.TryGetValue(programId, out var proc)) return;
                proc.Stop();
                _processors.Remove(programId);
                Console.WriteLine($"[Vision] Stopped processor for '{programId}'");
            }
        }

        /// <summary>When a program is saved, hot-swap the definition into any running processor.</summary>
        public void OnProgramSaved(VisionProgram updated)
        {
            lock (_lock)
            {
                if (_processors.TryGetValue(updated.Id, out var proc))
                    proc.UpdateProgram(updated);
            }
        }

        public VisionProcessor? GetProcessor(string programId)
        {
            lock (_lock) { _processors.TryGetValue(programId, out var p); return p; }
        }

        public VisionProgram? GetProgram(string programId) => _repo.Get(programId);

        public List<string> GetRunningIds()
        {
            lock (_lock) { return _processors.Keys.ToList(); }
        }

        public void StopAll()
        {
            lock (_lock)
            {
                foreach (var proc in _processors.Values) proc.Stop();
                _processors.Clear();
            }
        }
    }
}
