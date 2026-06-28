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

        public void StartProgram(string programId)
        {
            lock (_lock)
            {
                if (_processors.ContainsKey(programId)) return;

                var prog = _repo.Get(programId);
                if (prog == null) { Console.WriteLine($"[Vision] Program '{programId}' not found"); return; }

                var camera = _cameraManager.GetCamera(prog.CameraId);
                if (camera == null) { Console.WriteLine($"[Vision] Camera '{prog.CameraId}' not found"); return; }

                var proc = new VisionProcessor(prog, camera);
                _processors[programId] = proc;
                proc.Start();
                Console.WriteLine($"[Vision] Started processor for '{programId}'");
            }
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
