using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>
    /// One <see cref="CameraCalibration"/> per camera, stored as
    /// <c>&lt;dir&gt;/&lt;cameraId&gt;.json</c> (default <c>cameraCalibrations/</c> under the
    /// data directory). Loaded once, written atomically; thread-safe.
    /// </summary>
    public sealed class CameraCalibrationRepository
    {
        private readonly string _dir;
        private readonly object _lock = new();
        private readonly Dictionary<string, CameraCalibration> _items = new(StringComparer.Ordinal);

        public CameraCalibrationRepository(string dir = "cameraCalibrations")
        {
            _dir = dir;
            Load();
        }

        private void Load()
        {
            if (!Directory.Exists(_dir)) return;
            foreach (var file in Directory.GetFiles(_dir, "*.json"))
            {
                var cal = JsonFiles.Load<CameraCalibration>(file, logTag: "CameraCalibration");
                if (cal != null && !string.IsNullOrEmpty(cal.CameraId))
                    _items[cal.CameraId] = cal;
            }
        }

        private string FilePath(string cameraId)
        {
            var safe = string.Concat(cameraId.Split(Path.GetInvalidFileNameChars()));
            return Path.Combine(_dir, safe + ".json");
        }

        public CameraCalibration? Get(string? cameraId)
        {
            if (string.IsNullOrEmpty(cameraId)) return null;
            lock (_lock) return _items.TryGetValue(cameraId, out var c) ? c : null;
        }

        public bool IsCalibrated(string? cameraId) => Get(cameraId) != null;

        public List<CameraCalibration> List()
        {
            lock (_lock) return _items.Values.OrderBy(c => c.CameraId, StringComparer.Ordinal).ToList();
        }

        public void Save(CameraCalibration calibration)
        {
            if (string.IsNullOrEmpty(calibration.CameraId))
                throw new ArgumentException("Calibration has no cameraId");
            lock (_lock)
            {
                Directory.CreateDirectory(_dir);
                JsonFiles.Save(FilePath(calibration.CameraId), calibration);
                _items[calibration.CameraId] = calibration;
            }
        }

        public bool Delete(string cameraId)
        {
            lock (_lock)
            {
                bool had = _items.Remove(cameraId);
                var path = FilePath(cameraId);
                if (File.Exists(path)) { File.Delete(path); had = true; }
                return had;
            }
        }
    }
}
