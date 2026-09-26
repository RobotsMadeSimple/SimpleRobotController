using System.Text.Json;

namespace Controller.RobotControl.Persistence
{
    /// <summary>An item that can live in a <see cref="JsonListRepository{T}"/>.</summary>
    public interface IStoredItem
    {
        string Id { get; set; }
        long LastUpdatedUnixMs { get; set; }
    }

    /// <summary>
    /// Thread-safe, id-keyed collection persisted as a single JSON array file.
    /// Assigns a GUID id on first save and stamps LastUpdatedUnixMs on every upsert.
    /// </summary>
    public class JsonListRepository<T> where T : class, IStoredItem
    {
        private readonly string _file;
        private readonly string _logTag;
        private readonly object _lock = new();
        private readonly JsonSerializerOptions _opts;
        private Dictionary<string, T> _items = new();

        /// <summary>Unix ms of the last successful save (0 until the first save).</summary>
        public long LastUpdatedUnixMs { get; private set; }

        public JsonListRepository(string file, JsonSerializerOptions? options = null)
        {
            _file   = file;
            _logTag = typeof(T).Name + "Repository";
            _opts   = options ?? JsonDefaults.File;
            Load();
        }

        private void Load()
        {
            var list = JsonFiles.Load<List<T>>(_file, _opts, _logTag);
            if (list == null) return;
            _items = list
                .Where(i => !string.IsNullOrWhiteSpace(i.Id))
                .GroupBy(i => i.Id)
                .ToDictionary(g => g.Key, g => g.Last());
        }

        private void Save()
        {
            JsonFiles.Save(_file, _items.Values.ToList(), _opts);
            LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        }

        public T Upsert(T item)
        {
            lock (_lock)
            {
                if (string.IsNullOrWhiteSpace(item.Id))
                    item.Id = Guid.NewGuid().ToString();
                item.LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                _items[item.Id] = item;
                Save();
                return item;
            }
        }

        public bool Delete(string id)
        {
            lock (_lock)
            {
                if (!_items.Remove(id)) return false;
                Save();
                return true;
            }
        }

        public T? Get(string id)
        {
            lock (_lock) return _items.TryGetValue(id, out var item) ? item : null;
        }

        public List<T> GetAll()
        {
            lock (_lock) return _items.Values.ToList();
        }
    }
}
