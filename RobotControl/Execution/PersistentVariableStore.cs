using System.Text.Json;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// The on-disk store behind persistent program variables: one flat JSON object of
    /// <c>"programId:name" → value</c>, shared by every executor in the process.
    /// </summary>
    /// <remarks>
    /// Best-effort by design: a missing or unreadable file loads as empty and a failed
    /// write is dropped, because a persistence hiccup must never stop a running machine.
    /// </remarks>
    internal static class PersistentVariableStore
    {
        private const string FilePath = "persistent_vars.json";
        private static readonly object Lock = new();

        public static Dictionary<string, double> Load()
        {
            lock (Lock)
            {
                try
                {
                    if (!File.Exists(FilePath)) return new();
                    var json = File.ReadAllText(FilePath);
                    return JsonSerializer.Deserialize<Dictionary<string, double>>(json) ?? new();
                }
                catch { return new(); }
            }
        }

        public static void Write(Dictionary<string, double> values)
        {
            lock (Lock)
            {
                try
                {
                    File.WriteAllText(FilePath, JsonSerializer.Serialize(values));
                }
                catch { /* best-effort */ }
            }
        }
    }
}
