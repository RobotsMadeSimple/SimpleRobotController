namespace Controller.RobotControl.Persistence
{
    /// <summary>
    /// Crash-safe file writes. Every write goes to a sibling temp file first and is
    /// then renamed over the target, so a power loss or crash mid-write leaves either
    /// the old file or the new file on disk, never a truncated one.
    /// </summary>
    public static class AtomicFile
    {
        public static void WriteAllText(string path, string contents)
        {
            var tmp = TempPathFor(path);
            File.WriteAllText(tmp, contents);
            File.Move(tmp, path, overwrite: true);
        }

        public static void WriteAllBytes(string path, byte[] bytes)
        {
            var tmp = TempPathFor(path);
            File.WriteAllBytes(tmp, bytes);
            File.Move(tmp, path, overwrite: true);
        }

        public static async Task WriteAllBytesAsync(string path, byte[] bytes, CancellationToken ct = default)
        {
            var tmp = TempPathFor(path);
            await File.WriteAllBytesAsync(tmp, bytes, ct);
            File.Move(tmp, path, overwrite: true);
        }

        private static string TempPathFor(string path)
        {
            var dir = Path.GetDirectoryName(Path.GetFullPath(path));
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
            return path + ".tmp";
        }
    }
}
