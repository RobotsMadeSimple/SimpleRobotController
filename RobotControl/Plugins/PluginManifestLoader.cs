using System.Text.Json;

namespace Controller.RobotControl.Plugins;

/// <summary>Result of reading one plugin folder: the manifest (null when unreadable) and every problem found.</summary>
public sealed record PluginManifestLoadResult(string FolderName, PluginManifest? Manifest, IReadOnlyList<ManifestProblem> Problems)
{
    public bool IsValid => Manifest != null && Problems.Count == 0;
}

/// <summary>Reads and validates <c>plugins/&lt;id&gt;/plugin.json</c>.</summary>
public static class PluginManifestLoader
{
    public const string ManifestFileName = "plugin.json";

    /// <summary>Loads the manifest of <paramref name="pluginFolder"/>; also checks that the folder name equals the id.</summary>
    public static PluginManifestLoadResult Load(string pluginFolder, Func<string, bool>? isFunctionName = null)
    {
        string folderName = Path.GetFileName(Path.TrimEndingDirectorySeparator(pluginFolder));
        string path       = Path.Combine(pluginFolder, ManifestFileName);
        if (!File.Exists(path))
            return new(folderName, null, [new ManifestProblem("missingManifest", $"{ManifestFileName} not found")]);

        string json;
        try { json = File.ReadAllText(path); }
        catch (Exception ex) { return new(folderName, null, [new ManifestProblem("badManifest", ex.Message)]); }

        var result = Parse(json, isFunctionName);
        if (result.Manifest is { } m && !string.Equals(m.Id, folderName, StringComparison.Ordinal))
        {
            var problems = result.Problems.ToList();
            problems.Add(new ManifestProblem("idMismatch", $"folder '{folderName}' does not match id '{m.Id}'", "id"));
            return new(folderName, m, problems);
        }
        return result with { FolderName = folderName };
    }

    /// <summary>Parses and validates manifest JSON text (no folder checks).</summary>
    public static PluginManifestLoadResult Parse(string json, Func<string, bool>? isFunctionName = null)
    {
        PluginManifest manifest;
        try { manifest = PluginManifest.Parse(json); }
        catch (Exception ex) when (ex is JsonException or NotSupportedException or InvalidOperationException)
        {
            return new("", null, [new ManifestProblem("badManifest", $"plugin.json is not valid: {ex.Message}")]);
        }
        return new(manifest.Id, manifest, manifest.Validate(isFunctionName));
    }
}
