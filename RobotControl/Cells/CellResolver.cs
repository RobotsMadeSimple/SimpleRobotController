using System.Text.Json.Nodes;

namespace Controller.RobotControl.Cells;

/// <summary>
/// Builds what <c>GET /cells/active</c> and <c>GET /cells/{id}</c> return (docs/cell-package.md §4):
/// the package's cell.json with <c>robot</c> filled from the built-in for the controller's robot type
/// when absent/null, plus <c>settings</c> — the current value of every adjustable setting the robot
/// declares.
/// </summary>
public static class CellResolver
{
    /// <summary>The robot settings an <c>adjustable</c> axis may name, read from the live config.</summary>
    public static double? SettingValue(RobotConfig config, string? setting) => setting switch
    {
        "astroVerticalTravelMm" => config.AstroVerticalTravelMm,
        "astroRadialTravelMm"   => config.AstroRadialTravelMm,
        _                       => null,
    };

    /// <summary>
    /// Resolves <paramref name="cell"/> (a stored cell.json, not modified) for this controller. A null
    /// cell resolves to the built-in robot alone: <c>id</c>/<c>name</c> null, no tools or fixtures.
    /// </summary>
    public static JsonObject Resolve(JsonObject? cell, RobotConfig config)
    {
        var resolved = cell is null
            ? new JsonObject
            {
                ["format"]   = CellPackageValidator.Format,
                ["version"]  = CellPackageValidator.Version,
                ["id"]       = null,
                ["name"]     = null,
                ["robot"]    = null,
                ["tools"]    = new JsonArray(),
                ["fixtures"] = new JsonArray(),
            }
            : (JsonObject)cell.DeepClone();

        if (resolved["robot"] is not JsonObject)
            resolved["robot"] = BuiltInRobots.For(config.RobotType);
        resolved["tools"]    ??= new JsonArray();
        resolved["fixtures"] ??= new JsonArray();

        // settings: one entry per adjustable axis. A setting the controller doesn't know reads as
        // the axis's nominal, so it draws unchanged.
        var settings = new JsonObject();
        if (resolved["robot"] is JsonObject robot && robot["adjustable"] is JsonArray adjustable)
        {
            foreach (var axis in adjustable.OfType<JsonObject>())
            {
                string? setting = CellPackageValidator.Str(axis, "setting");
                if (string.IsNullOrEmpty(setting) || settings.ContainsKey(setting)) continue;
                double? nominal = axis["nominal"] is JsonValue n && n.TryGetValue(out double nv) ? nv : null;
                double? value = SettingValue(config, setting) ?? nominal;
                settings[setting] = value is { } v ? JsonValue.Create(v) : null;
            }
        }
        resolved["settings"] = settings;
        return resolved;
    }
}
