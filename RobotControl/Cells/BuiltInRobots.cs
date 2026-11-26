using System.Text.Json.Nodes;

namespace Controller.RobotControl.Cells;

/// <summary>
/// The robot definitions a cell package gets when it leaves <c>robot</c> out (docs/cell-package.md §3.1),
/// embedded in the controller per robot type. ASTRO is FusionRobotSim's blocky <c>robots/astro.json</c>
/// plus the <c>adjustable</c> axes and <c>adjust</c> shape tags; CNC4Axis has none.
/// </summary>
public static class BuiltInRobots
{
    private static readonly Lazy<JsonObject> Astro = new(() => Load("Cells.Robots.astro.json"));

    /// <summary>A fresh copy of the built-in robot for <paramref name="robotType"/>, or null when there is none.</summary>
    public static JsonObject? For(string? robotType) =>
        string.Equals(robotType, "ASTRO", StringComparison.OrdinalIgnoreCase)
            ? (JsonObject)Astro.Value.DeepClone()
            : null;

    private static JsonObject Load(string resource)
    {
        using var stream = typeof(BuiltInRobots).Assembly.GetManifestResourceStream(resource)
            ?? throw new InvalidOperationException($"Embedded robot definition '{resource}' is missing");
        return JsonNode.Parse(stream)?.AsObject()
            ?? throw new InvalidOperationException($"Embedded robot definition '{resource}' is empty");
    }
}
