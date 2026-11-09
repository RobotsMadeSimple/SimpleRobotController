using System.Text.Json;

namespace Controller.RobotControl.Plugins;

/// <summary>A plugin expression function as the expression engine sees it.</summary>
/// <param name="FullName"><c>id.name</c>.</param>
public sealed record PluginFunctionInfo(
    string PluginId, string Name, string FullName, int MinArgs, int MaxArgs,
    string Signature, string Description, int TimeoutMs, bool Running);

/// <summary>
/// A plugin function call failed. <see cref="Code"/> is <c>pluginNotRunning</c>,
/// <c>pluginFunctionTimeout</c>, <c>pluginFunctionFailed</c> or <c>unknownPluginFunction</c>.
/// </summary>
public sealed class PluginFunctionException : Exception
{
    public string Code { get; }
    public PluginFunctionException(string code, string message) : base(message) => Code = code;
}

/// <summary>A step or function targeted a plugin that is not running.</summary>
public sealed class PluginNotRunningException : Exception
{
    public string PluginId { get; }
    public PluginNotRunningException(string pluginId) : base($"Plugin '{pluginId}' is not running") => PluginId = pluginId;
}

/// <summary>What <c>step.execute</c> carries besides the ids (values already resolved per docs/plugins.md §6).</summary>
public sealed class PluginStepRequest
{
    public string ProgramName { get; set; } = "";
    public string? StepName { get; set; }
    public Dictionary<string, object?> Params { get; set; } = new(StringComparer.Ordinal);
    public bool IsBackground { get; set; }
}

/// <summary>The outcome of a <c>step.execute</c>.</summary>
public sealed class PluginStepReply
{
    public bool Ok { get; init; }
    public Dictionary<string, JsonElement> Outputs { get; init; } = new(StringComparer.Ordinal);
    /// <summary>Error code when not <see cref="Ok"/> (the plugin's, or <c>pluginDisconnected</c>).</summary>
    public string? Error { get; init; }
    public string? Message { get; init; }
}

/// <summary>Result of <c>variables.get</c>: <c>{ variables, lists, strings }</c>.</summary>
public sealed class VariablesSnapshot
{
    public Dictionary<string, double> Variables { get; set; } = new();
    public Dictionary<string, object?> Lists { get; set; } = new();
    public Dictionary<string, string> Strings { get; set; } = new();
}

/// <summary>Robot state sampled by the event poll for <c>robot.position</c> and the transition events.</summary>
public sealed record PluginRobotSnapshot(
    double X, double Y, double Z, double RX, double RY, double RZ, bool Moving,
    bool Homed, bool Faulted, int FaultJoint, string? FaultMessage);

/// <summary><c>GetPluginContributions</c> entry for a step.</summary>
public sealed record PluginStepContribution(string PluginId, string PluginName, bool Running, PluginStepDef Step);

/// <summary><c>GetPluginContributions</c> entry for a function.</summary>
public sealed record PluginFunctionContribution(string PluginId, string PluginName, bool Running, PluginFunctionDef Function);

/// <summary><c>GetPluginContributions</c> entry for a property (<c>value</c> while known).</summary>
public sealed record PluginPropertyContribution(string PluginId, string PluginName, bool Running, PluginPropertyDef Property,
    [property: System.Text.Json.Serialization.JsonIgnore(Condition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull)] double? Value);

/// <summary>Everything the builder needs from plugins in one call (docs/plugins.md §7).</summary>
public sealed record PluginContributions(
    List<PluginStepContribution> Steps,
    List<PluginFunctionContribution> Functions,
    List<PluginPropertyContribution> Properties);

/// <summary>Outcome of an install from a zip.</summary>
/// <param name="Error"><c>badZip</c>, <c>badManifest</c> or <c>idExists</c> when not ok.</param>
public sealed record PluginInstallResult(bool Ok, string? Id, PluginHost? Host, string? Error, string? Message)
{
    public static PluginInstallResult Fail(string error, string message) => new(false, null, null, error, message);
}
