using System.Text.Json;
using System.Text.Json.Serialization;

namespace SimpleRobot.PluginSdk;

/// <summary>Log levels understood by the controller's plugin log.</summary>
public enum LogLevel { Debug, Info, Warn, Error }

/// <summary>A robot pose as exchanged with the controller (<c>{x,y,z,rx,ry,rz}</c>).</summary>
public readonly record struct PluginPoint(
    [property: JsonPropertyName("x")] double X,
    [property: JsonPropertyName("y")] double Y,
    [property: JsonPropertyName("z")] double Z,
    [property: JsonPropertyName("rx")] double RX,
    [property: JsonPropertyName("ry")] double RY,
    [property: JsonPropertyName("rz")] double RZ);

/// <summary>Base class for SDK exceptions.</summary>
public class PluginException : Exception
{
    public PluginException(string message, Exception? inner = null) : base(message, inner) { }
}

/// <summary>The controller refused the token (WebSocket close code 4401).</summary>
public sealed class PluginAuthException : PluginException
{
    public PluginAuthException(string message) : base(message) { }
}

/// <summary>Another connection for this plugin replaced ours (close code 4409).</summary>
public sealed class PluginReplacedException : PluginException
{
    public PluginReplacedException(string message) : base(message) { }
}

/// <summary>The connection was lost while a request was outstanding.</summary>
public sealed class PluginConnectionException : PluginException
{
    public PluginConnectionException(string message, Exception? inner = null) : base(message, inner) { }
}

/// <summary>A <c>controller.command</c> (or other request) failed. <see cref="Code"/> is the controller's error code.</summary>
public sealed class CommandException : PluginException
{
    public string Code { get; }
    public CommandException(string code, string message) : base(message) { Code = code; }
}

/// <summary>Throw from a step handler to fail the step with a specific error code; the program fails with <see cref="Exception.Message"/>.</summary>
public sealed class StepException : Exception
{
    public string Code { get; }
    public StepException(string message, string code = "stepFailed") : base(message) { Code = code; }
}

/// <summary>Result of <c>variables.get</c>.</summary>
public sealed class VariablesSnapshot
{
    public IReadOnlyDictionary<string, JsonElement> Variables { get; init; } = new Dictionary<string, JsonElement>();
    public IReadOnlyDictionary<string, JsonElement> Lists { get; init; } = new Dictionary<string, JsonElement>();
    public IReadOnlyDictionary<string, JsonElement> Strings { get; init; } = new Dictionary<string, JsonElement>();
}

/// <summary>Marks a method as the handler of a plugin step. Signature: <c>(IPluginContext, StepParams) → [Task&lt;]StepResult[&gt;]</c>.</summary>
[AttributeUsage(AttributeTargets.Method)]
public sealed class PluginStepAttribute(string id) : Attribute { public string Id { get; } = id; }

/// <summary>Marks a method as an expression function. Signature: <c>(IPluginContext, double[]) → [Task&lt;]double[&gt;]</c>.</summary>
[AttributeUsage(AttributeTargets.Method)]
public sealed class PluginFunctionAttribute(string name) : Attribute { public string Name { get; } = name; }

/// <summary>Marks a method as an event handler. Signature: <c>(IPluginContext, JsonElement) → Task|void</c>.</summary>
[AttributeUsage(AttributeTargets.Method, AllowMultiple = true)]
public sealed class PluginEventAttribute(string eventName) : Attribute { public string EventName { get; } = eventName; }
