using System.Text.Json;
using Controller.RobotControl.Plugins;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// <c>variables.get</c> / <c>variables.set</c> for plugins (docs/plugins.md §4.3): find the
    /// executor holding a program (foreground first, then a running background one) or use
    /// the global store when no program is named.
    /// </summary>
    internal static class PluginVariables
    {
        /// <summary>The executor whose variables belong to <paramref name="programName"/>, or null.</summary>
        public static ProgramExecutor? FindExecutor(string programName, ProgramExecutor? foreground, BackgroundProgramManager? background)
        {
            if (foreground?.CurrentProgramName is { } fg && string.Equals(fg, programName, StringComparison.OrdinalIgnoreCase))
                return foreground;
            return background?.FindRunning(programName);
        }

        /// <summary>The live snapshot for a program, the globals when <paramref name="programName"/> is null,
        /// or null for a program no executor holds.</summary>
        public static VariablesSnapshot? Get(string? programName, ProgramExecutor? foreground, BackgroundProgramManager? background)
        {
            if (programName is null)
                return new VariablesSnapshot { Variables = background?.GlobalVars.Snapshot() ?? new() };
            return FindExecutor(programName, foreground, background)?.PluginSnapshot();
        }

        /// <summary>Applies the writes; null on success, else <c>unknownProgram</c>, <c>computedVariable</c> or <c>badValue</c>.</summary>
        public static string? Set(string? programName, Dictionary<string, JsonElement> values,
                                  ProgramExecutor? foreground, BackgroundProgramManager? background)
        {
            if (programName is null)
            {
                var globals = background?.GlobalVars;
                if (globals is null) return "unknownProgram";
                var scalars = new List<(string, double)>();
                foreach (var (name, v) in values)
                {
                    if (globals.TryGetComputed(name, out _)) return "computedVariable";
                    if (Scalar(v) is not { } d) return "badValue"; // the global store holds numbers only
                    scalars.Add((name, d));
                }
                foreach (var (name, d) in scalars) globals.Set(name, d);
                return null;
            }
            var executor = FindExecutor(programName, foreground, background);
            return executor is null ? "unknownProgram" : executor.PluginSetVariables(values);
        }

        public static double? Scalar(JsonElement v) => v.ValueKind switch
        {
            JsonValueKind.Number => v.TryGetDouble(out var d) && double.IsFinite(d) ? d : null,
            JsonValueKind.True   => 1,
            JsonValueKind.False  => 0,
            _                    => null,
        };

        /// <summary>
        /// The write <paramref name="value"/> means for <paramref name="name"/> (not yet applied), or
        /// null for a value that has no variable form: numbers/booleans → Set, strings → SetString,
        /// arrays → SetList (a declared list keeps its element type), a point object → a
        /// one-element points list.
        /// </summary>
        public static Action<VariableScope>? Write(string name, JsonElement value)
        {
            if (Scalar(value) is { } d) return vars => vars.Set(name, d);
            switch (value.ValueKind)
            {
                case JsonValueKind.String:
                {
                    var s = value.GetString() ?? "";
                    return vars => vars.SetString(name, s);
                }
                case JsonValueKind.Array:
                {
                    var arr = value.Clone();
                    var inferred = PluginSteps.InferElementType(arr, null);
                    if (inferred is null) return null;
                    return vars => vars.SetList(name, JsonVariableCodec.ListFromJson(arr,
                        vars.Lists.TryGetValue(name, out var existing) ? existing.ElementType : inferred.Value));
                }
                case JsonValueKind.Object:
                {
                    if (!PluginSteps.TryPoint(value, out var p)) return null;
                    return vars => vars.SetList(name, ListVar.OfPoints([p]));
                }
            }
            return null;
        }
    }
}
