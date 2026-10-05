using System.Globalization;
using System.Text.Json;
using System.Text.RegularExpressions;
using Controller.RobotControl.Plugins;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// The <c>Plugin</c> step (docs/plugins.md §6): resolves the declared params, sends
    /// <c>step.execute</c> and yields until the reply arrives (queued onto the loop thread
    /// through <see cref="ExecutionContext.PendingActions"/>), then writes the mapped outputs.
    /// Never blocks the loop.
    /// </summary>
    internal static class PluginSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r) => r[StepType.Plugin] = new PluginStep();

        /// <summary><c>"&lt;plugin name&gt;: &lt;step label&gt;"</c> — or ids when the plugin/step is unknown.</summary>
        public static string Label(ProgramStep step, PluginManager? manager)
        {
            var host = manager?.Get(step.PluginId ?? "");
            var def  = host?.Manifest?.Steps.FirstOrDefault(s => string.Equals(s.Id, step.PluginStepId, StringComparison.Ordinal));
            string plugin = host?.Name ?? step.PluginId ?? "?";
            string label  = def != null ? (string.IsNullOrWhiteSpace(def.Label) ? def.Id : def.Label!) : step.PluginStepId ?? "?";
            return $"{plugin}: {label}";
        }

        private sealed class PluginStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var st = ctx.PluginState;

                if (!frame.WaitStarted || st.InvocationId == null)
                    return Start(step, frame, ctx);

                if (st.Reply is { } reply)
                {
                    string label = st.Label;
                    var def = st.Definition!;
                    st.Clear();
                    frame.WaitStarted = false;
                    if (!reply.Ok)
                        return ctx.Finish(ProgramStatus.Error, $"{label}: {reply.Message ?? reply.Error ?? "failed"}");
                    if (ApplyOutputs(step, def, reply, ctx.Vars, label) is { } error)
                        return ctx.Finish(ProgramStatus.Error, error);
                    return StepOutcome.Advance;
                }

                if (st.TimeoutMs > 0 && Environment.TickCount64 - st.StartedTick >= st.TimeoutMs)
                {
                    string stepLabel = st.StepLabel;
                    int timeout = st.TimeoutMs;
                    st.Reset("timeout");
                    frame.WaitStarted = false;
                    return ctx.Finish(ProgramStatus.Error, $"Plugin step '{stepLabel}' timed out after {timeout} ms");
                }
                return StepOutcome.Yield;
            }

            private static StepOutcome Start(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var manager = ctx.Plugins;
                string pluginId = step.PluginId ?? "";
                var host = manager?.Get(pluginId);
                if (host is null || host.Problems.Count > 0 || host.Manifest is not { } manifest)
                    return ctx.Finish(ProgramStatus.Error, $"Plugin '{pluginId}' is not installed");
                var def = manifest.Steps.FirstOrDefault(s => string.Equals(s.Id, step.PluginStepId, StringComparison.Ordinal));
                if (def is null)
                    return ctx.Finish(ProgramStatus.Error, $"Plugin '{host.Id}' has no step '{step.PluginStepId}'");
                if (!host.IsRunning)
                    return ctx.Finish(ProgramStatus.Error, $"Plugin '{host.Id}' is not running");

                string label = $"{host.Name}: {(string.IsNullOrWhiteSpace(def.Label) ? def.Id : def.Label)}";
                var request = new PluginStepRequest
                {
                    ProgramName  = ctx.Program?.Name ?? "",
                    StepName     = string.IsNullOrEmpty(step.Name) ? null : step.Name,
                    IsBackground = ctx.IsBackground,
                };
                foreach (var p in def.Params)
                {
                    var (ok, value, error) = ResolveParam(p, step.PluginParams, ctx, label);
                    if (!ok) return ctx.Finish(ProgramStatus.Error, error!);
                    if (value is not Omit) request.Params[p.Key] = value;
                }

                var st  = ctx.PluginState;
                int gen = ctx.RunGeneration;
                string? invocation = null;
                try
                {
                    // The reply lands on a thread-pool thread: hand it to the loop thread, which
                    // drops it when the run (or this invocation) is no longer the one waiting.
                    invocation = manager!.ExecuteStep(host.Id, def.Id, request, reply =>
                        ctx.PendingActions.Enqueue(() =>
                        {
                            if (gen == ctx.RunGeneration && st.InvocationId != null && st.InvocationId == invocation)
                                st.Reply = reply;
                        }));
                }
                catch (PluginNotRunningException ex)
                {
                    return ctx.Finish(ProgramStatus.Error, ex.Message);
                }

                int timeout = step.PluginTimeoutMs ?? def.TimeoutMs;
                st.Begin(manager, invocation, frame, def, label,
                         string.IsNullOrWhiteSpace(def.Label) ? def.Id : def.Label!, Math.Max(0, timeout));
                st.Subscribe(ctx, gen);
                frame.WaitStarted = true;
                ctx.Progress.StepStarted(step);
                return StepOutcome.Yield;
            }
        }

        // ── Params ────────────────────────────────────────────────────────────

        /// <summary>A param with no text and no default that is not required: not sent.</summary>
        private sealed class Omit { public static readonly Omit Instance = new(); }

        private static (bool Ok, object? Value, string? Error) ResolveParam(
            PluginStepParam p, Dictionary<string, string>? texts, ExecutionContext ctx, string label)
        {
            string? text = null;
            if (texts != null)
                foreach (var (k, v) in texts)
                    if (string.Equals(k, p.Key, StringComparison.Ordinal)) { text = v; break; }

            if (string.IsNullOrWhiteSpace(text))
            {
                if (p.Default is { } d && d.ValueKind is not (JsonValueKind.Null or JsonValueKind.Undefined))
                    return (true, d, null);
                if (p.Required)
                    return (false, null, $"{label}: parameter '{p.Label ?? p.Key}' has no value and no default");
                return (true, Omit.Instance, null);
            }

            var vars = ctx.Vars;
            switch (p.Type)
            {
                case "number":
                    return (true, ctx.Eval.Evaluate(text), null);

                case "boolean":
                    return (true, ctx.Eval.Evaluate(text) != 0, null);

                case "string":
                    return (true, vars.Interpolate(text), null);

                case "enum":
                {
                    var t = text.Trim();
                    if (p.Options != null && !p.Options.Contains(t, StringComparer.Ordinal))
                        return (false, null, $"{label}: '{t}' is not an option of '{p.Label ?? p.Key}' ({string.Join(", ", p.Options)})");
                    return (true, t, null);
                }

                case "point":
                {
                    if (!TryResolvePoint(text, ctx, out var pt, out var error))
                        return (false, null, $"{label}: {error}");
                    return (true, new { x = pt.X, y = pt.Y, z = pt.Z, rx = pt.RX, ry = pt.RY, rz = pt.RZ }, null);
                }

                case "list":
                {
                    var name = VarName(text);
                    if (!vars.Lists.TryGetValue(name, out var lv))
                        return (false, null, $"{label}: '${name}' is not a list variable");
                    return (true, JsonVariableCodec.ListToJson(lv), null);
                }

                case "image":
                {
                    var name = VarName(text);
                    if (!vars.IsImage(name))
                        return (false, null, $"{label}: '${name}' is not an image variable");
                    return (true, vars.GetImage(name), null);
                }

                case "variable":
                    return (true, VarName(text), null);

                default:
                    return (true, text, null);
            }
        }

        private static string VarName(string text) => text.Trim().TrimStart('$').Trim();

        internal static readonly Regex GridStackRef =
            new(@"^(?<kind>grid|stack)\s*:\s*(?<name>[^\[]+?)\s*\[(?<idx>.*)\]$", RegexOptions.IgnoreCase | RegexOptions.CultureInvariant);

        /// <summary>
        /// A point param: <c>grid:&lt;name&gt;[row, col]</c> / <c>grid:&lt;name&gt;[index]</c> /
        /// <c>stack:&lt;name&gt;[index]</c> (names, each index an expression), a points-variable
        /// element (<c>$pts[$i]</c>), or a saved point name (a template, as a move's
        /// pointNameExpr). The local frame is not applied.
        /// </summary>
        internal static bool TryResolvePoint(string text, ExecutionContext ctx, out Vector6 point, out string error)
        {
            point = Vector6.Zero;
            error = "";
            var src  = ctx.TargetSources;
            var vars = ctx.Vars;
            var t = text.Trim();

            var m = GridStackRef.Match(t);
            if (m.Success)
            {
                string name = m.Groups["name"].Value.Trim();
                var idx = SplitArgs(m.Groups["idx"].Value);
                if (idx.Count == 0 || idx.Any(string.IsNullOrWhiteSpace))
                {
                    error = $"'{t}' needs an index";
                    return false;
                }
                int Eval(string e) => (int)Math.Round(ctx.Eval.Evaluate(e));

                if (m.Groups["kind"].Value.Equals("grid", StringComparison.OrdinalIgnoreCase))
                {
                    var grid = src.Grids.GetAll().FirstOrDefault(g => string.Equals(g.Name, name, StringComparison.OrdinalIgnoreCase))
                               ?? src.Grids.Get(name);
                    if (grid == null) { error = $"Grid not found: {name}"; return false; }
                    var basePoint = src.Points.Get(grid.BasePointName);
                    if (basePoint == null) { error = $"Grid base point not found: {grid.BasePointName}"; return false; }
                    int row, col;
                    if (idx.Count == 1)
                    {
                        if (!grid.ColCount.HasValue || grid.ColCount.Value <= 0)
                        {
                            error = $"Grid '{grid.Name}' requires colCount to use a single index";
                            return false;
                        }
                        int i = Eval(idx[0]);
                        row = i / grid.ColCount.Value;
                        col = i % grid.ColCount.Value;
                    }
                    else if (idx.Count == 2) { row = Eval(idx[0]); col = Eval(idx[1]); }
                    else { error = $"'{t}' takes [index] or [row, col]"; return false; }
                    point = MoveTargetResolver.GridCell(grid, basePoint, row, col);
                    return true;
                }

                var stack = src.Stacks.GetAll().FirstOrDefault(s => string.Equals(s.Name, name, StringComparison.OrdinalIgnoreCase))
                            ?? src.Stacks.Get(name);
                if (stack == null) { error = $"Stack not found: {name}"; return false; }
                var stackBase = src.Points.Get(stack.BasePointName);
                if (stackBase == null) { error = $"Stack base point not found: {stack.BasePointName}"; return false; }
                if (idx.Count != 1) { error = $"'{t}' takes one index"; return false; }
                point = MoveTargetResolver.StackSlot(stack, stackBase, Eval(idx[0]));
                return true;
            }

            if (MoveTargetResolver.TryParsePointsRef(t, out var refName, out var idxExpr) && vars.TryGetPointList(refName, out var list))
            {
                if (list.Count == 0) { error = $"Points variable '{refName}' is empty or not set"; return false; }
                int i = string.IsNullOrWhiteSpace(idxExpr) ? 0 : (int)Math.Round(ctx.Eval.Evaluate(idxExpr));
                var vp = list.Items[Math.Clamp(i, 0, list.Count - 1)].ToPoint();
                point = new Vector6(vp.X, vp.Y, vp.Z, vp.RX, vp.RY, vp.RZ);
                return true;
            }

            var pointName = vars.Interpolate(t).Trim();
            if (pointName.Length == 0) { error = $"Point '{t}' resolved to nothing"; return false; }
            var named = src.Points.Get(pointName);
            if (named == null)
            {
                error = pointName == t ? $"Point not found: {pointName}" : $"Point not found: {pointName} (from '{t}')";
                return false;
            }
            point = new Vector6(named.X, named.Y, named.Z, named.RX, named.RY, named.RZ);
            return true;
        }

        /// <summary>Splits "a, min(b, c)" at top-level commas.</summary>
        internal static List<string> SplitArgs(string s)
        {
            var parts = new List<string>();
            int depth = 0, start = 0;
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                if (c is '(' or '[' or '{') depth++;
                else if (c is ')' or ']' or '}') depth--;
                else if (c == ',' && depth == 0) { parts.Add(s[start..i].Trim()); start = i + 1; }
            }
            parts.Add(s[start..].Trim());
            return parts;
        }

        // ── Outputs ───────────────────────────────────────────────────────────

        /// <summary>Writes the mapped outputs; returns an error message (a type mismatch) or null.</summary>
        internal static string? ApplyOutputs(ProgramStep step, PluginStepDef def, PluginStepReply reply, VariableScope vars, string label)
        {
            foreach (var map in step.PluginOutputs ?? [])
            {
                if (map == null || string.IsNullOrWhiteSpace(map.Key) || string.IsNullOrWhiteSpace(map.VariableName)) continue;
                var output = def.Outputs.FirstOrDefault(o => string.Equals(o.Key, map.Key, StringComparison.Ordinal));
                if (output == null) continue;
                if (!reply.Outputs.TryGetValue(map.Key, out var value)) continue; // missing → skipped
                string name = VarName(map.VariableName);
                if (ApplyOutput(output.Type, value, name, vars) is { } problem)
                    return $"{label}: output '{output.Label ?? output.Key}' → '${name}': {problem} ({Validation.ValidationCodes.PluginOutputType})";
            }
            return null;
        }

        private static string Kind(JsonValueKind k) => k switch
        {
            JsonValueKind.String => "text",
            JsonValueKind.Number => "a number",
            JsonValueKind.True or JsonValueKind.False => "a boolean",
            JsonValueKind.Array  => "an array",
            JsonValueKind.Object => "an object",
            _                    => "null",
        };

        /// <summary>Null when written; otherwise what did not fit.</summary>
        private static string? ApplyOutput(string type, JsonElement value, string name, VariableScope vars)
        {
            bool isList = vars.Lists.TryGetValue(name, out var list);
            bool isString = vars.IsString(name), isImage = vars.IsImage(name);
            bool scalarTarget = !isList && !isString && !isImage;

            switch (type)
            {
                case "number":
                case "boolean":
                {
                    if (!scalarTarget) return $"a {type} cannot be written to a {(isList ? "list" : isString ? "text" : "image")} variable";
                    double v;
                    switch (value.ValueKind)
                    {
                        case JsonValueKind.Number: v = value.GetDouble(); break;
                        case JsonValueKind.True:   v = 1; break;
                        case JsonValueKind.False:  v = 0; break;
                        default: return $"expected a {type} but the plugin returned {Kind(value.ValueKind)}";
                    }
                    if (!double.IsFinite(v)) return "the plugin returned a non-finite number";
                    vars.Set(name, type == "boolean" ? (v != 0 ? 1 : 0) : v);
                    return null;
                }

                case "string":
                    if (!isString) return "text can only be written to a text variable";
                    if (value.ValueKind != JsonValueKind.String) return $"expected text but the plugin returned {Kind(value.ValueKind)}";
                    vars.SetString(name, value.GetString() ?? "");
                    return null;

                case "image":
                    if (isList || isString || (!isImage && vars.TryGetLocal(name, out _)))
                        return "an image can only be written to an image variable";
                    if (value.ValueKind != JsonValueKind.String) return $"expected a base64 image but the plugin returned {Kind(value.ValueKind)}";
                    vars.SetImage(name, value.GetString() ?? "");
                    return null;

                case "point":
                {
                    if (!isList || list!.ElementType != ListElementType.Point) return "a point can only be written to a points list variable";
                    if (!TryPoint(value, out var p)) return $"expected a point {{x,y,z,rx,ry,rz}} or [6 numbers] but the plugin returned {Kind(value.ValueKind)}";
                    vars.SetList(name, ListVar.OfPoints([p]));
                    return null;
                }

                case "list":
                {
                    if (!scalarTarget && !isList) return $"a list cannot be written to a {(isString ? "text" : "image")} variable";
                    if (!isList && vars.TryGetLocal(name, out _)) return "a list cannot be written to a number variable";
                    if (value.ValueKind != JsonValueKind.Array) return $"expected an array but the plugin returned {Kind(value.ValueKind)}";
                    var inferred = InferElementType(value, isList ? list!.ElementType : null);
                    if (inferred == null) return "the array mixes numbers and objects";
                    if (isList && !Compatible(list!.ElementType, inferred.Value))
                        return $"'${name}' is a {list.ElementType} list but the plugin returned {inferred} elements";
                    // A declared list keeps its element type (as HTTP inbound does); a new one takes the data's.
                    vars.SetList(name, JsonVariableCodec.ListFromJson(value, isList ? list!.ElementType : inferred.Value));
                    return null;
                }
            }
            return $"unknown output type '{type}'";
        }

        private static bool Compatible(ListElementType declared, ListElementType got) => declared switch
        {
            ListElementType.Number or ListElementType.Boolean => got is ListElementType.Number or ListElementType.Boolean,
            ListElementType.Point  => got == ListElementType.Point,
            _                      => got is ListElementType.Record or ListElementType.Point,
        };

        /// <summary>numbers/booleans → Number (Boolean when only booleans), objects with x/y/z → Point,
        /// other objects → Record; an empty array → <paramref name="fallback"/> or Number; mixed → null.</summary>
        internal static ListElementType? InferElementType(JsonElement arr, ListElementType? fallback)
        {
            bool any = false, scalars = false, bools = true, objects = false, points = true;
            foreach (var el in arr.EnumerateArray())
            {
                any = true;
                switch (el.ValueKind)
                {
                    case JsonValueKind.Number: scalars = true; bools = false; break;
                    case JsonValueKind.True: case JsonValueKind.False: scalars = true; break;
                    case JsonValueKind.Object:
                        objects = true;
                        if (!(el.TryGetProperty("x", out _) && el.TryGetProperty("y", out _) && el.TryGetProperty("z", out _))) points = false;
                        break;
                    default: return null;
                }
            }
            if (!any) return fallback ?? ListElementType.Number;
            if (scalars && objects) return null;
            if (scalars) return bools ? ListElementType.Boolean : ListElementType.Number;
            return points ? ListElementType.Point : ListElementType.Record;
        }

        internal static bool TryPoint(JsonElement el, out Vector6Val p)
        {
            p = new Vector6Val();
            if (el.ValueKind == JsonValueKind.Object)
            {
                double F(string k) => el.TryGetProperty(k, out var v) && v.ValueKind == JsonValueKind.Number ? v.GetDouble() : 0;
                if (!el.TryGetProperty("x", out _)) return false;
                p = new Vector6Val { X = F("x"), Y = F("y"), Z = F("z"), RX = F("rx"), RY = F("ry"), RZ = F("rz") };
                return true;
            }
            if (el.ValueKind == JsonValueKind.Array && el.GetArrayLength() == 6 && el.EnumerateArray().All(v => v.ValueKind == JsonValueKind.Number))
            {
                var a = el.EnumerateArray().Select(v => v.GetDouble()).ToArray();
                p = new Vector6Val { X = a[0], Y = a[1], Z = a[2], RX = a[3], RY = a[4], RZ = a[5] };
                return true;
            }
            return false;
        }
    }

    /// <summary>
    /// The outstanding <c>step.execute</c> of a run (at most one: steps run one at a time).
    /// Touched on the loop thread under the executor lock; the reply and progress callbacks
    /// only enqueue onto <see cref="ExecutionContext.PendingActions"/>.
    /// </summary>
    internal sealed class PluginStepState
    {
        public string? InvocationId { get; private set; }
        public PluginManager? Manager { get; private set; }
        public PluginStepDef? Definition { get; private set; }
        /// <summary><c>"&lt;plugin name&gt;: &lt;step label&gt;"</c>.</summary>
        public string Label { get; private set; } = "";
        /// <summary>The manifest step label alone (timeout message).</summary>
        public string StepLabel { get; private set; } = "";
        public int  TimeoutMs { get; private set; }
        public long StartedTick { get; private set; }
        /// <summary>Set (on the loop thread) when the reply for <see cref="InvocationId"/> arrived.</summary>
        public PluginStepReply? Reply { get; set; }

        private StepListFrame? _frame;
        private Action<string, string?, double?>? _progress;

        public void Begin(PluginManager manager, string invocationId, StepListFrame frame, PluginStepDef def,
                          string label, string stepLabel, int timeoutMs)
        {
            Manager      = manager;
            InvocationId = invocationId;
            _frame       = frame;
            Definition   = def;
            Label        = label;
            StepLabel    = stepLabel;
            TimeoutMs    = timeoutMs;
            StartedTick  = Environment.TickCount64;
            Reply        = null;
        }

        /// <summary>Hooks <see cref="PluginManager.StepProgress"/> for this invocation: progress
        /// text becomes the monitor description <c>"&lt;label&gt;: &lt;message&gt;"</c>.</summary>
        public void Subscribe(ExecutionContext ctx, int generation)
        {
            if (Manager is null || InvocationId is null) return;
            string id = InvocationId;
            _progress = (inv, message, percent) =>
            {
                if (inv != id) return;
                ctx.PendingActions.Enqueue(() =>
                {
                    if (generation != ctx.RunGeneration || InvocationId != id) return;
                    var text = message;
                    if (percent is { } pc)
                        text = string.IsNullOrEmpty(text)
                            ? $"{pc.ToString("0", CultureInfo.InvariantCulture)}%"
                            : $"{text} ({pc.ToString("0", CultureInfo.InvariantCulture)}%)";
                    ctx.Progress.StepProgress(string.IsNullOrEmpty(text) ? Label : $"{Label}: {text}");
                });
            };
            Manager.StepProgress += _progress;
        }

        /// <summary>Forgets the invocation (it completed). No <c>step.cancel</c>.</summary>
        public void Clear()
        {
            if (Manager != null && _progress != null) Manager.StepProgress -= _progress;
            _progress    = null;
            InvocationId = null;
            Manager      = null;
            Definition   = null;
            Reply        = null;
            _frame       = null;
            TimeoutMs    = 0;
        }

        /// <summary>
        /// Abandons an outstanding invocation: sends <c>step.cancel</c> with
        /// <paramref name="reason"/> (<c>reset</c>, <c>stopped</c>, <c>timeout</c>), so a late reply is
        /// discarded, and rewinds the frame so the step is sent again if the run continues
        /// (a foreground Stop is a pause: Continue re-executes the step).
        /// </summary>
        public void Reset(string reason)
        {
            if (InvocationId != null && Manager != null)
                Manager.CancelStep(InvocationId, reason);
            if (_frame != null) _frame.WaitStarted = false;
            Clear();
        }
    }
}
