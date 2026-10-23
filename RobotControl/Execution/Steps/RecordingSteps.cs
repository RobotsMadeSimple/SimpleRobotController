using System.Globalization;
using System.Text;
using System.Text.Json;
using Controller.RobotControl.History;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Recording + export program steps:
    ///  • StartRecording — begin capturing a load cell into a target list variable.
    ///  • StopRecording  — stop it and fill that list variable with the captured {t,v,r} records.
    ///  • ExportData     — write one or more variables to a CSV/JSON file on the controller.
    /// Recording reuses <see cref="IoHistoryService"/> (so it also appears in the app's Recordings
    /// browser); the list variable is filled on Stop. Export writes to controller disk at a
    /// $variable/$time_ms-templated path, like SaveImage.
    /// </summary>
    internal static class RecordingSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.StartRecording] = new StartRecordingStep();
            r[StepType.StopRecording]  = new StopRecordingStep();
            r[StepType.ExportData]     = new ExportDataStep();
        }

        private static string? ChannelKey(ProgramStep step) =>
            !string.IsNullOrEmpty(step.LoadCellDeviceId) && step.LoadCellPin is int pin
                ? IoHistoryService.LoadCellKey(step.LoadCellDeviceId, pin)
                : null;

        /// <summary>Begin capturing the selected load cell into the named list variable.</summary>
        private sealed class StartRecordingStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var channelKey = ChannelKey(step);
                if (channelKey == null || string.IsNullOrWhiteSpace(step.RecordVariableName))
                    return StepOutcome.Advance; // nothing to record without a cell + a target variable

                var meta = ctx.Controller.IoHistory.StartRecording(channelKey, step.RecordVariableName);
                ctx.ProgramRecordings[channelKey] = (meta.Id, step.RecordVariableName!);
                return StepOutcome.Advance;
            }
        }

        /// <summary>Stop the recording for the selected load cell and store its samples as a list of {t,v,r}.</summary>
        private sealed class StopRecordingStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var channelKey = ChannelKey(step);
                if (channelKey == null || !ctx.ProgramRecordings.TryGetValue(channelKey, out var h))
                    return StepOutcome.Advance; // no active recording started by this program

                ctx.Controller.IoHistory.StopRecording(h.RecordingId);
                var win = ctx.Controller.IoHistory.GetRecordingWindow(h.RecordingId, null, null, int.MaxValue);
                var records = (win?.Samples ?? new List<IoSampleDto>())
                    .Select(s => new ObjectRecord { ["t"] = s.T, ["v"] = s.V, ["r"] = s.R });
                ctx.Vars.SetList(h.VarName, ListVar.OfRecords(records));
                ctx.ProgramRecordings.Remove(channelKey);
                return StepOutcome.Advance;
            }
        }

        /// <summary>Write the configured variables to a CSV or JSON file (controller-side).</summary>
        private sealed class ExportDataStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (string.IsNullOrWhiteSpace(step.ExportPath))
                    return StepOutcome.Advance; // no destination — nothing to write

                var format = (step.ExportFormat ?? "csv").Trim().ToLowerInvariant();
                var cols   = (step.ExportColumns ?? new List<JsonKeyValue>())
                    .Where(c => !string.IsNullOrWhiteSpace(c.Key)).ToList();

                string content = format == "json" ? BuildJson(cols, ctx.Vars) : BuildCsv(cols, ctx.Vars);

                try
                {
                    var path = ctx.Vars.Interpolate(step.ExportPath!);
                    var resolved = Path.IsPathRooted(path) ? path : Path.Combine(AppContext.BaseDirectory, path);
                    var dir = Path.GetDirectoryName(resolved);
                    if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
                    File.WriteAllText(resolved, content);
                }
                catch (Exception ex)
                {
                    return ctx.Finish(ProgramStatus.Error, $"Export data: could not write file — {ex.Message}");
                }
                return StepOutcome.Advance;
            }
        }

        // ── Column resolution (shared by CSV + JSON) ───────────────────────────

        /// <summary>Resolve one configured column to exactly one of: a list, a string, or a number.</summary>
        private static (ListVar? List, string? Str, double? Num) Resolve(JsonKeyValue c, VariableScope scope)
        {
            if (!string.IsNullOrWhiteSpace(c.ListVar) && scope.Lists.TryGetValue(c.ListVar, out var lv))
                return (lv, null, null);
            if (!string.IsNullOrWhiteSpace(c.ImageVar))
                return (null, scope.GetImage(c.ImageVar), null);

            // A lone "$name" can name a list or a string variable (not just a numeric expression).
            if (TryLoneVar(c.Expr, out var vn))
            {
                if (scope.Lists.TryGetValue(vn, out var lv2)) return (lv2, null, null);
                if (scope.IsString(vn) && scope.TryGetString(vn, out var sv)) return (null, sv, null);
            }

            double num = 0;
            if (!string.IsNullOrWhiteSpace(c.Expr))
                try { num = ExpressionEvaluator.Evaluate(c.Expr, scope.Eval.Vars, scope.Lists, scope.Properties); }
                catch { /* leave 0 */ }
            return (null, null, num);
        }

        private static bool TryLoneVar(string? expr, out string name)
        {
            name = "";
            var t = (expr ?? "").Trim();
            if (t.Length < 2 || t[0] != '$') return false;
            for (int i = 1; i < t.Length; i++)
                if (!(char.IsLetterOrDigit(t[i]) || t[i] == '_')) return false;
            name = t[1..];
            return name.Length > 0;
        }

        private static string Num(double v) => v.ToString("R", CultureInfo.InvariantCulture);

        // ── JSON ────────────────────────────────────────────────────────────────

        internal static string BuildJson(List<JsonKeyValue> cols, VariableScope scope)
        {
            var body = new Dictionary<string, object>();
            foreach (var c in cols)
            {
                var (list, str, num) = Resolve(c, scope);
                body[c.Key] = list != null ? JsonVariableCodec.ListToJson(list)
                            : str != null  ? str
                            : num ?? 0.0;
            }
            return JsonSerializer.Serialize(body, new JsonSerializerOptions { WriteIndented = true });
        }

        // ── CSV ─────────────────────────────────────────────────────────────────

        internal static string BuildCsv(List<JsonKeyValue> cols, VariableScope scope)
        {
            var resolved = cols.Select(c => (c.Key, V: Resolve(c, scope))).ToList();
            var listCols = resolved.Where(r => r.V.List != null).ToList();
            int rows = listCols.Count == 0 ? 1 : listCols.Max(r => r.V.List!.Items.Count);

            // A single record/point list exports with its field names as the headers directly
            // (e.g. a recording → t,v,r); mixed sources prefix columns with the row's key.
            bool single = resolved.Count == 1 && listCols.Count == 1 &&
                (listCols[0].V.List!.ElementType is ListElementType.Record or ListElementType.Point);

            var headers = new List<string>();
            var cells   = new List<Func<int, string>>();

            foreach (var (key, v) in resolved)
            {
                if (v.List is { } lv)
                {
                    if (lv.ElementType is ListElementType.Record or ListElementType.Point)
                    {
                        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                        var fields = new List<string>();
                        foreach (var item in lv.Items)
                            foreach (var k in item.Keys)
                                if (seen.Add(k)) fields.Add(k);
                        foreach (var f in fields)
                        {
                            headers.Add(single ? f : $"{key}.{f}");
                            var items = lv.Items; var field = f;
                            cells.Add(row => row < items.Count && items[row].TryGetValue(field, out var dv) ? Num(dv) : "");
                        }
                    }
                    else
                    {
                        headers.Add(key);
                        var items = lv.Items;
                        cells.Add(row => row < items.Count ? Num(items[row].Scalar) : "");
                    }
                }
                else if (v.Str is { } s)
                {
                    headers.Add(key);
                    cells.Add(_ => s);
                }
                else
                {
                    headers.Add(key);
                    var n = v.Num ?? 0;
                    cells.Add(_ => Num(n));
                }
            }

            var sb = new StringBuilder();
            sb.Append(string.Join(",", headers.Select(Esc)));
            for (int row = 0; row < rows; row++)
            {
                sb.Append('\n');
                sb.Append(string.Join(",", cells.Select(fn => Esc(fn(row)))));
            }
            return sb.ToString();
        }

        private static string Esc(string field)
        {
            if (field.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0) return field;
            return "\"" + field.Replace("\"", "\"\"") + "\"";
        }
    }
}
