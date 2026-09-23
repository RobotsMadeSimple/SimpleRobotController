using System.Text;
using System.Text.Json;
using System.Threading.Channels;

namespace Controller.RobotControl.Execution
{
    /// <summary>JSON over HTTP: HttpRequest (outbound POST) and HttpReceive (inbound webhook).</summary>
    internal static class HttpSteps
    {
        // Reused across all requests.
        private static readonly HttpClient HttpClient = new();

        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.HttpRequest] = new HttpRequestStep();
            r[StepType.HttpReceive] = new HttpReceiveStep();
        }

        // ── HttpRequest ───────────────────────────────────────────────────────

        private sealed class HttpRequestStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var http = ctx.Http;
                bool waitForResponse = step.JsonWaitForResponse ?? true;

                if (waitForResponse)
                {
                    if (!frame.WaitStarted)
                    {
                        var snapshot = BuildJsonBody(step, ctx.Vars);
                        http.PendingJsonTask = FireJsonRequest(step.JsonUrl, snapshot, step.JsonTimeoutMs ?? 10_000);
                        frame.WaitStarted = true;
                        ctx.Progress.StepStarted(step);
                        return StepOutcome.Yield;
                    }

                    if (http.PendingJsonTask == null || !http.PendingJsonTask.IsCompleted) return StepOutcome.Yield;

                    var result = http.PendingJsonTask.IsCompletedSuccessfully ? http.PendingJsonTask.Result : null;
                    http.PendingJsonTask = null;
                    frame.WaitStarted    = false;
                    JsonVariableCodec.ApplyInbound(ctx.Vars, step.JsonInbound, result);
                    return StepOutcome.Advance;
                }

                // Fire and continue — evaluate expressions now (on loop thread), then let the
                // HTTP request finish in the background and queue variable writes back here.
                var body    = BuildJsonBody(step, ctx.Vars);
                var inbound = step.JsonInbound?.ToList();
                int runGen  = ctx.RunGeneration;
                FireJsonRequest(step.JsonUrl, body, step.JsonTimeoutMs ?? 10_000)
                    .ContinueWith(t =>
                    {
                        // Checked again on the loop thread: a response that lands after the
                        // run ended (or a new one started) must not write into the next run.
                        if (t.IsCompletedSuccessfully && t.Result != null && inbound != null)
                            ctx.PendingActions.Enqueue(() =>
                            {
                                if (runGen == ctx.RunGeneration) JsonVariableCodec.ApplyInbound(ctx.Vars, inbound, t.Result);
                            });
                    });
                return StepOutcome.Advance;
            }
        }

        /// <summary>Evaluates all outbound fields on the control-loop thread (numeric expressions +
        /// image base64) and returns a plain snapshot the async HTTP task can serialize safely.</summary>
        private static Dictionary<string, object> BuildJsonBody(ProgramStep step, VariableScope scope)
        {
            var body = new Dictionary<string, object>();
            var vars = scope.Eval.Vars;
            foreach (var kv in step.JsonOutbound ?? [])
            {
                if (string.IsNullOrWhiteSpace(kv.Key)) continue;
                if (!string.IsNullOrWhiteSpace(kv.ListVar))
                {
                    // An undeclared list sends [] rather than being skipped. A server that
                    // expects the key should see an empty list, not a body with the key
                    // missing — the second is far harder to diagnose from the other end.
                    body[kv.Key] = scope.Lists.TryGetValue(kv.ListVar, out var lv)
                        ? JsonVariableCodec.ListToJson(lv)
                        : new List<double>();
                }
                else if (!string.IsNullOrWhiteSpace(kv.ImageVar))
                {
                    body[kv.Key] = scope.GetImage(kv.ImageVar);
                }
                else
                {
                    double val = 0;
                    if (!string.IsNullOrWhiteSpace(kv.Expr))
                        try { val = ExpressionEvaluator.Evaluate(kv.Expr, vars, scope.Lists, scope.Properties); }
                        catch { /* leave as 0 */ }
                    body[kv.Key] = val;
                }
            }
            // Backwards compat: old programs that used the separate jsonImageOutbound field
            foreach (var m in step.JsonImageOutbound ?? [])
            {
                if (string.IsNullOrWhiteSpace(m.Key) || string.IsNullOrWhiteSpace(m.VariableName)) continue;
                body[m.Key] = scope.GetImage(m.VariableName);
            }
            return body;
        }

        private static async Task<Dictionary<string, JsonElement>?> FireJsonRequest(
            string? url, Dictionary<string, object> body, int timeoutMs)
        {
            if (string.IsNullOrWhiteSpace(url)) return null;
            try
            {
                var json    = JsonSerializer.Serialize(body);
                var content = new StringContent(json, Encoding.UTF8, "application/json");
                using var cts = new CancellationTokenSource(timeoutMs);
                var response = await HttpClient.PostAsync(url, content, cts.Token).ConfigureAwait(false);
                if (!response.IsSuccessStatusCode) return null;
                var responseBody = await response.Content.ReadAsStringAsync(cts.Token).ConfigureAwait(false);
                if (string.IsNullOrWhiteSpace(responseBody)) return null;
                return JsonSerializer.Deserialize<Dictionary<string, JsonElement>>(responseBody);
            }
            catch
            {
                return null;
            }
        }

        // ── HttpReceive ───────────────────────────────────────────────────────

        private sealed class HttpReceiveStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var http = ctx.Http;
                if (!frame.WaitStarted)
                {
                    var name = step.HttpReceiveName ?? "";
                    if (string.IsNullOrWhiteSpace(name))
                        return ctx.Finish(ProgramStatus.Error, "HttpReceive step has no webhook name set");

                    var cts = new CancellationTokenSource(step.HttpReceiveTimeoutMs ?? 30_000);
                    http.WebhookCts          = cts;
                    var id                   = ctx.Controller.WebhookManager.Subscribe(name, out var chan);
                    http.PendingWebhookSub   = (name, id);
                    http.PendingWebhookTask  = WaitForWebhook(chan, cts.Token);
                    frame.WaitStarted        = true;
                    ctx.Progress.StepStarted(step);
                    return StepOutcome.Yield;
                }

                if (http.PendingWebhookTask == null || !http.PendingWebhookTask.IsCompleted) return StepOutcome.Yield;

                var result = http.PendingWebhookTask.IsCompletedSuccessfully ? http.PendingWebhookTask.Result : null;
                http.PendingWebhookTask = null;
                http.WebhookCts?.Dispose();
                http.WebhookCts = null;
                if (http.PendingWebhookSub.HasValue)
                {
                    ctx.Controller.WebhookManager.Unsubscribe(http.PendingWebhookSub.Value.Name, http.PendingWebhookSub.Value.Id);
                    http.PendingWebhookSub = null;
                }
                frame.WaitStarted = false;

                if (result == null)
                    return ctx.Finish(ProgramStatus.Error,
                        $"HttpReceive: timeout waiting for webhook '{step.HttpReceiveName}'");

                JsonVariableCodec.ApplyInbound(ctx.Vars, step.HttpReceiveInbound, result);
                return StepOutcome.Advance;
            }
        }

        private static async Task<Dictionary<string, JsonElement>?> WaitForWebhook(
            Channel<Dictionary<string, JsonElement>> chan, CancellationToken ct)
        {
            try   { return await chan.Reader.ReadAsync(ct).ConfigureAwait(false); }
            catch { return null; }
        }
    }

    /// <summary>The pending HTTP work of a run: a request awaiting its response, and a
    /// webhook subscription awaiting a delivery.</summary>
    internal sealed class HttpState
    {
        public Task<Dictionary<string, JsonElement>?>? PendingJsonTask;
        public Task<Dictionary<string, JsonElement>?>? PendingWebhookTask;
        public (string Name, Guid Id)?                  PendingWebhookSub;
        public CancellationTokenSource?                 WebhookCts;

        /// <summary>Cancels a pending webhook wait and drops its subscription.</summary>
        public void ReleaseWebhook(RobotController controller)
        {
            WebhookCts?.Cancel();
            WebhookCts?.Dispose();
            WebhookCts         = null;
            PendingWebhookTask = null;
            if (PendingWebhookSub.HasValue)
            {
                controller.WebhookManager.Unsubscribe(PendingWebhookSub.Value.Name, PendingWebhookSub.Value.Id);
                PendingWebhookSub = null;
            }
        }
    }

    /// <summary>Conversions between program variables and JSON values.</summary>
    internal static class JsonVariableCodec
    {
        /// <summary>Writes each mapped key of an inbound JSON object into its variable.</summary>
        public static void ApplyInbound(VariableScope vars, List<JsonInboundMapping>? mappings, Dictionary<string, JsonElement>? data)
        {
            if (data == null || mappings == null) return;
            foreach (var m in mappings)
            {
                if (string.IsNullOrWhiteSpace(m.Key) || string.IsNullOrWhiteSpace(m.VariableName)) continue;
                if (!data.TryGetValue(m.Key, out var elem)) continue;
                if (elem.ValueKind == JsonValueKind.Array
                    && vars.Lists.TryGetValue(m.VariableName, out var target))
                {
                    vars.SetList(m.VariableName, ListFromJson(elem, target.ElementType));
                }
                else if (vars.IsImage(m.VariableName))
                {
                    var str = elem.ValueKind == JsonValueKind.String ? (elem.GetString() ?? "") : "";
                    vars.SetImage(m.VariableName, str);
                }
                else
                {
                    vars.Set(m.VariableName, ScalarFromJson(elem));
                }
            }
        }

        /// <summary>
        /// One list variable as something <see cref="JsonSerializer"/> turns into a JSON array.
        ///
        /// The element type picks the shape, and each one matches how that list reads in an
        /// expression: a Boolean list sends <c>[true, false]</c> because <c>$v[0]</c> is the
        /// value itself, while a point or record list sends objects because those are read by
        /// field name. So what goes on the wire is what the program would have seen.
        /// </summary>
        public static object ListToJson(ListVar lv) => lv.ElementType switch
        {
            ListElementType.Boolean => lv.Items.ConvertAll(r => r.Scalar != 0),
            ListElementType.Number  => lv.Items.ConvertAll(r => r.Scalar),
            // Points and records are already dictionaries of named doubles, which is
            // exactly a JSON object — no conversion needed.
            _                       => lv.Items,
        };

        /// <summary>One inbound JSON value as a number. Shared by scalar variables and by the
        /// elements of a number/boolean list so both coerce identically.</summary>
        public static double ScalarFromJson(JsonElement el)
        {
            double v = el.ValueKind switch
            {
                // TryGetDouble rather than GetDouble: throwing here would abort a program
                // mid-cycle over a malformed response from someone else's server.
                JsonValueKind.Number => el.TryGetDouble(out var d) ? d : 0,
                JsonValueKind.True   => 1,
                JsonValueKind.False  => 0,
                JsonValueKind.String => double.TryParse(el.GetString(), out var s) ? s : 0,
                _ => 0,
            };
            // A number too large for a double does not fail to parse — it succeeds as ±∞.
            // An infinity in a program variable then poisons every expression it reaches
            // while still comparing and evaluating like a normal value, so it never
            // surfaces as an error. 0 is also wrong, but it is wrong somewhere visible.
            return double.IsFinite(v) ? v : 0;
        }

        /// <summary>
        /// Rebuilds a list variable from an inbound JSON array.
        ///
        /// The element type the program declared wins over whatever arrived: a Boolean list
        /// stays Boolean whether the server sent <c>true</c> or <c>1</c>, so <c>$v[0]</c>,
        /// conditions and the editor all keep behaving the way the program was written
        /// against. The wire supplies values, not types.
        ///
        /// The list is replaced whole rather than merged, so its length follows the response —
        /// a shorter array shortens the list, and <c>$v.length</c> is how many came back.
        /// Anything that does not fit the declared shape becomes 0 instead of throwing,
        /// for the same reason as <see cref="ScalarFromJson"/>.
        /// </summary>
        public static ListVar ListFromJson(JsonElement arr, ListElementType type)
        {
            var items = new List<ObjectRecord>();
            bool structured = type is ListElementType.Point or ListElementType.Record;
            foreach (var el in arr.EnumerateArray())
            {
                if (!structured)
                {
                    items.Add(ObjectRecord.FromScalar(ScalarFromJson(el)));
                    continue;
                }
                // A structured element needs named fields. A non-object here (say a bare
                // number where {x,y,z} was expected) yields an empty record, which reads
                // as 0 on every field rather than shifting the rest of the list.
                var rec = new ObjectRecord();
                if (el.ValueKind == JsonValueKind.Object)
                    foreach (var p in el.EnumerateObject())
                        rec[p.Name] = ScalarFromJson(p.Value);
                items.Add(rec);
            }
            return new ListVar { ElementType = type, Items = items };
        }
    }
}
