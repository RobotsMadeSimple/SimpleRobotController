using System.Reflection;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;
using Controller.RobotControl;
using Controller.RobotControl.Language;
using Controller.RobotControl.Programs;
using K = Controller.RobotControl.Programs.FieldKinds;

namespace RobotControl.Tests.Language;

/// <summary>Shared helpers for the text-language tests: a name table, the round-trip
/// assertion and a catalog-driven random program generator.</summary>
internal static class SrsTestKit
{
    public static SrsNameTable Names()
    {
        var t = new SrsNameTable();
        t.Add(SrsRefKind.Point, "pt-1", "Pick1").Add(SrsRefKind.Point, "pt-2", "Pick 1").Add(SrsRefKind.Point, "pt-3", "Place1")
         .Add(SrsRefKind.Point, "point-guid-1", "P1").Add(SrsRefKind.Point, "point-guid-2", "P2").Add(SrsRefKind.Point, "pt-end", "end")
         .Add(SrsRefKind.Tool, "tool-1", "Gripper").Add(SrsRefKind.Tool, "tool-2", "Vacuum Cup")
         .Add(SrsRefKind.Local, "local-1", "Fixture1").Add(SrsRefKind.Local, "local-2", "Fixture 2")
         .Add(SrsRefKind.Grid, "grid-1", "Tray").Add(SrsRefKind.Grid, "grid-2", "Big Tray")
         .Add(SrsRefKind.Stack, "stack-1", "Bins")
         .Add(SrsRefKind.Routine, "routine-1", "Pick").Add(SrsRefKind.Routine, "routine-guid-1", "MyRoutine")
         .Add(SrsRefKind.Program, "bg-1", "Conveyor").Add(SrsRefKind.Program, "bg-guid-1", "BgProg")
         .Add(SrsRefKind.Vision, "vp-1", "Detect Parts").Add(SrsRefKind.Vision, "vp-2", "Find")
         .Add(SrsRefKind.Device, "nano-1", "Arduino Nano").Add(SrsRefKind.Device, "aux-1", "Aux")
         .Add(SrsRefKind.Camera, "cam-1", "Top").Add(SrsRefKind.Camera, "cam-2", "Side Cam");
        t.AddTool("vp-1", "t1", "Top edge").AddTool("vp-1", "t5", "Board").AddTool("vp-1", "t6", "dup").AddTool("vp-1", "t7", "dup")
         .AddTool("vp-2", "f1", "Parts");
        return t;
    }

    public static readonly JsonSerializerOptions Json = new()
    {
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        Converters = { new JsonStringEnumConverter() },
    };

    public static BuiltProgram Program(params ProgramStep[] steps) =>
        new() { Id = "prog-1", Name = "Test", Steps = steps.ToList() };

    /// <summary>Normalize(Parse(Format(P), base: P)) == Normalize(P), and Format is idempotent.
    /// Returns the formatted text.</summary>
    public static string AssertRoundTrip(BuiltProgram p, ISrsNames? names = null, bool includeIds = false)
    {
        names ??= Names();
        var f = SrsLanguage.Format(p, names, includeIds);
        var parsed = SrsLanguage.Parse(f.Text, p, names);
        var errors = parsed.Problems.Where(x => x.Severity == "error").ToList();
        if (!parsed.Ok)
            Assert.Fail("Formatted text does not parse:\n" + string.Join("\n", errors.Select(e => $"{e.Line}:{e.Col} {e.Code} {e.Message}")) + "\n---\n" + Numbered(f.Text));
        var want = SrsLanguage.NormalizeJson(p, names);
        var got = SrsLanguage.NormalizeJson(parsed.Program!, names);
        if (!JsonNode.DeepEquals(want, got))
            Assert.Fail("Round trip changed the program.\n--- text\n" + Numbered(f.Text) + "\n--- diff (want vs got)\n" + Diff(want, got));

        var again = SrsLanguage.Parse(f.Text, null, names);
        Assert.True(again.Ok);
        var f2 = SrsLanguage.Format(again.Program!, names, includeIds);
        if (f2.Text != f.Text) Assert.Fail("Format is not idempotent.\n--- first\n" + Numbered(f.Text) + "\n--- second\n" + Numbered(f2.Text));
        return f.Text;
    }

    public static string Numbered(string text) =>
        string.Join("\n", text.Split('\n').Select((l, i) => $"{i + 1,3}| {l}"));

    public static string Diff(JsonNode? a, JsonNode? b, string path = "$")
    {
        if (JsonNode.DeepEquals(a, b)) return "";
        if (a is JsonObject ao && b is JsonObject bo)
        {
            var sb = new System.Text.StringBuilder();
            foreach (var k in ao.Select(x => x.Key).Union(bo.Select(x => x.Key)))
                sb.Append(Diff(ao[k], bo[k], $"{path}.{k}"));
            return sb.ToString();
        }
        if (a is JsonArray aa && b is JsonArray ba && aa.Count == ba.Count)
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < aa.Count; i++) sb.Append(Diff(aa[i], ba[i], $"{path}[{i}]"));
            return sb.ToString();
        }
        return $"{path}: {a?.ToJsonString() ?? "(absent)"}  vs  {b?.ToJsonString() ?? "(absent)"}\n";
    }

    // ── Random programs ───────────────────────────────────────────────────

    private static readonly Dictionary<string, PropertyInfo> Props =
        typeof(ProgramStep).GetProperties()
            .Select(p => (p, n: p.GetCustomAttribute<JsonPropertyNameAttribute>()?.Name))
            .Where(x => x.n != null).ToDictionary(x => x.n!, x => x.p);

    public static readonly string[] Texts =
    [
        "", "Pick 1", "Pick1", "a\"quoted\"b", "line1\nline2", "Bin {$i}", "tab\there", "#hash", "back\\slash",
        "ünïcode 日本 🤖", "ident", "end", "on", "here", "  padded  ", "a, b", "x -> y", "{", "}", "$var", "P1", "dup",
    ];

    public static readonly string[] Exprs =
    [
        "$count + 1", "$a", "$pts[$i].x", "$pts[$i + 1][2]", "200", "-3.5", "(3)", "($a + 1) * 2", "$a == 1",
        "{$x + 1}", "abs($v) > 2 ? 1 : 0", "$a and $b", "$a or not $b", "$s.contains(\"x\")", "min($a, $b, 3)",
        "", " $x", "$x\n+1", "a #b", "\"q\"", "1e-5", "on", "true", "$x, $y", "$a -> $b", "max(", "$stb.in1",
        "$io.Arduino_Nano.Load_Cell >= 2000", "$robot.x", "$l.length", "2s", "500ms",
    ];

    public static readonly string[] VarNames = ["count", "pts", "ok", "bad name", "x1", "_", "vals", "label", "s"];

    public sealed class Gen(int seed)
    {
        private readonly Random _r = new(seed);
        private int _ids;

        public bool Chance(double p) => _r.NextDouble() < p;
        public T Pick<T>(IReadOnlyList<T> xs) => xs[_r.Next(xs.Count)];
        public string Id() => $"id-{seed}-{++_ids}";

        public double Number() => _r.Next(8) switch
        {
            0 => 0, 1 => _r.Next(-50, 500), 2 => Math.Round(_r.NextDouble() * 100, 3), 3 => -0.25,
            4 => 1e-5, 5 => 12345.678, 6 => 1500, _ => _r.Next(1, 10) * 1000,
        };

        public string Text() => Pick(Texts);
        public string Expr() => Pick(Exprs);
        public string VarName() => Pick(VarNames);

        public JsonNode Scalar(Type t)
        {
            t = Nullable.GetUnderlyingType(t) ?? t;
            if (t == typeof(int)) return JsonValue.Create(_r.Next(-3, 70000) / (Chance(0.5) ? 1 : 100))!;
            if (t == typeof(long)) return JsonValue.Create((long)_r.Next(-2000, 2000))!;
            if (t == typeof(double)) return JsonValue.Create(Number())!;
            if (t == typeof(bool)) return JsonValue.Create(Chance(0.5))!;
            return JsonValue.Create(Text())!;
        }

        public JsonObject Condition()
        {
            var items = new JsonArray();
            int n = _r.Next(0, 4);
            for (int i = 0; i < n; i++)
            {
                var it = new JsonObject { ["id"] = Chance(0.9) ? Id() : "" };
                if (Chance(0.75)) it["expr"] = Pick(["$a > 1", "$b", "$a and $b or $c", "not $x", "$i < 10", "$a ? 1 : 0", "", "$stb.in1 == 1", "($a or $b)", "$c && $d"]);
                else { it["left"] = Pick(["$i", "$s", "", "3", "$a + 1"]); it["operator"] = Pick(["==", "<", ">=", "contains", "!="]); it["right"] = Pick(["10", "abc", "", "$j"]); }
                if (i > 0 || Chance(0.2)) it["join"] = Pick(["AND", "OR", "and", "XOR"]);
                items.Add(it);
            }
            return new JsonObject { ["combinator"] = Pick(["ALL", "ANY"]), ["items"] = items };
        }

        public JsonObject Step(int depth)
        {
            var spec = Pick(StepCatalog.Steps);
            var type = Enum.Parse<StepType>(spec.Type);
            if (Chance(0.02)) { type = StepType.Unknown; }
            var j = new JsonObject { ["id"] = Id(), ["type"] = type.ToString() };
            if (Chance(0.2)) j["name"] = Text();
            if (Chance(0.25)) j["comment"] = Text();
            if (Chance(0.12)) j["enabled"] = Chance(0.7) ? false : true;
            if (type == StepType.Unknown) { j["unknownStepType"] = Pick(["MoveWithForce", "Fancy Step", ""]); return j; }

            foreach (var f in spec.Fields)
            {
                if (!Chance(f.Required ? 0.85 : 0.5)) continue;
                Field(j, type, f, depth);
            }
            // Something the catalog does not say for this type (rides in @with).
            if (Chance(0.15))
            {
                var p = Pick(Props.Where(kv => kv.Key is not ("id" or "type") && IsScalar(kv.Value.PropertyType)).ToList());
                j[p.Key] = Scalar(p.Value.PropertyType);
            }
            return j;
        }

        private static bool IsScalar(Type t)
        {
            t = Nullable.GetUnderlyingType(t) ?? t;
            return t == typeof(int) || t == typeof(long) || t == typeof(double) || t == typeof(bool) || t == typeof(string);
        }

        private void Expression(JsonObject j, string key)
        {
            if (j["expressions"] is not JsonObject ex) j["expressions"] = ex = new JsonObject();
            ex[key] = Expr();
        }

        private void Field(JsonObject j, StepType type, FieldSpec f, int depth)
        {
            var clr = Props.TryGetValue(f.Key, out var pi) ? pi.PropertyType : typeof(string);
            switch (f.Kind)
            {
                case K.Number:
                case K.Integer:
                case K.Bool:
                    if (f.ExprKey != null && Chance(0.35)) { Expression(j, f.ExprKey); if (Chance(0.5)) j[f.Key] = Scalar(clr); }
                    else if (f.Options != null && Chance(0.8)) j[f.Key] = JsonValue.Create(Convert.ToInt32(Pick(f.Options).Value));
                    else j[f.Key] = Scalar(clr);
                    break;
                case K.Enum:
                    j[f.Key] = Chance(0.85) && f.Options != null ? Convert.ToString(Pick(f.Options).Value) : Pick(["weird value", "", "STOP"]);
                    break;
                case K.Text:
                case K.Template:
                    j[f.Key] = Text();
                    break;
                case K.Expr:
                    j[f.Key] = Chance(0.8) ? Expr() : Text();
                    break;
                case K.Condition:
                    j[f.Key] = Condition();
                    break;
                case K.VarWrite:
                case K.VarRead:
                case K.ListVar:
                    j[f.Key] = VarName();
                    break;
                case K.PointRef: case K.ToolRef: case K.LocalRef: case K.RoutineRef: case K.ProgramRef: case K.VisionRef:
                {
                    var props = f.StoredProps;
                    var kind = f.Kind switch
                    {
                        K.PointRef => SrsRefKind.Point, K.ToolRef => SrsRefKind.Tool, K.LocalRef => SrsRefKind.Local,
                        K.RoutineRef => SrsRefKind.Routine, K.ProgramRef => SrsRefKind.Program, _ => SrsRefKind.Vision,
                    };
                    var known = Names().List(kind);
                    int mode = _r.Next(5);
                    if (mode <= 1 && known.Count > 0) { var k = Pick(known); j[props[0]] = k.Id; if (Chance(0.7)) j[props[1]] = Chance(0.8) ? k.Name : "Stale"; }
                    else if (mode == 2) { j[props[0]] = "dangling-" + _r.Next(9); if (Chance(0.5)) j[props[1]] = Text(); }
                    else if (mode == 3 && known.Count > 0) j[props[1]] = Pick(known).Name;
                    else j[props[1]] = Text();
                    break;
                }
                case K.DeviceRef:
                case K.CameraRef:
                {
                    var known = Names().List(f.Kind == K.DeviceRef ? SrsRefKind.Device : SrsRefKind.Camera);
                    j[f.Key] = Chance(0.7) ? Pick(known).Id : Pick(["dangling-dev", "Arduino Nano", "", "Top"]);
                    break;
                }
                case K.LabelRef:
                    if (Chance(0.6)) j["labelId"] = Pick(["lbl-1", "lbl-2", "lbl-x"]);
                    if (Chance(0.6)) j["labelName"] = Pick(["retry", "start", "Fancy Label", ""]);
                    break;
                case K.Target:
                    Target(j, f.Key == "via");
                    break;
                case K.Axes6:
                    foreach (var p in f.StoredProps)
                    {
                        if (Chance(0.3)) j[p] = Number();
                        if (Chance(0.1)) Expression(j, p);
                    }
                    break;
                case K.TextList:
                {
                    var a = new JsonArray();
                    int n = _r.Next(0, 4);
                    for (int i = 0; i < n; i++) a.Add(Chance(0.5) ? Pick(["waitTimeout", "httpFailed", "noPart", "OK"]) : Text());
                    j[f.Key] = a;
                    break;
                }
                case K.ExprMap:
                {
                    var o = new JsonObject();
                    int n = _r.Next(0, 4);
                    for (int i = 0; i < n; i++) o[Pick(["height", "count", "x", "z", "bad key", "end", "label", "send"])] = Chance(0.6) ? Expr() : Text();
                    j[f.Key] = o;
                    break;
                }
                case K.MappingOut:
                {
                    var a = new JsonArray();
                    int n = _r.Next(0, 3);
                    for (int i = 0; i < n; i++)
                    {
                        if (f.Key == "toolOutputs")
                            a.Add(new JsonObject { ["toolId"] = Pick(["t1", "t5", "t6", "ghost", "Top edge"]), ["value"] = Pick(["angle", "cells", "points", "pose.x", "in"]), ["variable"] = VarName() });
                        else a.Add(new JsonObject { ["key"] = Pick(["found", "move", "x", "bad key", "grams"]), ["variableName"] = VarName() });
                    }
                    j[f.Key] = a;
                    break;
                }
                case K.Rows:
                {
                    var a = new JsonArray();
                    int n = _r.Next(0, 3);
                    for (int i = 0; i < n; i++)
                    {
                        var row = new JsonObject { ["key"] = Pick(["n", "photo", "board", "weird \"key\""]), ["expr"] = "" };
                        switch (_r.Next(4))
                        {
                            case 0: row["listVar"] = VarName(); break;
                            case 1: row["imageVar"] = VarName(); break;
                            case 2: row["expr"] = Expr(); break;
                            default: row["expr"] = Expr(); row["listVar"] = VarName(); break;
                        }
                        a.Add(row);
                    }
                    j[f.Key] = a;
                    break;
                }
                case K.Steps:
                    j[f.Key] = Steps(depth + 1);
                    break;
                case K.Branches:
                {
                    var a = new JsonArray();
                    int n = _r.Next(0, 3);
                    for (int i = 0; i < n; i++) a.Add(new JsonObject { ["id"] = Id(), ["condition"] = Condition(), ["steps"] = Steps(depth + 1) });
                    j[f.Key] = a;
                    break;
                }
                case K.Cases:
                {
                    var a = new JsonArray();
                    int n = _r.Next(0, 3);
                    for (int i = 0; i < n; i++)
                    {
                        var vals = new JsonArray();
                        int m = _r.Next(0, 3);
                        for (int k = 0; k < m; k++) vals.Add(Pick(["1", "2", "$a", "$b + 1", "", "x, y"]));
                        a.Add(new JsonObject { ["id"] = Id(), ["values"] = vals, ["steps"] = Steps(depth + 1) });
                    }
                    j[f.Key] = a;
                    break;
                }
                case K.PluginRef:
                    j["pluginId"] = Pick(["scale", "my-plugin", "x.y"]);
                    j["pluginStepId"] = Pick(["weigh", "in", "step 2"]);
                    break;
                case K.Json:
                    j[f.Key] = new JsonObject { ["file"] = "part.svg", ["safeZ"] = 10, ["holes"] = new JsonArray() };
                    break;
            }
        }

        private void Target(JsonObject j, bool via)
        {
            if (via)
            {
                switch (_r.Next(4))
                {
                    case 0: j["viaPointId"] = "pt-3"; j["viaPointName"] = "Place1"; break;
                    case 1: j["viaPointNameExpr"] = Pick(["$pts[1]", "bin{$i}", "Pick1", "$t"]); break;
                    case 2: j["viaPointName"] = Text(); break;
                }
                return;
            }
            switch (_r.Next(7))
            {
                case 0: j["pointId"] = "pt-1"; j["pointName"] = "Pick1"; break;
                case 1: j["pointNameExpr"] = Pick(["$pts[$i]", "bin{$i}", "$target", "Pick1", "a\"b{$x}"]); break;
                case 2:
                {
                    var gp = new JsonObject { ["gridId"] = Pick(["grid-1", "grid-2", "ghost-grid", ""]), ["useGridIndex"] = Chance(0.4) };
                    if (Chance(0.7)) gp["rowIndex"] = _r.Next(0, 5);
                    if (Chance(0.7)) gp["colIndex"] = _r.Next(0, 5);
                    if (Chance(0.5)) gp["gridIndex"] = _r.Next(0, 9);
                    j["gridPoint"] = gp;
                    if (Chance(0.4)) Expression(j, Pick(["gridRowIndex", "gridColIndex", "gridGridIndex"]));
                    break;
                }
                case 3:
                {
                    var sp = new JsonObject { ["stackId"] = Pick(["stack-1", "ghost"]) };
                    if (Chance(0.7)) sp["index"] = _r.Next(0, 5);
                    j["stackPoint"] = sp;
                    if (Chance(0.4)) Expression(j, "stackIndex");
                    break;
                }
                case 4: j["varPointName"] = "pts"; if (Chance(0.5)) j["varPointIndex"] = "$i"; break;
                case 5: j["pointName"] = Text(); break;
            }
        }

        public JsonArray Steps(int depth)
        {
            var a = new JsonArray();
            if (depth > 3) return a;
            int n = _r.Next(0, depth == 0 ? 8 : 3);
            for (int i = 0; i < n; i++)
            {
                if (Chance(0.08)) a.Add(Note());
                else a.Add(Step(depth));
            }
            return a;
        }

        private JsonObject Note()
        {
            var j = new JsonObject { ["id"] = Id(), ["type"] = "Note", ["comment"] = Chance(0.8) ? Pick(["A note", "two\nlines", "", "## not a section", "x"]) : null };
            if (Chance(0.4)) j["noteStyle"] = Pick(["section", "note", "odd"]);
            return j;
        }

        public JsonObject Variable(int i)
        {
            var j = new JsonObject { ["id"] = Id(), ["name"] = i < VarNames.Length && Chance(0.8) ? VarNames[i] : $"v{i}" };
            switch (_r.Next(8))
            {
                case 0: j["value"] = Number(); break;
                case 1: j["isBoolean"] = true; j["value"] = Chance(0.5) ? 1 : 0; break;
                case 2: j["isString"] = true; if (Chance(0.8)) j["stringValue"] = Text(); break;
                case 3:
                {
                    var et = Pick(["Number", "Boolean", "Point", "Record"]);
                    var items = new JsonArray();
                    for (int k = 0; k < _r.Next(0, 3); k++)
                        items.Add(et switch
                        {
                            "Point" => new JsonObject { ["x"] = Number(), ["z"] = Number(), ["rz"] = 0 },
                            "Record" => new JsonObject { ["row"] = _r.Next(3), ["col coverage"] = 0.5 },
                            _ => new JsonObject { ["value"] = Chance(0.5) ? 1 : Number() },
                        });
                    j["items"] = items;
                    if (Chance(0.9)) j["elementType"] = et;
                    break;
                }
                case 4: j["isImage"] = true; break;
                case 5: j["isStopwatch"] = true; break;
                case 6: j["isComputed"] = true; j["valueExpression"] = Pick(["$w * $h", "$a > 1", "5"]); if (Chance(0.3)) j["isBoolean"] = true; break;
                default: j["valueExpression"] = Expr(); j["value"] = 7; break;
            }
            if (Chance(0.2)) j["isGlobal"] = true;
            if (Chance(0.2)) j["isPersistent"] = Chance(0.8);
            if (Chance(0.1)) j["isInput"] = true;
            if (Chance(0.2)) j["displayOnMonitor"] = true;
            if (Chance(0.2)) j["description"] = Text();
            if (Chance(0.15)) j["paramMode"] = Pick(["in", "out", "inOut", "sideways"]);
            return j;
        }

        public BuiltProgram Program()
        {
            var vars = new JsonArray();
            for (int i = 0; i < _r.Next(0, 6); i++) vars.Add(Variable(i));
            var steps = Steps(0);
            // Labels for gotos to land on.
            if (Chance(0.5)) steps.Add(new JsonObject { ["id"] = Id(), ["type"] = "Label", ["labelId"] = "lbl-1", ["labelName"] = "retry" });
            if (Chance(0.3)) steps.Insert(0, new JsonObject { ["id"] = Id(), ["type"] = "Label", ["labelId"] = "lbl-2", ["labelName"] = "start" });
            var p = new JsonObject { ["id"] = "prog", ["name"] = "Random", ["steps"] = steps, ["variables"] = vars };
            return p.Deserialize<BuiltProgram>(Json)!;
        }
    }
}
