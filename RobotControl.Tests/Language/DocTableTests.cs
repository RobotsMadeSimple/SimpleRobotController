using System.Text;
using Controller.RobotControl.Programs;
using K = Controller.RobotControl.Programs.FieldKinds;

namespace RobotControl.Tests.Language;

/// <summary>
/// docs/program-text.md's argument reference is generated from the step catalog, and this test
/// keeps it current: the generated block between the BEGIN/END markers must equal the file's.
/// Set <c>SRS_UPDATE_DOCS=1</c> to rewrite it.
/// </summary>
public class DocTableTests
{
    private const string Begin = "<!-- BEGIN GENERATED ARGUMENTS (DocTableTests) -->";
    private const string End = "<!-- END GENERATED ARGUMENTS -->";

    private static string DocPath([System.Runtime.CompilerServices.CallerFilePath] string here = "") =>
        Path.GetFullPath(Path.Combine(Path.GetDirectoryName(here)!, "..", "..", "docs", "program-text.md"));

    private static string ValueText(StepSpec s, FieldSpec f) => f.Kind switch
    {
        K.Number or K.Integer => f.Unit == "ms" ? "duration (`500ms`, `2s`)" : "number" + (f.ExprKey != null ? " or `(expr)`" : ""),
        K.Bool => "`on` / `off`" + (f.ExprKey != null ? " or `(expr)`" : ""),
        K.Enum => string.Join(" \\| ",(f.Options ?? []).Select(o => $"`{Word(s, f, Convert.ToString(o.Value)!)}`")),
        K.Text => "text",
        K.Template => "`\"text {$var}\"`",
        K.Expr => "expression",
        K.Condition => "condition",
        K.VarWrite or K.VarRead => "`$variable`",
        K.ListVar => "`$list`",
        K.PointRef => "point name",
        K.ToolRef => "tool name",
        K.LocalRef => "local name",
        K.RoutineRef => "routine name",
        K.ProgramRef => "program name",
        K.VisionRef => "vision program name",
        K.DeviceRef => "device name",
        K.CameraRef => "camera name",
        K.LabelRef => "label name",
        K.Target => "target",
        K.Axes6 => "`(x=…, y=…, z=…, rx=…, ry=…, rz=…)`",
        K.TextList => "`(a, b, …)`",
        _ => f.Kind,
    };

    private static string Word(StepSpec s, FieldSpec f, string v) =>
        Controller.RobotControl.Language.SrsParser.EnumWords.TryGetValue((Enum.Parse<Controller.RobotControl.StepType>(s.Type), f.Key), out var words)
            ? words.FirstOrDefault(w => w.Value == v).Word ?? v : v;

    public static string Generate()
    {
        var sb = new StringBuilder();
        foreach (var cat in StepCatalog.Categories)
        {
            var steps = StepCatalog.Steps.Where(s => s.Category == cat.Key).ToList();
            if (steps.Count == 0) continue;
            sb.Append($"\n#### {cat.Label}\n\n");
            foreach (var s in steps)
            {
                sb.Append($"**{s.Label}** — {s.Description}.  \n`{s.Syntax.Replace("|", "\\|")}`\n\n");
                var args = s.Fields.Where(f => !f.Hidden && f.Kind is not (K.Steps or K.Branches or K.Cases or K.Json)).ToList();
                if (args.Count == 0) { sb.Append('\n'); continue; }
                sb.Append("| In text | Field | Value | Unit | Default |\n|---|---|---|---|---|\n");
                foreach (var f in args)
                {
                    string where = f.Position != null ? $"position {f.Position + 1}" : f.Arg.Length > 0 ? $"`{f.Arg}=`" : "rows / sugar";
                    string def = f.Default switch { null => "", bool b => b ? "on" : "off", string[] a => string.Join(", ", a), var d => Convert.ToString(d, System.Globalization.CultureInfo.InvariantCulture)! };
                    sb.Append($"| {where} | {f.Label} | {ValueText(s, f)} | {f.Unit ?? ""} | {def} |\n");
                }
                sb.Append('\n');
            }
        }
        return sb.ToString();
    }

    [Fact]
    public void TheArgumentReferenceMatchesTheCatalog()
    {
        var path = DocPath();
        Assert.True(File.Exists(path), path);
        var doc = File.ReadAllText(path).Replace("\r\n", "\n");
        int b = doc.IndexOf(Begin, StringComparison.Ordinal), e = doc.IndexOf(End, StringComparison.Ordinal);
        Assert.True(b >= 0 && e > b, "markers missing in docs/program-text.md");
        var current = doc[(b + Begin.Length)..e];
        var generated = Generate();
        if (Environment.GetEnvironmentVariable("SRS_UPDATE_DOCS") == "1" && current != generated)
        {
            File.WriteAllText(path, doc[..(b + Begin.Length)] + generated + doc[e..]);
            return;
        }
        Assert.Equal(generated, current);
    }
}
