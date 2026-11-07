using Controller.RobotControl.Language;
using static RobotControl.Tests.Language.SrsTestKit;

namespace RobotControl.Tests.Language;

/// <summary>
/// The formatter's canonical text, pinned: each sample is parsed and formatted, and must equal
/// <c>Fixtures/srs/&lt;name&gt;.srs</c> exactly (and parsing that file must give it back). Set
/// <c>SRS_UPDATE_GOLDEN=1</c> to rewrite the files after an intended change, then review the diff.
/// </summary>
public class PrinterGoldenTests
{
    private static string FixtureDir([System.Runtime.CompilerServices.CallerFilePath] string here = "") =>
        Path.Combine(Path.GetDirectoryName(here)!, "..", "Fixtures", "srs");

    public static IEnumerable<object[]> SampleNames() => Samples.All.Select(s => new object[] { s.Name });

    [Theory]
    [MemberData(nameof(SampleNames))]
    public void FormatMatchesTheGoldenText(string name)
    {
        var names = Names();
        var text = Samples.All.First(s => s.Name == name).Text;
        var parsed = SrsLanguage.Parse(text, null, names);
        Assert.True(parsed.Ok);
        var formatted = SrsLanguage.Format(parsed.Program!, names).Text;
        var path = Path.Combine(FixtureDir(), name + ".srs");
        if (Environment.GetEnvironmentVariable("SRS_UPDATE_GOLDEN") == "1") File.WriteAllText(path, formatted);
        Assert.True(File.Exists(path), $"Missing golden {path} — run with SRS_UPDATE_GOLDEN=1");
        var golden = File.ReadAllText(path).Replace("\r\n", "\n");
        Assert.Equal(golden, formatted);

        // The golden text is itself canonical: parse + format gives it back.
        var again = SrsLanguage.Parse(golden, null, names);
        Assert.True(again.Ok);
        Assert.Equal(golden, SrsLanguage.Format(again.Program!, names).Text);
    }
}
