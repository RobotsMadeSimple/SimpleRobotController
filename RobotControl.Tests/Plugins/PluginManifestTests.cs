using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

public class PluginManifestTests
{
    // The example manifest of docs/plugins.md §2, verbatim.
    private const string DocExample = """
    {
      "id": "scale", "name": "Bench Scale", "version": "1.2.0",
      "description": "Reads a serial bench scale and exposes its weight.", "author": "Octane Coffee",
      "protocolVersion": 1, "runtime": "python", "entry": "main.py", "args": [],
      "python": { "minVersion": "3.9", "requirements": "requirements.txt" },
      "autoStart": true, "restart": { "mode": "always", "maxRestarts": 5, "backoffMs": 2000 }, "readyTimeoutMs": 15000,
      "configSchema": [
        { "key": "port", "label": "Serial port", "type": "string", "default": "COM3", "help": "e.g. COM3 or /dev/ttyUSB0" },
        { "key": "baud", "label": "Baud rate", "type": "enum", "default": "9600", "options": ["9600", "19200", "115200"] },
        { "key": "tareOnStart", "label": "Tare on start", "type": "boolean", "default": true },
        { "key": "apiKey", "label": "API key", "type": "password", "default": "" },
        { "key": "samples", "label": "Samples", "type": "number", "default": 5, "min": 1, "max": 100, "step": 1 }
      ],
      "steps": [{
        "id": "weigh", "label": "Weigh item", "description": "Averages N readings and writes the weight.",
        "params": [
          { "key": "samples", "label": "Samples", "type": "number", "default": 5, "min": 1, "required": true },
          { "key": "unit", "label": "Unit", "type": "enum", "default": "g", "options": ["g", "oz"] },
          { "key": "label", "label": "Label", "type": "string", "default": "" },
          { "key": "stable", "label": "Wait for stable", "type": "boolean", "default": true },
          { "key": "target", "label": "Drop point", "type": "point" },
          { "key": "weights", "label": "History list", "type": "list" },
          { "key": "photo", "label": "Photo", "type": "image" }
        ],
        "outputs": [
          { "key": "grams", "label": "Weight (g)", "type": "number" },
          { "key": "stable", "label": "Was stable", "type": "boolean" },
          { "key": "text", "label": "Display text", "type": "string" },
          { "key": "where", "label": "Pick point", "type": "point" },
          { "key": "series", "label": "Readings", "type": "list" },
          { "key": "snap", "label": "Annotated photo", "type": "image" }
        ],
        "timeoutMs": 0, "cancellable": true
      }],
      "functions": [
        { "name": "tare", "signature": "scale.tare()", "description": "Zero the scale; returns 1 on success", "minArgs": 0, "maxArgs": 0, "timeoutMs": 250 },
        { "name": "toOz", "signature": "scale.toOz(grams)", "description": "Grams to ounces", "minArgs": 1, "maxArgs": 1 }
      ],
      "properties": [
        { "name": "weight", "description": "Live weight (g)", "type": "number" },
        { "name": "stable", "description": "1 while the reading is stable", "type": "boolean" },
        { "name": "connected", "description": "1 while the scale answers", "type": "boolean" }
      ]
    }
    """;

    private static PluginManifest Doc() => PluginManifest.Parse(DocExample);

    private static List<string> Codes(PluginManifest m) => m.Validate().Select(p => p.Code).ToList();

    [Fact]
    public void DocExampleIsValidAndParsesEveryField()
    {
        var m = Doc();
        Assert.Empty(m.Validate());
        Assert.Equal("scale", m.Id);
        Assert.Equal("python", m.Runtime);
        Assert.Equal("3.9", m.Python!.MinVersion);
        Assert.Equal(5, m.ConfigSchema.Count);
        Assert.Equal(7, m.Steps[0].Params.Count);
        Assert.Equal(250, m.Functions[1].EffectiveTimeoutMs);
        Assert.Equal(1, m.Functions[1].EffectiveMaxArgs);
        Assert.Equal(3, m.Properties.Count);
    }

    [Theory]
    [InlineData("robot")] [InlineData("program")] [InlineData("time")] [InlineData("aux")] [InlineData("camera")]
    [InlineData("stb")] [InlineData("relay")] [InlineData("nano")] [InlineData("plugin")] [InlineData("global")]
    [InlineData("local")] [InlineData("list")] [InlineData("time_ms")]
    public void ReservedRootsAreRejected(string id)
    {
        var m = Doc(); m.Id = id;
        Assert.Contains("reservedId", Codes(m));
    }

    [Fact]
    public void ExistingFunctionNamesAreRejected()
    {
        Assert.True(ExpressionEvaluator.IsFunctionName("max"));
        var m = Doc(); m.Id = "max";
        Assert.Contains("reservedId", Codes(m));
    }

    [Theory]
    [InlineData("Scale")] [InlineData("1scale")] [InlineData("s")] [InlineData("my-scale")] [InlineData("")]
    [InlineData("a234567890123456789012345678901234")]
    public void BadIdsAreRejected(string id)
    {
        var m = Doc(); m.Id = id;
        Assert.Contains("badId", Codes(m));
    }

    [Fact]
    public void ProtocolRuntimeAndEntryRules()
    {
        var m = Doc(); m.ProtocolVersion = 2;
        Assert.Contains("unsupportedProtocol", Codes(m));

        m = Doc(); m.Runtime = "java";
        Assert.Contains("badRuntime", Codes(m));

        m = Doc(); m.Entry = null;
        Assert.Contains("missingEntry", Codes(m));

        m = Doc(); m.Entry = "../escape.py";
        Assert.Contains("entryOutsideFolder", Codes(m));

        m = Doc(); m.Entry = "/abs/main.py";
        Assert.Contains("entryOutsideFolder", Codes(m));

        m = Doc(); m.Runtime = "external"; m.Entry = null;
        Assert.Empty(m.Validate());

        m = Doc(); m.Restart.Mode = "sometimes";
        Assert.Contains("badRestart", Codes(m));
    }

    [Fact]
    public void StepFunctionAndPropertyRules()
    {
        var m = Doc(); m.Steps[0].Id = "Weigh";
        Assert.Contains("badStepId", Codes(m));

        m = Doc(); m.Steps.Add(new PluginStepDef { Id = "weigh" });
        Assert.Contains("duplicateStepId", Codes(m));

        m = Doc(); m.Steps[0].Params[0].Type = "matrix";
        Assert.Contains("badParamType", Codes(m));

        m = Doc(); m.Steps[0].Outputs[0].Type = "variable"; // variable is a param type only
        Assert.Contains("badOutputType", Codes(m));

        m = Doc(); m.Functions[0].Name = "Tare";
        Assert.Contains("badFunctionName", Codes(m));

        m = Doc(); m.Functions[0].TimeoutMs = 6000;
        Assert.Contains("badFunctionTimeout", Codes(m));

        m = Doc(); m.Functions[0].MinArgs = 2; m.Functions[0].MaxArgs = 1;
        Assert.Contains("badArity", Codes(m));

        m = Doc(); m.Properties[0].Type = "string";
        Assert.Contains("badPropertyType", Codes(m));

        m = Doc(); m.Properties[0].Name = "1weight";
        Assert.Contains("badPropertyName", Codes(m));

        m = Doc(); m.ConfigSchema[1].Options = null;
        Assert.Contains("badConfigField", Codes(m));

        m = Doc(); m.ConfigSchema[0].Type = "date";
        Assert.Contains("badConfigField", Codes(m));
    }

    [Fact]
    public void LoaderReportsIdMismatchMissingAndBadJson()
    {
        var dir = Path.Combine(Path.GetTempPath(), "srcmanifest-" + Guid.NewGuid().ToString("N"));
        try
        {
            var folder = Path.Combine(dir, "other");
            Directory.CreateDirectory(folder);
            Assert.Equal("missingManifest", PluginManifestLoader.Load(folder).Problems.Single().Code);

            File.WriteAllText(Path.Combine(folder, "plugin.json"), DocExample);
            var r = PluginManifestLoader.Load(folder);
            Assert.Equal("idMismatch", r.Problems.Single().Code);
            Assert.Equal("other", r.FolderName);

            File.WriteAllText(Path.Combine(folder, "plugin.json"), "{ not json");
            Assert.Equal("badManifest", PluginManifestLoader.Load(folder).Problems.Single().Code);
        }
        finally { Directory.Delete(dir, true); }
    }

    [Fact]
    public void ReadyOverrideReplacesContributionLists()
    {
        var m = Doc();
        var over = JsonSerializer.SerializeToElement(new { functions = new[] { new { name = "zero", minArgs = 0, maxArgs = 0 } } });
        var o = m.WithOverride(over);
        Assert.Equal("zero", o.Functions.Single().Name);
        Assert.Equal(3, o.Properties.Count); // untouched
        Assert.Equal(2, m.Functions.Count);  // original unchanged
    }
}
