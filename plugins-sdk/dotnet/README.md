# SimpleRobot.PluginSdk (C#)

Write [SimpleRobotController plugins](../../docs/plugins.md) in C#. The SDK speaks the plugin
WebSocket protocol (docs/plugins.md §4) so a plugin is a few handlers and one `RunAsync()`.

- Targets `net8.0` and `net10.0`; no third-party dependencies.
- Works for `runtime: "exe"` (self-contained binary), `runtime: "dotnet"` and `runtime: "external"`.

```
plugins-sdk/dotnet/
  SimpleRobot.PluginSdk/          the library
  SimpleRobot.PluginSdk.Tests/    xunit tests against a fake controller (Kestrel)
  SimpleRobot.PluginSdk.sln       SDK + tests + examples/dotnet/DemoScale
examples/dotnet/DemoScale/        a complete simulated-scale plugin
```

## Install

Project reference (simplest, e.g. from a plugin that lives in this repo):

```xml
<ProjectReference Include="..\..\..\plugins-sdk\dotnet\SimpleRobot.PluginSdk\SimpleRobot.PluginSdk.csproj" />
```

Or a local NuGet package:

```
dotnet pack plugins-sdk/dotnet/SimpleRobot.PluginSdk -c Release -o ./nupkgs
dotnet nuget add source ./nupkgs --name simplerobot-local     # once
dotnet add package SimpleRobot.PluginSdk
```

## Hello world

`plugin.json` (see docs/plugins.md §2) declares what the plugin contributes; the code implements it.

```csharp
using SimpleRobot.PluginSdk;

var plugin = new PluginHost();             // reads SRC_PLUGIN_* set by the controller

plugin.OnReady(ctx => { ctx.Log("ready"); return Task.CompletedTask; });

plugin.Step("weigh", async (ctx, p) =>
{
    ctx.Progress("settling…", 10);
    double grams = await ReadScale(p.GetInt("samples", 5));
    return new StepResult { ["grams"] = grams, ["stable"] = true };
});

plugin.Function("tare", (ctx, args) => Task.FromResult(1.0));    // keep it fast: < timeoutMs (250 ms default)

plugin.On("program.started", (ctx, e) => ctx.CommandAsync("SetSTBOutput", new { pin = 1, value = true }));

plugin.Background(async (ctx, ct) =>
{
    while (!ct.IsCancellationRequested)
    {
        ctx.SetProperties(new { weight = Read(), connected = 1 });
        await Task.Delay(50, ct);
    }
});

await plugin.RunAsync();   // connect, serve, reconnect with backoff, return after `shutdown`
```

`RunAsync` sends `plugin.ready` first (token + SDK name/version), serves requests concurrently,
reconnects with exponential backoff (1 s up to 30 s) when the socket drops, and returns once it has
replied to `shutdown`. It throws `PluginAuthException` if the controller rejects the token (close
code 4401) and `PluginReplacedException` if another connection took over (4409).

Orphan protection: the controller sets `SRC_PARENT_PID`. The host checks every 2 s that this process
still exists and, once it is gone (the controller was force-killed), logs to stderr and `RunAsync`
returns normally (so the example's `Main` exits 0). Set `PluginHostOptions.WatchParent = false` to
disable, or `ParentPid` to override the pid.

Notes:

- `On(...)` events are subscribed automatically at ready (globs such as `program.*` work). Use
  `plugin.SubscribeIntervals(positionIntervalMs: 50)` to set the default intervals, or
  `ctx.SubscribeAsync(...)` at runtime (it adds to the existing subscription).
- A context is created per connection. Background tasks are cancelled when the connection ends and
  started again after a reconnect; `OnReady` runs after every (re)connect.
- Step handlers receive an `IPluginContext` that is really an `IStepContext`: cast it to get
  `InvocationId`, `CancellationToken` (cancelled by `step.cancel`), `IsCancelled`, `CancelReason`.
  `ctx.Progress(...)` is only valid inside a step and throws elsewhere.
- Fail a step with `throw new StepException("scale timeout", "scaleTimeout")` (`ok:false`, that code;
  default code `stepFailed`). Any other exception is reported as `error: "exception"`.
- `ctx.CommandAsync(name, params)` can call any command from docs/websocket-api.md; failures throw
  `CommandException` with the controller's `Code`.
- Others: `ctx.GetVariablesAsync()`, `ctx.SetVariablesAsync(new { count = 3 })`,
  `ctx.SetStatus("degraded", "no scale")`, `ctx.ClearProperties()`, `ctx.Config`.

## Attribute style

```csharp
class ScalePlugin
{
    [PluginStep("weigh")]
    public async Task<StepResult> Weigh(IPluginContext ctx, StepParams p) => new() { ["grams"] = await Read(p.GetInt("samples")) };

    [PluginFunction("toOz")]
    public double ToOz(IPluginContext ctx, double[] args) => args[0] / 28.3495;

    [PluginEvent("program.started")]
    public Task Started(IPluginContext ctx, JsonElement e) { ctx.Log("started"); return Task.CompletedTask; }
}

var plugin = new PluginHost().Register(new ScalePlugin());
await plugin.RunAsync();
```

Signatures are checked at `Register` time; steps and functions may be sync or `Task<T>`, events `void` or `Task`.

## Values as the plugin sees them (docs/plugins.md §6)

Step params arrive already typed per the manifest; read them from `StepParams`:

| Param type | Getter | Notes |
|---|---|---|
| `number` | `GetDouble(key, default)` / `GetInt(key, default)` | |
| `boolean` | `GetBool(key, default)` | |
| `string`, `enum` | `GetString(key, default)` | templates already interpolated |
| `point` | `GetPoint(key)` → `PluginPoint(X, Y, Z, RX, RY, RZ)` | `TryGetPoint` for optional points; throws `badParams` if absent |
| `list` | `GetList<T>(key)` | `T` = `double`, `PluginPoint`, a record class, `JsonElement`… |
| `image` | `GetImageBytes(key)` → `byte[]?` | decodes the base64 JPEG |
| `variable` | `GetString(key)` | the raw variable name |
| anything | `Raw`, `Has(key)`, `TryGet(key, out JsonElement)` | |

A missing parameter returns the default you pass (`GetPoint`/`GetImageBytes` differ as above); a value of the wrong
kind throws a `StepException` with code `badParams`.

Outputs go in a `StepResult` (indexer accepts numbers, `bool`, `string`, `PluginPoint`, any `IEnumerable` of those,
dictionaries, `JsonElement` and `byte[]`):

| Output type | Put in the result |
|---|---|
| `number` / `boolean` | `double` / `bool` |
| `string` | `string` |
| `point` | `PluginPoint` |
| `list` | `double[]`, `List<PluginPoint>`, … |
| `image` | `byte[]` (sent as base64 JPEG; encode it as JPEG yourself) |

Expression functions take `double[]` and return `double` (booleans as 0/1). Properties are numbers
(booleans are sent as 0/1 by the controller).

## Development mode (`runtime: "external"`)

Install a plugin folder containing only `plugin.json` with `"runtime": "external"`, open its detail page in the
app to get the URL and token, then run your project from the IDE:

```csharp
var plugin = new PluginHost(new PluginHostOptions
{
    Url = "ws://192.168.1.50:5000/plugin",   // from the plugin detail page
    Token = "…",
    PluginId = "scale",
    // Optional: override the manifest's steps/functions/properties without editing the file.
    // ManifestOverride = new { steps = new[] { new { id = "weigh", label = "Weigh" } } },
});
```

Breakpoints and hot reload work; stop the process and run again to reconnect (a second connection
replaces the first). Logs appear in the plugin's log view.

## Publishing and installing

The controller launches the plugin with `<entry> <args>` from the plugin folder. For `runtime: "exe"` the entry is
OS-specific (`DemoScale.exe` on Windows, `DemoScale` on Linux), so publish once per target:

```
cd examples/dotnet/DemoScale
dotnet publish -c Release -r win-x64   --self-contained -p:PublishSingleFile=true -o dist/win-x64
dotnet publish -c Release -r linux-x64 --self-contained -p:PublishSingleFile=true -o dist/linux-x64
```

(`-o .` also works but writes into the project folder; a separate output folder keeps the source tree clean.)
Copy `plugin.json` next to the binary (the DemoScale project already copies it to its output), set `entry` to the
binary name for that OS, and zip the folder contents with `plugin.json` at the zip's root (or inside a single
top-level folder):

```
cd dist/linux-x64 && zip -r ../scale-linux.zip .
curl -X POST -H "Content-Type: application/zip" --data-binary @../scale-linux.zip "http://<controller>:5000/plugins/install?replace=true"
```

The app's Plugins page can install the same zip. The folder name (= plugin id) must match `plugin.json`'s `id`;
the installer sets the executable bit on Linux.

### Alternative: `runtime: "dotnet"`

When the controller machine has a .NET runtime installed (8 or newer), skip the self-contained publish:

```json
{ "runtime": "dotnet", "entry": "DemoScale.dll", "args": [] }
```

```
dotnet publish -c Release -o dist/dotnet      # framework-dependent, portable across OSes
```

Zip `dist/dotnet` (it contains `DemoScale.dll`, `SimpleRobot.PluginSdk.dll`, `DemoScale.runtimeconfig.json`, `plugin.json`).
The example's `RollForward=LatestMajor` lets the net8 build run on newer runtimes.

## Build and test

```
cd plugins-sdk/dotnet
dotnet build -c Release
dotnet test -c Release --nologo
```

The tests start a fake controller (Kestrel on a random loopback port serving `/plugin`) and cover the handshake,
bad token, step round trips, cancel, functions, config changes, events and auto-subscribe, shutdown and reconnect.
