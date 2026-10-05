using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;

namespace Controller.RobotControl.Plugins;

/// <summary>A launch that cannot succeed until something changes (missing runtime, old Python, failed venv).</summary>
public sealed class PluginLaunchException : Exception
{
    /// <summary><c>runtimeMissing</c>, <c>entryMissing</c>, <c>pythonTooOld</c>, <c>venvFailed</c>, <c>launchFailed</c>.</summary>
    public string Code { get; }
    public PluginLaunchException(string code, string message) : base(message) => Code = code;
}

/// <summary>A running plugin process.</summary>
public interface IPluginProcess
{
    int Pid { get; }
    /// <summary>Completes with the exit code when the process exits.</summary>
    Task<int> Exited { get; }
    /// <summary>Terminates the process and its children (SIGTERM, then SIGKILL on Linux; tree kill on Windows).</summary>
    void Kill();
}

/// <summary>Everything a launcher needs to start one plugin.</summary>
public sealed class PluginLaunchContext
{
    public required PluginManifest Manifest { get; init; }
    public required string PluginDir { get; init; }
    public required IReadOnlyDictionary<string, string> Environment { get; init; }
    public required PluginLog Log { get; init; }
    /// <summary>Called before a long venv bootstrap starts (the host shows state <c>installing</c>).</summary>
    public Action? OnInstalling { get; init; }
}

/// <summary>Starts plugin processes. Swapped for a fake in tests.</summary>
public interface IPluginProcessLauncher
{
    /// <summary>
    /// Bootstraps (python venv) and starts the process. Throws <see cref="PluginLaunchException"/>
    /// for problems a restart would not fix.
    /// </summary>
    Task<IPluginProcess> LaunchAsync(PluginLaunchContext context, CancellationToken ct);
}

/// <summary>
/// The real launcher: python (venv bootstrap when a requirements file exists and <c>.venv</c>
/// does not; the venv interpreter when present, else <c>python3</c> then <c>python</c> on PATH),
/// <c>dotnet &lt;entry&gt;</c>, or a native executable. Working directory is the plugin folder;
/// stdout/stderr go to the plugin log.
/// </summary>
public sealed class PluginProcessLauncher : IPluginProcessLauncher
{
    private static readonly bool IsWindows = RuntimeInformation.IsOSPlatform(OSPlatform.Windows);

    public async Task<IPluginProcess> LaunchAsync(PluginLaunchContext context, CancellationToken ct)
    {
        var m   = context.Manifest;
        var dir = context.PluginDir;
        string fileName;
        var args = new List<string>();

        switch (m.Runtime)
        {
            case "python":
            {
                string entry = Path.Combine(dir, m.Entry!);
                if (!File.Exists(entry)) throw new PluginLaunchException("entryMissing", $"Entry '{m.Entry}' not found");

                string reqRel  = m.Python?.Requirements is { Length: > 0 } r ? r : "requirements.txt";
                string reqPath = Path.Combine(dir, reqRel);
                string? venvPy = VenvPython(dir);
                if (venvPy == null && File.Exists(reqPath))
                {
                    string basePy = FindOnPath("python3", "python")
                        ?? throw new PluginLaunchException("runtimeMissing", "Python was not found on PATH (python3 / python)");
                    await CheckPythonVersionAsync(basePy, m.Python?.MinVersion, context.Log, ct);
                    context.OnInstalling?.Invoke();
                    venvPy = await BootstrapVenvAsync(basePy, dir, reqRel, context.Log, ct);
                }
                fileName = venvPy ?? FindOnPath("python3", "python")
                    ?? throw new PluginLaunchException("runtimeMissing", "Python was not found on PATH (python3 / python)");
                await CheckPythonVersionAsync(fileName, m.Python?.MinVersion, context.Log, ct);
                args.Add(m.Entry!);
                break;
            }
            case "dotnet":
            {
                if (!File.Exists(Path.Combine(dir, m.Entry!))) throw new PluginLaunchException("entryMissing", $"Entry '{m.Entry}' not found");
                fileName = FindOnPath("dotnet") ?? throw new PluginLaunchException("runtimeMissing", "dotnet was not found on PATH");
                args.Add(m.Entry!);
                break;
            }
            case "exe":
            {
                fileName = Path.GetFullPath(Path.Combine(dir, m.Entry!));
                if (!File.Exists(fileName)) throw new PluginLaunchException("entryMissing", $"Entry '{m.Entry}' not found");
                break;
            }
            default:
                throw new PluginLaunchException("launchFailed", $"Runtime '{m.Runtime}' cannot be launched");
        }
        args.AddRange(m.Args ?? new List<string>());

        var psi = BuildStartInfo(fileName, args, dir, context.Environment);
        var process = new Process { StartInfo = psi, EnableRaisingEvents = true };
        var log = context.Log;
        process.OutputDataReceived += (_, e) => { if (e.Data != null) log.Append("stdout", e.Data); };
        process.ErrorDataReceived  += (_, e) => { if (e.Data != null) log.Append("stderr", e.Data); };
        try
        {
            if (!process.Start()) throw new PluginLaunchException("launchFailed", $"Could not start {fileName}");
        }
        catch (System.ComponentModel.Win32Exception ex)
        {
            throw new PluginLaunchException("launchFailed", $"Could not start {fileName}: {ex.Message}");
        }
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();
        log.Append("info", $"Started {Path.GetFileName(fileName)} {string.Join(' ', args)} (pid {process.Id})");
        return new OsPluginProcess(process);
    }

    /// <summary>The start info for a plugin process (pure; unit-tested).</summary>
    public static ProcessStartInfo BuildStartInfo(string fileName, IEnumerable<string> args, string workingDir,
                                                  IReadOnlyDictionary<string, string> environment)
    {
        var psi = new ProcessStartInfo
        {
            FileName               = fileName,
            WorkingDirectory       = workingDir,
            UseShellExecute        = false,
            RedirectStandardOutput = true,
            RedirectStandardError  = true,
            RedirectStandardInput  = false,
            CreateNoWindow         = true,
        };
        foreach (var a in args) psi.ArgumentList.Add(a);
        foreach (var (k, v) in environment) psi.Environment[k] = v;
        psi.Environment["PYTHONUNBUFFERED"] = "1"; // live log lines from print()
        return psi;
    }

    /// <summary>The venv interpreter of <paramref name="pluginDir"/>, or null when there is no venv.</summary>
    public static string? VenvPython(string pluginDir)
    {
        string p = IsWindows
            ? Path.Combine(pluginDir, ".venv", "Scripts", "python.exe")
            : Path.Combine(pluginDir, ".venv", "bin", "python");
        return File.Exists(p) ? p : null;
    }

    /// <summary>The first of <paramref name="names"/> found on PATH (with PATHEXT on Windows).</summary>
    public static string? FindOnPath(params string[] names)
    {
        var path = Environment.GetEnvironmentVariable("PATH") ?? "";
        var dirs = path.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries);
        var exts = IsWindows
            ? (Environment.GetEnvironmentVariable("PATHEXT") ?? ".EXE;.CMD;.BAT").Split(';', StringSplitOptions.RemoveEmptyEntries)
            : new[] { "" };
        foreach (var name in names)
            foreach (var d in dirs)
                foreach (var ext in IsWindows ? exts.Prepend("") : exts)
                {
                    string candidate;
                    try { candidate = Path.Combine(d.Trim('"'), name + ext); }
                    catch (ArgumentException) { continue; }
                    if (!File.Exists(candidate)) continue;
                    // Skip the Windows Store "python" stub (a 0-byte app execution alias).
                    if (IsWindows && candidate.Contains("WindowsApps", StringComparison.OrdinalIgnoreCase)) continue;
                    return candidate;
                }
        return null;
    }

    /// <summary>Parses "Python 3.11.4" → 3.11.4.</summary>
    public static Version? ParsePythonVersion(string output)
    {
        var match = Regex.Match(output, @"(\d+)\.(\d+)(?:\.(\d+))?");
        return match.Success ? Version.Parse(match.Value) : null;
    }

    private static async Task CheckPythonVersionAsync(string python, string? minVersion, PluginLog log, CancellationToken ct)
    {
        if (string.IsNullOrWhiteSpace(minVersion)) return;
        var (code, output) = await RunToLogAsync(python, ["--version"], null, null, ct, quiet: true);
        var have = ParsePythonVersion(output);
        var need = Version.Parse(PluginManifest.NormalizeVersion(minVersion));
        if (code != 0 || have == null)
            throw new PluginLaunchException("runtimeMissing", $"Could not run '{python} --version'");
        if (have < need)
            throw new PluginLaunchException("pythonTooOld", $"Python {have} is older than the required {minVersion}");
    }

    private static async Task<string> BootstrapVenvAsync(string basePython, string dir, string requirements, PluginLog log, CancellationToken ct)
    {
        string venvDir = Path.Combine(dir, ".venv");
        log.Append("info", "Creating virtual environment (.venv)…");
        var (code, _) = await RunToLogAsync(basePython, ["-m", "venv", ".venv"], dir, log, ct);
        string? venvPy = VenvPython(dir);
        if (code != 0 || venvPy == null)
        {
            TryDelete(venvDir);
            throw new PluginLaunchException("venvFailed", $"python -m venv failed (exit {code})");
        }
        log.Append("info", $"Installing {requirements}…");
        (code, _) = await RunToLogAsync(venvPy, ["-m", "pip", "install", "-r", requirements], dir, log, ct);
        if (code != 0)
        {
            TryDelete(venvDir); // so the next start retries the install
            throw new PluginLaunchException("venvFailed", $"pip install -r {requirements} failed (exit {code})");
        }
        log.Append("info", "Virtual environment ready");
        return venvPy;
    }

    private static async Task<(int Code, string Output)> RunToLogAsync(string file, string[] args, string? cwd,
        PluginLog? log, CancellationToken ct, bool quiet = false)
    {
        var psi = BuildStartInfo(file, args, cwd ?? Environment.CurrentDirectory, new Dictionary<string, string>());
        using var p = new Process { StartInfo = psi };
        var output = new System.Text.StringBuilder();
        void Line(string? s, string level)
        {
            if (s == null) return;
            lock (output) output.AppendLine(s);
            if (!quiet) log?.Append(level, s);
        }
        p.OutputDataReceived += (_, e) => Line(e.Data, "stdout");
        p.ErrorDataReceived  += (_, e) => Line(e.Data, "stderr");
        try { p.Start(); }
        catch (System.ComponentModel.Win32Exception ex) { return (-1, ex.Message); }
        p.BeginOutputReadLine();
        p.BeginErrorReadLine();
        try { await p.WaitForExitAsync(ct); }
        catch (OperationCanceledException) { try { p.Kill(true); } catch { } throw; }
        return (p.ExitCode, output.ToString());
    }

    private static void TryDelete(string dir)
    {
        try { if (Directory.Exists(dir)) Directory.Delete(dir, recursive: true); } catch { /* best effort */ }
    }
}

/// <summary><see cref="IPluginProcess"/> over a <see cref="Process"/>.</summary>
internal sealed class OsPluginProcess : IPluginProcess
{
    private readonly Process _process;
    private readonly TaskCompletionSource<int> _exited = new(TaskCreationOptions.RunContinuationsAsynchronously);

    public OsPluginProcess(Process process)
    {
        _process = process;
        Pid = process.Id;
        process.Exited += (_, _) => Complete();
        if (process.HasExited) Complete();
    }

    public int Pid { get; }
    public Task<int> Exited => _exited.Task;

    private void Complete()
    {
        // WaitForExit() without a timeout also drains the redirected output.
        try { _process.WaitForExit(); } catch { }
        int code;
        try { code = _process.ExitCode; } catch { code = -1; }
        _exited.TrySetResult(code);
    }

    public void Kill()
    {
        try
        {
            if (_process.HasExited) return;
            if (!OperatingSystem.IsWindows())
            {
                // Polite SIGTERM first so the plugin can clean up; SIGKILL the tree shortly after.
                _ = sys_kill(Pid, SIGTERM);
                if (_process.WaitForExit(1000)) return;
            }
            _process.Kill(entireProcessTree: true);
        }
        catch (InvalidOperationException) { /* already exited */ }
        catch (System.ComponentModel.Win32Exception) { }
    }

    private const int SIGTERM = 15;

    [DllImport("libc", EntryPoint = "kill", SetLastError = true)]
    private static extern int sys_kill(int pid, int sig);
}
