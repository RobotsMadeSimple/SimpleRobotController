using System.Diagnostics;
using System.Text.Json;
using Controller.RobotControl.UsbRelay;

namespace Controller.RobotControl.Commands;

/// <summary>Identity, status polling, self-update and restart.</summary>
internal sealed class SystemCommands
{
    private readonly RobotController _robot;
    private readonly ProgramCycleManager _programs;
    private readonly BackgroundProgramManager _background;

    public SystemCommands(RobotController robot, ProgramCycleManager programs, BackgroundProgramManager background)
    {
        _robot      = robot;
        _programs   = programs;
        _background = background;
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("GetRobotInfo",      GetRobotInfo);
        d.Add("GetStatus",         GetStatus);
        d.Add("Update",            Update);
        d.Add("RestartController", RestartController);
        d.Add("SetRobotIdentity",  SetRobotIdentity);
    }

    private object? GetRobotInfo(CommandMessage msg)
    {
        var identity = _robot.Identity;
        return new
        {
            robotName    = identity.RobotName,
            robotType    = identity.RobotType,
            serialNumber = identity.SerialNumber,
        };
    }

    private void SetRobotIdentity(CommandMessage msg)
    {
        var p        = CommandJson.LoadParams<SetRobotIdentityParams>(msg);
        var identity = _robot.Identity;
        if (p.RobotName != null) identity.RobotName = p.RobotName;
        if (p.RobotType != null) identity.RobotType = p.RobotType;
        RobotIdentityService.Save(identity);
        _robot.OnIdentityChanged?.Invoke(identity);
    }

    private object? GetStatus(CommandMessage msg)
    {
        var robot      = _robot;
        var kinematics = robot.Kinematics;
        var position   = robot.LivePosition;
        var target     = robot.LiveTargetPosition;
        var config     = robot.Config;
        var stb        = robot.stb;
        var relay      = robot.RelayManager;

        Vector6? pose = kinematics.GetVisualRobotPose(position, robot.CurrentTool);
        var (j1, j2x, j2z, j4) = kinematics.GetJointAngles();

        // Position expressed in the active local's frame — the jog page
        // shows this so the readout tracks the selected local. Equals
        // the world position when no local is active.
        var localPos = robot.ActiveLocalOffset is { } locStat
            ? LocalFrame.Inverse(locStat, position)
            : position;

        var speeds = robot.MotionParameters;
        var fault  = robot.FaultStatus;
        string homingState = robot.HomingState;

        return new
        {
            moving = robot.IsMoving,
            wasHomed = robot.Homed,
            homingState,
            isHoming = homingState != HomingSequencer.IdleStateName,
            lastPointUpdate = robot.pointRepo.LastUpdatedUnixMs,
            driverConnected = stb.connected,
            driverOk = stb.connected && stb.status != 0,

            x = position.X,
            y = position.Y,
            z = position.Z,
            rx = position.RX,
            ry = position.RY,
            rz = position.RZ,

            localX = localPos.X,
            localY = localPos.Y,
            localZ = localPos.Z,
            localRZ = localPos.RZ,

            targetX = target.X,
            targetY = target.Y,
            targetZ = target.Z,
            targetRX = target.RX,
            targetRY = target.RY,
            targetRZ = target.RZ,

            joint1Angle = j1,
            joint2X = j2x,
            joint2Z = j2z,
            joint4Angle = j4,

            poseX = pose?.X ?? 0,
            poseY = pose?.Y ?? 0,
            poseZ = pose?.Z ?? 0,
            poseRX = pose?.RX ?? 0,
            poseRY = pose?.RY ?? 0,
            poseRZ = pose?.RZ ?? 0,

            speedS = speeds.SpeedS,
            accelS = speeds.AccelS,
            decelS = speeds.DecelS,

            speedJ = speeds.SpeedJ,
            accelJ = speeds.AccelJ,
            decelJ = speeds.DecelJ,

            // STB digital inputs
            input1 = stb.Input1,
            input2 = stb.Input2,
            input3 = stb.Input3,
            input4 = stb.Input4,

            // STB digital outputs
            output1 = stb.Output1,
            output2 = stb.Output2,
            output3 = stb.Output3,
            output4 = stb.Output4,

            // USB relay board — cached state (updated on Set + a 1s
            // board poll) so relays changed by a running program
            // update live in the app without a per-broadcast USB read.
            relay = new UsbRelayState
            {
                Connected = relay.IsConnected,
                Serial    = relay.GetSerial(),
                Relays    = relay.GetRelayStates(),
                Names     = relay.GetRelayNames(),
            },

            // Program cycle — summary only (no logs / images)
            programs = _programs.GetProgramsSummary(),

            // Tool repository
            lastToolUpdate = robot.toolRepo.LastUpdatedUnixMs,
            activeTool     = robot.ActiveToolName,

            // Local repository
            lastLocalUpdate = robot.localRepo.LastUpdatedUnixMs,
            activeLocal     = robot.ActiveLocalName,

            // Background programs currently running
            backgroundPrograms = _background.GetStatuses()
                .Select(s => new { name = s.Name, currentStep = s.CurrentStep })
                .ToList(),

            // Built program repository
            lastBuiltProgramUpdate = robot.builtProgramRepo.LastUpdatedUnixMs,

            // Grid repository
            lastGridUpdate = robot.gridRepo.LastUpdatedUnixMs,

            // Stack repository
            lastStackUpdate = robot.stackRepo.LastUpdatedUnixMs,

            speedOverridePercent = robot.SpeedOverrideFactor * 100.0,

            // Joint soft-limit fault state
            faulted            = fault.Faulted,
            faultJoint         = fault.Joint,
            faultDirection     = fault.Direction,
            faultMessage       = fault.Message,
            limitBypass        = fault.Bypass,
            jointLimitsEnabled = config.JointLimitsEnabled,
            robotType          = config.RobotType,

            version = RobotController.Version,
            isLinux = System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Linux),
        };
    }

    private object? Update(CommandMessage msg)
    {
        if (!System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Linux))
            return new { ok = false, error = "Update is only supported on Linux." };

        _ = Task.Run(async () =>
        {
            try
            {
                Console.WriteLine("[Update] Fetching latest release from GitHub…");
                using var http = new System.Net.Http.HttpClient();
                http.DefaultRequestHeaders.Add("User-Agent", "SimpleRobotController");

                var json = await http.GetStringAsync(
                    "https://api.github.com/repos/RobotsMadeSimple/SimpleRobotController/releases/latest");
                using var doc = JsonDocument.Parse(json);

                string? downloadUrl = null;
                foreach (var asset in doc.RootElement.GetProperty("assets").EnumerateArray())
                {
                    if (asset.GetProperty("name").GetString() == "SimpleRobotController")
                    {
                        downloadUrl = asset.GetProperty("browser_download_url").GetString();
                        break;
                    }
                }

                if (downloadUrl == null)
                {
                    Console.WriteLine("[Update] Linux binary not found in latest release.");
                    return;
                }

                var exePath = Environment.ProcessPath
                    ?? System.Diagnostics.Process.GetCurrentProcess().MainModule!.FileName;
                var tempPath = exePath + ".update";

                Console.WriteLine($"[Update] Downloading {downloadUrl}…");
                var bytes = await http.GetByteArrayAsync(downloadUrl);
                await File.WriteAllBytesAsync(tempPath, bytes);

                if (!OperatingSystem.IsWindows())
                    File.SetUnixFileMode(tempPath,
                        System.IO.UnixFileMode.UserRead   | System.IO.UnixFileMode.UserWrite  | System.IO.UnixFileMode.UserExecute |
                        System.IO.UnixFileMode.GroupRead  | System.IO.UnixFileMode.GroupExecute |
                        System.IO.UnixFileMode.OtherRead  | System.IO.UnixFileMode.OtherExecute);

                File.Move(tempPath, exePath, overwrite: true);
                Console.WriteLine("[Update] Binary replaced. Exiting for systemd restart…");
                await Task.Delay(500);
                Environment.Exit(0);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Update] Failed: {ex.Message}");
            }
        });

        return new { ok = true };
    }

    private void RestartController(CommandMessage msg)
    {
        _ = Task.Run(async () =>
        {
            await Task.Delay(500); // let the ack go out first
            try
            {
                RestartPlan.Execute();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Restart] Failed to launch the replacement: {ex}");
            }
            Console.WriteLine("[Restart] Exiting.");
            Environment.Exit(0);
        });
    }
}

/// <summary>
/// Relaunches the controller with the same arguments and working directory.
///
/// On a dev box (the project's .csproj sits three levels above the build output) the
/// helper script first rebuilds so the fresh code runs; the rebuild is best-effort — a
/// missing SDK or a compile error must never leave the robot without a controller, so
/// the existing binary is relaunched either way. The script waits for this process to
/// exit before building/launching (the binary and the port are ours until then).
///
/// Under systemd (INVOCATION_ID is set) the unit has Restart=always, so the right move
/// is simply to exit and let systemd bring the service back; spawning a child there
/// would race it for the port.
/// </summary>
internal static class RestartPlan
{
    public static void Execute()
    {
        var exePath = Environment.ProcessPath ?? throw new InvalidOperationException("ProcessPath is unknown");
        var args    = Environment.GetCommandLineArgs().Skip(1).ToArray();
        var cwd     = Directory.GetCurrentDirectory();
        var pid     = Environment.ProcessId;

        var baseDir    = AppContext.BaseDirectory;
        var projectDir = Path.GetFullPath(Path.Combine(baseDir, "..", "..", ".."));
        var csproj     = Directory.Exists(projectDir) ? Directory.GetFiles(projectDir, "*.csproj").FirstOrDefault() : null;
        var config     = baseDir.Contains($"{Path.DirectorySeparatorChar}Release{Path.DirectorySeparatorChar}") ? "Release" : "Debug";

        if (!OperatingSystem.IsWindows() && !string.IsNullOrEmpty(Environment.GetEnvironmentVariable("INVOCATION_ID")))
        {
            Console.WriteLine("[Restart] Running under systemd — exiting and letting the service manager restart the controller.");
            return;
        }

        var scriptPath = Path.Combine(cwd, OperatingSystem.IsWindows() ? "restart-controller.cmd" : "restart-controller.sh");
        var script     = OperatingSystem.IsWindows()
            ? WindowsScript(pid, exePath, args, cwd, csproj, config)
            : UnixScript(pid, exePath, args, cwd, csproj, config);
        File.WriteAllText(scriptPath, script);
        if (!OperatingSystem.IsWindows())
            File.SetUnixFileMode(scriptPath, UnixFileMode.UserRead | UnixFileMode.UserWrite | UnixFileMode.UserExecute);

        Console.WriteLine($"[Restart] {(csproj != null ? "Rebuilding and relaunching" : "Relaunching")} via {scriptPath}");
        Console.WriteLine($"[Restart]   exe:  {exePath}");
        Console.WriteLine($"[Restart]   args: {string.Join(" ", args)}");
        Console.WriteLine($"[Restart]   cwd:  {cwd}");

        // The helper must outlive this process. A child that shares our console dies
        // with it on Windows (the console is destroyed when we exit), so launch it
        // through ShellExecute, which gives cmd.exe a console of its own (hidden). On
        // Linux a throwaway shell backgrounds it with nohup so it is not our child
        // by the time we exit and cannot be hung up with us.
        var psi = new ProcessStartInfo { WorkingDirectory = cwd };
        if (OperatingSystem.IsWindows())
        {
            psi.UseShellExecute = true;
            psi.WindowStyle     = ProcessWindowStyle.Hidden;
            // Absolute path: the controller may have inherited a PATH without System32
            // (e.g. launched from a minimal shell), and ShellExecute would then fail to
            // find a bare "cmd.exe".
            psi.FileName        = Path.Combine(Environment.SystemDirectory, "cmd.exe");
            psi.Arguments       = $"/c \"\"{scriptPath}\"\"";
        }
        else
        {
            psi.UseShellExecute = false;
            psi.FileName        = "/bin/bash";
            psi.ArgumentList.Add("-c");
            psi.ArgumentList.Add($"nohup /bin/bash '{scriptPath.Replace("'", "'\\''")}' >/dev/null 2>&1 &");
        }
        Process.Start(psi);
    }

    /// <summary>The dotnet host to build with: the one that launched us if we run under the muxer,
    /// else the machine-wide install, else whatever "dotnet" resolves to on the script's PATH.</summary>
    private static string DotnetHost()
    {
        var host = Environment.GetEnvironmentVariable("DOTNET_HOST_PATH");
        if (!string.IsNullOrEmpty(host) && File.Exists(host)) return host;
        if (OperatingSystem.IsWindows())
        {
            var pf = Environment.GetEnvironmentVariable("ProgramFiles");
            var candidate = string.IsNullOrEmpty(pf) ? null : Path.Combine(pf, "dotnet", "dotnet.exe");
            if (candidate != null && File.Exists(candidate)) return candidate;
        }
        else
        {
            foreach (var c in new[] { "/usr/bin/dotnet", "/usr/share/dotnet/dotnet", "/usr/lib/dotnet/dotnet",
                                      Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".dotnet", "dotnet") })
                if (File.Exists(c)) return c;
        }
        return "dotnet";
    }

    private static string WindowsScript(int pid, string exe, string[] args, string cwd, string? csproj, string config)
    {
        static string Q(string s) => "\"" + s.Replace("\"", "\"\"") + "\"";
        // Every step appends to restart-controller.log next to the script, so a restart
        // that does not come back can be diagnosed from the data directory.
        var log = Q(Path.Combine(cwd, "restart-controller.log"));
        var sys = Environment.SystemDirectory;
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("@echo off");
        sb.AppendLine($"rem Waits for controller PID {pid} to exit, rebuilds (best effort), relaunches with the same arguments.");
        sb.AppendLine($"echo [%date% %time%] helper started for PID {pid}>> {log}");
        sb.AppendLine(":wait");
        sb.AppendLine($"{Q(Path.Combine(sys, "tasklist.exe"))} /FI \"PID eq {pid}\" 2>nul | {Q(Path.Combine(sys, "find.exe"))} \"{pid}\" >nul && ({Q(Path.Combine(sys, "timeout.exe"))} /t 1 /nobreak >nul & goto wait)");
        sb.AppendLine($"echo [%time%] old process gone>> {log}");
        if (csproj != null)
        {
            sb.AppendLine($"echo [%time%] building {config}>> {log}");
            sb.AppendLine($"{Q(DotnetHost())} build {Q(csproj)} -c {config} --nologo >> {log} 2>&1");
            sb.AppendLine($"echo [%time%] build exit %errorlevel%>> {log}");
        }
        sb.AppendLine($"cd /d {Q(cwd)}");
        sb.AppendLine($"start \"\" /D {Q(cwd)} {Q(exe)} {string.Join(" ", args.Select(Q))}");
        sb.AppendLine($"echo [%time%] launched %errorlevel%>> {log}");
        sb.AppendLine("exit /b 0");
        return sb.ToString();
    }

    private static string UnixScript(int pid, string exe, string[] args, string cwd, string? csproj, string config)
    {
        static string Q(string s) => "'" + s.Replace("'", "'\''") + "'";
        var sb = new System.Text.StringBuilder();
        sb.AppendLine("#!/bin/bash");
        sb.AppendLine($"# Waits for controller PID {pid} to exit, rebuilds (best effort), relaunches with the same arguments.");
        sb.AppendLine($"while kill -0 {pid} 2>/dev/null; do sleep 0.2; done");
        if (csproj != null)
            sb.AppendLine($"{Q(DotnetHost())} build {Q(csproj)} -c {config} --nologo || echo \"[restart] build failed; relaunching the existing binary\"");
        sb.AppendLine($"cd {Q(cwd)}");
        sb.AppendLine($"nohup {Q(exe)} {string.Join(" ", args.Select(Q))} >/dev/null 2>&1 &");
        return sb.ToString();
    }
}
