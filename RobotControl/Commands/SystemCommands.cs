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
            await Task.Delay(500);
            try
            {
                // On a dev machine the project source sits three levels above the
                // build output (…/RobotControl/bin/<Config>/net10.0/). When it's
                // present, rebuild the latest code and relaunch the fresh binary
                // instead of re-running the stale one. In production (published,
                // no .csproj) we just relaunch the current binary as before.
                var baseDir    = AppContext.BaseDirectory;
                var projectDir = Path.GetFullPath(Path.Combine(baseDir, "..", "..", ".."));
                var csproj     = Directory.Exists(projectDir)
                    ? Directory.GetFiles(projectDir, "*.csproj").FirstOrDefault()
                    : null;
                var exePath = Environment.ProcessPath;

                if (csproj != null && exePath != null)
                {
                    var sep    = Path.DirectorySeparatorChar;
                    var config = baseDir.Contains($"{sep}Release{sep}") ? "Release" : "Debug";
                    var psi = new System.Diagnostics.ProcessStartInfo
                    {
                        UseShellExecute  = true,
                        WorkingDirectory = projectDir,
                    };
                    if (OperatingSystem.IsWindows())
                    {
                        // Wait for this process to release its own binary, rebuild, then relaunch.
                        psi.FileName  = "cmd.exe";
                        psi.Arguments = $"/c timeout /t 2 /nobreak >nul & dotnet build \"{csproj}\" -c {config} --nologo && start \"\" \"{exePath}\"";
                    }
                    else
                    {
                        psi.FileName  = "/bin/bash";
                        psi.Arguments = $"-c \"sleep 2 && dotnet build '{csproj}' -c {config} --nologo && nohup '{exePath}' >/dev/null 2>&1 &\"";
                    }
                    System.Diagnostics.Process.Start(psi);
                }
                else if (exePath != null)
                {
                    System.Diagnostics.Process.Start(exePath);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Restart] Failed: {ex.Message}");
            }
            Environment.Exit(0);
        });
    }
}
