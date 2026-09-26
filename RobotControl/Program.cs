using Controller.RobotControl;
using Controller.RobotControl.Hosting;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using System.Runtime.InteropServices;

class Program
{
    static void Main(string[] args)
    {
        // ── CLI args: --port <n>  --data <dir> ────────────────────────────────
        int    port    = 9000;
        string dataDir = AppContext.BaseDirectory;

        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i] == "--port" && int.TryParse(args[i + 1], out int p)) port = p;
            if (args[i] == "--data") dataDir = Path.GetFullPath(args[i + 1]);
        }

        Directory.CreateDirectory(dataDir);
        // All relative-path file operations (repos, config, identity, relay) resolve
        // against this directory, so each robot instance gets its own isolated data.
        Directory.SetCurrentDirectory(dataDir);

        Console.WriteLine($"[Boot] Simple Robot Controller {RobotController.Version}");
        Console.WriteLine($"[Boot] Port: {port}  Data: {dataDir}");
        Console.WriteLine($"[Boot] OS: {RuntimeInformation.OSDescription}");
        Console.WriteLine($"[Boot] Runtime: {RuntimeInformation.FrameworkDescription}");

        // Last-resort diagnostic — catches unhandled exceptions from any thread that
        // weren't caught by the control-loop try/catch (e.g. background Task faults).
        AppDomain.CurrentDomain.UnhandledException += (_, e) =>
            Console.WriteLine($"[FATAL] Unhandled exception: {e.ExceptionObject}");

        // ── Controller ────────────────────────────────────────────────────────
        var identity = RobotIdentityService.Load();
        var config   = RobotConfigService.Load();

        var robot = new RobotController();
        robot.SetIdentity(identity);
        robot.SetConfig(config);
        // Start devices and the motion/program threads only once the real
        // identity and config are applied (the first ticks must not see defaults).
        robot.Start();

        // ── Web host ──────────────────────────────────────────────────────────
        var builder = WebApplication.CreateBuilder(args);
        var app     = builder.Build();
        app.UseWebSockets();

        var lifetime = app.Services.GetRequiredService<IHostApplicationLifetime>();

        var wsServer = new RobotWebSocketServer("/control", async cmd => await robot.AddCommand(cmd));
        // Wire shutdown token so active WebSocket receive loops unblock immediately.
        wsServer.SetShutdownToken(lifetime.ApplicationStopping);
        wsServer.Map(app);

        app.MapRobotEndpoints(robot, vectorFileDir: "dxf");

        // ── mDNS discovery ────────────────────────────────────────────────────
        // Optional: set "enableMdns": false in robot-config.json to run without
        // network discovery (the robot is then only reachable by direct address).
        if (config.EnableMdns)
        {
            var mdns = new MdnsAdvertiser(identity, port, lifetime);
            robot.OnIdentityChanged = mdns.IdentityChanged;
            mdns.Start();
        }
        else
        {
            Console.WriteLine("[mDNS] Disabled via robot-config.json (enableMdns=false) — not advertising");
        }

        app.Run($"http://0.0.0.0:{port}");
    }
}
