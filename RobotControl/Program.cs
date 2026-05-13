using Controller.RobotControl;
using Makaretu.Dns;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Hosting;
using System.Diagnostics;

class Program
{
    static void Main(string[] args)
    {
        // Ensure relative file paths work when running as a Windows Service
        Directory.SetCurrentDirectory(AppContext.BaseDirectory);

        var identity = RobotIdentityService.Load();
        var config   = RobotConfigService.Load();

        var robotController = new Controller.RobotControl.RobotController();
        robotController.SetIdentity(identity);
        robotController.SetConfig(config);

        // ---- Web server ----
        var builder = WebApplication.CreateBuilder(args);
        builder.Host.UseWindowsService();
        var app = builder.Build();
        app.UseWebSockets();

        var wsServer = new RobotWebSocketServer(
            "/control",
            async command =>
            {
                return await robotController.AddCommand(command);
            }
        );

        ServiceProfile BuildProfile(RobotIdentity id)
        {
            var p = new ServiceProfile(id.SerialNumber, "_robot._tcp", 9000);
            p.AddProperty("ControlEndpoint", "/control");
            p.AddProperty("SerialNumber",    id.SerialNumber);
            p.AddProperty("RobotType",       id.RobotType);
            p.AddProperty("RobotName",       id.RobotName);
            return p;
        }

        var sd = new ServiceDiscovery();
        var service = BuildProfile(identity);
        sd.Advertise(service);

        Console.WriteLine($"[mDNS] Advertising as '{identity.SerialNumber}._robot._tcp' " +
                          $"(Type: '{identity.RobotType}', Name: '{identity.RobotName}')");

        robotController.OnIdentityChanged = updated =>
        {
            _ = Task.Run(async () =>
            {
                sd.Unadvertise(service);
                await Task.Delay(250);
                service = BuildProfile(updated);
                sd.Advertise(service);
                Console.WriteLine($"[mDNS] Re-advertising with Type: '{updated.RobotType}', Name: '{updated.RobotName}'");
            });
        };

        var lifetime = app.Services.GetRequiredService<IHostApplicationLifetime>();

        // Wire shutdown token so active WebSocket receive loops unblock immediately
        wsServer.SetShutdownToken(lifetime.ApplicationStopping);

        // On shutdown: unadvertise mDNS so peers stop seeing this robot instantly
        lifetime.ApplicationStopping.Register(() =>
        {
            Console.WriteLine("[mDNS] Unadvertising on shutdown…");
            try { sd.Unadvertise(service); } catch { }
            try { sd.Dispose(); } catch { }
        });

        // Periodic re-announce — exits cleanly when ApplicationStopping fires
        _ = Task.Run(async () =>
        {
            try
            {
                while (true)
                {
                    await Task.Delay(3000, lifetime.ApplicationStopping);
                    try { sd.Announce(service); } catch { }
                }
            }
            catch (OperationCanceledException) { }
        });

        wsServer.Map(app);

        app.Run("http://0.0.0.0:9000");
    }
}
