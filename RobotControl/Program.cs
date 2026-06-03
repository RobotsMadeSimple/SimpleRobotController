using Controller.RobotControl;
using Makaretu.Dns;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Hosting;
using System.Diagnostics;

class Program
{
    static void Main(string[] args)
    {
        // Ensure config files load relative to the exe when launched from startup folder
        Directory.SetCurrentDirectory(AppContext.BaseDirectory);

        var version = Controller.RobotControl.RobotController.Version;
        Console.WriteLine($"[Boot] Simple Robot Controller {version}");
        Console.WriteLine($"[Boot] OS: {System.Runtime.InteropServices.RuntimeInformation.OSDescription}");
        Console.WriteLine($"[Boot] Runtime: {System.Runtime.InteropServices.RuntimeInformation.FrameworkDescription}");

        var identity = RobotIdentityService.Load();
        var config   = RobotConfigService.Load();

        var robotController = new Controller.RobotControl.RobotController();
        robotController.SetIdentity(identity);
        robotController.SetConfig(config);

        // ---- Web server ----
        var builder = WebApplication.CreateBuilder(args);
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
        var currentIdentity = identity;
        var service = BuildProfile(identity);
        sd.Advertise(service);

        Console.WriteLine($"[mDNS] Advertising as '{identity.SerialNumber}._robot._tcp' " +
                          $"(Type: '{identity.RobotType}', Name: '{identity.RobotName}')");

        robotController.OnIdentityChanged = updated =>
        {
            _ = Task.Run(async () =>
            {
                currentIdentity = updated;
                sd.Unadvertise(service);
                await Task.Delay(250);
                service = BuildProfile(updated);
                sd.Advertise(service);
                Console.WriteLine($"[mDNS] Re-advertising with Type: '{updated.RobotType}', Name: '{updated.RobotName}'");
            });
        };

        System.Net.NetworkInformation.NetworkChange.NetworkAddressChanged += (_, _) =>
        {
            _ = Task.Run(async () =>
            {
                await Task.Delay(2000); // wait for DHCP to assign the new IP
                sd.Unadvertise(service);
                await Task.Delay(250);
                service = BuildProfile(currentIdentity);
                sd.Advertise(service);
                Console.WriteLine("[mDNS] Network address changed — re-advertising");
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
