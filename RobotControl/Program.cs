using Controller.RobotControl;
using Makaretu.Dns;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Hosting;
using System.Diagnostics;

class Program
{
    static void Main(string[] args)
    {
        var identity = RobotIdentityService.Load();

        var robotController = new Controller.RobotControl.RobotController();
        robotController.SetIdentity(identity);

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

        _ = Task.Run(async () =>
        {
            while (true)
            {
                await Task.Delay(3000);
                try { sd.Announce(service); } catch { }
            }
        });

        wsServer.Map(app);

        app.Run("http://0.0.0.0:9000");
    }
}
