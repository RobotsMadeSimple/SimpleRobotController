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

        // Use serial number as the mDNS instance name so each robot is addressable uniquely
        var service = new ServiceProfile(identity.SerialNumber, "_robot._tcp", 9000);
        service.AddProperty("ControlEndpoint", "/control");
        service.AddProperty("SerialNumber",    identity.SerialNumber);
        service.AddProperty("RobotType",       identity.RobotType);
        service.AddProperty("RobotName",       identity.RobotName);
        var sd = new ServiceDiscovery();
        sd.Advertise(service);

        Console.WriteLine($"[mDNS] Advertising as '{identity.SerialNumber}._robot._tcp' " +
                          $"(Type: '{identity.RobotType}', Name: '{identity.RobotName}')");

        wsServer.Map(app);

        app.Run("http://0.0.0.0:9000");
    }
}
