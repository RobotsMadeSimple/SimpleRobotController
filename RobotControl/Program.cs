using Controller.RobotControl;
using Makaretu.Dns;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Hosting;
using System.Diagnostics;

class Program
{
    static void Main(string[] args)
    {
        var robotController = new Controller.RobotControl.RobotController();

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

        var service = new ServiceProfile("RobotController", "_robot._tcp", 9000);
        service.AddProperty("ControlEndpoint", "/control");
        service.AddProperty("RobotType", "TBot");
        service.AddProperty("SerialNumber", "1");
        service.AddProperty("RobotName", "TBot");
        var sd = new ServiceDiscovery();
        sd.Advertise(service);

        wsServer.Map(app);

        app.Run("http://0.0.0.0:9000");
    }
}
