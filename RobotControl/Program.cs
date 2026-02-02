using Controller.RobotControl;
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
            "/ws",
            async command =>
            {
                return await robotController.AddCommand(command);
            }
        );

        wsServer.Map(app);

        app.Run("http://0.0.0.0:5000");
    }
}
