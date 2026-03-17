using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;

public class RobotWebSocketServer
{
    private readonly PathString _path;
    private readonly Func<CommandMessage, Task<object?>> _commandHandler;

    public RobotWebSocketServer(
        PathString path,
        Func<CommandMessage, Task<object?>> commandHandler)
    {
        _path = path;
        _commandHandler = commandHandler;
    }

    public void Map(IApplicationBuilder app)
    {
        app.Map(_path, wsApp =>
        {
            wsApp.Run(async context =>
            {
                if (!context.WebSockets.IsWebSocketRequest)
                {
                    context.Response.StatusCode = 400;
                    return;
                }

                using var socket = await context.WebSockets.AcceptWebSocketAsync();
                await HandleClient(socket);
            });
        });
    }

    private async Task HandleClient(WebSocket socket)
    {
        var buffer = new byte[4096];

        while (socket.State == WebSocketState.Open)
        {
            var result = await socket.ReceiveAsync(buffer, CancellationToken.None);
            if (result.Count > 0)
            {
                var json = Encoding.UTF8.GetString(buffer, 0, result.Count);
                await HandleMessage(socket, json);
            }
        }

        await socket.CloseAsync(
            WebSocketCloseStatus.NormalClosure,
            "Closing",
            CancellationToken.None);
    }

    private static readonly JsonSerializerOptions JsonOpts = new()
    {
        PropertyNameCaseInsensitive = true
    };

    private async Task HandleMessage(WebSocket socket, string json)
    {
        //Console.WriteLine("New Message" + json);
        string? type;

        try
        {
            using var doc = JsonDocument.Parse(json);
            if (!doc.RootElement.TryGetProperty("type", out var typeProp))
            {
                await SendJson(socket, new
                {
                    Type = "error",
                    Error = "missingType"
                });
                return;
            }

            type = typeProp.GetString();
        }
        catch (JsonException)
        {
            await SendJson(socket, new
            {
                Type = "error",
                Error = "invalidJson",
                ReturnData = json
            });
            return;
        }

        if (type == null)
            return;

        switch (type)
        {
            case "Command":
                {
                    var cmd = JsonSerializer.Deserialize<CommandMessage>(json, JsonOpts);
                    if (cmd == null)
                        return;

                    var resultObj = await _commandHandler(cmd);

                    // Base ACK object
                    var ack = new Dictionary<string, object?>
                    {
                        ["type"] = "ack",
                        ["command"] = cmd.Command.Clone(),
                        ["id"] = cmd.Id,
                        ["ok"] = true
                    };

                    if (resultObj != null)
                    {
                        // Serialize result object to JSON
                        var resultJson = JsonSerializer.Serialize(resultObj);

                        // Parse into JsonDocument
                        using var doc = JsonDocument.Parse(resultJson);

                        if (doc.RootElement.ValueKind == JsonValueKind.Object)
                        {
                            foreach (var prop in doc.RootElement.EnumerateObject())
                            {
                                ack[prop.Name] = prop.Value.Clone();
                            }
                        }
                    }

                    await SendJson(socket, ack);
                    break;
                }


            default:
                await SendJson(socket, new
                {
                    Type = "error",
                    Error = "unknownMessageType",
                    Received = type
                });
                break;
        }
    }



    private static async Task SendJson(WebSocket socket, object payload)
    {
        var json = JsonSerializer.Serialize(payload);
        var bytes = Encoding.UTF8.GetBytes(json);

        await socket.SendAsync(
            bytes,
            WebSocketMessageType.Text,
            true,
            CancellationToken.None);
    }
}
