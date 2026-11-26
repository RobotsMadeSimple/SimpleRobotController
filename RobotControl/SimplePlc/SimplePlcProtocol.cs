using System.Text.Json;

namespace Controller.RobotControl.SimplePlc
{
    /// <summary>
    /// Constants and small helpers for the SimplePLC head's native host protocol: UDP discovery on
    /// 47810, newline-delimited JSON over TCP 47811. One protocol version (<see cref="ProtoVersion"/>).
    /// The message shapes are built with anonymous objects and read as <see cref="JsonElement"/>; this
    /// type only holds the wire constants and the type-id → name map.
    /// </summary>
    public static class SimplePlcProtocol
    {
        public const int ProtoVersion   = 1;
        public const int DiscoveryPort  = 47810;
        public const int DefaultTcpPort = 47811;
        /// <summary>Max bytes per line on the wire (protocol §4.2).</summary>
        public const int MaxLineBytes   = 1024;

        /// <summary>The client name we present in <c>hello</c>.</summary>
        public const string ClientName = "SimpleRobotController";

        public static readonly JsonSerializerOptions Json = new()
        {
            PropertyNamingPolicy = null, // keys are already the exact protocol spelling
        };

        /// <summary>Module type name from its type id (for display and the IO channel labels).</summary>
        public static string TypeName(int typeId) => typeId switch
        {
            1 => "DI8",
            2 => "DO8",
            3 => "RLY4",
            4 => "AI4",
            5 => "AO2",
            6 => "DO8N",   // sinking/NPN twin of DO8 — wire-identical (8 outputs, field power)
            _ => "",
        };

        /// <summary>Serialize a request object to one wire line's JSON (no trailing newline).</summary>
        public static string Encode(object request) => JsonSerializer.Serialize(request, Json);

        /// <summary>The <c>op</c> field of a parsed message, or "" if absent.</summary>
        public static string Op(JsonElement msg) =>
            msg.ValueKind == JsonValueKind.Object && msg.TryGetProperty("op", out var op) && op.ValueKind == JsonValueKind.String
                ? op.GetString() ?? "" : "";
    }
}
