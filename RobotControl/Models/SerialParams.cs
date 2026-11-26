using System.Collections.Generic;
using System.IO.Ports;
using System.Text.Json.Serialization;
using Controller.RobotControl.Serial;

namespace Controller.RobotControl;

// ── Generic serial device command params ────────────────────────────────────────

public class AddSerialDeviceParams
{
    [JsonPropertyName("name")]            public string    Name            { get; set; } = "";
    [JsonPropertyName("portName")]        public string    PortName        { get; set; } = "";
    [JsonPropertyName("baudRate")]        public int       BaudRate        { get; set; } = 9600;
    [JsonPropertyName("dataBits")]        public int?      DataBits        { get; set; }
    [JsonPropertyName("parity")]          public Parity?   Parity          { get; set; }
    [JsonPropertyName("stopBits")]        public StopBits? StopBits        { get; set; }
    [JsonPropertyName("writeLineEnding")] public string?   WriteLineEnding { get; set; }
}

public class SerialDeviceIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class SetSerialDeviceConfigParams
{
    [JsonPropertyName("id")]              public string    Id              { get; set; } = "";
    [JsonPropertyName("name")]            public string?   Name            { get; set; }
    [JsonPropertyName("portName")]        public string?   PortName        { get; set; }
    [JsonPropertyName("baudRate")]        public int?      BaudRate        { get; set; }
    [JsonPropertyName("dataBits")]        public int?      DataBits        { get; set; }
    [JsonPropertyName("parity")]          public Parity?   Parity          { get; set; }
    [JsonPropertyName("stopBits")]        public StopBits? StopBits        { get; set; }
    [JsonPropertyName("writeLineEnding")] public string?   WriteLineEnding { get; set; }
    [JsonPropertyName("autoConnect")]     public bool?     AutoConnect     { get; set; }
    [JsonPropertyName("commands")]        public List<SerialCommandDef>? Commands { get; set; }
}

public class SendSerialCommandParams
{
    [JsonPropertyName("id")]             public string  Id             { get; set; } = "";
    [JsonPropertyName("commandId")]      public string? CommandId      { get; set; }
    [JsonPropertyName("text")]           public string? Text           { get; set; }
    [JsonPropertyName("expectReply")]    public bool?   ExpectReply    { get; set; }
    [JsonPropertyName("replyTimeoutMs")] public int?    ReplyTimeoutMs { get; set; }
}

public class SetSerialDeviceConnectedParams
{
    [JsonPropertyName("id")]        public string Id        { get; set; } = "";
    [JsonPropertyName("connected")] public bool   Connected { get; set; }
}
