using System.Collections.Generic;
using System.IO.Ports;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Serial
{
    // ── Config model — serial_devices.json ─────────────────────────────────────

    /// <summary>Root of serial_devices.json — the user-defined generic serial devices.</summary>
    public sealed class SerialDevicesConfig
    {
        [JsonPropertyName("serialDevices")]
        public List<SerialDeviceConfig> SerialDevices { get; set; } = new();
    }

    /// <summary>One generic serial device: a user-chosen port, its framing, and named commands.</summary>
    public sealed class SerialDeviceConfig
    {
        [JsonPropertyName("id")]              public string Id              { get; set; } = "";
        [JsonPropertyName("name")]            public string Name            { get; set; } = "";
        [JsonPropertyName("portName")]        public string PortName        { get; set; } = "";
        [JsonPropertyName("baudRate")]        public int    BaudRate        { get; set; } = 9600;
        [JsonPropertyName("dataBits")]        public int    DataBits        { get; set; } = 8;
        [JsonPropertyName("parity")]          public Parity Parity          { get; set; } = Parity.None;
        [JsonPropertyName("stopBits")]        public StopBits StopBits      { get; set; } = StopBits.One;
        /// <summary>lf (\n) | crlf (\r\n) | cr (\r) | none (no terminator).</summary>
        [JsonPropertyName("writeLineEnding")] public string WriteLineEnding { get; set; } = "lf";
        [JsonPropertyName("autoConnect")]     public bool   AutoConnect     { get; set; } = true;
        [JsonPropertyName("commands")]        public List<SerialCommandDef> Commands { get; set; } = new();

        /// <summary>The byte(s) appended to a sent payload for this device's writeLineEnding.</summary>
        public static string LineEnding(string? writeLineEnding) =>
            (writeLineEnding ?? "lf").Trim().ToLowerInvariant() switch
            {
                "crlf" => "\r\n",
                "cr"   => "\r",
                "none" => "",
                _      => "\n",
            };
    }

    /// <summary>A saved, named command a device can send.</summary>
    public sealed class SerialCommandDef
    {
        [JsonPropertyName("id")]             public string Id             { get; set; } = "";
        [JsonPropertyName("name")]           public string Name           { get; set; } = "";
        [JsonPropertyName("payload")]        public string Payload        { get; set; } = "";
        [JsonPropertyName("expectReply")]    public bool   ExpectReply    { get; set; }
        [JsonPropertyName("replyTimeoutMs")] public int    ReplyTimeoutMs { get; set; } = 1000;
    }

    // ── Wire state ─────────────────────────────────────────────────────────────

    /// <summary>One line in a device's rolling console buffer.</summary>
    public sealed class SerialLineEntry
    {
        /// <summary>"tx" (sent) | "rx" (received).</summary>
        [JsonPropertyName("dir")]  public string Dir  { get; set; } = "";
        [JsonPropertyName("text")] public string Text { get; set; } = "";
    }

    /// <summary>Live state broadcast for one serial device (GetStatus / GetSerialDevices).</summary>
    public sealed class SerialDeviceState
    {
        [JsonPropertyName("id")]              public string Id              { get; set; } = "";
        [JsonPropertyName("name")]            public string Name            { get; set; } = "";
        [JsonPropertyName("portName")]        public string PortName        { get; set; } = "";
        [JsonPropertyName("baudRate")]        public int    BaudRate        { get; set; }
        [JsonPropertyName("dataBits")]        public int    DataBits        { get; set; } = 8;
        [JsonPropertyName("parity")]          public Parity Parity          { get; set; } = Parity.None;
        [JsonPropertyName("stopBits")]        public StopBits StopBits      { get; set; } = StopBits.One;
        [JsonPropertyName("writeLineEnding")] public string WriteLineEnding { get; set; } = "lf";
        [JsonPropertyName("autoConnect")]     public bool   AutoConnect     { get; set; } = true;
        [JsonPropertyName("connected")]       public bool   Connected       { get; set; }
        [JsonPropertyName("recentLines")]     public List<SerialLineEntry> RecentLines { get; set; } = new();
        [JsonPropertyName("commands")]        public List<SerialCommandDef> Commands   { get; set; } = new();
    }

    /// <summary>A serial port with its descriptive info and in-use status (ListSerialPorts).</summary>
    public sealed class SerialPortDescriptor
    {
        [JsonPropertyName("portName")]     public string PortName     { get; set; } = "";
        [JsonPropertyName("description")]  public string Description   { get; set; } = "";
        [JsonPropertyName("manufacturer")] public string Manufacturer { get; set; } = "";
        [JsonPropertyName("vid")]          public string Vid          { get; set; } = "";
        [JsonPropertyName("pid")]          public string Pid          { get; set; } = "";
        [JsonPropertyName("serialNumber")] public string SerialNumber { get; set; } = "";
        [JsonPropertyName("inUse")]        public bool   InUse        { get; set; }
        [JsonPropertyName("usedBy")]       public string UsedBy       { get; set; } = "";
    }
}
