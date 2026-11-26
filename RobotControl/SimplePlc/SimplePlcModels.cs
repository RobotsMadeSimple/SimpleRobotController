using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.SimplePlc
{
    // SimplePLC: a 24 V Ethernet IO head with side-plugging IO modules (DI8 inputs, DO8 outputs, …).
    // The controller talks to it over the head's native host protocol: UDP discovery on 47810, then a
    // newline-delimited-JSON TCP session on 47811 (claim to own outputs, a host watchdog the owner keeps
    // fed, pushed input/module/fault events). Each module slot's bits are exposed as controller IO so
    // programs and the IO dashboard read inputs and write outputs like any other device.
    //
    // Config (below) is persisted in simple_plc.json; the live wire state is rebuilt from the head.

    // ── Config file models (simple_plc.json) ───────────────────────────────────

    /// <summary>Root of simple_plc.json — the user-added SimplePLC heads.</summary>
    public sealed class SimplePlcConfig
    {
        [JsonPropertyName("simplePlcDevices")]
        public List<SimplePlcDeviceConfig> SimplePlcDevices { get; set; } = new();
    }

    /// <summary>One configured SimplePLC head: where to reach it and the settings we push on connect.</summary>
    public sealed class SimplePlcDeviceConfig
    {
        /// <summary>Stable controller-side id (PLC_001). Never the head MAC — the user may re-point a name.</summary>
        [JsonPropertyName("id")]          public string Id          { get; set; } = "";
        /// <summary>Human-readable label shown in the UI.</summary>
        [JsonPropertyName("name")]        public string Name        { get; set; } = "";
        /// <summary>Head IP or hostname (from discovery, or typed by the user).</summary>
        [JsonPropertyName("host")]        public string Host        { get; set; } = "";
        [JsonPropertyName("port")]        public int    Port        { get; set; } = SimplePlcProtocol.DefaultTcpPort;
        /// <summary>Head MAC as reported by discovery ("02:53:50:1A:2B:3C"); the hardware identity, used to
        /// recognise the same head if its IP changes. Empty until first seen.</summary>
        [JsonPropertyName("mac")]         public string Mac         { get; set; } = "";
        /// <summary>Open the session (and claim ownership) automatically when the controller starts.</summary>
        [JsonPropertyName("autoConnect")] public bool   AutoConnect { get; set; } = true;

        // Settings pushed to the head with `config` once owned (4.4). Kept here so a reconnect restores them.
        /// <summary>Host watchdog (ms): the head faults if the owner goes quiet this long. 0 = off. 50..60000.</summary>
        [JsonPropertyName("watchdogMs")]  public int    WatchdogMs  { get; set; } = 500;
        /// <summary>Input debounce filter (ms) applied on the head.</summary>
        [JsonPropertyName("filterMs")]    public int    FilterMs    { get; set; } = 3;
        /// <summary>What outputs do when the watchdog trips: "hold" (keep state, default) or "off".</summary>
        [JsonPropertyName("onTimeout")]   public string OnTimeout   { get; set; } = "hold";
    }

    // ── Wire state (GetStatus / GetSimplePlcDevices) ────────────────────────────

    /// <summary>One module slot's live state, broadcast for the config screen.</summary>
    public sealed class SimplePlcSlotState
    {
        [JsonPropertyName("slot")]    public int    Slot    { get; set; }
        /// <summary>Module type name reported by the head: "DI8", "DO8", … ("" if unknown).</summary>
        [JsonPropertyName("type")]    public string Type    { get; set; } = "";
        [JsonPropertyName("typeId")]  public int    TypeId  { get; set; }
        /// <summary>Input bit count for this slot.</summary>
        [JsonPropertyName("inBits")]  public int    InBits  { get; set; }
        /// <summary>Output bit count for this slot.</summary>
        [JsonPropertyName("outBits")] public int    OutBits { get; set; }
        /// <summary>"ok" or "lost" (stopped answering on the module bus).</summary>
        [JsonPropertyName("state")]   public string State   { get; set; } = "ok";
        /// <summary>Latest input bits packed into an integer (bit 0 = input 0).</summary>
        [JsonPropertyName("inValue")]  public int   InValue  { get; set; }
        /// <summary>Latest commanded output bits packed into an integer.</summary>
        [JsonPropertyName("outValue")] public int   OutValue { get; set; }
        /// <summary>For output modules that report it: whether their 24 V field supply is present.
        /// Null for modules that don't have a field-power rail (e.g. inputs).</summary>
        [JsonPropertyName("fieldPower")] public bool? FieldPower { get; set; }
    }

    /// <summary>One active fault on a head (watchdog / supply_low / field_power / module_restart).</summary>
    public sealed class SimplePlcFault
    {
        /// <summary>Fault code, e.g. "watchdog", "supply_low", "field_power".</summary>
        [JsonPropertyName("code")] public string Code { get; set; } = "";
        /// <summary>Module slot the fault is about (field_power); null for head-wide faults.</summary>
        [JsonPropertyName("slot")] public int?   Slot { get; set; }
        /// <summary>Measured supply millivolts, when the fault carries it (supply_low).</summary>
        [JsonPropertyName("mv")]   public int?   Mv   { get; set; }
    }

    /// <summary>Live state broadcast for one SimplePLC head.</summary>
    public sealed class SimplePlcDeviceState
    {
        [JsonPropertyName("id")]          public string Id          { get; set; } = "";
        [JsonPropertyName("name")]        public string Name        { get; set; } = "";
        [JsonPropertyName("host")]        public string Host        { get; set; } = "";
        [JsonPropertyName("port")]        public int    Port        { get; set; }
        [JsonPropertyName("mac")]         public string Mac         { get; set; } = "";
        [JsonPropertyName("autoConnect")] public bool   AutoConnect { get; set; } = true;
        /// <summary>TCP session is open.</summary>
        [JsonPropertyName("connected")]   public bool   Connected   { get; set; }
        /// <summary>We hold the write claim (ownership) on the head.</summary>
        [JsonPropertyName("owned")]       public bool   Owned       { get; set; }
        /// <summary>The head's own name (`SimplePLC-1A2B3C` or a renamed one).</summary>
        [JsonPropertyName("headName")]    public string HeadName    { get; set; } = "";
        [JsonPropertyName("firmware")]    public string Firmware    { get; set; } = "";
        /// <summary>Where the head's MAC comes from: "eeprom" (unique) or "default" (shared — needs a UI
        /// warning, since every default-MAC head on a network collides). "" if the head didn't report it.</summary>
        [JsonPropertyName("macSource")]   public string MacSource   { get; set; } = "";
        /// <summary>Head 24 V supply in millivolts (0 if the head didn't report it).</summary>
        [JsonPropertyName("supplyMv")]    public int    SupplyMv    { get; set; }
        [JsonPropertyName("watchdogMs")]  public int    WatchdogMs  { get; set; }
        [JsonPropertyName("filterMs")]    public int    FilterMs    { get; set; }
        [JsonPropertyName("onTimeout")]   public string OnTimeout   { get; set; } = "hold";
        /// <summary>Primary active fault code for simple UIs ("watchdog"/"supply_low"/…), else "". See
        /// <see cref="Faults"/> for the full set.</summary>
        [JsonPropertyName("fault")]       public string Fault       { get; set; } = "";
        /// <summary>All active faults on the head (a head can fault on several things at once).</summary>
        [JsonPropertyName("faults")]      public List<SimplePlcFault> Faults { get; set; } = new();
        /// <summary>Last connection error, for the UI ("" when fine).</summary>
        [JsonPropertyName("lastError")]   public string LastError   { get; set; } = "";
        [JsonPropertyName("slots")]       public List<SimplePlcSlotState> Slots { get; set; } = new();
    }

    /// <summary>A head found by UDP discovery (DiscoverSimplePlc).</summary>
    public sealed class SimplePlcDiscovered
    {
        [JsonPropertyName("name")]    public string Name    { get; set; } = "";
        [JsonPropertyName("mac")]     public string Mac     { get; set; } = "";
        [JsonPropertyName("host")]    public string Host    { get; set; } = "";
        [JsonPropertyName("port")]    public int    Port    { get; set; } = SimplePlcProtocol.DefaultTcpPort;
        [JsonPropertyName("firmware")] public string Firmware { get; set; } = "";
        [JsonPropertyName("modules")] public int    Modules { get; set; }
        /// <summary>True when a configured device already points at this MAC.</summary>
        [JsonPropertyName("added")]   public bool   Added   { get; set; }
    }
}
