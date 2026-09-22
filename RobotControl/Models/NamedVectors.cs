using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

// ── Named-vector identity contract (shared by Point and Tool) ─────────────────

public interface INamedVector
{
    string? Name { get; set; }
    long LastUpdatedUnixMs { get; set; }
}

// ── Point ─────────────────────────────────────────────────────────────────────

public class Point : Vector6, INamedVector
{
    public string? Name { get; set; }

    // Unix ms when this point was last created or modified
    public long LastUpdatedUnixMs { get; set; }
}

// ── Tool ──────────────────────────────────────────────────────────────────────

/// <summary>TCP offset tool frame stored in the tool repository.</summary>
public class Tool : Vector6, INamedVector
{
    public string? Name { get; set; }
    public string Description { get; set; } = "";
    public long LastUpdatedUnixMs { get; set; }
}

public class ToolHistoryEntry
{
    public long TimestampUnixMs { get; set; }
    public Tool Tool { get; set; } = new();
}

// ── Tool command params ───────────────────────────────────────────────────────

public class EditToolParams
{
    [JsonPropertyName("name")]        public string  Name        { get; set; } = default!;
    [JsonPropertyName("newName")]     public string? NewName     { get; set; }
    [JsonPropertyName("description")] public string? Description { get; set; }
    [JsonPropertyName("x")]           public double? X           { get; set; }
    [JsonPropertyName("y")]           public double? Y           { get; set; }
    [JsonPropertyName("z")]           public double? Z           { get; set; }
    [JsonPropertyName("rx")]          public double? RX          { get; set; }
    [JsonPropertyName("ry")]          public double? RY          { get; set; }
    [JsonPropertyName("rz")]          public double? RZ          { get; set; }
}

public class ToolNameParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
}

// ── Local ─────────────────────────────────────────────────────────────────────

/// <summary>Named user/work-frame coordinate system stored in the local repository.</summary>
public class Local : Vector6, INamedVector
{
    public string? Name { get; set; }
    public string Description { get; set; } = "";
    public long LastUpdatedUnixMs { get; set; }
}

public class LocalHistoryEntry
{
    public long TimestampUnixMs { get; set; }
    public Local Local { get; set; } = new();
}

// ── Local command params ──────────────────────────────────────────────────────

public class EditLocalParams
{
    [JsonPropertyName("name")]        public string  Name        { get; set; } = default!;
    [JsonPropertyName("newName")]     public string? NewName     { get; set; }
    [JsonPropertyName("description")] public string? Description { get; set; }
    [JsonPropertyName("x")]           public double? X           { get; set; }
    [JsonPropertyName("y")]           public double? Y           { get; set; }
    [JsonPropertyName("z")]           public double? Z           { get; set; }
    [JsonPropertyName("rx")]          public double? RX          { get; set; }
    [JsonPropertyName("ry")]          public double? RY          { get; set; }
    [JsonPropertyName("rz")]          public double? RZ          { get; set; }
}

public class LocalNameParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
}

public class TeachPointParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
}

public class EditPointParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;

    [JsonPropertyName("newName")]
    public string? NewName { get; set; }

    [JsonPropertyName("x")]
    public double? X { get; set; }

    [JsonPropertyName("y")]
    public double? Y { get; set; }

    [JsonPropertyName("z")]
    public double? Z { get; set; }

    [JsonPropertyName("rx")]
    public double? RX { get; set; }

    [JsonPropertyName("ry")]
    public double? RY { get; set; }

    [JsonPropertyName("rz")]
    public double? RZ { get; set; }
}

public class PointHistoryEntry
{
    public long TimestampUnixMs { get; set; }
    public Point Point { get; set; } = new();
}


