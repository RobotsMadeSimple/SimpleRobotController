using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Named-vector identity contract (shared by Point and Tool) ─────────────────

public interface INamedVector
{
    string? Name { get; set; }
    /// <summary>
    /// Stable identity assigned once on create and preserved across renames/re-teaches.
    /// Program steps link by this id so a rename never breaks the link; Name is only a
    /// display label / convenience alias. Backfilled for data saved before ids existed
    /// (see NamedVectorRepository.EnsureIdentity).
    /// </summary>
    string? Id { get; set; }
    /// <summary>Optional free-text note describing the item's purpose. Preserved across
    /// re-teach (see NamedVectorRepository.SaveItem) so it isn't lost when the pose changes.</summary>
    string Description { get; set; }
    long LastUpdatedUnixMs { get; set; }
}

// ── Point ─────────────────────────────────────────────────────────────────────

public class Point : Vector6, INamedVector
{
    public string? Name { get; set; }

    /// <summary>
    /// Stable identity, assigned once on create and preserved across renames and
    /// re-teaches. Program steps reference a point by this id so renaming the point
    /// never breaks the link. Serialized as lowercase "id" to match the app.
    /// Backfilled for points saved before ids existed (see PointRepository).
    /// </summary>
    [JsonPropertyName("id")]
    public string? Id { get; set; }

    /// <summary>Optional free-text note describing what the point is for. Edited any time;
    /// never required when teaching. Mirrors <see cref="Tool.Description"/> / <see cref="Local.Description"/>.</summary>
    public string Description { get; set; } = "";

    /// <summary>Program points only: true until the point has been taught a position (added in the
    /// builder with a name but not yet jogged to). Stored so the untaught state survives save/reload.
    /// Global library points are always taught and leave this false.</summary>
    [JsonPropertyName("untaught")]
    public bool Untaught { get; set; }

    // Unix ms when this point was last created or modified
    public long LastUpdatedUnixMs { get; set; }
}

// ── Tool ──────────────────────────────────────────────────────────────────────

/// <summary>TCP offset tool frame stored in the tool repository.</summary>
public class Tool : Vector6, INamedVector
{
    public string? Name { get; set; }

    /// <summary>Stable identity — see <see cref="INamedVector.Id"/>. Program steps link by this.</summary>
    [JsonPropertyName("id")]
    public string? Id { get; set; }

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

    /// <summary>Stable identity — see <see cref="INamedVector.Id"/>. Program steps link by this.</summary>
    [JsonPropertyName("id")]
    public string? Id { get; set; }

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

    /// <summary>Optional description to set when teaching a brand-new point in one step.</summary>
    [JsonPropertyName("description")]
    public string? Description { get; set; }
}

public class EditPointParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;

    [JsonPropertyName("newName")]
    public string? NewName { get; set; }

    [JsonPropertyName("description")]
    public string? Description { get; set; }

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


