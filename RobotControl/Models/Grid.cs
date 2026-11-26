using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Grid ──────────────────────────────────────────────────────────────────────

/// <summary>
/// A 2D grid of positions defined by a base point, row/column offsets, and an
/// optional rotation about the base point's Z-axis.
/// </summary>
public class Grid : Controller.RobotControl.Persistence.IStoredItem
{
    [JsonPropertyName("id")]             public string Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string Name           { get; set; } = "";
    /// <summary>Optional free-text note about the grid (what it is, where/how to teach it).</summary>
    [JsonPropertyName("description")]    public string Description    { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string BasePointName  { get; set; } = "";
    /// <summary>Stable id of the base point — resolved id-first so a renamed base still works.</summary>
    [JsonPropertyName("basePointId")]    public string BasePointId    { get; set; } = "";

    [JsonPropertyName("rowOffsetX")]     public double RowOffsetX     { get; set; }
    [JsonPropertyName("rowOffsetY")]     public double RowOffsetY     { get; set; }
    [JsonPropertyName("rowOffsetZ")]     public double RowOffsetZ     { get; set; }

    [JsonPropertyName("colOffsetX")]     public double ColOffsetX     { get; set; }
    [JsonPropertyName("colOffsetY")]     public double ColOffsetY     { get; set; }
    [JsonPropertyName("colOffsetZ")]     public double ColOffsetZ     { get; set; }

    [JsonPropertyName("rowCount")]       public int?   RowCount       { get; set; }
    [JsonPropertyName("colCount")]       public int?   ColCount       { get; set; }

    /// <summary>Degrees — rotates the row/column offsets around the base-point Z-axis.</summary>
    [JsonPropertyName("rotation")]       public double Rotation       { get; set; }

    /// <summary>How the grid was set up: "2pt" | "3pt" | "4pt" | "manual". Drives the app's re-fit.</summary>
    [JsonPropertyName("method")]         public string Method         { get; set; } = "";

    /// <summary>2-point only: which side of the first row the next row lies on ("left" | "right").</summary>
    [JsonPropertyName("rowSide")]        public string RowSide        { get; set; } = "";

    /// <summary>The taught corner points (global points) this grid was fitted from. The app re-fits the
    /// offsets from their live poses when one is re-taught or edited; empty for a manual grid.</summary>
    [JsonPropertyName("corners")]        public List<GridCorner>? Corners { get; set; }

    [JsonPropertyName("lastUpdatedUnixMs")] public long LastUpdatedUnixMs { get; set; }
}

/// <summary>A grid's taught corner: a reference to a global point plus which corner it is.</summary>
public class GridCorner
{
    /// <summary>"A" (the base/origin) | "B" | "C" | "D".</summary>
    [JsonPropertyName("role")]      public string Role      { get; set; } = "";
    [JsonPropertyName("pointId")]   public string PointId   { get; set; } = "";
    [JsonPropertyName("pointName")] public string PointName { get; set; } = "";
}

/// <summary>Identifies a cell in a named grid — used as the target position in a MoveL/MoveJ step.</summary>
public class GridPointRef
{
    [JsonPropertyName("gridId")]       public string  GridId       { get; set; } = "";
    [JsonPropertyName("rowIndex")]     public double? RowIndex     { get; set; }
    [JsonPropertyName("colIndex")]     public double? ColIndex     { get; set; }
    [JsonPropertyName("gridIndex")]    public double? GridIndex    { get; set; }
    [JsonPropertyName("useGridIndex")] public bool    UseGridIndex { get; set; }
}


public class SaveGridParams
{
    [JsonPropertyName("id")]             public string  Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string  Name           { get; set; } = "";
    [JsonPropertyName("description")]    public string  Description    { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string  BasePointName  { get; set; } = "";
    [JsonPropertyName("basePointId")]    public string  BasePointId    { get; set; } = "";
    [JsonPropertyName("rowOffsetX")]     public double  RowOffsetX     { get; set; }
    [JsonPropertyName("rowOffsetY")]     public double  RowOffsetY     { get; set; }
    [JsonPropertyName("rowOffsetZ")]     public double  RowOffsetZ     { get; set; }
    [JsonPropertyName("colOffsetX")]     public double  ColOffsetX     { get; set; }
    [JsonPropertyName("colOffsetY")]     public double  ColOffsetY     { get; set; }
    [JsonPropertyName("colOffsetZ")]     public double  ColOffsetZ     { get; set; }
    [JsonPropertyName("rowCount")]       public int?    RowCount       { get; set; }
    [JsonPropertyName("colCount")]       public int?    ColCount       { get; set; }
    [JsonPropertyName("rotation")]       public double  Rotation       { get; set; }
    [JsonPropertyName("method")]         public string  Method         { get; set; } = "";
    [JsonPropertyName("rowSide")]        public string  RowSide        { get; set; } = "";
    [JsonPropertyName("corners")]        public List<GridCorner>? Corners { get; set; }
}

public class GridIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}


