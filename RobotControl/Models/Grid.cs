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
    [JsonPropertyName("basePointName")]  public string BasePointName  { get; set; } = "";

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

    [JsonPropertyName("lastUpdatedUnixMs")] public long LastUpdatedUnixMs { get; set; }
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
    [JsonPropertyName("basePointName")]  public string  BasePointName  { get; set; } = "";
    [JsonPropertyName("rowOffsetX")]     public double  RowOffsetX     { get; set; }
    [JsonPropertyName("rowOffsetY")]     public double  RowOffsetY     { get; set; }
    [JsonPropertyName("rowOffsetZ")]     public double  RowOffsetZ     { get; set; }
    [JsonPropertyName("colOffsetX")]     public double  ColOffsetX     { get; set; }
    [JsonPropertyName("colOffsetY")]     public double  ColOffsetY     { get; set; }
    [JsonPropertyName("colOffsetZ")]     public double  ColOffsetZ     { get; set; }
    [JsonPropertyName("rowCount")]       public int?    RowCount       { get; set; }
    [JsonPropertyName("colCount")]       public int?    ColCount       { get; set; }
    [JsonPropertyName("rotation")]       public double  Rotation       { get; set; }
}

public class GridIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}


