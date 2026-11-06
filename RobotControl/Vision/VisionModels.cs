using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Vision
{
    [JsonConverter(typeof(JsonStringEnumConverter))]
    public enum VisionZoneShape { Rectangle, Circle, Polygon, Quad }

    /// <summary>Normalized (0–1) zone geometry relative to the camera frame.</summary>
    public class VisionZoneGeometry
    {
        [JsonPropertyName("shape")]  public VisionZoneShape Shape  { get; set; } = VisionZoneShape.Rectangle;

        // Rectangle — top-left corner + size (all 0-1 fractions)
        [JsonPropertyName("x")]      public double X      { get; set; }
        [JsonPropertyName("y")]      public double Y      { get; set; }
        [JsonPropertyName("width")]  public double Width  { get; set; } = 1;
        [JsonPropertyName("height")] public double Height { get; set; } = 1;

        /// <summary>
        /// Rectangle tilt in degrees, clockwise, about the rectangle's own center. 0 means
        /// axis-aligned and every rectangle path takes its original untilted route, so an
        /// existing zone behaves exactly as it did before rotation existed.
        ///
        /// Applied in pixel space, never in this normalized space: 0–1 coordinates are scaled
        /// by width and height independently, and rotating inside a non-uniform scale shears
        /// the rectangle instead of turning it. Ignored by circles (meaningless) and polygons
        /// (whose points are already absolute).
        /// </summary>
        [JsonPropertyName("rotation")] public double Rotation { get; set; }

        // Circle — center + radius (radius is fraction of min(width, height))
        [JsonPropertyName("cx")]     public double Cx     { get; set; } = 0.5;
        [JsonPropertyName("cy")]     public double Cy     { get; set; } = 0.5;
        [JsonPropertyName("radius")] public double Radius { get; set; } = 0.25;

        // Polygon — list of [x, y] pairs (0-1 each).
        // Quad — exactly 4 corners TL, TR, BR, BL: a skewed (perspective) zone whose grid cells
        // follow the perspective, for a board seen at an angle.
        [JsonPropertyName("points")] public List<double[]> Points { get; set; } = new();
    }

    public class BlobDetectionParams
    {
        [JsonPropertyName("minArea")]              public float MinArea             { get; set; } = 100;
        [JsonPropertyName("maxArea")]              public float MaxArea             { get; set; } = 10000;
        [JsonPropertyName("filterByCircularity")]  public bool  FilterByCircularity { get; set; } = false;
        [JsonPropertyName("minCircularity")]       public float MinCircularity      { get; set; } = 0.5f;
        [JsonPropertyName("filterByConvexity")]    public bool  FilterByConvexity   { get; set; } = false;
        [JsonPropertyName("minConvexity")]         public float MinConvexity        { get; set; } = 0.8f;
        [JsonPropertyName("filterByInertia")]      public bool  FilterByInertia     { get; set; } = false;
        [JsonPropertyName("minInertiaRatio")]      public float MinInertiaRatio     { get; set; } = 0.1f;
        [JsonPropertyName("minThreshold")]         public float MinThreshold        { get; set; } = 10;
        [JsonPropertyName("maxThreshold")]         public float MaxThreshold        { get; set; } = 200;
        [JsonPropertyName("filterByColor")]        public bool  FilterByColor       { get; set; } = false;
        [JsonPropertyName("blobColor")]            public int   BlobColor           { get; set; } = 0; // 0=dark, 255=light
    }

    /// <summary>
    /// Splits a zone into a rows×cols lattice, laid out over the zone's bounding box, so an
    /// inspection pointing at the zone is measured once per cell instead of once overall.
    /// Cells are clipped to the zone shape, so a corner cell of a circular zone only covers
    /// the part inside the circle. Null, or 1×1, means "no grid" and behaviour is unchanged.
    ///
    /// Only color coverage inspections read this today; the other inspection types ignore it.
    /// </summary>
    public class VisionZoneGrid
    {
        [JsonPropertyName("rows")] public int Rows { get; set; } = 1;
        [JsonPropertyName("cols")] public int Cols { get; set; } = 1;
    }

    /// <summary>A named spatial region — geometry only, no detection config.</summary>
    public class VisionZone
    {
        [JsonPropertyName("id")]       public string             Id       { get; set; } = "";
        [JsonPropertyName("name")]     public string             Name     { get; set; } = "";
        [JsonPropertyName("geometry")] public VisionZoneGeometry Geometry { get; set; } = new();
        [JsonPropertyName("grid")]     public VisionZoneGrid?    Grid     { get; set; }
        /// <summary>
        /// Id of a frame inspection that aligns this zone each frame: the zone is warped by the
        /// homography from the frame's reference corners to the corners detected this frame.
        /// Null = the zone is used as drawn.
        /// </summary>
        [JsonPropertyName("alignTo")]  public string?            AlignTo  { get; set; }
    }

    /// <summary>
    /// What every inspection type has in common. Lets code that only cares about identity,
    /// enablement and zone (ordering, zone overrides, used-zone outlines) treat the six typed
    /// lists on <see cref="VisionProgram"/> as one sequence. Not serialized — each type still
    /// persists through its own list.
    /// </summary>
    public interface IVisionInspection
    {
        string  Id      { get; }
        string  Name    { get; }
        bool    Enabled { get; }
        string? ZoneId  { get; set; }
    }

    /// <summary>Blob detection run; optionally restricted to a zone by center-point containment.</summary>
    public class BlobInspection : IVisionInspection
    {
        [JsonPropertyName("id")]         public string              Id         { get; set; } = "";
        [JsonPropertyName("name")]       public string              Name       { get; set; } = "";
        [JsonPropertyName("enabled")]    public bool                Enabled    { get; set; } = true;
        [JsonPropertyName("zoneId")]     public string?             ZoneId     { get; set; } = null;
        [JsonPropertyName("blobParams")] public BlobDetectionParams BlobParams { get; set; } = new();
    }

    public class ColorEntry
    {
        [JsonPropertyName("id")]        public string Id        { get; set; } = "";
        [JsonPropertyName("r")]         public int    R         { get; set; }
        [JsonPropertyName("g")]         public int    G         { get; set; }
        [JsonPropertyName("b")]         public int    B         { get; set; }
        /// <summary>0–100; maps to per-channel ±(tolerance * 2.55) in RGB space.</summary>
        [JsonPropertyName("tolerance")] public int    Tolerance { get; set; } = 20;
    }

    public class ColorCoverageInspection : IVisionInspection
    {
        [JsonPropertyName("id")]          public string           Id          { get; set; } = "";
        [JsonPropertyName("name")]        public string           Name        { get; set; } = "";
        [JsonPropertyName("enabled")]     public bool             Enabled     { get; set; } = true;
        [JsonPropertyName("zoneId")]      public string?          ZoneId      { get; set; }
        [JsonPropertyName("colors")]      public List<ColorEntry> Colors      { get; set; } = new();
        [JsonPropertyName("minCoverage")] public double?          MinCoverage { get; set; } = 50;
        [JsonPropertyName("maxCoverage")] public double?          MaxCoverage { get; set; }
    }

    /// <summary>One cell of a gridded color coverage inspection, measured on its own.</summary>
    public class ColorCellResult
    {
        [JsonPropertyName("row")]      public int    Row      { get; set; }
        [JsonPropertyName("col")]      public int    Col      { get; set; }
        /// <summary>Row-major position in the grid: row * cols + col.</summary>
        [JsonPropertyName("index")]    public int    Index    { get; set; }
        [JsonPropertyName("coverage")] public double Coverage { get; set; }
        [JsonPropertyName("passed")]   public bool   Passed   { get; set; }
    }

    public class ColorCoverageResult
    {
        [JsonPropertyName("inspectionId")] public string InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string Name         { get; set; } = "";
        /// <summary>Coverage over the whole zone, gridded or not.</summary>
        [JsonPropertyName("coverage")]     public double Coverage     { get; set; }
        /// <summary>
        /// Whole-zone min/max test — except on a gridded zone, where it means *every cell*
        /// passed. A zone-wide average hides half-full cells, which is the thing a grid
        /// exists to catch.
        /// </summary>
        [JsonPropertyName("passed")]       public bool   Passed       { get; set; }
        /// <summary>One entry per cell, row-major. Null when the zone has no grid.</summary>
        [JsonPropertyName("cells")]        public List<ColorCellResult>? Cells { get; set; }
        /// <summary>How many cells passed. Null when the zone has no grid.</summary>
        [JsonPropertyName("cellsPassed")]  public int?   CellsPassed  { get; set; }
    }

    public class PolygonInspection : IVisionInspection
    {
        [JsonPropertyName("id")]           public string  Id           { get; set; } = "";
        [JsonPropertyName("name")]         public string  Name         { get; set; } = "";
        [JsonPropertyName("enabled")]      public bool    Enabled      { get; set; } = true;
        [JsonPropertyName("zoneId")]       public string? ZoneId       { get; set; }
        [JsonPropertyName("sides")]        public int     Sides        { get; set; } = 4;
        [JsonPropertyName("minArea")]      public float   MinArea      { get; set; } = 1000;
        [JsonPropertyName("maxArea")]      public float   MaxArea      { get; set; } = 100000;
        /// <summary>ApproxPolyDP accuracy factor — fraction of perimeter (0.01–0.1, default 0.04).</summary>
        [JsonPropertyName("epsilon")]      public double  Epsilon      { get; set; } = 0.04;
        [JsonPropertyName("minThreshold")]    public float  MinThreshold    { get; set; } = 50;
        [JsonPropertyName("maxThreshold")]    public float  MaxThreshold    { get; set; } = 200;
        [JsonPropertyName("invertThreshold")] public bool   InvertThreshold { get; set; } = false;
    }

    public class PolygonResult
    {
        [JsonPropertyName("inspectionId")] public string InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string Name         { get; set; } = "";
        [JsonPropertyName("count")]        public int    Count        { get; set; }
        [JsonPropertyName("found")]        public bool   Found        { get; set; }
        /// <summary>Orientation angle in degrees from MinAreaRect of the largest matching polygon.</summary>
        [JsonPropertyName("angle")]        public double Angle        { get; set; }
        /// <summary>Normalized centroid X (0–1) of the largest matching polygon.</summary>
        [JsonPropertyName("centerX")]      public double CenterX      { get; set; }
        /// <summary>Normalized centroid Y (0–1) of the largest matching polygon.</summary>
        [JsonPropertyName("centerY")]      public double CenterY      { get; set; }
    }

    /// <summary>Line detection inspection using Canny + HoughLinesP.</summary>
    public class LineInspection : IVisionInspection
    {
        [JsonPropertyName("id")]              public string  Id              { get; set; } = "";
        [JsonPropertyName("name")]            public string  Name            { get; set; } = "";
        [JsonPropertyName("enabled")]         public bool    Enabled         { get; set; } = true;
        [JsonPropertyName("zoneId")]          public string? ZoneId          { get; set; }
        [JsonPropertyName("cannyThreshold1")] public double  CannyThreshold1 { get; set; } = 50;
        [JsonPropertyName("cannyThreshold2")] public double  CannyThreshold2 { get; set; } = 150;
        [JsonPropertyName("houghThreshold")]  public int     HoughThreshold  { get; set; } = 50;
        [JsonPropertyName("minLineLength")]   public double  MinLineLength   { get; set; } = 30;
        [JsonPropertyName("maxLineGap")]      public double  MaxLineGap      { get; set; } = 10;
        [JsonPropertyName("filterByAngle")]   public bool    FilterByAngle   { get; set; } = false;
        [JsonPropertyName("minAngle")]        public double  MinAngle        { get; set; } = 0;
        [JsonPropertyName("maxAngle")]        public double  MaxAngle        { get; set; } = 180;
    }

    public class LineSegment
    {
        /// <summary>Normalized (0–1) start X.</summary>
        [JsonPropertyName("x1")]     public double X1     { get; set; }
        /// <summary>Normalized (0–1) start Y.</summary>
        [JsonPropertyName("y1")]     public double Y1     { get; set; }
        /// <summary>Normalized (0–1) end X.</summary>
        [JsonPropertyName("x2")]     public double X2     { get; set; }
        /// <summary>Normalized (0–1) end Y.</summary>
        [JsonPropertyName("y2")]     public double Y2     { get; set; }
        /// <summary>Undirected angle in degrees (0–180): 0=horizontal, 90=vertical.</summary>
        [JsonPropertyName("angle")]  public double Angle  { get; set; }
        /// <summary>Pixel length of the detected segment.</summary>
        [JsonPropertyName("length")] public double Length { get; set; }
    }

    public class LineResult
    {
        [JsonPropertyName("inspectionId")] public string            InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string            Name         { get; set; } = "";
        [JsonPropertyName("count")]        public int               Count        { get; set; }
        [JsonPropertyName("found")]        public bool              Found        { get; set; }
        [JsonPropertyName("lines")]        public List<LineSegment> Lines        { get; set; } = new();
    }

    /// <summary>ArUco marker detection inspection.</summary>
    public class ArucoInspection : IVisionInspection
    {
        [JsonPropertyName("id")]            public string  Id            { get; set; } = "";
        [JsonPropertyName("name")]          public string  Name          { get; set; } = "";
        [JsonPropertyName("enabled")]       public bool    Enabled       { get; set; } = true;
        [JsonPropertyName("zoneId")]        public string? ZoneId        { get; set; }
        /// <summary>OpenCV predefined dictionary ID (1=4x4_100 default, see ArUco dictionary list).</summary>
        [JsonPropertyName("dictionaryId")]  public int     DictionaryId  { get; set; } = 1;
        [JsonPropertyName("minMarkerArea")] public float   MinMarkerArea { get; set; } = 100;
        [JsonPropertyName("maxMarkerArea")] public float   MaxMarkerArea { get; set; } = 100000;
    }

    public class ArucoMarkerResult
    {
        [JsonPropertyName("markerId")] public int    MarkerId { get; set; }
        /// <summary>Normalized (0–1) center X of the detected marker.</summary>
        [JsonPropertyName("centerX")]  public double CenterX  { get; set; }
        /// <summary>Normalized (0–1) center Y of the detected marker.</summary>
        [JsonPropertyName("centerY")]  public double CenterY  { get; set; }
    }

    public class ArucoResult
    {
        [JsonPropertyName("inspectionId")] public string                  InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string                  Name         { get; set; } = "";
        [JsonPropertyName("count")]        public int                     Count        { get; set; }
        [JsonPropertyName("found")]        public bool                    Found        { get; set; }
        [JsonPropertyName("markers")]      public List<ArucoMarkerResult> Markers      { get; set; } = new();
    }

    public class BarcodeInspection : IVisionInspection
    {
        [JsonPropertyName("id")]      public string       Id      { get; set; } = "";
        [JsonPropertyName("name")]    public string       Name    { get; set; } = "";
        [JsonPropertyName("enabled")] public bool         Enabled { get; set; } = true;
        [JsonPropertyName("zoneId")]  public string?      ZoneId  { get; set; }
        /// <summary>ZXing BarcodeFormat names to scan for; empty = all formats.</summary>
        [JsonPropertyName("formats")] public List<string> Formats { get; set; } = new();
    }

    public class BarcodeCodeResult
    {
        [JsonPropertyName("value")]   public string Value   { get; set; } = "";
        [JsonPropertyName("format")]  public string Format  { get; set; } = "";
        [JsonPropertyName("centerX")] public double CenterX { get; set; }
        [JsonPropertyName("centerY")] public double CenterY { get; set; }
    }

    public class BarcodeResult
    {
        [JsonPropertyName("inspectionId")] public string                  InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string                  Name         { get; set; } = "";
        [JsonPropertyName("count")]        public int                     Count        { get; set; }
        [JsonPropertyName("found")]        public bool                    Found        { get; set; }
        [JsonPropertyName("codes")]        public List<BarcodeCodeResult> Codes        { get; set; } = new();
    }

    // ── Frame / corner / edge / texture inspections ────────────────────────────────────────

    /// <summary>
    /// Finds a four-cornered frame (a board's outline) so other zones can be aligned to it.
    /// method: "contour" (largest 4-sided contour), "lines" (two pairs of dominant edges,
    /// intersected), "corners" (Shi-Tomasi corners, the four extreme hull points) or
    /// "checker" (fits a gridRows × gridCols checkerboard to the squares themselves, starting
    /// from the search zone drawn roughly over them — for a board whose frame is hidden, cut
    /// off or blends with its dark squares).
    /// </summary>
    public class FrameInspection : IVisionInspection
    {
        [JsonPropertyName("id")]                public string  Id                { get; set; } = "";
        [JsonPropertyName("name")]              public string  Name              { get; set; } = "";
        [JsonPropertyName("enabled")]           public bool    Enabled           { get; set; } = true;
        [JsonPropertyName("zoneId")]            public string? ZoneId            { get; set; }
        [JsonPropertyName("method")]            public string  Method            { get; set; } = "contour";
        [JsonPropertyName("cannyLow")]          public double  CannyLow          { get; set; } = 50;
        [JsonPropertyName("cannyHigh")]         public double  CannyHigh         { get; set; } = 150;
        /// <summary>contour: smallest frame accepted, as % of the search area.</summary>
        [JsonPropertyName("minAreaPct")]        public double  MinAreaPct        { get; set; } = 10;
        /// <summary>contour: approxPolyDP epsilon as a fraction of the contour perimeter.</summary>
        [JsonPropertyName("approxEpsilon")]     public double  ApproxEpsilon     { get; set; } = 0.02;
        [JsonPropertyName("houghThreshold")]    public int     HoughThreshold    { get; set; } = 60;
        /// <summary>lines: shortest segment, as % of the search area's smaller side.</summary>
        [JsonPropertyName("minLineLengthPct")]  public double  MinLineLengthPct  { get; set; } = 20;
        [JsonPropertyName("maxLineGap")]        public double  MaxLineGap        { get; set; } = 10;
        [JsonPropertyName("maxCorners")]        public int     MaxCorners        { get; set; } = 200;
        [JsonPropertyName("cornerQuality")]     public double  CornerQuality     { get; set; } = 0.01;
        [JsonPropertyName("minCornerDistance")] public double  MinCornerDistance { get; set; } = 10;
        /// <summary>checker: squares across and down the checkerboard.</summary>
        [JsonPropertyName("gridRows")]          public int     GridRows          { get; set; } = 8;
        [JsonPropertyName("gridCols")]          public int     GridCols          { get; set; } = 8;
        /// <summary>
        /// checker: found when the grid lines carry at least this many times the edge
        /// strength of the lines halfway between them (square centres). A fitted board is
        /// ~8–14; a grid a quarter square off, or a hand over the board, is under ~4.
        /// </summary>
        [JsonPropertyName("minEdgeRatio")]      public double  MinEdgeRatio      { get; set; } = 4;
        /// <summary>The frame's corners (TL, TR, BR, BL, normalized) when the aligned zones were
        /// drawn. Aligned zones are warped from these to the corners detected each frame.</summary>
        [JsonPropertyName("referenceCorners")]  public List<double[]>? ReferenceCorners { get; set; }
    }

    public class FrameResult
    {
        [JsonPropertyName("inspectionId")] public string         InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string         Name         { get; set; } = "";
        [JsonPropertyName("found")]        public bool           Found        { get; set; }
        /// <summary>TL, TR, BR, BL, normalized; empty when not found.</summary>
        [JsonPropertyName("corners")]      public List<double[]> Corners      { get; set; } = new();
        /// <summary>Angle of the TL→TR edge against +x, degrees (pixel space).</summary>
        [JsonPropertyName("angle")]        public double         Angle        { get; set; }
        /// <summary>Frame area as % of the image.</summary>
        [JsonPropertyName("areaPct")]      public double         AreaPct      { get; set; }
        [JsonPropertyName("method")]       public string         Method       { get; set; } = "";
        /// <summary>checker: the fit's edge ratio (see FrameInspection.MinEdgeRatio); null for other methods.</summary>
        [JsonPropertyName("score")]        public double?        Score        { get; set; }
    }

    /// <summary>Corner (feature point) detection: Shi-Tomasi or Harris.</summary>
    public class CornerInspection : IVisionInspection
    {
        [JsonPropertyName("id")]          public string  Id          { get; set; } = "";
        [JsonPropertyName("name")]        public string  Name        { get; set; } = "";
        [JsonPropertyName("enabled")]     public bool    Enabled     { get; set; } = true;
        [JsonPropertyName("zoneId")]      public string? ZoneId      { get; set; }
        [JsonPropertyName("method")]      public string  Method      { get; set; } = "shiTomasi";
        [JsonPropertyName("maxCorners")]  public int     MaxCorners  { get; set; } = 50;
        [JsonPropertyName("quality")]     public double  Quality     { get; set; } = 0.01;
        [JsonPropertyName("minDistance")] public double  MinDistance { get; set; } = 10;
        [JsonPropertyName("blockSize")]   public int     BlockSize   { get; set; } = 3;
        /// <summary>found = at least this many corners.</summary>
        [JsonPropertyName("minCount")]    public int     MinCount    { get; set; } = 1;
    }

    public class CornerResult
    {
        [JsonPropertyName("inspectionId")] public string         InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string         Name         { get; set; } = "";
        [JsonPropertyName("count")]        public int            Count        { get; set; }
        [JsonPropertyName("found")]        public bool           Found        { get; set; }
        /// <summary>Normalized [x, y], strongest first.</summary>
        [JsonPropertyName("points")]       public List<double[]> Points       { get; set; } = new();
    }

    /// <summary>Edge detection (Canny): how much of the zone is edge, and its dominant direction.</summary>
    public class EdgeInspection : IVisionInspection
    {
        [JsonPropertyName("id")]         public string  Id         { get; set; } = "";
        [JsonPropertyName("name")]       public string  Name       { get; set; } = "";
        [JsonPropertyName("enabled")]    public bool    Enabled    { get; set; } = true;
        [JsonPropertyName("zoneId")]     public string? ZoneId     { get; set; }
        [JsonPropertyName("cannyLow")]   public double  CannyLow   { get; set; } = 50;
        [JsonPropertyName("cannyHigh")]  public double  CannyHigh  { get; set; } = 150;
        /// <summary>passed when the edge density (% of zone pixels) is within [min, max].</summary>
        [JsonPropertyName("minDensity")] public double? MinDensity { get; set; } = 0;
        [JsonPropertyName("maxDensity")] public double? MaxDensity { get; set; }
    }

    public class EdgeResult
    {
        [JsonPropertyName("inspectionId")]  public string  InspectionId  { get; set; } = "";
        [JsonPropertyName("name")]          public string  Name          { get; set; } = "";
        [JsonPropertyName("density")]       public double  Density       { get; set; }
        [JsonPropertyName("passed")]        public bool    Passed        { get; set; }
        /// <summary>Direction of the strongest straight edge, 0–180°, null when none.</summary>
        [JsonPropertyName("dominantAngle")] public double? DominantAngle { get; set; }
    }

    /// <summary>
    /// Fine detail in the zone, or in each cell of its grid: the spread of brightness left once
    /// slow shading is blurred away. A ringed or patterned object on a plain surface scores
    /// high even when both are the same colour (a black checker on a black square), while glare
    /// and soft shadows don't count. Each cell is measured on its centre only (inset by
    /// <see cref="CellInset"/>) so the cell's own borders and a slightly-off grid don't count.
    /// </summary>
    public class TextureInspection : IVisionInspection
    {
        [JsonPropertyName("id")]        public string  Id        { get; set; } = "";
        [JsonPropertyName("name")]      public string  Name      { get; set; } = "";
        [JsonPropertyName("enabled")]   public bool    Enabled   { get; set; } = true;
        [JsonPropertyName("zoneId")]    public string? ZoneId    { get; set; }
        /// <summary>Margin trimmed off each side of a cell before measuring, % of the cell (0–45).</summary>
        [JsonPropertyName("cellInset")] public double  CellInset { get; set; } = 25;
        /// <summary>passed when the texture score is within [min, max].</summary>
        [JsonPropertyName("minScore")]  public double? MinScore  { get; set; } = 4;
        [JsonPropertyName("maxScore")]  public double? MaxScore  { get; set; }
    }

    public class TextureCellResult
    {
        [JsonPropertyName("row")]    public int    Row    { get; set; }
        [JsonPropertyName("col")]    public int    Col    { get; set; }
        [JsonPropertyName("index")]  public int    Index  { get; set; }
        [JsonPropertyName("score")]  public double Score  { get; set; }
        [JsonPropertyName("passed")] public bool   Passed { get; set; }
    }

    public class TextureResult
    {
        [JsonPropertyName("inspectionId")] public string                   InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string                   Name         { get; set; } = "";
        /// <summary>The zone's score, or on a grid the highest cell score.</summary>
        [JsonPropertyName("score")]        public double                   Score        { get; set; }
        /// <summary>On a grid: true when at least one cell passed (a cell list is the real answer).</summary>
        [JsonPropertyName("passed")]       public bool                     Passed       { get; set; }
        [JsonPropertyName("cells")]        public List<TextureCellResult>? Cells        { get; set; }
        [JsonPropertyName("cellsPassed")]  public int?                     CellsPassed  { get; set; }
    }

    /// <summary>The outline a zone was actually used with this frame, after alignment.</summary>
    public class AlignedZone
    {
        [JsonPropertyName("zoneId")] public string         ZoneId { get; set; } = "";
        [JsonPropertyName("points")] public List<double[]> Points { get; set; } = new();
    }

    public class VisionProgram
    {
        [JsonPropertyName("id")]                  public string                        Id                 { get; set; } = "";
        [JsonPropertyName("name")]                public string                        Name               { get; set; } = "";
        [JsonPropertyName("description")]         public string                        Description        { get; set; } = "";
        [JsonPropertyName("cameraId")]            public string                        CameraId           { get; set; } = "";
        [JsonPropertyName("zones")]               public List<VisionZone>              Zones              { get; set; } = new();
        [JsonPropertyName("inspections")]         public List<BlobInspection>          Inspections        { get; set; } = new();
        [JsonPropertyName("colorInspections")]    public List<ColorCoverageInspection> ColorInspections   { get; set; } = new();
        [JsonPropertyName("polygonInspections")]  public List<PolygonInspection>       PolygonInspections { get; set; } = new();
        [JsonPropertyName("arucoInspections")]    public List<ArucoInspection>         ArucoInspections    { get; set; } = new();
        [JsonPropertyName("lineInspections")]     public List<LineInspection>          LineInspections     { get; set; } = new();
        [JsonPropertyName("barcodeInspections")] public List<BarcodeInspection>       BarcodeInspections  { get; set; } = new();
        [JsonPropertyName("frameInspections")]  public List<FrameInspection>         FrameInspections    { get; set; } = new();
        [JsonPropertyName("cornerInspections")] public List<CornerInspection>        CornerInspections   { get; set; } = new();
        [JsonPropertyName("edgeInspections")]   public List<EdgeInspection>          EdgeInspections     { get; set; } = new();
        [JsonPropertyName("textureInspections")] public List<TextureInspection>      TextureInspections  { get; set; } = new();
        // Order of inspections as a flat list of ids across all the typed lists above. The
        // editor lets the user drag inspections into any order; the processor runs (and
        // stacks labels) in this order. Ids missing from the list run after the listed ones,
        // in type order (see AllInspections).
        [JsonPropertyName("inspectionOrder")]     public List<string>                 InspectionOrder     { get; set; } = new();
        [JsonPropertyName("lastUpdatedUnixMs")]   public long                          LastUpdatedUnixMs   { get; set; }

        /// <summary>
        /// Every inspection across the typed lists, in type order: blob, color, polygon,
        /// ArUco, line, barcode, frame, corner, edge, texture — each list in its stored order. Ignores InspectionOrder.
        /// A list sent as JSON null is treated as empty.
        /// </summary>
        public IEnumerable<IVisionInspection> AllInspections()
        {
            foreach (var i in Inspections ?? [])        yield return i;
            foreach (var i in ColorInspections ?? [])   yield return i;
            foreach (var i in PolygonInspections ?? []) yield return i;
            foreach (var i in ArucoInspections ?? [])   yield return i;
            foreach (var i in LineInspections ?? [])    yield return i;
            foreach (var i in BarcodeInspections ?? []) yield return i;
            foreach (var i in FrameInspections ?? [])   yield return i;
            foreach (var i in CornerInspections ?? [])  yield return i;
            foreach (var i in EdgeInspections ?? [])    yield return i;
            foreach (var i in TextureInspections ?? []) yield return i;
        }
    }

    // ── Runtime results ───────────────────────────────────────────────────────────

    public class BlobResult
    {
        [JsonPropertyName("x")]    public float X    { get; set; }
        [JsonPropertyName("y")]    public float Y    { get; set; }
        [JsonPropertyName("size")] public float Size { get; set; }
    }

    public class InspectionResult
    {
        [JsonPropertyName("inspectionId")] public string           InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string           Name         { get; set; } = "";
        [JsonPropertyName("blobs")]        public List<BlobResult> Blobs        { get; set; } = new();
    }

    public class VisionResult
    {
        [JsonPropertyName("programId")]       public string                    ProgramId      { get; set; } = "";
        [JsonPropertyName("timestampMs")]     public long                      TimestampMs    { get; set; }
        /// <summary>Size of the analysed frame, px (0 when unknown).</summary>
        [JsonPropertyName("imageWidth")]      public int                       ImageWidth     { get; set; }
        [JsonPropertyName("imageHeight")]     public int                       ImageHeight    { get; set; }
        [JsonPropertyName("inspections")]     public List<InspectionResult>    Inspections    { get; set; } = new();
        [JsonPropertyName("colorResults")]    public List<ColorCoverageResult> ColorResults   { get; set; } = new();
        [JsonPropertyName("polygonResults")]  public List<PolygonResult>       PolygonResults { get; set; } = new();
        [JsonPropertyName("arucoResults")]    public List<ArucoResult>    ArucoResults    { get; set; } = new();
        [JsonPropertyName("lineResults")]     public List<LineResult>     LineResults     { get; set; } = new();
        [JsonPropertyName("barcodeResults")] public List<BarcodeResult>  BarcodeResults  { get; set; } = new();
        [JsonPropertyName("frameResults")]   public List<FrameResult>    FrameResults    { get; set; } = new();
        [JsonPropertyName("cornerResults")]  public List<CornerResult>   CornerResults   { get; set; } = new();
        [JsonPropertyName("edgeResults")]    public List<EdgeResult>     EdgeResults     { get; set; } = new();
        [JsonPropertyName("textureResults")] public List<TextureResult>  TextureResults  { get; set; } = new();
        /// <summary>Zones warped by a frame inspection this frame, as the outline actually used.</summary>
        [JsonPropertyName("alignedZones")]   public List<AlignedZone>    AlignedZones    { get; set; } = new();
        // How long each inspection took to run this frame, in milliseconds, keyed by
        // inspection id. Shown on the editor's inspection cards.
        [JsonPropertyName("timings")]         public Dictionary<string, double> Timings { get; set; } = new();
        /// <summary>Per-inspection error messages from this pass, formatted "{inspectionId}: {message}".</summary>
        [JsonPropertyName("errors")]         public List<string>        Errors          { get; set; } = new();
    }
}
