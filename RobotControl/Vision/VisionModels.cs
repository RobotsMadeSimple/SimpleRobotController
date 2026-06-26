using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Vision
{
    [JsonConverter(typeof(JsonStringEnumConverter))]
    public enum VisionZoneShape { Rectangle, Circle, Polygon }

    /// <summary>Normalized (0–1) zone geometry relative to the camera frame.</summary>
    public class VisionZoneGeometry
    {
        [JsonPropertyName("shape")]  public VisionZoneShape Shape  { get; set; } = VisionZoneShape.Rectangle;

        // Rectangle — top-left corner + size (all 0-1 fractions)
        [JsonPropertyName("x")]      public double X      { get; set; }
        [JsonPropertyName("y")]      public double Y      { get; set; }
        [JsonPropertyName("width")]  public double Width  { get; set; } = 1;
        [JsonPropertyName("height")] public double Height { get; set; } = 1;

        // Circle — center + radius (radius is fraction of min(width, height))
        [JsonPropertyName("cx")]     public double Cx     { get; set; } = 0.5;
        [JsonPropertyName("cy")]     public double Cy     { get; set; } = 0.5;
        [JsonPropertyName("radius")] public double Radius { get; set; } = 0.25;

        // Polygon — list of [x, y] pairs (0-1 each)
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

    /// <summary>A named spatial region — geometry only, no detection config.</summary>
    public class VisionZone
    {
        [JsonPropertyName("id")]       public string             Id       { get; set; } = "";
        [JsonPropertyName("name")]     public string             Name     { get; set; } = "";
        [JsonPropertyName("geometry")] public VisionZoneGeometry Geometry { get; set; } = new();
    }

    /// <summary>Blob detection run; optionally restricted to a zone by center-point containment.</summary>
    public class BlobInspection
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

    public class ColorCoverageInspection
    {
        [JsonPropertyName("id")]          public string           Id          { get; set; } = "";
        [JsonPropertyName("name")]        public string           Name        { get; set; } = "";
        [JsonPropertyName("enabled")]     public bool             Enabled     { get; set; } = true;
        [JsonPropertyName("zoneId")]      public string?          ZoneId      { get; set; }
        [JsonPropertyName("colors")]      public List<ColorEntry> Colors      { get; set; } = new();
        [JsonPropertyName("minCoverage")] public double?          MinCoverage { get; set; } = 50;
        [JsonPropertyName("maxCoverage")] public double?          MaxCoverage { get; set; }
    }

    public class ColorCoverageResult
    {
        [JsonPropertyName("inspectionId")] public string InspectionId { get; set; } = "";
        [JsonPropertyName("name")]         public string Name         { get; set; } = "";
        [JsonPropertyName("coverage")]     public double Coverage     { get; set; }
        [JsonPropertyName("passed")]       public bool   Passed       { get; set; }
    }

    public class PolygonInspection
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
    public class LineInspection
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
    public class ArucoInspection
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
        [JsonPropertyName("arucoInspections")]    public List<ArucoInspection>         ArucoInspections   { get; set; } = new();
        [JsonPropertyName("lineInspections")]     public List<LineInspection>          LineInspections    { get; set; } = new();
        [JsonPropertyName("lastUpdatedUnixMs")]   public long                          LastUpdatedUnixMs  { get; set; }
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
        [JsonPropertyName("inspections")]     public List<InspectionResult>    Inspections    { get; set; } = new();
        [JsonPropertyName("colorResults")]    public List<ColorCoverageResult> ColorResults   { get; set; } = new();
        [JsonPropertyName("polygonResults")]  public List<PolygonResult>       PolygonResults { get; set; } = new();
        [JsonPropertyName("arucoResults")]    public List<ArucoResult>         ArucoResults   { get; set; } = new();
        [JsonPropertyName("lineResults")]     public List<LineResult>          LineResults    { get; set; } = new();
    }
}
