using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

/// <summary>Hole position for CNC threading (robot coordinates, mm).</summary>
public class CncHole
{
    [JsonPropertyName("x")] public double X { get; set; }
    [JsonPropertyName("y")] public double Y { get; set; }
}

/// <summary>
/// CNC toolpath specification. Holes are threaded and contours followed from the
/// same block; the executor expands this into MoveL/ThreadMove steps at runtime
/// so large toolpaths don't bloat the stored program.
/// </summary>
public class CncSpec
{
    [JsonPropertyName("file")]  public string? File  { get; set; }
    [JsonPropertyName("safeZ")] public double  SafeZ { get; set; }

    // Holes — drilled or threaded, per HoleOp ("thread" default, "drill" plunges)
    [JsonPropertyName("holes")]            public List<CncHole>? Holes            { get; set; }
    [JsonPropertyName("holeIndexes")]      public List<int>?     HoleIndexes      { get; set; }
    [JsonPropertyName("holeOp")]           public string?        HoleOp           { get; set; }
    [JsonPropertyName("holeDepth")]        public double?        HoleDepth        { get; set; }
    [JsonPropertyName("threadPitch")]      public double?        ThreadPitch      { get; set; }
    [JsonPropertyName("holePeck")]         public bool?          HolePeck         { get; set; }
    [JsonPropertyName("holePeckDepth")]    public double?        HolePeckDepth    { get; set; }
    [JsonPropertyName("threadReverseOut")] public bool?          ThreadReverseOut { get; set; }

    // Contours — baked robot-space polylines ([x0,y0,x1,y1,…] per contour)
    // plus the placement/motion settings needed to re-edit in the app.
    [JsonPropertyName("paths")]          public List<List<double>>? Paths          { get; set; }
    [JsonPropertyName("contourIndexes")] public List<int>?          ContourIndexes { get; set; }
    [JsonPropertyName("scale")]          public double?             Scale          { get; set; }
    [JsonPropertyName("offsetX")]        public double?             OffsetX        { get; set; }
    [JsonPropertyName("offsetY")]        public double?             OffsetY        { get; set; }
    [JsonPropertyName("flipY")]          public bool?               FlipY          { get; set; }
    [JsonPropertyName("activeZ")]          public double?             ActiveZ          { get; set; }
    [JsonPropertyName("activeSpeed")]      public double?             ActiveSpeed      { get; set; }
    [JsonPropertyName("activeAccel")]      public double?             ActiveAccel      { get; set; }
    [JsonPropertyName("activeDecel")]      public double?             ActiveDecel      { get; set; }
    // Dynamics for safe-Z travel between contours and the retract; the plunge uses active dynamics.
    [JsonPropertyName("travelSpeed")]    public double?             TravelSpeed    { get; set; }
    [JsonPropertyName("travelAccel")]    public double?             TravelAccel    { get; set; }
    [JsonPropertyName("travelDecel")]    public double?             TravelDecel    { get; set; }
    [JsonPropertyName("blendRadius")]    public double?             BlendRadius    { get; set; }
    [JsonPropertyName("detail")]         public double?             Detail         { get; set; }
    /// <summary>Max endpoint gap (mm) for chaining touching segments into one pass.</summary>
    [JsonPropertyName("joinTolerance")]  public double?             JoinTolerance  { get; set; }
    // Tool-radius compensation applied when the paths were baked (app-side).
    [JsonPropertyName("offsetMode")]     public string?             OffsetMode     { get; set; }
    [JsonPropertyName("offsetDistance")] public double?             OffsetDistance { get; set; }
    /// <summary>
    /// "absolute" (default): baked coordinates are used as-is. "current": the
    /// robot's position when the block starts becomes the origin — X/Y/Z of
    /// every generated move is relative to it (within the active local).
    /// </summary>
    [JsonPropertyName("originMode")]     public string?             OriginMode     { get; set; }
    // Direction/start-point choices — baked into Paths by the app; stored only
    // so the CNC builder can restore its editing state.
    [JsonPropertyName("contourReversed")] public List<int>?                   ContourReversed { get; set; }
    [JsonPropertyName("contourStarts")]   public Dictionary<string, CncHole>? ContourStarts   { get; set; }
    /// <summary>
    /// $variable expressions for motion fields, keyed by spec field name
    /// (safeZ, activeZ, activeSpeed, activeAccel, activeDecel, travelSpeed,
    /// travelAccel, travelDecel, blendRadius, holeDepth, threadPitch,
    /// holePeckDepth). Attached to the generated steps and evaluated at run
    /// time; drill-structural values resolve once at block start.
    /// </summary>
    [JsonPropertyName("expressions")]    public Dictionary<string, string>? Expressions { get; set; }
}


