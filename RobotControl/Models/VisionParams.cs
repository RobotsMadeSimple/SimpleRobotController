using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Vision command params ─────────────────────────────────────────────────────

public class DeleteVisionProgramParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class StartStopVisionParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

/// <summary>
/// Run a vision program against a single supplied image instead of a live camera — for the
/// API / testing. Give either <see cref="ProgramId"/> (a saved program) or an inline
/// <see cref="Program"/>; the image is a base64 PNG/JPEG, with or without a data-URL prefix.
/// </summary>
public class RunVisionOnImageParams
{
    [JsonPropertyName("programId")]        public string?                ProgramId        { get; set; }
    [JsonPropertyName("program")]          public Vision.VisionProgram?  Program          { get; set; }
    [JsonPropertyName("image")]            public string?                Image            { get; set; }
    /// <summary>Also return the annotated frame as a base64 JPEG. Off by default — it is large.</summary>
    [JsonPropertyName("includeAnnotated")] public bool                   IncludeAnnotated { get; set; }
}

/// <summary>SetVisionToolReference: a pose tool's reference := its pose in the latest result (or cleared).</summary>
public class SetVisionToolReferenceParams
{
    [JsonPropertyName("programId")] public string ProgramId { get; set; } = "";
    [JsonPropertyName("toolId")]    public string ToolId    { get; set; } = "";
    /// <summary>True: clear the reference instead (it is captured again on the next successful run).</summary>
    [JsonPropertyName("clear")]     public bool   Clear     { get; set; }
}

/// <summary>TeachVisionTool: cut a patternMatch / contourMatch template from a frame and the tool's region.</summary>
public class TeachVisionToolParams
{
    [JsonPropertyName("programId")] public string  ProgramId { get; set; } = "";
    [JsonPropertyName("toolId")]    public string  ToolId    { get; set; } = "";
    /// <summary>Optional base64 PNG/JPEG to teach from; default the program's latest camera frame.</summary>
    [JsonPropertyName("image")]     public string? Image     { get; set; }
    /// <summary>
    /// Optional area to teach from instead of the tool's region, normalized to the image as it is
    /// (no fixture) — e.g. a rotated rectangle: the pattern is cut deskewed and its angle is 0°.
    /// </summary>
    [JsonPropertyName("region")]    public Vision.VisionRegion? Region { get; set; }
    /// <summary>True: return the template (and a preview PNG) without changing or saving the program.</summary>
    [JsonPropertyName("preview")]   public bool    Preview   { get; set; }
}
