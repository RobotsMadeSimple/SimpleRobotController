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
    [JsonPropertyName("programId")]        public string?                               ProgramId        { get; set; }
    [JsonPropertyName("program")]          public Vision.VisionProgram?                 Program          { get; set; }
    [JsonPropertyName("image")]            public string?                               Image            { get; set; }
    /// <summary>Optional: point every inspection at this zone for this run (a runtime override).</summary>
    [JsonPropertyName("zoneId")]           public string?                               ZoneId           { get; set; }
    /// <summary>Also return the annotated frame as a base64 JPEG. Off by default — it is large.</summary>
    [JsonPropertyName("includeAnnotated")] public bool                                  IncludeAnnotated { get; set; }
}


