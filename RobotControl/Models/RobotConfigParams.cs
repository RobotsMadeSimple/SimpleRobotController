using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

public class SetRobotIdentityParams
{
    [JsonPropertyName("robotName")] public string? RobotName { get; set; }
    [JsonPropertyName("robotType")] public string? RobotType { get; set; }
}

public class SetRobotConfigParams
{
    [JsonPropertyName("robotType")]                 public string? RobotType                 { get; set; }
    [JsonPropertyName("homingSpeed")]               public double? HomingSpeed               { get; set; }
    [JsonPropertyName("homingSlowSpeed")]           public double? HomingSlowSpeed           { get; set; }
    [JsonPropertyName("homingBackoffMm")]           public double? HomingBackoffMm           { get; set; }
    [JsonPropertyName("j1HomeOffsetDeg")]           public double? J1HomeOffsetDeg           { get; set; }
    [JsonPropertyName("verticalHomePosition")]      public double? VerticalHomePosition      { get; set; }
    [JsonPropertyName("horizontalHomePosition")]    public double? HorizontalHomePosition    { get; set; }
    [JsonPropertyName("verticalHomingDirection")]   public int?    VerticalHomingDirection   { get; set; }
    [JsonPropertyName("horizontalHomingDirection")] public int?    HorizontalHomingDirection { get; set; }
    [JsonPropertyName("j1HomingDirection")]         public int?    J1HomingDirection         { get; set; }
    [JsonPropertyName("j4HomeOffsetDeg")]           public double? J4HomeOffsetDeg           { get; set; }
    [JsonPropertyName("m1Direction")]               public int?    M1Direction               { get; set; }
    [JsonPropertyName("m2Direction")]               public int?    M2Direction               { get; set; }
    [JsonPropertyName("m3Direction")]               public int?    M3Direction               { get; set; }
    [JsonPropertyName("m4Direction")]               public int?    M4Direction               { get; set; }
    [JsonPropertyName("enableNanoCards")]           public bool?   EnableNanoCards           { get; set; }
    [JsonPropertyName("enableRelayCard")]           public bool?   EnableRelayCard           { get; set; }
    [JsonPropertyName("enableAuxAxis")]             public bool?   EnableAuxAxis             { get; set; }
    [JsonPropertyName("enableCameras")]             public bool?   EnableCameras             { get; set; }
    [JsonPropertyName("jogSlowSpeed")]              public double? JogSlowSpeed              { get; set; }
    [JsonPropertyName("jogNormalSpeed")]            public double? JogNormalSpeed            { get; set; }
    [JsonPropertyName("jogFastSpeed")]              public double? JogFastSpeed              { get; set; }
    [JsonPropertyName("cncStepsPerRevX")]           public int?    CncStepsPerRevX           { get; set; }
    [JsonPropertyName("cncStepsPerRevY")]           public int?    CncStepsPerRevY           { get; set; }
    [JsonPropertyName("cncStepsPerRevZ")]           public int?    CncStepsPerRevZ           { get; set; }
    [JsonPropertyName("cncStepsPerRevRZ")]          public int?    CncStepsPerRevRZ          { get; set; }
    [JsonPropertyName("cncMmPerRevX")]              public double? CncMmPerRevX              { get; set; }
    [JsonPropertyName("cncMmPerRevY")]              public double? CncMmPerRevY              { get; set; }
    [JsonPropertyName("cncMmPerRevZ")]              public double? CncMmPerRevZ              { get; set; }
    [JsonPropertyName("cncDegPerRevRZ")]            public double? CncDegPerRevRZ            { get; set; }
    [JsonPropertyName("cncXHomePosition")]          public double? CncXHomePosition          { get; set; }
    [JsonPropertyName("cncYHomePosition")]          public double? CncYHomePosition          { get; set; }
    [JsonPropertyName("cncZHomePosition")]          public double? CncZHomePosition          { get; set; }
    [JsonPropertyName("cncRzHomePosition")]         public double? CncRzHomePosition         { get; set; }
    [JsonPropertyName("cncXHomingDirection")]       public int?    CncXHomingDirection       { get; set; }
    [JsonPropertyName("cncYHomingDirection")]       public int?    CncYHomingDirection       { get; set; }
    [JsonPropertyName("cncZHomingDirection")]       public int?    CncZHomingDirection       { get; set; }

    // ── Joint soft limits ─────────────────────────────────────────────────
    [JsonPropertyName("jointLimitsEnabled")]        public bool?   JointLimitsEnabled        { get; set; }
    [JsonPropertyName("joint1Min")]                 public double? Joint1Min                 { get; set; }
    [JsonPropertyName("joint1Max")]                 public double? Joint1Max                 { get; set; }
    [JsonPropertyName("joint2Min")]                 public double? Joint2Min                 { get; set; }
    [JsonPropertyName("joint2Max")]                 public double? Joint2Max                 { get; set; }
    [JsonPropertyName("joint3Min")]                 public double? Joint3Min                 { get; set; }
    [JsonPropertyName("joint3Max")]                 public double? Joint3Max                 { get; set; }
    [JsonPropertyName("joint4Min")]                 public double? Joint4Min                 { get; set; }
    [JsonPropertyName("joint4Max")]                 public double? Joint4Max                 { get; set; }
}

public class SetLimitBypassParams
{
    [JsonPropertyName("enable")] public bool Enable { get; set; }
}


public class SetSpeedOverrideParams
{
    [JsonPropertyName("percent")] public double Percent { get; set; } = 100.0;
}
