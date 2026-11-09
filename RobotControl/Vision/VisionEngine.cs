using Controller.RobotControl.Vision.Tools;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// Runs a vision program on one frame: layers on demand, tools in list order, each tool's
    /// region moved by its fixture, limits judged, failed fixtures propagated as notRun, and a
    /// pose tool's reference captured on its first successful run when it has none.
    /// </summary>
    internal static class VisionEngine
    {
        /// <summary>The outcome of one run.</summary>
        internal sealed class Run
        {
            public required VisionResult Result { get; init; }
            /// <summary>Tools whose reference was captured on this run (the program was changed in place).</summary>
            public List<VisionTool> CapturedReferences { get; } = new();
        }

        /// <summary>
        /// Runs <paramref name="program"/> on <paramref name="src"/>. <paramref name="stopBefore"/>
        /// stops just before that tool (used to resolve a tool's region for teaching); the layer
        /// cache is then handed to <paramref name="inspect"/> before it is released.
        /// </summary>
        public static Run Execute(VisionProgram program, Mat src, VisionRuntime runtime,
                                  string? stopBefore = null, Action<LayerCache, VisionResult>? inspect = null)
        {
            var result = new VisionResult
            {
                ProgramId   = program.Id,
                Timestamp   = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                ImageWidth  = src.Width,
                ImageHeight = src.Height,
            };
            var run = new Run { Result = result };
            using var layers = new LayerCache(src, program.Layers);
            var tools = program.Tools ?? [];
            var seen = new Dictionary<string, VisionTool>();

            foreach (var tool in tools)
            {
                if (tool == null || string.IsNullOrEmpty(tool.Id)) continue;
                if (stopBefore != null && tool.Id == stopBefore) break;
                var before = new Dictionary<string, VisionTool>(seen);
                seen.TryAdd(tool.Id, tool);
                if (!tool.Enabled) continue;

                var sw = Stopwatch.StartNew();
                var tr = new ToolResult { Id = tool.Id, Type = tool.Type };
                try
                {
                    RunTool(tool, before, layers, result, runtime, tr);
                }
                catch (Exception ex)
                {
                    tr.Status  = ToolStatus.Error;
                    tr.Message = ex.Message;
                    tr.Pose    = null;
                    result.Errors.Add($"{tool.Id}: {ex.Message}");
                }
                tr.Passed = tr.Ok && LimitsPass(tool, tr);
                tr.TimeMs = Math.Round(sw.Elapsed.TotalMilliseconds, 2);
                result.Timings[tool.Id] = tr.TimeMs;
                result.Tools.Add(tr);

                if (tr.Ok && tr.Pose != null && tool.Reference == null && ToolRegistry.Get(tool.Type) is { MakesPose: true })
                {
                    tool.Reference = tr.Pose.Clone();
                    run.CapturedReferences.Add(tool);
                }
            }

            result.Layers.AddRange(layers.Computed);
            foreach (var kv in layers.Timings) result.Timings[$"layer:{kv.Key}"] = kv.Value;
            result.Passed = result.Tools.Where(t => tools.FirstOrDefault(x => x.Id == t.Id)?.IncludeInPass ?? false).All(t => t.Passed);
            inspect?.Invoke(layers, result);
            return run;
        }

        private static void RunTool(VisionTool tool, Dictionary<string, VisionTool> earlier, LayerCache layers,
                                    VisionResult result, VisionRuntime runtime, ToolResult tr)
        {
            var impl = ToolRegistry.Get(tool.Type);
            if (impl == null) throw new InvalidOperationException($"unknown tool type '{tool.Type}'");
            if (!layers.Exists(tool.Input)) throw new InvalidOperationException($"input layer '{tool.Input}' does not exist");

            // Sources must be earlier tools that ran and succeeded.
            foreach (var sid in impl.Sources(tool))
                if (NotRunBecause(sid, "source", earlier, result) is { } why) { tr.Status = ToolStatus.NotRun; tr.Message = why; return; }

            // A shared region: the area (and fixture) of the tool it comes from.
            var owner = impl.UsesRegion ? RegionOwner(tool, earlier) : tool;
            var transform = FixtureTransform.Identity;
            if (!string.IsNullOrEmpty(owner.FixtureId))
            {
                if (NotRunBecause(owner.FixtureId, "fixture", earlier, result) is { } why) { tr.Status = ToolStatus.NotRun; tr.Message = why; return; }
                var fixtureTool = earlier[owner.FixtureId];
                var fr = result.Tool(owner.FixtureId)!;
                if (fr.Pose == null) { tr.Status = ToolStatus.NotRun; tr.Message = $"fixture '{Label(fixtureTool)}' has no pose"; return; }
                // A fixture without a reference yet (taught this very frame) stays where drawn.
                var reference = fixtureTool.Reference ?? fr.Pose;
                var t = FixtureTransform.Between(reference, fr.Pose);
                if (t == null) { tr.Status = ToolStatus.NotRun; tr.Message = $"fixture '{Label(fixtureTool)}' pose is degenerate"; return; }
                transform = t;
            }

            var region = impl.UsesRegion
                ? PixelRegion.FromNormalized(owner.Region, layers.Width, layers.Height).Transform(transform)
                : PixelRegion.Whole(layers.Width, layers.Height);
            if (impl.UsesRegion) tr.Region = Geo.ToPx(region.Outline());

            impl.Run(new ToolContext
            {
                Layers = layers, Tool = tool, Region = region, Transform = transform,
                Result = result, Out = tr, Runtime = runtime,
            });
            if (!tr.Ok) tr.Pose = null;
        }

        /// <summary>Why a tool that depends on <paramref name="id"/> cannot run; null when it can.</summary>
        private static string? NotRunBecause(string id, string role, Dictionary<string, VisionTool> earlier, VisionResult result)
        {
            if (!earlier.TryGetValue(id, out var dep)) return $"{role} '{id}' is not an earlier tool in this program";
            if (!dep.Enabled) return $"{role} '{Label(dep)}' is disabled";
            var r = result.Tool(id);
            if (r == null) return $"{role} '{Label(dep)}' did not run";
            if (r.Status == ToolStatus.NotRun) return $"{role} '{Label(dep)}' did not run ({r.Message})";
            if (!r.Ok) return $"{role} '{Label(dep)}' not found";
            return null;
        }

        private static string Label(VisionTool t) => string.IsNullOrEmpty(t.Name) ? t.Id : t.Name;

        /// <summary>
        /// The tool whose region and fixture <paramref name="tool"/> uses: itself, or the end of its
        /// <c>regionFrom</c> chain. Each link must be an earlier tool (enabled or not — only its
        /// region is borrowed).
        /// </summary>
        internal static VisionTool RegionOwner(VisionTool tool, IReadOnlyDictionary<string, VisionTool> earlier)
        {
            var cur = tool;
            var seen = new HashSet<string> { tool.Id };
            while (!string.IsNullOrEmpty(cur.RegionFrom))
            {
                if (!earlier.TryGetValue(cur.RegionFrom, out var src))
                    throw new InvalidOperationException($"region source '{cur.RegionFrom}' is not an earlier tool in this program");
                if (!seen.Add(src.Id)) throw new InvalidOperationException($"region source '{Label(src)}' leads back to itself");
                cur = src;
            }
            return cur;
        }

        /// <summary>
        /// Every limit satisfied: each keyed value must exist and be in range — except a gridded
        /// tool's main value, which means every cell passed (each cell was judged on it).
        /// </summary>
        internal static bool LimitsPass(VisionTool tool, ToolResult tr)
        {
            if (tool.Limits == null) return true;
            string? main = ToolRegistry.Get(tool.Type)?.MainValue(tool);
            foreach (var (key, limit) in tool.Limits)
            {
                if (limit == null) continue;
                if (key == "passed") continue;
                if (key == main && tr.Cells.Count > 0)
                {
                    if (!tr.Cells.All(c => c.Passed)) return false;
                    continue;
                }
                if (!tr.TryGetNumber(key, out var v) || !limit.Accepts(v)) return false;
            }
            return true;
        }

        // ── Teaching ─────────────────────────────────────────────────────────────

        /// <summary>
        /// Teaches a patternMatch / contourMatch tool from <paramref name="src"/>: earlier tools
        /// run first so a fixtured region is where the fixture puts it, then the template is cut
        /// from the tool's region on its input layer and stored in params.template.
        /// <paramref name="area"/> (normalized, in the frame as it is — no fixture) teaches from
        /// that area instead, e.g. a rotated rectangle drawn round the part: the pattern is cut
        /// deskewed to the rectangle's axes and its angle becomes the pattern's 0°.
        /// </summary>
        public static string Teach(VisionProgram program, string toolId, Mat src, VisionRuntime runtime, VisionRegion? area = null)
        {
            var tool = program.Tool(toolId) ?? throw new InvalidOperationException($"no tool '{toolId}' in this program");
            if (tool.Type is not ("patternMatch" or "contourMatch"))
                throw new InvalidOperationException($"tool '{Label(tool)}' is a {tool.Type}; only patternMatch and contourMatch are taught");
            string summary = "";
            Execute(program, src, runtime, toolId, (layers, result) =>
            {
                var earlier = new Dictionary<string, VisionTool>();
                foreach (var t in program.Tools ?? [])
                {
                    if (t == null || string.IsNullOrEmpty(t.Id)) continue;
                    if (t.Id == toolId) break;
                    earlier.TryAdd(t.Id, t);
                }
                var owner = RegionOwner(tool, earlier);
                var transform = FixtureTransform.Identity;
                if (area == null && !string.IsNullOrEmpty(owner.FixtureId))
                {
                    var fr = result.Tool(owner.FixtureId);
                    var ft = program.Tool(owner.FixtureId);
                    if (fr?.Pose == null || ft == null) throw new InvalidOperationException($"fixture '{owner.FixtureId}' not found on this image");
                    transform = FixtureTransform.Between(ft.Reference ?? fr.Pose, fr.Pose) ?? FixtureTransform.Identity;
                }
                var region = area != null
                    ? PixelRegion.FromNormalized(area, layers.Width, layers.Height)
                    : PixelRegion.FromNormalized(owner.Region, layers.Width, layers.Height).Transform(transform);
                if (!layers.Exists(tool.Input)) throw new InvalidOperationException($"input layer '{tool.Input}' does not exist");
                var gray = layers.Gray(tool.Input);
                if (tool.Type == "patternMatch")
                {
                    var t = PatternMatchTool.Teach(gray, region, area);
                    tool.SetParam("template", t);
                    summary = $"pattern {t.Width}×{t.Height} px";
                }
                else
                {
                    var ctx = new ToolContext
                    {
                        Layers = layers, Tool = tool, Region = region, Transform = transform, Result = result,
                        Out = new ToolResult(), Runtime = runtime,
                    };
                    var (bin, box) = ToolMath.Binarize(ctx);
                    using (bin)
                    {
                        Cv2.FindContours(bin, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
                        var best = contours.OrderByDescending(ct => Cv2.ContourArea(ct)).FirstOrDefault();
                        if (best == null || best.Length < 3) throw new InvalidOperationException("no shape in the region to teach (check polarity / threshold)");
                        var r = Cv2.BoundingRect(best);
                        double bw = Math.Max(1, r.Width), bh = Math.Max(1, r.Height);
                        var t = new ContourMatchTool.Template
                        {
                            Width = bw, Height = bh,
                            Points = best.Select(q => new VisionPoint(Math.Round((q.X - r.X) / bw, 4), Math.Round((q.Y - r.Y) / bh, 4))).ToList(),
                        };
                        tool.SetParam("template", t);
                        summary = $"contour of {t.Points.Count} points, {bw}×{bh} px";
                    }
                }
            });
            return summary;
        }
    }
}
