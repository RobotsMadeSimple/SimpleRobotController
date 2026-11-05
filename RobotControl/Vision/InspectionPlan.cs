using Controller.RobotControl.Vision.Inspections;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// One enabled inspection bound to its strategy and resolved zone, ready to run on a frame.
    /// </summary>
    internal sealed class InspectionStep
    {
        /// <summary>
        /// The zone to use this frame: the drawn one, or one warped by its alignment frame.
        /// Skip = the frame was not found, so the inspection must not run on a guessed zone.
        /// </summary>
        internal readonly record struct ZoneThisFrame(VisionZone? Zone, bool Skip, string? Error);

        private readonly Action<FrameContext, Mat, LabelStack, VisionResult> _execute;
        private readonly Action<VisionResult>? _onFailure;

        private InspectionStep(string id, Action<FrameContext, Mat, LabelStack, VisionResult> execute,
                               Action<VisionResult>? onFailure)
        {
            Id         = id;
            _execute   = execute;
            _onFailure = onFailure;
        }

        public string Id { get; }

        /// <summary>Detects, annotates, then adds the result to <paramref name="result"/>.</summary>
        public void Execute(FrameContext ctx, Mat annotated, LabelStack labels, VisionResult result) =>
            _execute(ctx, annotated, labels, result);

        /// <summary>Records whatever the inspection type reports when it throws (blob: an empty result).</summary>
        public void RecordFailure(VisionResult result) => _onFailure?.Invoke(result);

        public static InspectionStep Create<TInsp, TResult>(
            IInspectionStrategy<TInsp, TResult> strategy, TInsp insp, string id, VisionZone? zone,
            Action<VisionResult, TResult> publish, Action<VisionResult>? onFailure = null,
            Func<FrameContext, VisionResult, ZoneThisFrame>? align = null)
        {
            return new InspectionStep(id, (ctx, annotated, labels, result) =>
            {
                var useZone = zone;
                if (align != null)
                {
                    var z = align(ctx, result);
                    if (z.Error != null) result.Errors.Add($"{id}: {z.Error}");
                    if (z.Skip) return;
                    useZone = z.Zone;
                }
                var r = strategy.Run(ctx, insp, useZone);
                try
                {
                    strategy.Annotate(annotated, r, insp, useZone, labels);
                    publish(result, r);
                }
                finally
                {
                    (r as IDisposable)?.Dispose();
                }
            }, onFailure);
        }
    }

    /// <summary>
    /// A program compiled for the processing loop: enabled inspections bound to strategies
    /// and zones, in run order, plus the zones to outline. Built once per program update so
    /// the per-frame loop does no lookups or parsing. Immutable once built.
    /// </summary>
    internal sealed class InspectionPlan
    {
        // Stateless strategies are shared by every processor.
        public static readonly BlobStrategy          Blob    = new();
        public static readonly ColorCoverageStrategy Color   = new();
        public static readonly PolygonStrategy       Polygon = new();
        public static readonly LineStrategy          Line    = new();
        public static readonly FrameStrategy         Frame   = new();
        public static readonly CornerStrategy        Corner  = new();
        public static readonly EdgeStrategy          Edge    = new();

        private InspectionPlan(VisionProgram program, IReadOnlyList<InspectionStep> steps, IReadOnlyList<VisionZone> borderZones)
        {
            Program     = program;
            Steps       = steps;
            BorderZones = borderZones;
        }

        public VisionProgram                 Program     { get; }
        public IReadOnlyList<InspectionStep> Steps       { get; }
        /// <summary>Zones used by at least one enabled inspection, in zone-list order.</summary>
        public IReadOnlyList<VisionZone>     BorderZones { get; }

        public static InspectionPlan Build(VisionProgram prog, ArucoStrategy aruco)
        {
            var barcode = new BarcodeStrategy(prog.BarcodeInspections ?? []);
            VisionZone? Zone(string? id) => ResolveZone(prog, id);
            Func<FrameContext, VisionResult, InspectionStep.ZoneThisFrame>? Align(string? id) =>
                Zone(id) is { AlignTo: { Length: > 0 } } z ? (ctx, vr) => AlignZone(prog, z, vr, ctx.Width, ctx.Height) : null;

            // Frame inspections run before everything else (in their stored order), because
            // zones aligned to them need this frame's corners.
            var frames = new List<InspectionStep>();
            foreach (var i in prog.FrameInspections ?? [])
                if (i.Enabled)
                    frames.Add(InspectionStep.Create(Frame, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.FrameResults.Add(d.Result)));

            // Type order; Order() then applies the stored InspectionOrder on top.
            var steps = new List<InspectionStep>();
            foreach (var i in prog.Inspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Blob, i, i.Id, Zone(i.ZoneId),
                        (vr, r) => vr.Inspections.Add(r),
                        vr => vr.Inspections.Add(BlobStrategy.Empty(i)), Align(i.ZoneId)));
            foreach (var i in prog.ColorInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Color, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.ColorResults.Add(d.Result), align: Align(i.ZoneId)));
            foreach (var i in prog.PolygonInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Polygon, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.PolygonResults.Add(d.Result), align: Align(i.ZoneId)));
            foreach (var i in prog.ArucoInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(aruco, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.ArucoResults.Add(d.Result), align: Align(i.ZoneId)));
            foreach (var i in prog.LineInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Line, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.LineResults.Add(d.Result), align: Align(i.ZoneId)));
            foreach (var i in prog.BarcodeInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(barcode, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.BarcodeResults.Add(d.Result), align: Align(i.ZoneId)));
            foreach (var i in prog.CornerInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Corner, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.CornerResults.Add(d.Result), align: Align(i.ZoneId)));
            foreach (var i in prog.EdgeInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Edge, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.EdgeResults.Add(d.Result), align: Align(i.ZoneId)));

            // Outline only zones actually used by an enabled inspection. This reflects any
            // runtime zone override (all inspections then point at the chosen zone) and shows
            // the zone genuinely in use rather than every defined zone.
            var usedZoneIds = prog.AllInspections()
                .Where(i => i.Enabled && !string.IsNullOrEmpty(i.ZoneId))
                .Select(i => i.ZoneId!)
                .ToHashSet();
            // An aligned zone is outlined where it was actually used (VisionResult.AlignedZones),
            // not where it was drawn.
            var borderZones = (prog.Zones ?? [])
                .Where(z => usedZoneIds.Contains(z.Id) && string.IsNullOrEmpty(z.AlignTo)).ToList();

            var ordered = Order(frames, s => s.Id, prog.InspectionOrder);
            ordered.AddRange(Order(steps, s => s.Id, prog.InspectionOrder));
            return new InspectionPlan(prog, ordered, borderZones);
        }

        /// <summary>
        /// The zone warped onto this frame by its alignment frame: the homography from the
        /// frame's reference corners to the corners found now. Not found → skip (an inspection
        /// on a guessed zone would report the wrong cells); no reference → used as drawn.
        /// </summary>
        internal static InspectionStep.ZoneThisFrame AlignZone(VisionProgram prog, VisionZone zone, VisionResult vr, int w, int h)
        {
            var frame = prog.FrameInspections?.FirstOrDefault(f => f.Id == zone.AlignTo);
            if (frame == null)
                return new(zone, false, $"zone '{zone.Name}': alignment frame '{zone.AlignTo}' is not in this program");
            if (!frame.Enabled)
                return new(zone, false, $"zone '{zone.Name}': alignment frame '{frame.Name}' is disabled — used as drawn");
            var found = vr.FrameResults.FirstOrDefault(r => r.InspectionId == frame.Id);
            if (found is not { Found: true })
                return new(null, true, $"zone '{zone.Name}': alignment frame '{frame.Name}' not found");
            if (frame.ReferenceCorners is not { Count: 4 } reference)
                return new(zone, false, $"zone '{zone.Name}': frame '{frame.Name}' has no reference — set one");
            var hNorm = ZoneGeometry.Between(reference, found.Corners);
            if (hNorm == null)
                return new(null, true, $"zone '{zone.Name}': frame '{frame.Name}' corners do not form a usable quad");

            var warped = ZoneGeometry.Warp(zone, hNorm, w, h);
            if (!vr.AlignedZones.Any(a => a.ZoneId == zone.Id))
                vr.AlignedZones.Add(new AlignedZone
                {
                    ZoneId = zone.Id,
                    Points = ZoneGeometry.Outline(warped.Geometry, w, h)
                        .Select(p => new[] { Math.Round(p[0], 4), Math.Round(p[1], 4) }).ToList(),
                });
            return new(warped, false, null);
        }

        /// <summary>The zone an inspection points at; null for none or an unknown id.</summary>
        public static VisionZone? ResolveZone(VisionProgram prog, string? zoneId) =>
            string.IsNullOrEmpty(zoneId) ? null : prog.Zones?.FirstOrDefault(z => z.Id == zoneId);

        /// <summary>
        /// Sorts <paramref name="typeOrdered"/> by each item's position in
        /// <paramref name="order"/> (first occurrence wins). Items whose id is not listed keep
        /// their incoming relative order and run after every listed item.
        /// </summary>
        internal static List<T> Order<T>(IEnumerable<T> typeOrdered, Func<T, string> idOf, IReadOnlyList<string>? order)
        {
            var rank = new Dictionary<string, int>();
            if (order != null)
                for (int i = 0; i < order.Count; i++)
                    if (order[i] != null) rank.TryAdd(order[i], i);

            // OrderBy is stable, so equal ranks (all the unlisted ones) keep type order.
            return typeOrdered
                .OrderBy(x => rank.TryGetValue(idOf(x), out var r) ? r : int.MaxValue)
                .ToList();
        }
    }
}
