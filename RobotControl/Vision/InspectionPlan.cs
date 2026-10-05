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
            Action<VisionResult, TResult> publish, Action<VisionResult>? onFailure = null)
        {
            return new InspectionStep(id, (ctx, annotated, labels, result) =>
            {
                var r = strategy.Run(ctx, insp, zone);
                try
                {
                    strategy.Annotate(annotated, r, insp, zone, labels);
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

            // Type order; Order() then applies the stored InspectionOrder on top.
            var steps = new List<InspectionStep>();
            foreach (var i in prog.Inspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Blob, i, i.Id, Zone(i.ZoneId),
                        (vr, r) => vr.Inspections.Add(r),
                        vr => vr.Inspections.Add(BlobStrategy.Empty(i))));
            foreach (var i in prog.ColorInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Color, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.ColorResults.Add(d.Result)));
            foreach (var i in prog.PolygonInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Polygon, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.PolygonResults.Add(d.Result)));
            foreach (var i in prog.ArucoInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(aruco, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.ArucoResults.Add(d.Result)));
            foreach (var i in prog.LineInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(Line, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.LineResults.Add(d.Result)));
            foreach (var i in prog.BarcodeInspections ?? [])
                if (i.Enabled)
                    steps.Add(InspectionStep.Create(barcode, i, i.Id, Zone(i.ZoneId), (vr, d) => vr.BarcodeResults.Add(d.Result)));

            // Outline only zones actually used by an enabled inspection. This reflects any
            // runtime zone override (all inspections then point at the chosen zone) and shows
            // the zone genuinely in use rather than every defined zone.
            var usedZoneIds = prog.AllInspections()
                .Where(i => i.Enabled && !string.IsNullOrEmpty(i.ZoneId))
                .Select(i => i.ZoneId!)
                .ToHashSet();
            var borderZones = (prog.Zones ?? []).Where(z => usedZoneIds.Contains(z.Id)).ToList();

            return new InspectionPlan(prog, Order(steps, s => s.Id, prog.InspectionOrder), borderZones);
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
