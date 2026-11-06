using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    using Point = OpenCvSharp.Point;

    /// <summary>Draws tool results over a frame: regions, cells, edges, lines, poses and a status label per tool.</summary>
    internal static class VisionOverlay
    {
        public static void DrawAll(Mat img, VisionProgram program, VisionResult result)
        {
            var labels = new LabelStack(img.Height);
            foreach (var tr in result.Tools)
            {
                var tool = program.Tool(tr.Id);
                DrawTool(img, tr);
                labels.DrawStatusLabel(img, Label(tool, tr), tr.Passed);
            }
        }

        private static string Label(VisionTool? tool, ToolResult tr)
        {
            string name = string.IsNullOrEmpty(tool?.Name) ? tr.Id : tool!.Name;
            if (tr.Status == ToolStatus.NotRun) return $"{name}: not run";
            if (tr.Status == ToolStatus.Error)  return $"{name}: error";
            if (tr.Status == ToolStatus.Fail)   return $"{name}: not found";
            if (tr.Cells.Count > 0 && tr.TryGetNumber("cellsPassed", out var cp)) return $"{name}: {cp:0}/{tr.Cells.Count}";
            if (tr.TryGetNumber("count", out var n)) return $"{name}: {n:0}";
            return name;
        }

        public static void DrawTool(Mat img, ToolResult tr)
        {
            var regionColor = tr.Status == ToolStatus.NotRun ? VisionPalette.DebugIgnored
                            : tr.Passed ? VisionPalette.ZoneBorder : VisionPalette.Fail;
            if (tr.Region.Count >= 2)
                Cv2.Polylines(img, new[] { Pts(tr.Region) }, true, regionColor, 1, LineTypes.AntiAlias);

            foreach (var cell in tr.Cells)
                if (cell.Quad is { Count: 4 } q)
                {
                    var pts = Pts(q);
                    Cv2.Polylines(img, new[] { pts }, true, cell.Passed ? VisionPalette.Pass : VisionPalette.Fail, cell.Passed ? 2 : 1);
                    var at = new Point((int)pts.Average(p => p.X) - 8, (int)pts.Average(p => p.Y) + 4);
                    Cv2.PutText(img, $"{cell.Value:0}", at, VisionDrawing.Font, VisionDrawing.CellFontScale, VisionPalette.GridCellText, 1);
                }

            foreach (var l in tr.Lines)
                Cv2.Line(img, P(l.X1, l.Y1), P(l.X2, l.Y2), VisionPalette.LineSegment, VisionDrawing.OverlayThickness, LineTypes.AntiAlias);

            foreach (var p in tr.Points.Take(500))
                Cv2.Circle(img, P(p.X, p.Y), 3, VisionPalette.Corner, -1, LineTypes.AntiAlias);

            foreach (var item in tr.Items.Take(200))
                if (item.TryGetValue("x", out var xo) && item.TryGetValue("y", out var yo) && xo is double x && yo is double y)
                    Cv2.DrawMarker(img, P(x, y), VisionPalette.Blob, MarkerTypes.Cross, 12, 2);

            if (tr.Pose is { } pose)
            {
                var o = P(pose.X, pose.Y);
                double a = pose.Angle * Math.PI / 180;
                Cv2.ArrowedLine(img, o, P(pose.X + 30 * Math.Cos(a), pose.Y + 30 * Math.Sin(a)), VisionPalette.PolygonArrow, 2, LineTypes.AntiAlias, 0, 0.25);
                Cv2.Circle(img, o, 4, VisionPalette.PolygonArrow, -1, LineTypes.AntiAlias);
                if (pose.Quad is { Count: 4 } pq)
                    Cv2.Polylines(img, new[] { Pts(pq) }, true, VisionPalette.Frame, 2, LineTypes.AntiAlias);
            }
        }

        private static Point P(double x, double y) =>
            new((int)Math.Round(double.IsFinite(x) ? Math.Clamp(x, -1e5, 1e5) : 0), (int)Math.Round(double.IsFinite(y) ? Math.Clamp(y, -1e5, 1e5) : 0));

        private static Point[] Pts(IEnumerable<PxPoint> pts) => pts.Select(p => P(p.X, p.Y)).ToArray();
    }
}
