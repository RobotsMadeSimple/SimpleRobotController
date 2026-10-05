using OpenCvSharp;
using System;
using System.Collections.Generic;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// One decoded frame plus the intermediates inspections derive from it, built lazily and
    /// shared so that several inspections on the same frame convert to grayscale, blur, run
    /// Canny or rasterise a zone once between them rather than once each.
    ///
    /// Every Mat handed out is owned by the context and disposed with it: callers must treat
    /// them as read-only (write into a new Mat instead) and must not dispose them. The source
    /// Mat is borrowed, not owned. Single-threaded: one context per frame per thread.
    /// </summary>
    internal sealed class FrameContext : IDisposable
    {
        /// <summary>Gaussian kernel size (square) used for <see cref="Blurred"/>.</summary>
        public const int BlurKernelSize = 5;

        private Mat? _gray;
        private Mat? _blurred;
        private Mat? _fullMask;
        private readonly Dictionary<(double, double), Mat> _canny     = new();
        private readonly Dictionary<string, Mat>           _zoneMasks = new();
        private bool _disposed;

        public FrameContext(Mat source)
        {
            Source = source;
            Width  = source.Width;
            Height = source.Height;
        }

        /// <summary>The decoded BGR frame. Borrowed — the caller that decoded it disposes it.</summary>
        public Mat Source { get; }
        public int Width  { get; }
        public int Height { get; }
        public Size Size => new(Width, Height);

        /// <summary>Single-channel grayscale of <see cref="Source"/>.</summary>
        public Mat Gray
        {
            get
            {
                ThrowIfDisposed();
                if (_gray == null)
                {
                    var gray = new Mat();
                    Cv2.CvtColor(Source, gray, ColorConversionCodes.BGR2GRAY);
                    _gray = gray;
                }
                return _gray;
            }
        }

        /// <summary><see cref="Gray"/> smoothed with a <see cref="BlurKernelSize"/>² Gaussian.</summary>
        public Mat Blurred
        {
            get
            {
                ThrowIfDisposed();
                if (_blurred == null)
                {
                    var blurred = new Mat();
                    Cv2.GaussianBlur(Gray, blurred, new Size(BlurKernelSize, BlurKernelSize), 0);
                    _blurred = blurred;
                }
                return _blurred;
            }
        }

        /// <summary>Canny edges of <see cref="Gray"/> for one threshold pair.</summary>
        public Mat Canny(double threshold1, double threshold2)
        {
            ThrowIfDisposed();
            var key = (threshold1, threshold2);
            if (!_canny.TryGetValue(key, out var edges))
            {
                edges = new Mat();
                Cv2.Canny(Gray, edges, threshold1, threshold2);
                _canny[key] = edges;
            }
            return edges;
        }

        /// <summary>
        /// White-on-black mask of the zone, the size of the frame. A null zone means "the
        /// whole frame" and yields an all-white mask. Cached by zone id — within one frame a
        /// zone id always names the same geometry.
        /// </summary>
        public Mat ZoneMask(VisionZone? zone)
        {
            ThrowIfDisposed();
            if (zone == null)
                return _fullMask ??= new Mat(Size, MatType.CV_8UC1, Scalar.White);

            if (!_zoneMasks.TryGetValue(zone.Id, out var mask))
            {
                mask = new Mat(Size, MatType.CV_8UC1, Scalar.Black);
                ZoneGeometry.FillZoneMask(mask, zone.Geometry, Width, Height);
                _zoneMasks[zone.Id] = mask;
            }
            return mask;
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(FrameContext));
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _gray?.Dispose();
            _blurred?.Dispose();
            _fullMask?.Dispose();
            foreach (var m in _canny.Values)     m.Dispose();
            foreach (var m in _zoneMasks.Values) m.Dispose();
            _canny.Clear();
            _zoneMasks.Clear();
        }
    }
}
