using OpenCvSharp;

namespace Controller.RobotControl.Vision.Inspections
{
    /// <summary>
    /// One inspection type's detection, drawing and debug rendering.
    ///
    /// <typeparamref name="TResult"/> is the strategy's own detection record: the public
    /// result that goes into <see cref="VisionResult"/> plus whatever pixel-space data the
    /// overlay needs. If it is <see cref="System.IDisposable"/> the caller disposes it once
    /// the frame has been annotated.
    /// </summary>
    internal interface IInspectionStrategy<TInsp, TResult>
    {
        /// <summary>Detection only — reads the frame, never draws.</summary>
        TResult Run(FrameContext ctx, TInsp insp, VisionZone? zone);

        /// <summary>Draws this inspection's overlay and status label onto the annotated frame.</summary>
        void Annotate(Mat annotated, TResult result, TInsp insp, VisionZone? zone, LabelStack labels);

        /// <summary>A JPEG showing the intermediate pipeline stages, for the /debug endpoints.</summary>
        byte[]? RenderDebug(FrameContext ctx, TInsp insp, VisionZone? zone) => null;
    }
}
