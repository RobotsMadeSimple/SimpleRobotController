namespace Controller.RobotControl.Conveyors
{
    /// <summary>A latch-pin edge reported by an aux board, with the step counts captured at the edge.</summary>
    /// <param name="StopwatchTicks">Controller <see cref="System.Diagnostics.Stopwatch"/> time of the edge.</param>
    public readonly record struct ConveyorLatch(string DeviceId, int Pin, int Value, double[] AxisSteps, long StopwatchTicks);

    /// <summary>
    /// Everything <see cref="ConveyorService"/> needs from the hardware: aux-axis live positions and
    /// latch edges, the IO a sensor or a timed belt reads. The controller's implementation is
    /// <see cref="ControllerConveyorSource"/>; tests use a fake.
    /// </summary>
    public interface IConveyorTravelSource
    {
        /// <summary>The axis' live step count and step rate, extrapolated to now. Thread-safe and
        /// non-blocking (called from the motion thread). False when the device is not streaming.</summary>
        bool TryGetLivePosition(string deviceId, int axis, out double steps, out double stepsPerSec);

        /// <summary>Steps per mm of a Linear axis with mmPerRev; 0 when it has no physical unit.</summary>
        double StepsPerMm(string deviceId, int axis);

        /// <summary>Turn on live position streaming on an aux device (0 = off).</summary>
        void SetPositionStreaming(string deviceId, int periodMs);

        /// <summary>Raised on an aux-board latch-pin edge (any thread).</summary>
        event Action<ConveyorLatch>? InputLatched;

        /// <summary>The current state of a sensor input (<c>io</c> / <c>stb</c> kinds), or null when unknown.</summary>
        bool? ReadInput(ConveyorSensor sensor);

        /// <summary>The current state of an output (the timed source's runOutput), or null when unknown.</summary>
        bool? ReadOutput(ConveyorIoRef output);
    }
}
