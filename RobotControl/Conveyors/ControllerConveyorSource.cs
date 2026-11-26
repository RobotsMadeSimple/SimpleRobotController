using Controller.RobotControl.AuxAxis;

namespace Controller.RobotControl.Conveyors
{
    /// <summary>
    /// <see cref="IConveyorTravelSource"/> over the running controller: aux live positions, latch
    /// edges and streaming from <see cref="AuxAxisManager"/>; sensor inputs from the driver board
    /// and the IO devices; the timed belt's run output as SetOutput reads it.
    /// </summary>
    internal sealed class ControllerConveyorSource : IConveyorTravelSource
    {
        private readonly RobotController _robot;
        private AuxAxisManager? _aux;

        public ControllerConveyorSource(RobotController robot)
        {
            _robot = robot;
            _aux   = robot.AuxAxisManager;
            if (_aux != null) _aux.InputLatched += OnLatched;
        }

        public event Action<ConveyorLatch>? InputLatched;

        private void OnLatched(AuxLatchEvent e) =>
            InputLatched?.Invoke(new ConveyorLatch(e.DeviceId, e.Pin, e.Value, e.AxisSteps, e.StopwatchTicks));

        public bool TryGetLivePosition(string deviceId, int axis, out double steps, out double stepsPerSec)
        {
            if (_aux != null) return _aux.TryGetLivePosition(deviceId, axis, out steps, out stepsPerSec);
            steps = stepsPerSec = 0;
            return false;
        }

        public double StepsPerMm(string deviceId, int axis)
        {
            var cfg = _aux?.GetAxisConfig(deviceId, axis);
            return cfg is { AxisType: AuxAxisType.Linear } ? cfg.StepsPerUnit() : 0;
        }

        public void SetPositionStreaming(string deviceId, int periodMs) => _aux?.SetPositionStreaming(deviceId, periodMs);

        public bool? ReadInput(ConveyorSensor sensor)
        {
            switch (sensor.Kind)
            {
                case ConveyorSensorKinds.Stb:
                    return sensor.Input switch
                    {
                        1 => _robot.stb.Input1, 2 => _robot.stb.Input2,
                        3 => _robot.stb.Input3, 4 => _robot.stb.Input4,
                        _ => null,
                    };
                case ConveyorSensorKinds.Io:
                case ConveyorSensorKinds.AuxPin:
                {
                    if (string.IsNullOrEmpty(sensor.DeviceId) || sensor.Pin is not { } pin) return null;
                    // A Nano first (cheap, no allocation); else an aux board's spare-pin IO.
                    if (_robot.NanoManager?.GetDevice(sensor.DeviceId) is { } nano)
                        return nano.Connected ? nano.GetPinState(pin)?.Value : null;
                    var dev = _aux?.GetAllIoStates().FirstOrDefault(d => d.Id == sensor.DeviceId);
                    if (dev == null || !dev.Connected) return null;
                    return dev.Pins.FirstOrDefault(p => p.Pin == pin)?.Value;
                }
                default:
                    return null;
            }
        }

        public bool? ReadOutput(ConveyorIoRef output) =>
            Execution.IoSteps.ReadOutput(_robot, output.OutputCard ?? "stb", output.OutputNumber, output.OutputDeviceId);
    }
}
