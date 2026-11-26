using System.Diagnostics;
using Controller.RobotControl;
using Controller.RobotControl.Conveyors;
using Controller.RobotControl.Programs;

namespace RobotControl.Tests.Conveyors;

/// <summary>
/// Conveyor tracking on the real motion path: a never-connected controller (identity kinematics),
/// its motion thread played tick by tick on the test thread (RunCommands + RunMotion, exactly what
/// MotionLoop does), and a fake belt running on the real clock. Proves the TCP follows a moving part
/// and that ending the tracking freezes it.
/// </summary>
public sealed class ConveyorMotionTests : IDisposable
{
    private const double BeltMmS = 40;        // 400 steps/s at 10 steps/mm
    private readonly RobotController _robot = new RobotController().WithIdentityKinematics();
    private readonly FakeBelt _belt = new(Stopwatch.GetTimestamp);
    private readonly ConveyorService _svc;
    private readonly Conveyor _c;

    public ConveyorMotionTests()
    {
        (_svc, var saved) = ConveyorKit.Service(_belt, Stopwatch.GetTimestamp, ConveyorKit.Aux("Belt", rz: 30));
        _c = saved[0];
        _c.Window.EndMm = 2000;
        _svc.Save(_c);
        _robot.Conveyors = _svc;
        _belt.StepsPerSec = BeltMmS * _belt.Spm;
    }

    public void Dispose() => _svc.Dispose();

    /// <summary>Plays the motion thread for <paramref name="seconds"/> of real time (or until
    /// <paramref name="until"/> holds).</summary>
    private void Run(double seconds, Func<bool>? until = null)
    {
        var sw = Stopwatch.StartNew();
        while (sw.Elapsed.TotalSeconds < seconds)
        {
            _robot.RunCommands();
            _robot.RunMotion();
            if (until?.Invoke() == true) return;
        }
    }

    /// <summary>Where the point <paramref name="onPart"/> (part-frame coordinates) is in the world now.</summary>
    private Vector6 PartPointNow(Vector6 onPart)
    {
        var x = _svc.TrackedPartX()!.Value;
        return LocalFrame.Apply(_c.PartFrame(x), onPart);
    }

    /// <summary>A tracked move as the program dispatcher builds it: the target resolved in the part
    /// frame now, stamped with the travel it was resolved at.</summary>
    private RobotCommand TrackedMove(string type, Vector6 onPart)
    {
        Assert.True(_svc.TryPartFrame(out var frame, out var travel));
        var t = LocalFrame.Apply(frame, onPart);
        return new RobotCommand
        {
            CommandType = type, X = t.X, Y = t.Y, Z = t.Z, RX = t.RX, RY = t.RY, RZ = t.RZ,
            Speed = 300, Accel = 2000, Decel = 2000, TrackTravel = travel,
        };
    }

    private static double Dist(Vector6 a, Vector6 b) => a.Distance3(b);

    [Fact]
    public void ATrackedMoveLCatchesThePartAndTheRobotThenHoldsOnIt()
    {
        var onPart = new Vector6(5, -3, 20);
        long part = _svc.AddPart(_c.Id, 120);
        Assert.True(_svc.StartTracking(_c.Id, part, owner: null));
        _robot.RunMotion();                                      // the motion thread sees the session

        _robot.QueuedCommands.Enqueue(TrackedMove("MoveL", onPart));
        Run(3, () => _robot.QueuedCommands.IsEmpty && !_robot.IsMoving);
        Assert.False(_robot.IsMoving, $"q={_robot.QueuedCommands.Count} pos={_robot.GetCurrentPosition().X:0.##},{_robot.GetCurrentPosition().Y:0.##},{_robot.GetCurrentPosition().Z:0.##} tgt={_robot.LiveTargetPosition.X:0.##},{_robot.LiveTargetPosition.Y:0.##} fault={_robot.FaultStatus.Message}");
        // The move ended on the moving part …
        Assert.True(Dist(_robot.GetCurrentPosition(), PartPointNow(onPart)) < 0.5,
            $"TCP {_robot.GetCurrentPosition().X:0.##},{_robot.GetCurrentPosition().Y:0.##} vs part {PartPointNow(onPart).X:0.##},{PartPointNow(onPart).Y:0.##}");

        // … and with no move running the robot keeps riding along with it.
        var start = _robot.GetCurrentPosition();
        for (int i = 0; i < 5; i++)
        {
            Run(0.06);
            Assert.True(_robot.TrackingHold);
            Assert.True(Dist(_robot.GetCurrentPosition(), PartPointNow(onPart)) < 0.5);
        }
        double moved = Dist(_robot.GetCurrentPosition(), start);
        Assert.InRange(moved, BeltMmS * 0.25, BeltMmS * 0.6);   // ≈ 0.3 s × 40 mm/s along the belt
        Assert.False(_robot.IsMoving);                           // a hold is no move: the program is not held up

        // A synchronised move from the hold: lift 30 mm off the part while following it.
        var lifted = new Vector6(onPart.X, onPart.Y, onPart.Z + 30);
        _robot.QueuedCommands.Enqueue(TrackedMove("MoveL", lifted));
        Run(3, () => _robot.QueuedCommands.IsEmpty && !_robot.IsMoving);
        Assert.True(Dist(_robot.GetCurrentPosition(), PartPointNow(lifted)) < 0.5);

        // EndTracking freezes the offset: the robot stays where it is while the belt runs on.
        _svc.EndTracking(release: false);
        Run(0.02);
        var frozen = _robot.GetCurrentPosition();
        Run(0.2);
        Assert.True(Dist(_robot.GetCurrentPosition(), frozen) < 1e-6);
        Assert.False(_robot.TrackingHold);
        Assert.Equal(1, _svc.Status().Single().Picked);

        // Later moves are ordinary ones.
        var plain = new Vector6(frozen.X - 20, frozen.Y, frozen.Z);
        _robot.MoveL(plain, 300, 2000, 2000, null);
        Run(3, () => !_robot.IsMoving);
        Assert.True(Dist(_robot.GetCurrentPosition(), plain) < 1e-6);
    }

    [Fact]
    public void ATrackedMoveJAimsAtTheInterceptAndTheNextMoveLSynchronises()
    {
        var onPart = new Vector6(0, 0, 15);
        long part = _svc.AddPart(_c.Id, 150);
        _svc.StartTracking(_c.Id, part, owner: null);
        _robot.RunMotion();

        _robot.QueuedCommands.Enqueue(TrackedMove("MoveJ", onPart));
        Run(3, () => _robot.QueuedCommands.IsEmpty && !_robot.IsMoving);
        // Planned untracked to where the part would be when the move ends.
        Assert.True(Dist(_robot.GetCurrentPosition(), PartPointNow(onPart)) < 2.0,
            $"intercept off by {Dist(_robot.GetCurrentPosition(), PartPointNow(onPart)):0.##} mm");
        Assert.False(_robot.TrackingHold);                       // a MoveJ does not synchronise

        _robot.QueuedCommands.Enqueue(TrackedMove("MoveL", onPart));
        Run(3, () => _robot.QueuedCommands.IsEmpty && !_robot.IsMoving);
        Assert.True(Dist(_robot.GetCurrentPosition(), PartPointNow(onPart)) < 0.5);
        Assert.True(_robot.TrackingHold);
    }

    [Fact]
    public void ABeltFasterThanTheRobotStopsTrackingWithConveyorSpeed()
    {
        var owner = new object();
        long part = _svc.AddPart(_c.Id, 120);
        _svc.StartTracking(_c.Id, part, owner);
        _robot.RunMotion();
        _robot.QueuedCommands.Enqueue(TrackedMove("MoveL", new Vector6(0, 0, 10)));
        Run(3, () => _robot.QueuedCommands.IsEmpty && !_robot.IsMoving);
        Assert.True(_robot.TrackingHold);

        _belt.StepsPerSec = (_robot.TrackingSpeedLimit + 50) * _belt.Spm; // the belt speeds up past the robot
        Run(0.05);
        Assert.Null(_svc.Tracking);
        Assert.Equal(ErrorCodes.ConveyorSpeed, _svc.ConsumeFault(owner)?.Code);
        var frozen = _robot.GetCurrentPosition();
        Run(0.1);
        Assert.True(Dist(_robot.GetCurrentPosition(), frozen) < 1e-6);
    }

    [Fact]
    public void AHardStopStopsTheHoldDead()
    {
        long part = _svc.AddPart(_c.Id, 120);
        _svc.StartTracking(_c.Id, part, owner: null);
        _robot.RunMotion();
        _robot.QueuedCommands.Enqueue(TrackedMove("MoveL", new Vector6(0, 0, 10)));
        Run(3, () => _robot.QueuedCommands.IsEmpty && !_robot.IsMoving);
        Assert.True(_robot.TrackingHold);

        // What MotionLoop does with a HardStop() request at the top of its next tick.
        typeof(RobotController).GetMethod("ExecuteHardStop",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!.Invoke(_robot, null);
        Run(0.02);
        var stopped = _robot.GetCurrentPosition();
        Run(0.1);
        Assert.True(Dist(_robot.GetCurrentPosition(), stopped) < 1e-6);
        Assert.NotNull(_svc.Tracking);                           // the session stays: Continue blends back on
    }
}
