using Controller.RobotControl.Conveyors;
using Controller.RobotControl.MotionProfilers;

namespace Controller.RobotControl
{
    // Conveyor tracking on the motion thread (docs/conveyor-tracking.md, "Motion integration").
    // Nothing here changes the motion loop's cadence or touches the STB loops: RunMotion calls
    // TrackingSample() once per tick (a lock-free travel read) and adds the belt offset to the
    // Cartesian target before IK.
    //
    //   tracked MoveL  output = profile(s) + X̂ · (travel − ref) · blend(s)
    //                  blend ramps 0 → 1 (smoothstep over the move's progress) when the move starts
    //                  untracked, and is 1 when the robot was already following.
    //   hold on part   output = base + X̂ · (travel − ref) while no move runs and the robot follows.
    //   MoveJ          aims at the predicted intercept (target + X̂ · speed · duration), untracked.
    //   tracking ends  the offset freezes: the robot stays where it is.
    internal partial class RobotController
    {
        /// <summary>Configured conveyors and their runtime (travel, queue, tracked part).</summary>
        public ConveyorService Conveyors { get; internal set; } = null!;

        /// <summary>How often the motion thread re-checks reach of the followed pose (s).</summary>
        private const double TrackReachCheckPeriodS = 0.02;
        /// <summary>The reach check looks this far ahead along the belt (s).</summary>
        private const double TrackReachLookaheadS = 0.2;

        // ── Motion-thread tracking state (motion thread only) ─────────────────
        private bool    _trkSession;          // a tracking session is active (as seen this tick)
        private long    _trkGeneration;       // the session last seen
        private double  _trkDirX = 1, _trkDirY;
        private double  _trkTravel, _trkSpeed; // this tick's belt sample
        private bool    _trkFollowing;        // holding on the part: output = _trkBase + X̂·(travel − _trkRef)
        private Vector6 _trkBase = new();
        private double  _trkRef;
        private bool    _trkMove;             // the running linear move is tracked
        private double  _trkMoveRef;
        private bool    _trkMoveRamp;
        private bool    _trkFrozen;           // tracking ended mid-move: keep the last offset
        private double  _trkOffX, _trkOffY;   // the offset applied on the last tick
        private long    _trkNextReachCheck;
        private volatile bool _trkHolding;    // published: following with no move (stb.moving)

        /// <summary>The fastest belt the robot can follow: its linear speed (Set Speed (Linear)),
        /// since it has to catch up with the part on top of the belt's motion.</summary>
        internal double TrackingSpeedLimit => SpeedS;

        /// <summary>True while the robot follows a tracked part with no move running.</summary>
        internal bool TrackingHold => _trkHolding;

        private static readonly double TicksPerSecond = System.Diagnostics.Stopwatch.Frequency;

        /// <summary>
        /// Once per motion tick, before any profiler runs: reads the tracking session and the belt.
        /// A session that ended (EndTracking, a fault, the run stopped) freezes the offset; a new
        /// session starts unsynchronised (the first tracked MoveL blends in).
        /// </summary>
        private void TrackingSample()
        {
            var t = Conveyors?.Tracking;
            if (t == null)
            {
                if (_trkSession)
                {
                    _trkSession   = false;
                    _trkFollowing = false;
                    if (_trkMove) _trkFrozen = true;
                }
                return;
            }
            if (!_trkSession || t.Generation != _trkGeneration)
            {
                _trkSession    = true;
                _trkGeneration = t.Generation;
                _trkFollowing  = false;
                _trkMove       = false;
                _trkFrozen     = false;
                _trkDirX       = t.DirX;
                _trkDirY       = t.DirY;
            }
            Conveyors!.TrySample(t.ConveyorId, Conveyors.Now, out _trkTravel, out _trkSpeed);

            // Belt faster than the robot can move linearly: it could never keep up.
            if ((_trkFollowing || (_trkMove && !_trkFrozen)) && Math.Abs(_trkSpeed) > TrackingSpeedLimit + 1e-6)
            {
                Conveyors.RaiseFault(Programs.ErrorCodes.ConveyorSpeed,
                    $"Conveyor '{t.Config.Name}' runs at {Math.Abs(_trkSpeed):0.#} mm/s — faster than the robot's linear speed ({TrackingSpeedLimit:0.#} mm/s)");
                EndTrackingOnMotionThread();
            }
        }

        // The session ended on this thread (a fault it raised): freeze right away.
        private void EndTrackingOnMotionThread()
        {
            _trkSession   = false;
            _trkFollowing = false;
            if (_trkMove) _trkFrozen = true;
        }

        /// <summary>Any untracked motion (a plain move, a jog, a joint move) ends the hold.</summary>
        private void StopFollowing()
        {
            _trkFollowing = false;
        }

        /// <summary>The belt offset for a tracked linear move at progress <paramref name="s"/>, added
        /// to <see cref="CurrentPosition"/> in place.</summary>
        private void ApplyMoveTracking(double s, bool finished)
        {
            if (!_trkFrozen && _trkSession)
            {
                double b = _trkMoveRamp ? SmoothStep(finished ? 1 : s) : 1;
                double d = (_trkTravel - _trkMoveRef) * b;
                _trkOffX = _trkDirX * d;
                _trkOffY = _trkDirY * d;
                if (!finished) CheckTrackedReach();
            }
            CurrentPosition = new Vector6(CurrentPosition.X + _trkOffX, CurrentPosition.Y + _trkOffY, CurrentPosition.Z,
                                          CurrentPosition.RX, CurrentPosition.RY, CurrentPosition.RZ);
            if (!finished) return;

            // The move reached the part: follow it from here (base + X̂·(travel − ref) is exactly
            // this tick's output). A frozen move just stops.
            _trkMove = false;
            if (_trkSession && !_trkFrozen)
            {
                _trkFollowing = true;
                _trkBase      = new Vector6(TargetPosition.X, TargetPosition.Y, TargetPosition.Z,
                                            TargetPosition.RX, TargetPosition.RY, TargetPosition.RZ);
                _trkRef       = _trkMoveRef;
            }
            _trkFrozen = false;
        }

        private static double SmoothStep(double s)
        {
            s = Math.Clamp(s, 0, 1);
            return s * s * (3 - 2 * s);
        }

        /// <summary>Hold on the part: no move runs, the robot rides along with the belt.</summary>
        private void RunTrackingHold()
        {
            double d = _trkTravel - _trkRef;
            CurrentPosition = new Vector6(_trkBase.X + _trkDirX * d, _trkBase.Y + _trkDirY * d, _trkBase.Z,
                                          _trkBase.RX, _trkBase.RY, _trkBase.RZ);
            CheckTrackedReach();
            if (!_trkFollowing) return; // the reach check stopped it
            CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
            UpdateJointTargets();
            if (_lastClampViolated) CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
        }

        /// <summary>
        /// Every <see cref="TrackReachCheckPeriodS"/>: can the robot still reach where the belt takes
        /// it a little ahead? If not, tracking stops with the catchable <c>outOfReach</c> before a
        /// joint hits its limit.
        /// </summary>
        private void CheckTrackedReach()
        {
            long now = Conveyors.Now;
            if (now < _trkNextReachCheck) return;
            _trkNextReachCheck = now + (long)(TrackReachCheckPeriodS * TicksPerSecond);
            double ahead = _trkSpeed * TrackReachLookaheadS;
            var probe = new Vector6(CurrentPosition.X + _trkDirX * ahead, CurrentPosition.Y + _trkDirY * ahead, CurrentPosition.Z,
                                    CurrentPosition.RX, CurrentPosition.RY, CurrentPosition.RZ);
            var r = CheckReach(probe, CurrentTool);
            if (r.Ok) return;
            Conveyors.RaiseFault(Programs.ErrorCodes.OutOfReach,
                $"The tracked part is carrying the robot out of reach at ({probe.X:0.#}, {probe.Y:0.#}, {probe.Z:0.#}) — {r.Reason}");
            EndTrackingOnMotionThread();
        }

        /// <summary>
        /// MoveL with a tracked target (see <see cref="RobotCommand.TrackTravel"/>): sets up the
        /// profile start and the offset mode. Returns the start pose for the profiler.
        /// </summary>
        private Vector6 BeginTrackedLinear(double? trackTravel, double lineSpeed)
        {
            TrackingSample();
            _trkMove = false;
            _trkFrozen = false;
            if (trackTravel is not { } r || !_trkSession)
            {
                StopFollowing();
                return CurrentPosition;
            }
            _trkMove       = true;
            _trkMoveRef    = r;
            _trkOffX = _trkOffY = 0;
            if (_trkFollowing)
            {
                // Already synchronised: plan from the nominal start (this pose without the belt
                // travel since the target was resolved), offset at full weight from the first tick.
                _trkMoveRamp  = false;
                _trkFollowing = false;
                double d = _trkTravel - r;
                return new Vector6(CurrentPosition.X - _trkDirX * d, CurrentPosition.Y - _trkDirY * d, CurrentPosition.Z,
                                   CurrentPosition.RX, CurrentPosition.RY, CurrentPosition.RZ);
            }
            _trkMoveRamp = true;
            return CurrentPosition;
        }

        /// <summary>
        /// MoveJ with a tracked target: aim at where the part will be when the move ends (two
        /// passes of duration × belt speed), plan it untracked. The next MoveL synchronises.
        /// Returns the joint profiler, or null when the intercept is out of reach (tracking stopped).
        /// </summary>
        private Vector6MotionProfiler? PlanTrackedJoint(Vector6 target, double trackTravel,
            double speed, double accel, double decel, MotionProfileSettings profile)
        {
            TrackingSample();
            StopFollowing();
            _trkMove = false;
            if (!_trkSession)
            {
                TargetJoints = _kinematics.InverseKinematics(target, CurrentTool);
                return new(CurrentJointTargets, TargetJoints, speed, accel, decel, profile);
            }
            double now = _trkTravel - trackTravel; // belt moved since the target was resolved
            double lead = 0;
            Vector6MotionProfiler? prof = null;
            Vector6 aim = target;
            for (int pass = 0; pass < 3; pass++)
            {
                double d = now + lead;
                aim = new Vector6(target.X + _trkDirX * d, target.Y + _trkDirY * d, target.Z, target.RX, target.RY, target.RZ);
                TargetJoints = _kinematics.InverseKinematics(aim, CurrentTool);
                prof = new(CurrentJointTargets, TargetJoints, speed, accel, decel, profile);
                lead = _trkSpeed * prof.Duration;
            }
            var reach = CheckReach(aim, CurrentTool);
            if (!reach.Ok)
            {
                Conveyors.RaiseFault(Programs.ErrorCodes.OutOfReach,
                    $"The predicted pick point ({aim.X:0.#}, {aim.Y:0.#}, {aim.Z:0.#}) is out of reach — {reach.Reason}");
                EndTrackingOnMotionThread();
                return null;
            }
            TargetPosition = aim;
            return prof;
        }

        /// <summary>Hard stop / joint fault: the robot stops dead; the session (if any) stays, so the
        /// next tracked move blends back onto the part.</summary>
        private void ClearTrackingMotion()
        {
            _trkFollowing = false;
            _trkMove      = false;
            _trkFrozen    = false;
            _trkHolding   = false;
        }

        /// <summary>A stop condition ended a tracked move: keep following the part from where the
        /// robot is now (a probe that touched the part stays on it).</summary>
        private void StopTrackedMoveInPlace()
        {
            if (_trkMove && _trkSession && !_trkFrozen)
            {
                _trkFollowing = true;
                _trkBase      = new Vector6(CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z,
                                            CurrentPosition.RX, CurrentPosition.RY, CurrentPosition.RZ);
                _trkRef       = _trkTravel;
            }
            _trkMove   = false;
            _trkFrozen = false;
        }
    }
}
