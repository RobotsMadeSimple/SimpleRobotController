using System;
using System.Diagnostics;
using System.Numerics;

namespace Controller.RobotControl.MotionProfilers
{
    public class ToolJoggingMotionProfiler
    {
        private double _speed = 0;
        private readonly Stopwatch _sw = new();
        private bool _started;

        private Stopwatch autoResetSw = new();
        private double defaultResetTime = 0.2;
        private double resetTime = 0.6;

        public bool IsFinished { get; private set; } = true;
        public double StopEpsilon { get; set; } = 1e-6;

        private Vector6 jogAxis = new();
        private double speed;
        private double accel;
        private double decel;

        private Vector6 _capturedDir = Vector6.Zero;
        private bool _hasCapturedDir = false;
        private Vector6 _lockedJogAxis = Vector6.Zero;
        private bool _stopRequested = false;

        public Vector6 Update(Vector6 currentPosition)
        {
            bool enabled = !IsFinished && !_stopRequested && autoResetSw.Elapsed.TotalSeconds < resetTime;

            double dt = GetDtSec();
            if (dt <= 0) return currentPosition;

            double maxSpeed = Math.Abs(speed);

            if (!_hasCapturedDir && enabled)
            {
                // Translate linear jog direction from tool frame to world frame
                Vector3 localLinear = new((float)jogAxis.X, (float)jogAxis.Y, (float)jogAxis.Z);
                if (localLinear.LengthSquared() > 0)
                    localLinear = Vector3.Normalize(localLinear);

                Vector3 worldLinear = TransformToolToWorld(localLinear, currentPosition);
                if (worldLinear.LengthSquared() > 0)
                    worldLinear = Vector3.Normalize(worldLinear);

                // Rotation direction stays in tool frame
                Vector3 rotDir = new((float)jogAxis.RX, (float)jogAxis.RY, (float)jogAxis.RZ);
                float rotMag = rotDir.Length();
                if (rotMag > 0)
                    rotDir /= rotMag;

                // Normalize combined 6D direction vector so speed is uniform across all axes
                double combinedMag = Math.Sqrt(
                    worldLinear.X * worldLinear.X + worldLinear.Y * worldLinear.Y + worldLinear.Z * worldLinear.Z +
                    rotDir.X * rotDir.X + rotDir.Y * rotDir.Y + rotDir.Z * rotDir.Z
                );

                _capturedDir = combinedMag > 0
                    ? new Vector6(
                        worldLinear.X / combinedMag, worldLinear.Y / combinedMag, worldLinear.Z / combinedMag,
                        rotDir.X / combinedMag, rotDir.Y / combinedMag, rotDir.Z / combinedMag
                      )
                    : Vector6.Zero;

                _hasCapturedDir = true;
            }

            double targetSpeed = enabled ? maxSpeed : 0;
            double rate = enabled ? Math.Abs(accel) : Math.Abs(decel);
            double maxDv = rate * dt;

            _speed = (maxDv <= 0) ? targetSpeed : MoveTowards(_speed, targetSpeed, maxDv);

            if (!enabled && Math.Abs(_speed) <= StopEpsilon)
            {
                _speed = 0;
                IsFinished = true;
                _started = false;
                _hasCapturedDir = false;
                _lockedJogAxis = Vector6.Zero;
                _stopRequested = false;
                return currentPosition;
            }

            IsFinished = false;

            return new Vector6(
                currentPosition.X  + _capturedDir.X  * _speed * dt,
                currentPosition.Y  + _capturedDir.Y  * _speed * dt,
                currentPosition.Z  + _capturedDir.Z  * _speed * dt,
                currentPosition.RX + _capturedDir.RX * _speed * dt,
                currentPosition.RY + _capturedDir.RY * _speed * dt,
                currentPosition.RZ + _capturedDir.RZ * _speed * dt
            );
        }

        public void Jog(Vector6 jogAxis, double speed, double accel, double decel, double? resetTime = null)
        {
            this.resetTime = resetTime ?? defaultResetTime;

            Vector6 normalizedAxis = Normalize6(jogAxis);

            if (!IsFinished)
            {
                // Only extend the jog if the direction matches the locked axis
                if (!DirectionsMatch(normalizedAxis, _lockedJogAxis))
                    return;
            }
            else
            {
                // Starting fresh — lock the new direction
                _lockedJogAxis = normalizedAxis;
                _started = false;
                _speed = 0;
                _hasCapturedDir = false;
                IsFinished = false;
            }

            this.jogAxis = jogAxis;
            this.speed = speed;
            this.accel = accel;
            this.decel = decel;
            _stopRequested = false;

            autoResetSw.Restart();
        }

        public void StopJog() => _stopRequested = true;

        public void ForceStop()
        {
            _speed = 0;
            IsFinished = true;
            _started = false;
            _hasCapturedDir = false;
            _lockedJogAxis = Vector6.Zero;
        }

        private static Vector6 Normalize6(Vector6 v)
        {
            double mag = Math.Sqrt(
                v.X * v.X + v.Y * v.Y + v.Z * v.Z +
                v.RX * v.RX + v.RY * v.RY + v.RZ * v.RZ
            );
            return mag > 0
                ? new Vector6(v.X / mag, v.Y / mag, v.Z / mag, v.RX / mag, v.RY / mag, v.RZ / mag)
                : Vector6.Zero;
        }

        private static bool DirectionsMatch(Vector6 a, Vector6 b, double threshold = 0.9999)
        {
            double dot = a.X * b.X + a.Y * b.Y + a.Z * b.Z +
                         a.RX * b.RX + a.RY * b.RY + a.RZ * b.RZ;
            return dot >= threshold;
        }

        private Vector3 TransformToolToWorld(Vector3 local, Vector6 pose)
        {
            float rx = DegToRad((float)pose.RX);
            float ry = DegToRad((float)pose.RY);
            float rz = DegToRad((float)pose.RZ);

            Matrix4x4 rot =
                Matrix4x4.CreateRotationZ(rz) *
                Matrix4x4.CreateRotationY(ry) *
                Matrix4x4.CreateRotationX(rx);

            return Vector3.Transform(local, rot);
        }

        private static float DegToRad(float deg) => deg * (float)Math.PI / 180f;

        private double GetDtSec()
        {
            if (!_started)
            {
                _started = true;
                _sw.Restart();
                return 0.0;
            }

            double dt = _sw.Elapsed.TotalSeconds;
            _sw.Restart();

            if (dt > 0.25) dt = 0.25;
            return dt;
        }

        private static double MoveTowards(double current, double target, double maxDelta)
        {
            double delta = target - current;
            if (Math.Abs(delta) <= maxDelta) return target;
            return current + Math.Sign(delta) * maxDelta;
        }

    }
}