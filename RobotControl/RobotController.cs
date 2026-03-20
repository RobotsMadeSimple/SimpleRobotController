using Controller.RobotControl.MotionProfilers;
using Controller.RobotControl.Robots.TBot;
using System.Diagnostics;
using System.Numerics;
using System.Text.Json;

namespace Controller.RobotControl
{
    internal class RobotController
    {
        public PointRepository pointRepo = new();
        public STB4100 stb = new();
        public ScalarMotionProfiler mp = new();
        public TBotKinematics TBot = new();

        // Hard-stop flag — set from any thread, consumed exclusively on the control loop thread
        private volatile bool _hardStopRequested;

        // Joint motion profiler
        private Vector6MotionProfiler? jointMotionProfiler;
        private Vector6 TargetJoints = new();
        private double SpeedJ = 100;
        private double AccelJ = 100;
        private double DecelJ = 100;

        // Linear Positioning
        private Vector6MotionProfiler? linearMotionProfiler;
        private Vector6 TargetPosition = new();
        private double SpeedS = 100; // Linear Max Velocity
        private double AccelS = 100; // Linear Acceleration
        private double DecelS = 100; // Linear Deceleration

        // Current Status of Robot
        private Vector6 CurrentPosition = new();  // Actual position of the robot
        public bool IsMoving => linearMotionProfiler is not null || jointMotionProfiler is not null || IsJogging || IsJointJogging || IsToolJogging;
        // X is away from flange, Y is towards the inside of the robot, Z is Vertical
        public Vector6 CurrentTool = new(0, 0, 0);
        // Current Pose Of the Joints
        private Vector6 CurrentJointTargets = new();

        // If the Robot was homed from startup
        private bool homed = false;
        private bool startHoming = false;
        private String homingState = "WaitingForStart";
        private double homedJointDeg = -11; // J1 when homed is at 0
        private double verticalHomed = 445; // Z Height when homed
        private double horizontalHomed = 413; // Horizontal distance when homed

        private JoggingMotionProfiler joggingMotionProfiler = new();
        private JoggingMotionProfiler jointJoggingProfiler = new();
        private ToolJoggingMotionProfiler toolJoggingMotionProfiler = new();

        private bool IsJogging => !joggingMotionProfiler.IsFinished;
        private bool IsJointJogging => !jointJoggingProfiler.IsFinished;
        private bool IsToolJogging => !toolJoggingMotionProfiler.IsFinished;

        public List<RobotCommand> QueuedCommands = new();

        public RobotController()
        {
            stb.Start();

            stb.Motor3.InvertDirection = true;
            stb.Motor4.InvertDirection = true;

            new Thread(ControlLoop) { IsBackground = true }.Start();
        }

        private void ControlLoop()
        {
            var sw = Stopwatch.StartNew();
            long nextTick = 0;

            double periodSec = 0.004; // 4ms
            long periodTicks = (long)(periodSec * Stopwatch.Frequency);

            while (true)
            {
                long now = sw.ElapsedTicks;

                if (now >= nextTick)
                {
                    Loop();
                    nextTick += periodTicks;

                }

                // Small spin wait to reduce CPU burn but keep timing tight
                Thread.SpinWait(50);
            }
        }

        public void Loop()
        {
            while (true)
            {
                // Consume hard-stop flag before anything else touches the profilers
                if (_hardStopRequested)
                    ExecuteHardStop();

                // Execute pending robot commands
                RunCommands();

                // Run the robot motion control
                RunMotion();

                // Execute Homing
                RunHoming();

                // Let the stepper motor drive towards the new targets
                stb.moving = IsMoving;
            }
        }

        public void RunMotion()
        {
            if (linearMotionProfiler is not null)
            {
                CurrentPosition = linearMotionProfiler.Update();
                if (linearMotionProfiler.IsFinished)
                {
                    // Set the position to the final postiion
                    CurrentPosition.Copy(TargetPosition);
                    // Destroy the profiler
                    linearMotionProfiler = null;
                }

                // Calculate IK to get the joint targets for the next interpolated linear movement
                CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
            }
            else if (jointMotionProfiler is not null)
            {
                // Always update first (matches linear pattern)
                CurrentJointTargets = jointMotionProfiler.Update();

                if (jointMotionProfiler.IsFinished)
                {
                    // Snap to exact target joints on the same iteration the profiler finishes,
                    // so the motors are commanded to the precise endpoint rather than whatever
                    // floating-point value the profiler's last step returned
                    CurrentJointTargets.Copy(TargetJoints);

                    // Destroy the profiler
                    jointMotionProfiler = null;
                }

                // Update the joint angles with the new calculated ones
                UpdateJointTargets();

                // Recalculate the Cartesian Coordinate position to keep it current
                CurrentPosition = TBot.TcpPosition(CurrentTool);
            }
            else if (IsJogging)
            {
                CurrentPosition = joggingMotionProfiler.Update(CurrentPosition);
                // Calculate IK to get the joint targets for the next Jog movement
                CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
            }
            else if (IsJointJogging)
            {
                // Continue Jogging the joints that are moving
                CurrentJointTargets = jointJoggingProfiler.Update(CurrentJointTargets);

                // Update the joint angles with the new calculated ones
                UpdateJointTargets();

                // Recalculate the Cartesian Coordinate position to keep it current
                CurrentPosition = TBot.TcpPosition(CurrentTool);
            }
            else if (IsToolJogging)
            {
                CurrentPosition = toolJoggingMotionProfiler.Update(CurrentPosition);
                // Calculate IK to get the joint targets for the next Jog movement
                CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
            }
        }
        public void UpdateJointTargets()
        {
            // Have the robot update its joints
            TBot.UpdateJointTargets(CurrentJointTargets, out double m1Deg, out double m2Deg, out double m3Deg, out double m4Deg);

            // Drive the motors to the target
            stb.SetMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
        }

        public Task<object> AddCommand(CommandMessage command)
        {
            object? payload = null;

            switch (command.Command)
            {
                case "Home":
                    startHoming = true;
                    break;

                case "SetHomed":
                    SetAllHomed();
                    break;

                case "Reset":
                    stb.Reset();
                    break;

                case "HardStop":
                    HardStop();
                    break;

                case "StopJog":
                    joggingMotionProfiler.StopJog();
                    jointJoggingProfiler.StopJog();
                    toolJoggingMotionProfiler.StopJog();
                    break;

                case "GetPoints":
                    payload = new
                    {
                        points = pointRepo.pointsJson
                    };
                    break;

                case "TeachPoint":
                    {
                        var tp = LoadParams<TeachPointParams>(command);
                        pointRepo.SavePoint(tp.Name, CurrentPosition);
                    }
                    break;

                case "DeletePoint":
                    {
                        var dp = LoadParams<TeachPointParams>(command);
                        pointRepo.DeletePoint(dp.Name);
                    }
                    break;

                case "EditPoint":
                    {
                        var ep = LoadParams<EditPointParams>(command);
                        var values = new Dictionary<string, object?>();
                        if (ep.NewName != null)   values["Name"] = ep.NewName;
                        if (ep.X.HasValue)        values["X"]    = ep.X.Value;
                        if (ep.Y.HasValue)        values["Y"]    = ep.Y.Value;
                        if (ep.Z.HasValue)        values["Z"]    = ep.Z.Value;
                        if (ep.RX.HasValue)       values["RX"]   = ep.RX.Value;
                        if (ep.RY.HasValue)       values["RY"]   = ep.RY.Value;
                        if (ep.RZ.HasValue)       values["RZ"]   = ep.RZ.Value;
                        pointRepo.EditPoint(ep.Name, values);
                    }
                    break;

                case "GetStatus":
                    {
                        Vector6 pose = TBot.GetVisualRobotPose(CurrentPosition, CurrentTool);

                        payload = new
                        {
                            moving = IsMoving,
                            wasHomed = homed,
                            homingState = this.homingState,
                            isHoming = this.homingState != "WaitingForStart",
                            lastPointUpdate = pointRepo.LastUpdatedUnixMs,
                            driverConnected = stb.connected,

                            x = CurrentPosition.X,
                            y = CurrentPosition.Y,
                            z = CurrentPosition.Z,
                            rx = CurrentPosition.RX,
                            ry = CurrentPosition.RY,
                            rz = CurrentPosition.RZ,

                            targetX = this.TargetPosition.X,
                            targetY = this.TargetPosition.Y,
                            targetZ = this.TargetPosition.Z,
                            targetRX = this.TargetPosition.RX,
                            targetRY = this.TargetPosition.RY,
                            targetRZ = this.TargetPosition.RZ,

                            joint1Angle = this.TBot.CurrentJoint1.JointAngleDeg,
                            joint2X = this.TBot.CurrentJoint2.Cartesian.x,
                            joint2Z = this.TBot.CurrentJoint2.Cartesian.z,

                            poseX = pose.X,
                            poseY = pose.Y,
                            poseZ = pose.Z,
                            poseRX = pose.RX,
                            poseRY = pose.RY,
                            poseRZ = pose.RZ,

                            speedS = SpeedS,
                            accelS = AccelS,
                            decelS = DecelS,

                            speedJ = SpeedJ,
                            accelJ = AccelJ,
                            decelJ = DecelJ,

                            // STB Axis Limits
                            input1 = stb.Input1,
                            input2 = stb.Input2,
                            input3 = stb.Input3,
                            input4 = stb.Input4,
                        };
                        break;
                    }

                default:
                    RobotCommand NewCommand = LoadParams<RobotCommand>(command);
                    NewCommand.CommandType = command.Command;
                    QueuedCommands.Add(NewCommand);
                    break;

            }
            payload ??= new { };

            // instance logic here
            return Task.FromResult((object)payload);
        }
        public void RunCommands()
        {
            if (QueuedCommands.Count == 0)
                return;

            RobotCommand? Command = QueuedCommands[0];
            Vector6? target = null;

            if (Command is null)
                return;

            string CommandType = Command.CommandType ?? "";

            if (IsMoving && !CommandType.Contains("Jog"))
                return;

            switch (CommandType)
            {
                case "MoveL":
                    {
                        target = ResolveVector(Command);
                        MoveL(target, Command.Speed, Command.Accel, Command.Decel, Command.ToolOffsetVector6);
                    }
                    break;

                case "OffsetL":
                    Vector6 NewPosition = CurrentPosition + Command.Vector6;
                    MoveL(NewPosition, Command.Speed, Command.Accel, Command.Decel, Command.ToolOffsetVector6);
                    break;

                case "MoveJ":
                    {
                        target = ResolveVector(Command);
                        MoveJ(target, Command.Speed, Command.Accel, Command.Decel, Command.ToolOffsetVector6);
                    }
                    break;

                case "SetTool":
                    this.CurrentTool = Command.Vector6;
                    break;

                case "SpeedS":
                    this.SpeedS = Command.Speed ??= this.SpeedS;
                    break;

                case "AccelS":
                    this.AccelS = Command.Accel ??= this.AccelS;
                    this.DecelS = Command.Decel ??= this.DecelS;
                    break;

                case "SpeedJ":
                    this.SpeedJ = Command.Speed ?? this.SpeedJ;
                    break;

                case "AccelJ":
                    this.AccelJ = Command.Accel ?? this.AccelJ;
                    this.DecelJ = Command.Decel ?? this.DecelJ;
                    break;

                case "JogL":
                    JogL(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                case "JogJ":
                    JogJ(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                case "JogTool":
                    JogTool(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                default:
                    break;
            }

            // The command was successful, destroy it
            QueuedCommands.Remove(Command);
        }

        public void SetAllHomed()
        {
            double m1Deg, m2Deg, m3Deg, m4Deg;
            // Offset the current joint angle
            TBot.InterpolatedJoint1.JointAngleDeg = homedJointDeg;
            TBot.CurrentJoint1.JointAngleDeg = homedJointDeg;
            TBot.InterpolatedJoint2.Cartesian = (horizontalHomed, TBot.InterpolatedJoint2.Cartesian.z);
            TBot.CurrentJoint2.Cartesian = (horizontalHomed, TBot.InterpolatedJoint2.Cartesian.z);
            TBot.InterpolatedJoint2.Cartesian = (TBot.InterpolatedJoint2.Cartesian.x, verticalHomed);
            TBot.CurrentJoint2.Cartesian = (TBot.CurrentJoint2.Cartesian.x, verticalHomed);

            // Recalculate the position and joint targets
            CurrentPosition = TBot.TcpPosition(CurrentTool);
            CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);

            // Have the robot update its joints
            TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

            // Drive the motors to the target
            stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
        }
        public void RunHoming()
        {
            double m1Deg, m2Deg, m3Deg, m4Deg;

            switch (homingState)
            {
                case "WaitingForStart":
                    if (startHoming)
                        homingState = "HomeVertical";
                    break;

                case "HomeVertical":
                    jointJoggingProfiler.Jog(new(0, 0, 1), 20, 100, 10000000, 0.001);
                    if (stb.Input2)
                    {
                        ExecuteHardStop();
                        homingState = "WaitVerticalMoveComplete";
                    }
                    break;

                case "WaitVerticalMoveComplete":
                    if (!IsMoving)
                        homingState = "SetVerticalHomed";
                    break;

                case "SetVerticalHomed":
                    // Offset the current joint angle
                    TBot.InterpolatedJoint2.Cartesian = (TBot.InterpolatedJoint2.Cartesian.x, verticalHomed);
                    TBot.CurrentJoint2.Cartesian = (TBot.CurrentJoint2.Cartesian.x, verticalHomed);

                    CurrentPosition = TBot.TcpPosition(CurrentTool);
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, Vector6.Zero);

                    // Have the robot update its joints
                    TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

                    // Drive the motors to the target
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomeHorizontal";
                    break;

                case "HomeHorizontal":
                    jointJoggingProfiler.Jog(new(0, 1), 20, 100, 10000000, 0.001);
                    if (stb.Input3)
                    {
                        ExecuteHardStop();
                        homingState = "WaitHorizontalMoveComplete";
                    }
                    break;

                case "WaitHorizontalMoveComplete":
                    if (!IsMoving)
                    {
                        homingState = "SetHorizontalHomed";
                    }
                    break;

                case "SetHorizontalHomed":
                    // Offset the current joint angle
                    TBot.InterpolatedJoint2.Cartesian = (horizontalHomed, TBot.InterpolatedJoint2.Cartesian.z);
                    TBot.CurrentJoint2.Cartesian = (horizontalHomed, TBot.InterpolatedJoint2.Cartesian.z);

                    CurrentPosition = TBot.TcpPosition(CurrentTool);
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, Vector6.Zero);

                    // Have the robot update its joints
                    TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

                    // Drive the motors to the target
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomeJ1";
                    break;

                case "HomeJ1":
                    Vector6 J1JogDirection = new(1);
                    jointJoggingProfiler.Jog(J1JogDirection, 20, 100, 10000000, 0.001);
                    if (stb.Input1)
                    {
                        ExecuteHardStop();
                        homingState = "WaitJ1MoveComplete";
                    }
                    break;

                case "WaitJ1MoveComplete":
                    if (!IsMoving)
                    {
                        homingState = "SetJ1MotorHomed";
                    }
                    break;

                case "SetJ1MotorHomed":
                    // Offset the current joint angle
                    TBot.InterpolatedJoint1.JointAngleDeg = homedJointDeg;
                    TBot.CurrentJoint1.JointAngleDeg = homedJointDeg;

                    CurrentPosition = TBot.TcpPosition(CurrentTool);
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, Vector6.Zero);

                    // Have the robot update its joints
                    TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

                    // Set the motors to the target
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomingComplete";
                    break;

                case "HomingComplete":
                    startHoming = false;
                    homingState = "WaitingForStart";
                    break;

                default:
                    break;
            }
        }

        /// <summary>
        /// Thread-safe: sets a flag that is consumed at the top of the next control loop iteration.
        /// Never touches the profilers directly from outside the loop thread.
        /// </summary>
        public void HardStop()
        {
            _hardStopRequested = true;
        }

        /// <summary>
        /// Must only be called from the control loop thread.
        /// Clears all motion state immediately and safely.
        /// </summary>
        private void ExecuteHardStop()
        {
            _hardStopRequested = false;
            linearMotionProfiler = null;
            jointMotionProfiler = null;
            joggingMotionProfiler.ForceStop();
            jointJoggingProfiler.ForceStop();
            toolJoggingMotionProfiler.ForceStop();
            QueuedCommands.Clear();
            startHoming = false;
            homingState = "WaitingForStart";
        }

        public void MoveJ(Vector6 TargetPosition, double? Speed, double? Accel, double? Decel, Vector6? ToolOffset)
        {
            if (IsMoving)
                return;

            // Gather the commands motion params if there specified otherwise default to the last set ones
            double jointSpeed = Speed ??= this.SpeedJ;
            double jointAccel = Accel ??= this.AccelJ;
            double jointDecel = Decel ??= this.DecelJ;

            if (ToolOffset is not null)
            {
                this.TargetPosition = ApplyToolOffset(TargetPosition, ToolOffset);
            }
            else
            {
                // Copy the Command Position to the Target Position
                this.TargetPosition = TargetPosition;
            }

            // Calculate the joint positions for the target position and the current tooling
            this.TargetJoints = TBotKinematics.InverseKinematics(this.TargetPosition, this.CurrentTool);

            // Generate a joint motion profile using the current and target joint positions
            jointMotionProfiler = new(CurrentJointTargets, this.TargetJoints, jointSpeed, jointAccel, jointDecel);
        }

        

        public void MoveL(Vector6 TargetPosition, double? Speed, double? Accel, double? Decel, Vector6? ToolOffset)
        {
            if (IsMoving)
                return;

            // Gather the commands motion params if there specified otherwise default to the last set ones
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;

            if (ToolOffset is not null)
            {
                this.TargetPosition = ApplyToolOffset(TargetPosition, ToolOffset);
            }
            else
            {
                // Copy the Command Position to the Target Position
                this.TargetPosition = TargetPosition;
            }

            // Generate a new linear motion profiler for this move
            linearMotionProfiler = new(CurrentPosition, this.TargetPosition, lineSpeed, lineAccel, lineDecel);
        }

        public void JogJ(Vector6 jogJointDirection, double? Speed, double? Accel, double? Decel)
        {
            // Gather the commands motion params if there specified otherwise default to the last set ones
            double jointSpeed = Speed ??= this.SpeedJ;
            double jointAccel = Accel ??= this.AccelJ;
            double jointDecel = Decel ??= this.DecelJ;


            jointJoggingProfiler.Jog(jogJointDirection, jointSpeed, jointAccel, jointDecel);
        }

        public void JogL(Vector6 jogDirection, double? Speed, double? Accel, double? Decel)
        {
            // Gather the commands motion params if there specified otherwise default to the last set ones
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;

            joggingMotionProfiler.Jog(jogDirection, lineSpeed, lineAccel, lineDecel);
        }

        public void JogTool(Vector6 jogDirection, double? Speed, double? Accel, double? Decel)
        {
            // Gather the commands motion params if there specified otherwise default to the last set ones
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;

            toolJoggingMotionProfiler.Jog(jogDirection, lineSpeed, lineAccel, lineDecel);
        }

        /// <summary>
        /// Offsets a pose in its own tool frame by the given offset Vector6.
        /// The linear offset (X/Y/Z) is rotated into world space using the pose's current orientation,
        /// then added to the pose's position. The rotation offset (RX/RY/RZ) is applied after translation.
        /// </summary>
        public static Vector6 ApplyToolOffset(Vector6 pose, Vector6 offset)
        {
            float rx = (float)(pose.RX * Math.PI / 180.0);
            float ry = (float)(pose.RY * Math.PI / 180.0);
            float rz = (float)(pose.RZ * Math.PI / 180.0);

            Matrix4x4 rot =
                Matrix4x4.CreateRotationZ(rz) *
                Matrix4x4.CreateRotationY(ry) *
                Matrix4x4.CreateRotationX(rx);

            Vector3 worldOffset = Vector3.Transform(
                new Vector3((float)offset.X, (float)offset.Y, (float)offset.Z),
                rot
            );

            return new Vector6(
                pose.X  + worldOffset.X,
                pose.Y  + worldOffset.Y,
                pose.Z  + worldOffset.Z,
                pose.RX + offset.RX,
                pose.RY + offset.RY,
                pose.RZ + offset.RZ
            );
        }

        public static T LoadParams<T>(CommandMessage msg)
        {
            if (msg.Params == null)
                throw new InvalidOperationException("Command has no params");

            return msg.Params.Value.Deserialize<T>()!;
        }

        private Vector6 ResolveVector(RobotCommand command)
        {
            if (!string.IsNullOrWhiteSpace(command.Name))
            {
                if (!pointRepo.Points.TryGetValue(command.Name, out var point))
                    throw new InvalidOperationException($"Point '{command.Name}' not found");

                return new Vector6(point.X, point.Y, point.Z, point.RX, point.RY, point.RZ);
            }

            return command.Vector6;
        }
        
    }
}
