using Controller.RobotControl.Robots.TBot;
using Controller.RobotControl.MotionProfilers;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;
using System.Text.Json;
using System.Runtime.InteropServices.Swift;
using System.Data;

namespace Controller.RobotControl
{
    internal class RobotController
    {
        public PointRepository pointRepo = new();
        public STB4100 stb = new();
        public ScalarMotionProfiler mp = new();
        public TBotKinematics TBot = new();

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
        public bool IsMoving => linearMotionProfiler is not null || jointMotionProfiler is not null || IsJogging || IsJointJogging;
        // X is away from flange, Y is towards the inside of the robot, Z is Vertical
        public Vector6 CurrentTool = new(0, 50, 0);
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

        private bool IsJogging => !joggingMotionProfiler.IsFinished;
        private bool IsJointJogging => !jointJoggingProfiler.IsFinished;

        public List<RobotCommand> QueuedCommands = new();

        public RobotController()
        {
            stb.Start();

            stb.Motor3.InvertDirection = true;

            new Thread(Loop) { IsBackground = true }.Start();
        }

        public void Loop()
        {
            while (true)
            {
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
                if (jointMotionProfiler.IsFinished)
                {
                    // Set the joint targets to the final target position
                    CurrentJointTargets.Copy(TargetJoints);

                    // Destroy the profiler
                    jointMotionProfiler = null;
                }
                else
                {
                    // Calculate the new joint angles for the next interpolated joint movement
                    CurrentJointTargets = jointMotionProfiler.Update();
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

                case "GetPoints":
                    payload = new
                    {
                        points = pointRepo.pointsJson
                    };
                    break;

                case "Teach":
                    pointRepo.SavePoint("Name", CurrentPosition);
                    break;

                case "GetStatus":
                    {
                        Vector6 pose = TBot.GetVisualRobotPose(CurrentPosition, CurrentTool);

                        payload = new
                        {
                            moving = IsMoving,
                            wasHomed = homed,
                            homingState = this.homingState,
                            lastPointUpdate = pointRepo.LastUpdatedUnixMs,

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
            
            if (Command is null)
                return;

            string CommandType = Command.CommandType ?? "";

            if (IsMoving && !CommandType.Contains("Jog"))
                return;

            switch (CommandType)
            {
                case "MoveL":
                    MoveL(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                case "OffsetL":
                    Vector6 NewPosition = CurrentPosition + Command.Vector6;
                    MoveL(NewPosition, Command.Speed, Command.Accel, Command.Decel);
                    break;

                case "MoveJ":
                    MoveJ(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
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
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);

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
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);

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
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);

                    // Have the robot update its joints
                    TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

                    // Drive the motors to the target
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

        public void MoveJ(Vector6 TargetPosition, double? Speed, double? Accel, double? Decel)
        {
            if (IsMoving)
                return;

            // Copy the Command Position to the Target Position
            this.TargetPosition = TargetPosition;

            // Gather the commands motion params if there specified otherwise default to the last set ones
            double jointSpeed = Speed ??= this.SpeedJ;
            double jointAccel = Accel ??= this.AccelJ;
            double jointDecel = Decel ??= this.DecelJ;

            // Calculate the joint positions for the target position and the current tooling
            this.TargetJoints = TBotKinematics.InverseKinematics(this.TargetPosition, this.CurrentTool);

            // Generate a joint motion profile using the current and target joint positions
            jointMotionProfiler = new(CurrentJointTargets, this.TargetJoints, jointSpeed, jointAccel, jointDecel);
        }

        

        public void MoveL(Vector6 TargetPosition, double? Speed, double? Accel, double? Decel)
        {
            if (IsMoving)
                return;

            // Copy the Command Position to the Target Position
            this.TargetPosition = TargetPosition;

            // Gather the commands motion params if there specified otherwise default to the last set ones
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;

            // Copy the Command position to the TargetPosition
            this.TargetPosition = TargetPosition;

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

        public static T LoadParams<T>(CommandMessage msg)
        {
            if (msg.Params == null)
                throw new InvalidOperationException("Command has no params");

            return msg.Params.Value.Deserialize<T>()!;
        }
        
    }
}
