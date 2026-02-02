using Controller.RobotControl.Robots.TBot;
using Controller.RobotControl.MotionProfilers;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.Json;

namespace Controller.RobotControl
{
    internal class RobotController
    {
        public STB4100 stb = new();
        public ScalarMotionProfiler mp = new();
        public StepperMotor motor1;
        public StepperMotor motor2;
        public StepperMotor motor3;
        public StepperMotor motor4;
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
        public bool IsMoving => linearMotionProfiler is not null || jointMotionProfiler is not null || IsJogging;
        // X is away from flange, Y is towards the inside of the robot, Z is Vertical
        public Vector6 CurrentTool = new Vector6(0, 50, 50);
        // Current Pose Of the Joints
        private Vector6 CurrentJointTargets = new();


        private JoggingMotionProfiler joggingMotionProfiler = new();

        private bool IsJogging => !joggingMotionProfiler.IsFinished;

        public List<RobotCommand> QueuedCommands = new();

        public RobotController()
        {
            // ---- Robot startup ----
            motor1 = stb.AddMotor("1", 1600, 1, 1, 0);
            motor2 = stb.AddMotor("2", 1600, 1, 2, 0);
            motor3 = stb.AddMotor("3", 1600, 1, 3, 0);
            motor4 = stb.AddMotor("4", 1600, 1, 4, 0);

            stb.Reset();
            //stb.Start();

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

                // Let the stepper motor drive towards the new targets
                stb.Loop(linearMotionProfiler is not null || jointMotionProfiler is not null);
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
                    CurrentJointTargets = TargetJoints;

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
        }
        public void UpdateJointTargets()
        {
            // Have the robot update its joints
            TBot.UpdateJointTargets(CurrentJointTargets, out double m1Deg, out double m2Deg, out double m3Deg);

            // Drive the motors to the target
            motor1.SetTarget(m1Deg);
            motor2.SetTarget(m2Deg);
            motor3.SetTarget(m3Deg);
        }

        public Task<object> AddCommand(CommandMessage command)
        {
            object? payload = null;

            switch (command.Command)
            {
                case "GetStatus":
                    {
                        Vector6 pose = TBot.GetVisualRobotPose(CurrentPosition, CurrentTool);

                        payload = new
                        {
                            moving = IsMoving,

                            x = CurrentPosition.X,
                            y = CurrentPosition.Y,
                            z = CurrentPosition.Z,
                            rx = CurrentPosition.RX,
                            ry = CurrentPosition.RY,
                            rz = CurrentPosition.RZ,

                            targetX = this.TargetPosition.X,
                            targetY = this.TargetPosition.Y,
                            targetZ = this.TargetPosition.Z,
                            targetRx = this.TargetPosition.RX,
                            targetRy = this.TargetPosition.RY,
                            targetRz = this.TargetPosition.RZ,

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
                            decelJ = DecelJ
                        };
                        break;
                    }

                default:
                    RobotCommand NewCommand = LoadParams<RobotCommand>(command);
                    NewCommand.CommandType = command.Command;
                    if (NewCommand is null)
                    {
                        Console.WriteLine($"Adding null command for some reason? {command.Command}");
                        break;
                    }
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
            {
                return;
            }

            double Speed;
            double Accel;
            double Decel;

            switch (Command.CommandType)
            {
                case "MoveL":
                    // If already moving, wait for the movement to complete
                    if (IsMoving)
                        return;

                    Speed = Command.Speed ??= this.SpeedS;
                    Accel = Command.Accel ??= this.AccelS;
                    Decel = Command.Decel ??= this.DecelS;

                    // Copy the Command position to the TargetPosition
                    this.TargetPosition = Command.Vector6;

                    // Generate a new linear motion profiler for this move
                    linearMotionProfiler = new(CurrentPosition, this.TargetPosition, Speed, Accel, Decel);

                    break;

                case "MoveJ":
                    // If already moving, wait for the movement to complete
                    if (IsMoving)
                        return;
                    
                    // Copy the Command Position to the Target Position
                    this.TargetPosition = Command.Vector6;

                    // Calculate the joint positions for the target position and the current tooling
                    this.TargetJoints = TBotKinematics.InverseKinematics(this.TargetPosition, this.CurrentTool);

                    // Generate a joint motion profile using the current and target joint positions
                    jointMotionProfiler = new(CurrentJointTargets, this.TargetJoints, 100, 100, 100);

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
                    Vector6 jogDirection = Command.Vector6;

                    Speed = Command.Speed ??= this.SpeedS;
                    Accel = Command.Accel ??= this.AccelS;
                    Decel = Command.Decel ??= this.DecelS;

                    joggingMotionProfiler.Jog(jogDirection, Speed, Accel, Decel);
                    break;

                default:
                    break;
            }

            // The command was successful, destroy it
            QueuedCommands.Remove(Command);
        }

        public static T LoadParams<T>(CommandMessage msg)
        {
            if (msg.Params == null)
                throw new InvalidOperationException("Command has no params");

            return msg.Params.Value.Deserialize<T>()!;
        }
        
    }
}
