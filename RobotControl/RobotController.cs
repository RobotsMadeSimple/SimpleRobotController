using Controller.RobotControl.MotionProfilers;
using Controller.RobotControl.Nano;
using Controller.RobotControl.Robots.TBot;
using System.Diagnostics;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl
{
    internal class RobotController
    {
        public PointRepository       pointRepo       = new();
        public ToolRepository        toolRepo        = new();
        public BuiltProgramRepository builtProgramRepo = new();
        public STB4100 stb = new();
        private RobotIdentity _identity = new();
        public ScalarMotionProfiler mp = new();
        public TBotKinematics TBot = new();
        private readonly ProgramCycleManager programManager = new();
        private ProgramExecutor? programExecutor;

        // Shared deserialisation options — handles string enums and camelCase from the client
        private static readonly JsonSerializerOptions _jsonOptions = new()
        {
            Converters = { new JsonStringEnumConverter() },
            PropertyNameCaseInsensitive = true
        };

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

        // Active tool name — "" means no tool (origin Vector6)
        private string activeTool = "";

        // If the Robot was homed from startup
        private bool homed = false;
        private bool startHoming = false;
        private String homingState = "WaitingForStart";
        private double homedJointDeg = -17; // J1 when homed is at 0
        private double verticalHomed = 445; // Z Height when homed
        private double horizontalHomed = 413; // Horizontal distance when homed

        private JoggingMotionProfiler joggingMotionProfiler = new();
        private JoggingMotionProfiler jointJoggingProfiler = new();
        private ToolJoggingMotionProfiler toolJoggingMotionProfiler = new();

        private bool IsJogging => !joggingMotionProfiler.IsFinished;
        private bool IsJointJogging => !jointJoggingProfiler.IsFinished;
        private bool IsToolJogging => !toolJoggingMotionProfiler.IsFinished;

        public List<RobotCommand> QueuedCommands = new();

        // ── Nano IO ───────────────────────────────────────────────────────────
        public NanoManager NanoManager { get; private set; } = null!;
        private long _lastStatusLightMs = 0;

        public RobotController()
        {
            NanoManager = new NanoManager("nano_config.json");
            NanoManager.Start();

            stb.Start();

            stb.Motor3.InvertDirection = true;
            stb.Motor4.InvertDirection = true;

            programExecutor = new ProgramExecutor(this, programManager, pointRepo, toolRepo);

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

                // Step through any active built program
                programExecutor?.Update();

                // Run the robot motion control
                RunMotion();

                // Execute Homing
                RunHoming();

                // Let the stepper motor drive towards the new targets
                stb.moving = IsMoving;

                // Update the status-light neopixel strip at ~2 Hz
                long nowMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                if (nowMs - _lastStatusLightMs >= 500)
                {
                    _lastStatusLightMs = nowMs;
                    UpdateStatusLight();
                }
            }
        }

        /// <summary>
        /// Sets all pixels on the first configured Neopixel strip to a colour that
        /// reflects the current robot state.  Colours:
        ///   Purple  — Nano not connected
        ///   Red     — Motor driver not connected
        ///   Yellow  — Homing in progress
        ///   Orange  — Not yet homed
        ///   Blue    — Moving
        ///   Green   — Idle and ready
        /// </summary>
        private void UpdateStatusLight()
        {
            var neoResult = NanoManager.FindFirstNeopixel();
            if (neoResult == null) return;

            var (device, neoPin) = neoResult.Value;

            NeoPixelColor color;

            if (!device.Connected)
                color = NeoPixelColor.Purple;
            else if (!stb.connected)
                color = NeoPixelColor.Red;
            else if (startHoming || homingState != "WaitingForStart")
                color = NeoPixelColor.Yellow;
            else if (!homed)
                color = NeoPixelColor.Orange;
            else if (IsMoving)
                color = NeoPixelColor.Blue;
            else
                color = NeoPixelColor.Green;

            var colors = new NeoPixelColor[neoPin.PixelCount];
            for (int i = 0; i < colors.Length; i++)
                colors[i] = color;

            NanoManager.SetNeoPixel(device.Id, neoPin.Pin, colors);
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

        public void SetIdentity(RobotIdentity identity)
        {
            _identity = identity;
        }

        public Task<object> AddCommand(CommandMessage command)
        {
            object? payload = null;

            switch (command.Command)
            {
                case "GetRobotInfo":
                    payload = new
                    {
                        robotName    = _identity.RobotName,
                        robotType    = _identity.RobotType,
                        serialNumber = _identity.SerialNumber,
                    };
                    break;

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

                            // STB digital inputs
                            input1 = stb.Input1,
                            input2 = stb.Input2,
                            input3 = stb.Input3,
                            input4 = stb.Input4,

                            // STB digital outputs
                            output1 = stb.Output1,
                            output2 = stb.Output2,
                            output3 = stb.Output3,
                            output4 = stb.Output4,

                            // Program cycle — summary only (no logs / images)
                            programs = programManager.GetProgramsSummary(),

                            // Tool repository
                            lastToolUpdate = toolRepo.LastUpdatedUnixMs,
                            activeTool     = this.activeTool,

                            // Built program repository
                            lastBuiltProgramUpdate = builtProgramRepo.LastUpdatedUnixMs,
                        };
                        break;
                    }

                // ── Program cycle ─────────────────────────────────────────

                case "SetAvailablePrograms":
                    {
                        var p = LoadParams<SetAvailableProgramsParams>(command);
                        programManager.SetAvailablePrograms(p.Programs);
                    }
                    break;

                case "SetProgramStatus":
                    {
                        var update = LoadParams<ProgramCycleUpdate>(command);
                        programManager.ApplyStatusUpdate(update);
                    }
                    break;

                case "GetProgramImages":
                    {
                        // Merge live in-memory images (Python/external) with persisted images
                        // for built programs so idle built programs still show their image.
                        var merged = programManager.GetAllImages();
                        foreach (var kv in builtProgramRepo.GetAllImages())
                            if (kv.Value != null) merged[kv.Key] = kv.Value;
                        payload = new { images = merged };
                    }
                    break;

                case "GetProgramLogs":
                    {
                        var p = LoadParams<GetProgramLogsParams>(command);
                        var logs = programManager.GetProgramLogs(p.ProgramName, p.Start, p.End);
                        payload = new
                        {
                            programName = p.ProgramName,
                            totalCount  = programManager.GetLogCount(p.ProgramName),
                            start       = p.Start ?? 0,
                            logs
                        };
                    }
                    break;

                case "StartProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                            programExecutor?.Start(built);
                        else
                            programManager.SetFlag(p.ProgramName, "Start");
                    }
                    break;

                case "StopProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                            programExecutor?.Stop();
                        else
                            programManager.SetFlag(p.ProgramName, "Stop");
                    }
                    break;

                case "ResetProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                        {
                            programExecutor?.Reset();
                            programManager.ResetToReady(p.ProgramName,
                                ProgramExecutor.CountSteps(built.Steps));
                        }
                        else
                        {
                            programManager.SetFlag(p.ProgramName, "Reset");
                        }
                    }
                    break;

                case "AbortProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                        {
                            // Abort for built programs = full reset to Ready (program persists in the list)
                            programExecutor?.Reset();
                            programManager.ResetToReady(p.ProgramName,
                                ProgramExecutor.CountSteps(built.Steps));
                        }
                        else
                        {
                            programManager.SetFlag(p.ProgramName, "Abort");
                        }
                    }
                    break;

                case "ClearProgramActions":
                    {
                        var p = LoadParams<ProgramActionParams>(command);
                        programManager.ClearActions(p.ProgramName);
                    }
                    break;

                // ── Tool repository ───────────────────────────────────────

                case "GetTools":
                    payload = new { tools = toolRepo.toolsJson };
                    break;

                case "CreateTool":
                    {
                        var ep = LoadParams<EditToolParams>(command);
                        var v  = new Vector6(ep.X ?? 0, ep.Y ?? 0, ep.Z ?? 0,
                                             ep.RX ?? 0, ep.RY ?? 0, ep.RZ ?? 0);
                        var tool = toolRepo.SaveTool(ep.Name, v);
                        if (!string.IsNullOrEmpty(ep.Description))
                        {
                            toolRepo.EditTool(ep.Name, new()
                            {
                                ["Description"] = ep.Description
                            });
                        }
                    }
                    break;

                case "EditTool":
                    {
                        var ep     = LoadParams<EditToolParams>(command);
                        var values = new Dictionary<string, object?>();
                        if (ep.NewName      != null) values["Name"]        = ep.NewName;
                        if (ep.Description  != null) values["Description"] = ep.Description;
                        if (ep.X.HasValue)           values["X"]           = ep.X.Value;
                        if (ep.Y.HasValue)           values["Y"]           = ep.Y.Value;
                        if (ep.Z.HasValue)           values["Z"]           = ep.Z.Value;
                        if (ep.RX.HasValue)          values["RX"]          = ep.RX.Value;
                        if (ep.RY.HasValue)          values["RY"]          = ep.RY.Value;
                        if (ep.RZ.HasValue)          values["RZ"]          = ep.RZ.Value;
                        toolRepo.EditTool(ep.Name, values);

                        // Keep activeTool name in sync after a rename
                        if (ep.NewName != null && activeTool == ep.Name)
                            activeTool = ep.NewName;
                    }
                    break;

                case "DeleteTool":
                    {
                        var tp = LoadParams<ToolNameParams>(command);
                        toolRepo.DeleteTool(tp.Name);
                        // Clear active tool if the deleted one was active
                        if (activeTool == tp.Name)
                        {
                            activeTool          = "";
                            CurrentTool         = Vector6.Zero;
                            CurrentPosition     = TBot.TcpPosition(CurrentTool);
                            CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);
                        }
                    }
                    break;

                // ── Built program repository ──────────────────────────────

                case "SaveBuiltProgram":
                    {
                        var p = LoadParams<SaveBuiltProgramParams>(command);
                        builtProgramRepo.Save(new BuiltProgram
                        {
                            Name        = p.Name,
                            Description = p.Description,
                            Steps       = p.Steps,
                        });
                    }
                    break;

                case "DeleteBuiltProgram":
                    {
                        var p = LoadParams<BuiltProgramNameParams>(command);
                        builtProgramRepo.Delete(p.Name);
                        programManager.RemoveProgram(p.Name);
                    }
                    break;

                case "SaveBuiltProgramImage":
                    {
                        var p = LoadParams<SaveBuiltProgramImageParams>(command);
                        var bytes = Convert.FromBase64String(p.Image);
                        builtProgramRepo.SaveImage(p.Name, bytes);
                    }
                    break;

                case "GetBuiltPrograms":
                    {
                        var list = builtProgramRepo.GetAll();
                        var json = System.Text.Json.JsonSerializer.Serialize(list, new System.Text.Json.JsonSerializerOptions
                        {
                            Converters = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
                            PropertyNamingPolicy = System.Text.Json.JsonNamingPolicy.CamelCase,
                        });
                        payload = new { programs = json };
                    }
                    break;

                case "ExecuteBuiltProgram":
                    {
                        var p    = LoadParams<BuiltProgramNameParams>(command);
                        var prog = builtProgramRepo.Get(p.Name);
                        if (prog != null)
                        {
                            // Register image in ProgramCycleManager so the monitor shows it while running
                            var imgBytes = builtProgramRepo.GetImage(p.Name);
                            programExecutor?.Start(prog, imgBytes != null ? Convert.ToBase64String(imgBytes) : null);
                        }
                    }
                    break;

                case "StopBuiltProgram":
                    programExecutor?.Stop();
                    break;

                case "SetActiveTool":
                    {
                        var tp = LoadParams<ToolNameParams>(command);
                        if (string.IsNullOrEmpty(tp.Name) || tp.Name == "None")
                        {
                            activeTool  = "";
                            CurrentTool = Vector6.Zero;
                        }
                        else
                        {
                            var tool = toolRepo.Get(tp.Name);
                            if (tool != null)
                            {
                                activeTool  = tp.Name;
                                CurrentTool = new Vector6(tool.X, tool.Y, tool.Z,
                                                          tool.RX, tool.RY, tool.RZ);
                            }
                        }
                        // Recalculate position with new tool offset
                        CurrentPosition     = TBot.TcpPosition(CurrentTool);
                        CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    }
                    break;

                // ── STB4100 IO ────────────────────────────────────────────────

                case "SetSTBOutput":
                    {
                        var p = LoadParams<SetNanoOutputParams>(command); // reuse same params shape
                        stb.SetOutput(p.Pin, p.Value);
                    }
                    break;

                // ── Nano IO ───────────────────────────────────────────────────

                case "GetIO":
                    {
                        var states = NanoManager.GetAllStates();
                        var json   = JsonSerializer.Serialize(states, new JsonSerializerOptions
                        {
                            Converters           = { new JsonStringEnumConverter() },
                            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                        });
                        payload = new { nanos = json };
                    }
                    break;

                case "SetNanoOutput":
                    {
                        var p = LoadParams<SetNanoOutputParams>(command);
                        NanoManager.SetOutput(p.NanoId, p.Pin, p.Value);
                    }
                    break;

                case "SetNeoPixel":
                    {
                        var p = LoadParams<SetNeoPixelParams>(command);
                        var colors = p.Colors
                            .Select(c => new NeoPixelColor(c.R, c.G, c.B))
                            .ToArray();
                        NanoManager.SetNeoPixel(p.NanoId, p.Pin, colors);
                    }
                    break;

                case "RenameNanoPin":
                    {
                        var p = LoadParams<RenameNanoPinParams>(command);
                        NanoManager.RenamePin(p.NanoId, p.Pin, p.Name);
                    }
                    break;

                case "ConfigureNanoPin":
                    {
                        var p    = LoadParams<ConfigureNanoPinParams>(command);
                        var type = p.Type switch
                        {
                            "Output"       => Nano.PinType.Output,
                            "Neopixel"     => Nano.PinType.Neopixel,
                            "Unconfigured" => Nano.PinType.Unconfigured,
                            _              => Nano.PinType.Input,
                        };
                        NanoManager.SetPinType(p.NanoId, p.Pin, type, p.PixelCount);
                    }
                    break;

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

            // Apply any status update that was attached to this command at send-time
            if (Command.StatusUpdate != null)
                programManager.ApplyStatusUpdate(Command.StatusUpdate);

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
                    this.CurrentTool    = Command.Vector6;
                    CurrentPosition     = TBot.TcpPosition(CurrentTool);
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);
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
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);

                    // Have the robot update its joints
                    TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

                    // Drive the motors to the target
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomeJ1";
                    break;

                case "HomeJ1":
                    Vector6 J1JogDirection = new(-1);
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
                    CurrentJointTargets = TBotKinematics.InverseKinematics(CurrentPosition, CurrentTool);

                    // Have the robot update its joints
                    TBot.UpdateJointTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);

                    // Set the motors to the target
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomingComplete";
                    break;

                case "HomingComplete":
                    startHoming = false;
                    homingState = "WaitingForStart";
                    homed = true;
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

            return msg.Params.Value.Deserialize<T>(_jsonOptions)!;
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
