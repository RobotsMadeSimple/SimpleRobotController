# SimpleRobotController — Architecture

A map of the codebase for anyone about to change it. The WebSocket protocol is
documented in `websocket-api.md`, program step types in `program-blocks.md`, and
the STB4100 loop timing constraints in `stb-loop-timing.md`.

## Process shape

One process, three long-lived threads plus device threads:

| Thread | Owner | Tick | Owns |
|---|---|---|---|
| `MotionLoop` | `RobotController` | unthrottled (burns a core on purpose) | profilers, current/target pose, kinematics, homing, joint limits, the motion command queue |
| `ProgramLoop` | `RobotController` | ~1 ms | `ProgramExecutor.Update()`, background executors, status light |
| ASP.NET thread pool | Kestrel | per request | WebSocket commands, HTTP endpoints |
| one per device | `NanoDevice`, `AuxAxisDevice`, `CameraDevice`, `VisionProcessor`, `STB4100` (two) | device-specific | serial/HID/camera I/O |

Rules that everything else depends on:

- **Only the motion thread mutates motion state.** Other threads post work with
  `RobotController.PostToMotionThread(Action)` (drained at the top of each tick)
  or enqueue a `RobotCommand` on `QueuedCommands`. `docs/stb-loop-timing.md`
  explains why the loop must not sleep or gate.
- **The executor is serialised by one lock.** `Start/Stop/Resume/Reset`,
  `Update()` and the monitor readers all hold `_controlLock`; nothing under
  `Execution/` is thread-safe on its own. Never call into the executor from the
  motion thread synchronously (see `LatchFault`, which posts `Stop()` to a task).
- **Motion errors are latched, not thrown.** A move the motion thread cannot
  execute calls `LatchMotionError`; the executor calls `ConsumeMotionError()`
  after every awaited move and fails the step.

## Startup (`Program.cs`)

`Program.Main` parses `--port`/`--data`, sets the data directory as the current
directory (every data file path is relative to it), loads identity and config,
constructs `RobotController`, applies identity/config, then `Start()`s devices and
threads. `Hosting/HttpEndpoints` maps the HTTP routes, `RobotWebSocketServer`
maps `/control`, and `Hosting/MdnsAdvertiser` advertises `_robot._tcp`.

## Folders

| Folder | What lives there |
|---|---|
| `Commands/` | One class per command domain (`PointCommands`, `IoCommands`, …). `CommandDispatcher.Create` registers every WebSocket command name → handler. Adding a command means adding one method and one `Add(...)` line; a duplicate name throws at startup. |
| `Execution/` | Program execution. `ExecutionContext` is what step handlers see; `VariableScope` holds variables; `EvalContext` snapshots variables+IO once per tick; `MoveTargetResolver` is pure target math; `MotionDispatcher` sends moves and handles pause/resume; `ProgressReporter` pushes status. |
| `Execution/Steps/` | `IStepHandler` implementations grouped by domain. A handler returns `Advance` (executor completes the step), `Yield` (come back next tick) or `Finished`. Register new step types in `StepHandlers.Build`. |
| `Models/` | DTOs: program cycle, steps, variables, built programs, command params, grid/stack. All in the root namespace; JSON names are fixed by `[JsonPropertyName]`. |
| `Geometry/`, `Motion/` | `Vector6`, `LocalFrame`, `JointLimiter`, `RobotCommand`, `HomingSequencer` (table-driven; the axis tables are built in `RobotController`). |
| `MotionProfilers/` | Trapezoidal/scalar profilers, jogging, blended continuous paths. Pure math, unit-tested. |
| `Robots/` | `IRobotKinematics` and the ASTRO / CNC4Axis implementations. `Joints/` holds the joint models ASTRO composes. |
| `Persistence/` | `AtomicFile` (temp + rename), `JsonFiles` (load/save, corrupt files quarantined as `*.corrupt-<stamp>`), `JsonListRepository<T>`, the named-vector repositories (points/tools/locals with history) and `BuiltProgramRepository` (one file per program). |
| `Hosting/` | HTTP endpoints, WebSocket server, mDNS. |
| `Vision/` | `VisionProcessor` (thread + loop), `InspectionPlan` (program → ordered run-ready steps), `FrameContext` (cached grey/blur/edges/masks per frame), `ZoneGeometry`, drawing helpers. |
| `Vision/Inspections/` | One `IInspectionStrategy` per inspection type: detection, annotation, debug rendering. Adding a type means one strategy class plus its list in `VisionProgram`. |
| `Nano/`, `AuxAxis/`, `Serial/` | Arduino serial devices on the shared `SerialLineDevice` base (port scan, `ID?` probe, port claim registry, session loop). |
| `Controllers/STB4100/` | The HID motion board: loops (timing-critical), `Stb4100Packets` (pure packet builders, byte-exact tests). |
| `Camera/`, `UsbRelay/` | USB camera capture and the HID relay board. |

## Conventions

- Build with `-c Release` when a controller exe is running from `bin/Debug`
  (it locks the Debug output). CI runs build + tests on Linux and Windows.
- Every data/config file goes through `JsonFiles`/`AtomicFile`; never
  `File.WriteAllText` app data directly.
- Wire format (WebSocket command names, params, response shapes) is shared with
  SimpleRobotApp. Add fields freely; renaming or reshaping requires a matching
  app change.
- Enums serialise as strings (`JsonStringEnumConverter`). Prefer an enum over a
  string field for any "mode"/"kind" value.
- Log with a `[Tag]` prefix on `Console.WriteLine` (journald captures stdout on
  Linux). Hot-path diagnostics go behind `Diag.Enabled` (`RMS_DIAG=1`).

## Known follow-ups

- `Vector6` is a mutable class used as a value; a `readonly record struct` with
  reference-swap publishing would remove the remaining torn-read window in
  `GetStatus`.
- Step "mode" fields (`LoopMode`, `WaitMode`, `Combinator`, `StopwatchAction`,
  `OutputCard`, …) are still strings compared by literal; converting them to
  enums touches the app's step JSON only by casing, which the converter tolerates.
- `ProgramStep` is one flat class for 37 step types; `JsonPolymorphic` on
  `"type"` would give typed step classes without changing the JSON.
- Responses such as `GetIO`/`GetAuxState` embed JSON as a string inside JSON;
  returning objects directly is a coordinated app+controller change.
- No dependency injection or `ILogger` yet; `RobotController` still constructs
  its device managers.
