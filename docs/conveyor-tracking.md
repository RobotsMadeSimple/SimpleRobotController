# Conveyor tracking

The robot picks (or places on) parts that are **moving** on a conveyor. The controller knows how far
the belt has travelled, latches each part's belt position when it passes a sensor (or when a program
or vision adds it), and during a tracked move adds the belt's travel to the robot's target every
motion tick so the tool rides along with the part.

This doc is the contract between the firmware, controller and app work. Names here are the wire
names; keep them exactly.

## Concepts

| Term | Meaning |
|---|---|
| **Conveyor** | A configured belt: where its travel comes from, its frame, its sensor and its pick window. Stored in `conveyors.json`, linked by stable `id`. |
| **Travel** | Belt distance moved since the controller started (mm, monotonic for a forward-running belt, can go back when reversed). |
| **Conveyor frame** | A frame in robot base coordinates (`x,y,z,rz`). Its **+X is the direction of belt travel**. Y is across the belt, Z up from the belt surface. |
| **Part** | An entry in the conveyor's queue: `{ id, travelAtLatch }`. Its current X in the conveyor frame is `sensorOffsetMm + (travelNow − travelAtLatch)`. |
| **Window** | `[startMm, endMm]` along conveyor X where the robot may track. A part beyond `endMm` is dropped as **missed**. |
| **Tracking** | A program state: one part is the *tracked part*; Cartesian targets are resolved in the **part frame** (the conveyor frame shifted along X to the part) and the motion thread keeps adding belt travel while tracking stays on. |

## Travel sources

`source.kind`:

- **`aux`** — an aux stepper axis drives the belt (`deviceId`, `axis`). Travel (mm) = live step
  count ÷ `StepsPerUnit()` of that axis (axis must be `Linear` with `mmPerRev`, or the conveyor's own
  `mmPerStep` override is used). The Uno streams live positions (below); the controller extrapolates
  between reports with the last measured velocity.
- **`timed`** — an externally driven belt (relay/VFD). The belt is "running" while an output reads on
  (`runOutput`: an IO ref, same shape the SetOutput step uses), at a fixed `speedMmS`. Travel is
  integrated on the controller. Less accurate; good enough for slow belts with wide windows.
- **`manual`** — no motion source; travel only changes via `ConveyorSetTravel` (tests, simulation).

`scale` (default 1.0) multiplies counted travel to correct belt slip / pulley error (set by calibration).

## Firmware (Uno `AuxStepperDriver`)

- Continuous (`C:`) mode **counts steps** too (signed, per axis), as do M moves and soft stops. A
  signed `long` position per axis lives on the board; `Z:<axis>` zeroes it.
- `POS?` → `POS:<ms>,<p0>,<p1>,<p2>,<p3>` (board `millis()`, signed step counts).
- `PS:<periodMs>` turns on streaming of the same `POS:` line every `periodMs` (0 = off, min 10).
  The controller sends `PS:20` when any conveyor uses that device.
- **Latch inputs:** `CFG:<pin>,L` = input-pullup scanned **every loop pass** (not the 10 ms IO scan);
  on change it emits `LAT:<pin>,<val>,<ms>,<p0>,<p1>,<p2>,<p3>` with the step counts captured at the
  edge. Regular `I` pins keep emitting `CHG:`. Wire the part sensor to the Uno for exact latching.
- Serial output must never block stepping: build lines into a small buffer and only write when
  `Serial.availableForWrite()` has room; drop a stream sample rather than stall.
- Bump `fw=` and add `pos,latch` to `caps=`.

As implemented (firmware **1.2.0**, `caps=stepper,io,pos,latch`):

- Counter sign: +1 per step CW (`D:<axis>,0`), −1 CCW — the same sign the controller's dead-reckoned
  position uses. 32-bit on the board; the controller unwraps it into a `long`.
- An unsolicited `POS:` is also sent whenever an axis comes to rest (M done — *before* `DONE:` —
  soft stop finished, `X`), so the final position is exact even with streaming off.
- Every line goes through one 256-byte TX ring fed to the UART only as far as
  `availableForWrite()` allows; stream/latch lines are formatted one number per loop pass from a
  snapshot. A stream sample is dropped when the ring is full; a `LAT:` waits for room (queue of 4
  edges). Latch pins have a 3 ms bounce hold-off after an edge (the settled level is then reported if
  it differs) and report as `I` in `STATE:`.
- `C:` on an axis that is already moving ramps from its current speed (up or down) instead of
  restarting from the minimum speed; `D:` that flips a moving axis drops it to the minimum speed.
- The controller streams at a baseline `PS:100` on every `pos` board (keeps `GetPosition` live during
  jogs); the conveyor layer asks for faster with `SetPositionStreaming(deviceId, 20)`.

### Controller aux layer (`RobotControl/AuxAxis/`)

- `AuxAxisDevice.TryGetLivePosition(int axis, out double steps, out double stepsPerSec)` (and
  `AuxAxisManager.TryGetLivePosition(deviceId, axis, …)`): position in the same space as
  `GetPosition` (board counter + an offset that absorbs `SetPosition`/zero and the gripper clamp),
  extrapolated from the newest `POS:` with the velocity measured over the last ~100 ms of samples, at
  most 3 stream periods (≥ 60 ms) ahead. Board `millis()` is mapped onto the controller `Stopwatch` by
  the least-delayed sample (lower envelope of receive − board time, following resonator drift with a
  400 ms time constant). Takes a short lock that is never held across serial I/O.
- `SetPositionStreaming(int periodMs)` (0 = back to the baseline), re-sent on every reconnect.
- `event Action<AuxLatchEvent> InputLatched`, `record AuxLatchEvent(DeviceId, Pin, Value, double[]
  AxisSteps, long StopwatchTicks)`: `LAT:` → firmware-captured positions + edge time mapped to the
  controller clock; `CHG:` → positions interpolated to the controller receive time.
- Latch pin config: `ConfigureIoPin` with `type: "Latch"` (aux boards only) stores `latch: true` on an
  **Input** pin; on firmware without `latch` it is configured as a plain input (`CHG:` events).
- **Old firmware (no `pos`)**: dead reckoning — `DONE:` deltas for M moves plus the commanded
  continuous velocity integrated on the controller (ramped at the commanded accel/decel). Good enough
  for slow belts; it drifts with any missed or ramp-shaped steps. The offline simulator uses the same
  integrator, so `TryGetLivePosition` works with no board connected.

## Controller

### Model (`RobotControl/Conveyors/ConveyorModels.cs`)

```jsonc
{
  "id": "c7f1…", "name": "Conveyor 1",
  "source": { "kind": "aux", "deviceId": "uno-1", "axis": 0, "mmPerStep": null,
              "runOutput": null, "speedMmS": 0 },
  "scale": 1.0,
  "frame": { "x": 0, "y": 0, "z": 0, "rz": 0 },
  "sensor": { "kind": "none|auxPin|io|stb", "deviceId": null, "pin": null, "input": null,
              "activeHigh": false, "offsetMm": 0, "minSpacingMm": 10 },
  "window": { "startMm": 0, "endMm": 300 },
  "maxQueue": 32
}
```

`sensor.kind`: `auxPin` = Uno latch pin (exact), `io` = any IO-device input (edge time = controller
receive time, belt position interpolated), `stb` = STB input (~20 ms polling; slow belts only).
`minSpacingMm` ignores re-triggers closer than that.

### Runtime (`ConveyorService`)

- Keeps per-conveyor travel, speed (mm/s), the part queue, counts (`latched`, `picked`, `missed`).
- `TravelAt(long stopwatchTicks)` is safe to call from the motion thread (lock-free snapshot or a
  tiny lock; never blocks on serial).
- Parts past `window.endMm` are dropped as missed (unless currently tracked — that raises the
  catchable error `conveyorWindow` in the program and stops tracking).

### Motion integration

- **Do not touch `MotionLoop` cadence, the STB4100 loops or `StepperMotor`** (see `stb-loop-timing.md`).
- In `RunMotion`, after the profiler sample and before IK: when tracking is active add
  `frameX̂ · (travelNow − travelAtPlan) · blend` to the Cartesian target. `blend` ramps 0→1 over the
  move's progress for a **MoveL** that starts untracked, and is 1 for moves that start already
  synchronised. When no move is running and tracking is on, the robot **holds on the part** (current
  pose + belt delta) so waits/gripper actions happen while following.
- **MoveJ** during tracking: predict the intercept (estimated duration × belt speed) and plan to it
  untracked; the next MoveL synchronises. (Documented limitation.)
- Belt speed above the robot's linear speed limit, or the tracked target leaving reach, raises a
  catchable error and stops tracking (`conveyorSpeed`, `outOfReach`).
- `ConveyorEndTracking` freezes the offset: the robot stays where it is and later moves are normal.

### Program steps (append after the current last `StepType`)

| StepType | Fields | Behaviour |
|---|---|---|
| `ConveyorTrack` | `conveyorId`, `trackWait` (bool, default true), `trackTimeoutMs` (number, optional) | Picks the oldest queued part inside the window and starts tracking it. With `trackWait`, waits for one to enter the window (timeout → catchable `conveyorTimeout`). Sets `$conveyor.<id>.part`. |
| `ConveyorEndTracking` | `conveyorId` (optional, default = currently tracked) | Stops tracking; marks the part `picked` (or `released` when `conveyorRelease` = true). |
| `ConveyorAddPart` | `conveyorId`, `partOffsetMm` (expr, X of the part in the conveyor frame now) | Software trigger — e.g. from a vision result. |
| `ConveyorClearQueue` | `conveyorId` | Drops all queued parts. |

While tracking, MoveL/MoveJ targets resolve in the **part frame** instead of the active local (a
taught point is "where on the part", relative to the part origin). Existing `ConveyorAction`
(forward/reverse/stop of an aux conveyor) stays and is fixed to convert mm/s → steps/s.

### Expressions (`$conveyor.<id>.…`)

`travel` (mm), `speed` (mm/s), `count` (queued parts), `inWindow` (parts in window), `tracking`
(bool), `part.x` (tracked part X in conveyor frame), `part.id`, `picked`, `missed`.

### WebSocket commands

| Command | Params | Result |
|---|---|---|
| `GetConveyors` | – | `{ conveyors: Conveyor[] }` |
| `SaveConveyor` | `conveyor` (id empty → new) | `{ conveyor }` |
| `DeleteConveyor` | `id` | – |
| `ConveyorAddPart` | `id`, `offsetMm` | `{ partId }` |
| `ConveyorClearQueue` | `id` | – |
| `ConveyorSetTravel` | `id`, `travelMm` | – (`manual` source only) |
| `ConveyorSample` | `id` | `{ x, y, z, rz, travelMm }` — current TCP + belt travel, for calibration |
| `ConveyorCalibrate` | `id`, `samples: [{x,y,z,travelMm}, …]` (≥2, same mark on the belt), optional `sensorSample: {x,y,z,travelMm, latchTravelMm}` | `{ frame, scale, sensorOffsetMm, residualMm }` proposal (not saved) |

Status broadcast gains `conveyors: [{ id, travelMm, speedMmS, running, tracking, trackedPartId,
parts: [{ id, x }], picked, missed }]` (≤ 10 Hz; parts capped at 16).

### Calibration (app wizard, modal flow)

1. Put a mark on the belt upstream. Jog the tool tip onto it → **Sample** (A).
2. Run the belt forward some distance, stop, jog onto the same mark → **Sample** (B). (Optionally a
   third, further downstream.) → frame direction = A→B, origin = A projected, `scale` = robot
   distance ÷ counted travel.
3. Sensor offset: let a part pass the sensor (latched), stop the belt, jog onto the part's leading
   reference → **Sample**; `offsetMm` = part X in the frame − travel since latch.
4. Window: drag/type start and end along X (defaults: sensor offset → +300 mm).

## Controller implementation notes (as built)

Where the controller fills in or goes beyond the contract above. Code: `RobotControl/Conveyors/`
(model, repository, `ConveyorService`, `ConveyorCalibration`, `ControllerConveyorSource`),
`RobotController.Conveyor.cs` (motion thread), `Execution/Steps/ConveyorSteps.cs`,
`Commands/ConveyorCommands.cs`. Tests: `RobotControl.Tests/Conveyors/`.

**Model / wire**
- Ids are generated as `c` + 12 hex digits and must be `[A-Za-z0-9_]` (they sit in
  `$conveyor.<id>.…`); `SaveConveyor` refuses anything else. Lookups ignore case.
- `source.runOutput` is `{ outputCard: "stb"|"relay"|"io", outputDeviceId, outputNumber }` — the
  SetOutput step's own field names.
- Status entries carry extras beyond the contract: `latched`, `lastLatchTravelMm` (travel at the
  last sensor latch — the calibration wizard's `latchTravelMm`), `sourceOk` (false while an aux
  source is silent), and `travelAtLatch` on each part. Released parts are not listed.
- `$conveyor.<id>.latched` is published too. `part.id` reads 0 when nothing is tracked.
- Step field kind for `conveyorId` is `conveyorRef` (new); the text language writes the
  conveyor's name and reads it back to the id. Validation code / text warning: `unknownConveyor`.
- Changing a conveyor's `source` or `scale` clears its queue (parts were latched against the old
  count). A rename keeps it.

**Travel**
- aux: belt mm/step = `source.mmPerStep` when set, else 1 ÷ the axis' `StepsPerUnit()` (Linear
  axes only). The board counter's sign is used as is — calibration's `scale` comes out negative
  when the count runs against the belt, which corrects it.
- A jump in the step count that the belt speed cannot explain (`SetPosition` / `ZeroPosition`,
  the stream resuming after dead reckoning) is folded into a per-conveyor offset, so travel stays
  continuous and queued parts keep their place.
- `scale` does not apply to a `manual` conveyor's set travel. Sensors of kind `io` / `stb` are
  polled every ~2 ms on the `ConveyorLoop` thread (edge time = poll time); `auxPin` uses the
  board's latch event with its captured step counts.
- `PS:20` (`SetPositionStreaming(id, 20)`) is sent at load / save and again every 2 s while a
  source is not reporting.

**Steps**
- `ConveyorTrack` with `trackWait: false` and no part in the window **continues untracked**
  (`$conveyor.<id>.tracking` = 0) rather than failing. Starting a new track while one is active
  ends the old one (counted picked).
- `ConveyorEndTracking` with `conveyorRelease` marks the part *released*: it stays on the belt,
  is not counted picked or missed, and is never tracked again.
- The "robot's linear speed limit" is its current linear speed (Set Speed (Linear) / `SpeedS`):
  `conveyorSpeed` when the belt is faster, at ConveyorTrack and on the motion thread.
- Tracking belongs to the run that started it (foreground only; both tracking steps are skipped in
  background programs). The run ending — complete, error or reset — ends it; a user Stop keeps it,
  and Continue re-dispatches the interrupted (stamped) move, which blends back onto the part.
- Off-thread faults (`conveyorWindow`, `conveyorSpeed`, the motion thread's `outOfReach`) are
  raised for the step running when the executor next ticks; a move in flight is stopped.

**Motion**
- The hold starts once a tracked MoveL has synchronised (ConveyorTrack alone does not move the
  robot). Holding drives the STB like a move (`stb.moving`), but `MotionBusy` stays false so the
  program carries on. Any untracked motion (jog, plain move, MoveJoints, blended run) ends the hold.
- The untracked-start blend is a smoothstep of the profile's progress. Blending (corner rounding)
  is off while tracking — each move is dispatched on its own.
- The program-side reach check of a tracked move also checks its end carried on by the belt for the
  estimated move time; the motion thread re-checks the followed pose with a 0.2 s look-ahead every
  20 ms. A MoveJ intercept that is out of reach drops the move and raises `outOfReach`.
- A hard stop / e-stop / joint fault stops the robot dead (no more riding); a stop condition that
  ends a tracked move keeps following from where the robot is.
