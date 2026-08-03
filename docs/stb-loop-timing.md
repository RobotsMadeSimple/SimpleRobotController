# STB4100 Loop Timing & Sensor Reads — Do Not Drift

**Status: load-bearing. Changing the timing in this section has broken homing
five times (PRs #110–#114). It was restored to the known-good PR #100 shape in
PR #115 (`d4dda23`), which fixed it on hardware. Read this before touching any
of the three loops below.**

## The three loops that must stay as they are

There are three independent `while(true)` loops involved in driving the motors
and reading the homing sensors. They are tuned as a set — changing one in
isolation is what caused every regression.

| Loop | File / method | Cadence that works |
|------|---------------|--------------------|
| Motion loop | `RobotController.ControlLoop` | **Unthrottled** — no gate, no `Sleep`, no `SpinWait` |
| STB command (write) loop | `STB4100.ControlLoop` | Gated **5 ms** moving / **20 ms** idle, `Sleep(1)` between |
| STB status (read) loop | `STB4100.StatusLoop` | `GetStatus()` on a **≥20 ms** gate, `Sleep(15)` between |

Plus:

- `STB4100.GetStatus()` — a **plain blocking single read**. One report per call,
  applied immediately. No `ReadTimeout`, no drain loop, `void` return.
- `STB4100.Connect()` — does **not** set `_stream.ReadTimeout` (HidSharp default).
- The jog-stop handshake in `STB4100.Loop()` is the **original** three-state
  machine with **no** "cancel stop when motion resumes" guard.

## Why the motion loop is unthrottled

At the PR #100 era, `RobotController.Loop()` contained its *own* `while(true)`
and never returned. The 4 ms `nextTick` gate that used to sit in `ControlLoop`
was therefore **dead code** — the motion loop, including `RunHoming()`'s sensor
checks, ran flat out. That is the configuration that homes reliably.

The current code keeps the modern single-pass `Loop()` (the test suite depends
on it) and a tick-level `try/catch`, but runs it **unthrottled** to reproduce
that same effective behaviour. `RunHoming` reacts to a sensor input on the
tick it sees it, so any latency added here becomes overshoot distance. Do not
add a gate, `Sleep`, or `SpinWait` back into this loop.

## What NOT to do (each of these was tried and broke homing)

- ❌ **Gate the motion loop** (4 ms tick). Delays sensor reaction → overshoot.
  (#112)
- ❌ **Full-throttle the STB *write* loop** (~1 kHz). The write rate *is* the
  board's report rate (one report per command). Flooding the reader backs up
  the HID input queue and starves the sensor inputs. (#112/#113)
- ❌ **Read continuously / drain-to-newest in `GetStatus`.** The drain loop
  livelocks when reports arrive faster than the read timeout — it keeps
  consuming and never applies the parse, so the homing switch is ignored while
  the axis drives through it. (#113)
- ❌ **Lower `ReadTimeout`** to make a drain terminate. Symptom of the above;
  don't set `ReadTimeout` at all. (#113)
- ❌ **Remove the idle `Sleep`s** from the STB loops. (#113)

## The one rule

The STB **write rate is the board's report rate**, and the reader consumes at
a fixed cadence. Keep the producer (write loop) bounded to what the consumer
(status loop) provably keeps up with — the 5 ms / 20 ms gating does exactly
that. Every overshoot regression came from breaking this balance, not from loop
cadence being "too slow."

## If homing overshoots again

It is almost certainly **not** these loops — they are the empirically-good
baseline. Look at the board side or the sensor wiring first, and measure
overshoot directly from the board's reported step counts (present in every
status report) before changing any timing here.

## History

- `d4dda23` (PR #100) — the known-good baseline this section is pinned to.
- PRs #110, #111, #112, #113, #114 — successive attempts to "improve" this
  timing. Each compiled, passed tests, and regressed homing on hardware.
- PR #115 — wholesale restore of the PR #100 STB4100 file + unthrottled motion
  loop. Fixed homing.
