# Hardwired emergency stop

> **This is not a certified safety function.** The e-stop circuit described here is a practical,
> hobby-grade way to cut motor power in a hurry. It has not been assessed against ISO 13850,
> ISO 13849 or IEC 60204-1, and the controller's software reaction is *monitoring only*. If people
> can be hurt by your robot, have the cell assessed and use certified safety components (a safety
> relay, a category-rated e-stop circuit). Never rely on software to stop a robot.

The e-stop has two halves:

1. **Hardware (does the stopping):** an NC (normally closed) mushroom-head e-stop button opens a
   contactor or relay that cuts the DC supply to the stepper drivers (DM542T). Pressing it stops the
   motors whether the controller is running, crashed or switched off.
2. **Software (knows it happened):** a second contact of the same button (or a sense tap on the
   same circuit) is wired to a controller input. The controller watches that input and, when it
   trips, stops everything in software too, aborts programs, and refuses to move until the operator
   resets it and homes the robot again.

Nothing here is built for you. This page documents how the controller expects it to be wired.

---

## 1. Wiring

### Parts

| Part | Notes |
|---|---|
| E-stop button | Red mushroom head, twist or pull to release, **two NC contact blocks** (most buttons take stackable contact blocks). |
| Contactor or relay | Coil voltage to suit your control supply (e.g. 24 V DC). Contacts rated for the driver supply's voltage and current with margin (DC breaking is harder than AC; derate). |
| Flyback diode | Across a DC coil (e.g. 1N4007), cathode to +. |
| Fuse | On the driver supply, sized for the drivers. |

### Power circuit (cuts the drivers)

```
 Control +24V ──[E-stop NC contact A]──[optional reset/start button]──┬── Contactor coil + ──┐
                                                                     │                       │
                                                                  (diode)                    │
 Control 0V  ────────────────────────────────────────────────────────┴── Contactor coil − ──┘

 Driver PSU +V ──[fuse]──[contactor NO main contact]──► DM542T  +V  (all four drivers)
 Driver PSU 0V ─────────────────────────────────────────► DM542T GND
```

- The button's NC contact A carries the contactor coil current. Press the button and the coil
  drops, the main contact opens, and the drivers lose power.
- A broken wire anywhere in the coil circuit has the same effect as pressing the button (fail-safe).
- Switch the drivers' **DC supply**, not the controller PC or the STB4100 board. The controller
  keeps running and can report what happened.
- Optional but recommended: a momentary "power on" button with a self-holding contact, so the
  drivers do not re-energise by themselves when the e-stop is released.

### Sense circuit (tells the controller)

Wire the button's second NC contact (contact B) to a controller input:

```
 Input supply ──[E-stop NC contact B]──► controller input (STB4100 IN4, or an IO board pin)
```

- **STB4100 input:** use an input that is not a homing switch. On ASTRO, inputs 1–3 are the
  homing switches, so use **input 4**.
- **IO board pin (Nano / aux stepper board):** configure the pin as an Input on the I/O page and
  wire the contact between the pin and ground (the board's pull-up makes it read on when the
  circuit is closed — check what your board reports and set the polarity to match).
- Keep the sense contact galvanically separate from the coil circuit (that is why the button has
  two contact blocks) so a fault in one cannot hide a fault in the other.

---

## 2. Controller setup

Robot → Configure → **Emergency stop** in the app, or `SetEstopConfig` (see
[websocket-api.md](websocket-api.md#safety--alarms)):

| Setting | Values | Default |
|---|---|---|
| Enabled | on / off | **off** — nothing is monitored until you turn it on |
| Source | STB4100 input (`stb`, input 1–4) or an IO device pin (`io`, device id + pin) | STB input 4 |
| Polarity | **Normally closed** (fail-safe: an open circuit is an e-stop) or Normally open | Normally closed |

The page shows the input's live state, so you can press and release the button and watch it change
before turning monitoring on. The device is chosen by its stable id, so renaming it does not break
the link.

The config is saved in `robot-config.json` under `estop`, and **Reset all settings** keeps it.

---

## 3. What the controller does

The controller reads the input from state it already has — the STB4100's published inputs, or the
IO board's reported pin state — on its own "SafetyMonitor" thread (every 5 ms). It adds no work to
the STB4100 loops or the motion loop (see [stb-loop-timing.md](stb-loop-timing.md)).

### States

| State | Meaning |
|---|---|
| `estopActive` | Monitoring is on and the input reads tripped now, **or the source is offline** (fail-safe: an unplugged board means the e-stop cannot be seen). |
| `estopLatched` | A trip happened and has not been reset. Stays set after the button is released. |

A trip latches only after the input has read healthy at least once since start-up, enabling or the
last reset. A controller that boots with the button already pressed, or before its IO board has
connected, blocks motion (`estopActive`) but does not latch and raises no alarm — it is unhomed and
idle, so there is nothing to react to. Turning monitoring on while the input reads tripped (or a
second press before a reset) does not latch either, but if the robot is homed, moving or running a
program it gets the full reaction below; the active state, and its alarm, clear by themselves when
the input reads healthy again.

### On a trip

1. The same path as **HardStop**: profilers cleared, jogs stopped, the motion queue drained, homing
   cancelled, aux axes stopped.
2. The foreground program and every background program end with the **`emergencyStop`** error. It
   is uncatchable (a Try block never sees it) and the program cannot be continued, only restarted.
3. Every joint is marked **unreferenced** — the drivers lost power, so the arm may have moved. The
   robot must be homed again.
4. The `robot.estop` plugin event fires: `{ active, latched, reason, source }` (it fires again on
   reset with `latched: false`).
5. An `emergencyStop` alarm (severity critical) is raised in the alarm history.

### While active or latched

Moves, jogs, homing, `SetHomed` / `SetJointPosition`, aux moves and program starts are refused:

```json
{ "ok": false, "error": "estopActive",  "message": "Emergency stop is active (…). Release the e-stop, reset it, then home the robot." }
{ "ok": false, "error": "estopLatched", "message": "The emergency stop has not been reset. Reset it, then home the robot." }
```

Reads, `HardStop`, `StopJog`, configuration and IO outputs still work. The app shows a red banner on
every screen with a **Reset** button.

### Reset

`ResetEstop` (the banner's Reset button) succeeds only when the input reads healthy (or monitoring
is off). Otherwise it returns `estopNotHealthy` with the reason. After a reset, home the robot.

Recovery checklist:

1. Find out why the e-stop was pressed and make the cell safe.
2. Release the button (twist or pull).
3. Re-energise the drivers (your power-on button, if fitted).
4. Press **Reset** in the app.
5. **Home** the robot.
6. Start the program again.

---

## 4. Testing it

Test the e-stop every time you change the wiring, and regularly after that:

1. With the robot homed and idle, press the e-stop: the drivers must lose power (motor shafts turn
   freely by hand), the app must show the e-stop banner, and the status must read
   `estopActive: true, estopLatched: true`.
2. Try to jog: it must be refused.
3. Press Reset with the button still pressed: it must be refused.
4. Release, re-energise, Reset, home. The robot must work normally.
5. Unplug the sense wire (or the IO board): the controller must trip (fail-safe).
6. Run a slow program and press the e-stop mid-move: the arm must stop and the program must end with
   `emergencyStop`.
