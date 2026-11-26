# Silo role (stacked-part lift)

A **silo** is a tube holding a stack of identical flat parts on a platform that an aux stepper lifts.
The robot always picks the top part at the rim; the silo raises the stack one part at a time. First
user: the twin checker silos (one per colour, 12 × 3.9 mm checkers each).

## Mechanics → numbers

| Field (`AuxAxisChannelConfig`) | Meaning | Checker silo |
|---|---|---|
| `role` | `"Silo"` | |
| `axisType` / `mmPerRev` / `stepsPerRev` | Linear lift, mm per motor rev | Linear, 40, 200 × microstep |
| `invertDirection` | Set so **positive = platform up** (twin motors are mirrored, so one silo is inverted) | per silo |
| `siloPitchMm` | Height of one part | 3.9 |
| `siloCapacity` | Parts when full | 12 |
| `siloClearanceMm` | Gap between a full stack's top and the rim | 0.5 |
| `siloHomeOverTravelMm` | Extra distance driven down past full travel when homing (the motor skips against the hard stop) | 5 |
| `siloHomeSpeed` | Homing speed, mm/s (slow; set the driver current low). 0 = `actionVelocity` | 5 |
| `actionVelocity` | Lift speed, mm/s | 20 |
| `siloCount` | Parts currently in the silo (persisted in `aux_config.json`, survives restarts). Changed only by SiloAction — `SetAuxAxisConfig` keeps it (clamped to a smaller capacity) and keeps any silo field it leaves out | |

Derived: **travel** = `siloCapacity × siloPitchMm + siloClearanceMm` (47.3 mm).
With `k` parts left, **present position** (absolute, from home) = `travel − (k − 1) × siloPitchMm`,
which lifts the top part's bottom face level with the rim. Absolute targets so errors never accumulate.

The tracked aux position counts motor steps, so on an inverted axis it runs negative as the platform
rises; the silo reads and targets it in platform-up steps (`× −1` when `invertDirection`). The
role needs `axisType` Linear with `mmPerRev` set.

## Behaviour

- **Home**: AuxMove of −(travel + over-travel) at `siloHomeSpeed`, wait, then zero the axis. Sets
  the runtime `siloHomed` flag. Count is unchanged.
- **Present**: requires homed (else catchable `siloNotHomed`) and `siloCount ≥ 1` (else catchable
  `siloEmpty`). Moves to the present position for `k = siloCount`, waits, then `siloCount −= 1`
  (the part is considered taken by the robot).
- **Accept** (the inverse — a part is added to the top, e.g. a captured checker): requires homed
  (`siloNotHomed`) and `siloCount < siloCapacity` (else catchable `siloFull`). Moves to the present
  position for `k = siloCount + 1` (one pitch lower than a Present, so the new part lands at the same
  rim pose), waits, then `siloCount += 1`. The robot places the part at the rim after the lift stops.
- **Refill**: sets `siloCount = siloCapacity`, then homes (the platform must be at the bottom to load
  from the top).
- **SetCount**: sets `siloCount` to the given number (0…capacity) without moving.
- Any other aux move / jog / zero / set-position of the axis, and a board connect or disconnect, clears
  `siloHomed` (runtime only, never persisted). A silo move stopped part-way does not finish: a cut-short
  Home leaves the silo un-homed, a cut-short Present leaves the count unchanged.
- Errors: `siloNotHomed`, `siloEmpty` and `siloFull` are catchable; `siloConfig` (the axis is not a
  Silo-role linear axis with a capacity and a part height, or the action is unknown) is fatal.

## Program step

`SiloAction` — fields `siloAction` (`"Home" | "Present" | "Accept" | "Refill" | "SetCount"`), `siloCount`
(int, expression-capable, SetCount only), plus the usual aux device/axis fields
(`auxDeviceId`, `auxAxisIndex`). Text form: `SiloAction home|present|refill|setcount[, count=…][, device=…, axis=…]`,
e.g. `SiloAction present, device=Aux, axis=1` / `SiloAction setcount, count=12, device=Aux, axis=1`.
Home / Present / Accept / Refill wait for the lift to stop before the program moves on.

## WebSocket

`SiloAction { deviceId, axis, action, count? }` → acks **at once** with `{ siloCount, siloHomed, moving }` —
the established pattern for motion commands (`MoveAux`, `Home`): a socket handles its commands one at
a time, so an ack held until the lift stops would also hold back a `StopAux` sent meanwhile. When
`moving` is true the finish (Home/Refill: zero + homed; Present: `siloCount − 1`; Accept:
`siloCount + 1`) runs when the axis stops; watch `GetAuxState`, whose axes gain `siloCount` and
`siloHomed` (plus the silo config fields). Refusals:
`{ ok:false, error: siloNotHomed | siloEmpty | siloFull | siloConfig, message }`. E-stop gated.

## Expressions

`$aux.<deviceId>.<axis>.siloCount`, `$aux.<deviceId>.<axis>.siloHomed`.

## Hardware notes

- Plain CNC-shield drivers can't sense a stall: homing just drives into the bottom hard stop and lets
  the motor skip. Keep the speed low and the driver current modest.
- Optional later: an IR reflective sensor in the tube's sight window at the rim, so Present can lift
  *until the sensor sees a part*.
