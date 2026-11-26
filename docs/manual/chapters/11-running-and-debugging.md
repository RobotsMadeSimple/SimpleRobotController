# Running and Debugging

## Checking a program before it runs

The controller checks a program every time it changes. The problem count appears in the editor
header; tap it to see the list. Each problem names its step (and field), and tapping it jumps there.

![Problems found before a run: an unknown point (warning) and a misspelt variable (error). Tap a problem to jump to its step.](../screens/validation.png)

- **Errors** block running: an unknown variable, an expression that does not parse, a missing
  required field, a reference to something that does not exist, a loop or branch with no steps.
- **Warnings** do not: an unknown point you have not taught yet, an unused variable, a disabled step,
  a step after an endless loop.

In Text view the same problems are underlined on their lines, together with parse problems such as a
missing `end`.

![In Text view the same problems are underlined on their lines.](../screens/text-problems.png)

The codes are listed in [Error and Problem Codes](#error-codes).

## Program states

![Program states. Stop, a Pause Program step, a breakpoint or step mode all pause the run (Stopped) with its place kept; Continue resumes, Step runs one step and pauses again.](../figures/run-states.svg)

| State | Meaning |
|---|---|
| Ready | Not running; the next start begins at the top. |
| Running | Executing steps. |
| Stopped | Paused with its place kept — by the operator's Stop, a Pause Program step, a breakpoint or step mode. A move that was interrupted resumes on Continue. |
| Complete | Finished normally: after the last step, an `exit`, or a `return` in the main program. |
| Error | Ended by an uncaught error; the status carries the message and `errorCode`, and the failed step is marked. |

## The monitor

The monitor shows the running program: its status text (from [Status Update](#ref-StatusUpdate)),
the current step, the variables marked **monitor**, display images, the run controls (start, stop,
and while paused Continue and Step) and the operator speed override.

![The monitor while a program runs: status, current step, monitor variables and the run controls.](../screens/monitor.png)

Program moves are scaled by the speed override; manual jogging is not. The run log (per program)
records each step, skipped steps, caught errors and status messages.

## Breakpoints

Set a breakpoint on a step in the editor (in Text view: in the gutter). The program pauses
**before** that step, with pause reason *breakpoint*; Continue runs on. A breakpoint inside a loop
pauses on every pass. Breakpoints are kept per program in the app and can be changed while the
program runs.

![Paused at a breakpoint: the pause reason is shown, with Continue, Step and Stop.](../screens/monitor-breakpoint.png)

## Step mode

In step mode the program pauses before **every** step (pause reason *step*). **Step** runs the paused
step and pauses before the next; **Continue** leaves step mode and runs on. Notes and disabled steps
never pause, and a step that takes time (a Wait, the legs of a Jump, a vision run) is never paused
half-way.

## Run from step

Run from step starts the program at a chosen step:

- enclosing loops start at their first pass (count 0, for-each element 0; a while condition is not
  checked), and enclosing branches, Try bodies or catches are entered without evaluating anything;
- the steps before it do not run, so variables start at their declared values;
- a step inside a routine, a disabled step or a step inside a CNC block cannot be a starting point.

Background programs ignore breakpoints, step mode and run from step; a plain start clears them.

## Alarms

The controller keeps an alarm history (the newest 500): program errors, joint-limit faults, the
e-stop, the motor driver or an IO board disconnecting, and plugin crashes. Open it from Robot →
Alarms, or from the badge on the e-stop banner. Active alarms are listed first, then the history;
each one expands to its cause and remedy. **Acknowledge** marks an alarm seen; it leaves the active
list once its condition has also gone (a program error has no lasting condition, so acknowledging
it is enough). **Clear** empties the history of everything that has returned to normal. Clients use
`GetAlarms`, `AcknowledgeAlarm`, `ClearAlarms` and `GetAlarmCatalog`; `GetStatus` carries
`activeAlarmCount` and `lastAlarmUpdate`.

## Emergency stop

With the hardwired e-stop monitored (Robot → Configure → Emergency stop; wiring in the controller's
`docs/estop.md` — **not a certified safety function**), pressing it stops motion, ends every program
with `emergencyStop` and marks the robot unhomed. A red banner shows on every screen until it is
reset. Moves, jogs, homing and program starts are refused meanwhile (`estopActive` /
`estopLatched`). To recover: release the button, re-energise the drivers, press **Reset** on the
banner (refused while the input still reads open), home the robot, then run the program again.

## Backup and restore

Robot → Backup & restore downloads one zip of everything you made on this controller: programs,
points, tools, locals, grids, stacks, robot and device configuration, vision programs, calibrations,
cell packages, persistent variables and plugin settings (plugins themselves are listed, not
included — reinstall them). Restoring is a step-by-step dialog: pick the zip, review what it will
replace, confirm. The controller saves a pre-restore backup first, refuses while a program runs or
the robot moves, then restarts with the restored data. Home the robot afterwards. Over HTTP this is
`GET /backup` and `POST /backup/restore` (`?apply=true` to apply).

## Debugging tips

- Put `status "…{$var}…"` steps at interesting places — the message appears on the monitor and in
  the log.
- Mark variables **monitor** to watch them live.
- Use `SetSpeedFactor 25` at the top while proving out a new program, and remove it after.
- Disable (`@off`) a section instead of deleting it while you test the rest.
- Test a formula with the expression tester before running it.
