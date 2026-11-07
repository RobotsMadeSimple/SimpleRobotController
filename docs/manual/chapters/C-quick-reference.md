# Quick Reference Cards

## Card 1 — Program skeleton

```srs
#!srs 1
var count = 0                    # number (default type)
var ok: bool = off
var msg: string
var pts: list point
computed done: bool = $count >= 12

## Setup
SetTool Gripper
SetSpeedL 200, accel=800, decel=800

## Main
repeat 12 as $i
  …
end
status "Done: {$count}"
```

## Card 2 — Motion

| Text | Does |
|---|---|
| `MoveJ Home` | joint move to a saved point |
| `MoveL Pick1, speed=50` | straight line, own speed |
| `MoveL here, offset=(z=20)` | relative move |
| `MoveL Pick1, offset=(z=20), blend=5` | 20 mm above, rounded corner |
| `MoveL $pts[$i]` | to a Points-list element |
| `MoveL "bin{$i}"` | to the point named by a template |
| `JumpL Place1, z=60` | lift to Z 60, travel, lower |
| `JumpL grid Tray[$r, $c], z=40` | to a grid cell (`Tray[$n]` by index) |
| `MoveL stack Bins[$i]` | to a stack slot |
| `MoveC Place1, via=Via1` | arc through a via point |
| `MoveL Pick1, until=$stb.in2, untilFlag=$hit` | stop when an input comes on |
| `SetSpeedFactor 25` | this program at 25 % |
| `SetLocal Fixture1` … `ClearLocal` | work in a local frame |
| `SavePosition $pts, mode=append` | remember where the robot is |

## Card 3 — Flow

```text
if $a > 5 and $ok          switch $grade              try retry=2, delay=1s
  …                        case 1                       …
elif $a == 2                 …                        catch $msg, code=$code
  …                        case 2, 3                    …
else                         …                        end
  …                        default
end                          …                        raise "Empty", code=empty
                           end
repeat 3 as $i             while $stb.in1 == 0        for $v, $i in $vals
  …                          Wait 10ms                  …
end                        end                        end

break · continue · return · exit "msg" · pause · label again · goto again
call PickPart(slot=$i), out picked=$ok
```

## Card 4 — Data and I/O

| Text | Does |
|---|---|
| `$count = $count + 1` | assign |
| `$label = "Bin {$i}"` | assign text (template) |
| `$vals.append(5)` · `$vals.insert(0, 5)` · `$vals.removeAt($i)` · `$vals.clear()` | list edits |
| `$pts.append(x=1, y=2, z=3)` · `$pts.append()` | add a point / the current position |
| `$pts[$i].z = 10` · `$vals.sort(by=value, desc)` · `$copy.copyFrom($vals)` | more list edits |
| `stopwatch start $lap` | start / stop / reset a stopwatch |
| `SetOutput stb 1 on` · `SetOutput relay 2 on, pulse=250ms` · `SetOutput stb 2 toggle` | outputs |
| `Wait 500ms` · `Wait until $stb.in1, timeout=5s, onTimeout=error` | waits |
| `ReadLoadCell $g, device=Arduino_Nano, pin=2` | read a load cell |
| `GripperAction open` · `GripperAction close` · `GripperAction to 25` | gripper |
| `status "…"` · `status warning "…"` · `prompt "…", buttons=("Yes", "No"), result=$c` | operator |

## Card 5 — Expressions

| | |
|---|---|
| Arithmetic | `+ - * / % ^` |
| Compare | `== != < <= > >=` (→ 1 / 0) |
| Logic | `and or not`, `c ? a : b` |
| Numbers | `abs` `sign` `sqrt` `pow` `min` `max` `clamp` `round` `floor` `ceil` `trunc` `mod` |
| Angles (degrees) | `sin` `cos` `tan` `asin` `acos` `atan` `atan2` `deg` `rad` |
| Geometry | `hypot` `dist` `dist3` `inBox` `inCircle` |
| Lists | `len` `sum` `avg` `minOf` `maxOf`, `$l.length`, `$l[i]`, `$pts[i].x` |
| Other | `if(c, a, b)` `rand()` `map(x, a, b, c, d)` `lerp(a, b, t)` |
| Text | `$s.contains("x")` `$s.startsWith` `$s.endsWith` `$s.indexOf` `$s.toNumber()` `$s.length` |
| Robot | `$robot.x` … `$robot.rz`, `$robot.moving`, `$robot.homed`, `$robot.speedOverride` |
| Program / time | `$program.runCount`, `$program.elapsedMs`, `$time.hour`, `$time_ms` |
| IO | `$stb.in1`, `$relay.3`, `$io.Arduino_Nano.Load_Cell` |

## Card 6 — Error codes

| Catchable | Fatal |
|---|---|
| `waitTimeout` · `visionTimeout` · `visionFailed` · `httpFailed` · `httpTimeout` · `receiveTimeout` · `pluginFailed` · `pluginTimeout` · `pluginNotRunning` · `promptTimeout` · `pointNotFound` · `arcDegenerate` · `indexOutOfRange` · `userError` · your own `raise` codes | `motionFault` · `jointLimit` · `driverDisconnected` · `emergencyStop` · `userStop` · `unknownVariable` · `expressionSyntax` · `computedVariable` · `badListOperation` |
