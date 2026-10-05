#!/usr/bin/env python
"""
MCP server exposing SimpleRobotController to an agent: read state, author and
save built programs, and - behind an explicit gate - drive the hardware.

Speaks MCP over stdio directly rather than through the `mcp` SDK, so it runs on
a stock Python with only `websocket-client` (and `zeroconf` for discovery, which
is optional if RMS_ROBOT_HOST is set). That keeps setup to editing .mcp.json.

Safety
------
Motion, program execution and output switching are refused unless
RMS_MCP_ALLOW_MOTION=1 is set in the server's environment. Everything else -
status, points, program read/write, validation - is always available, because
authoring a program is not the same as running one. `robot_stop` is the single
exception that ignores the gate: a stop must never be the thing that is
unavailable.

Environment
-----------
RMS_ROBOT_URL        Full endpoint, e.g. ws://192.168.4.23:9000/control
RMS_ROBOT_HOST       Host or IP; the URL is built as ws://<host>:9000/control
RMS_MCP_ALLOW_MOTION "1" to unlock the motion/execution/output tools
RMS_MCP_TIMEOUT      Seconds to wait for a command ACK (default 10)

With neither URL nor HOST set the server discovers the robot over mDNS
(_robot._tcp), which is how the app finds it.
"""

import json
import os
import sys
import time
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step_schema  # noqa: E402

SERVER_NAME    = "simple-robot-controller"
SERVER_VERSION = "1.0.0"
PROTOCOL       = "2024-11-05"

ACK_TIMEOUT   = float(os.environ.get("RMS_MCP_TIMEOUT", "10"))
ALLOW_MOTION  = os.environ.get("RMS_MCP_ALLOW_MOTION", "") == "1"

GATE_MESSAGE = (
    "Blocked: this tool drives physical hardware and the motion gate is off. "
    "The operator can enable it by setting RMS_MCP_ALLOW_MOTION=1 in the MCP server's "
    "env in .mcp.json and restarting the session. Do not work around this - ask them. "
    "Reading state and saving programs works without the gate; only running them needs it."
)


def log(msg):
    """stdout is the JSON-RPC channel, so anything human-readable goes to stderr."""
    print(f"[{SERVER_NAME}] {msg}", file=sys.stderr, flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# Robot link
# ─────────────────────────────────────────────────────────────────────────────

class RobotError(Exception):
    pass


class RobotLink:
    """
    One lazily-opened WebSocket to the controller, with request/response layered
    on top of it. The controller broadcasts status to every client on a timer, so
    a reply is found by draining frames until the ACK carrying our correlation id
    turns up - anything else on the wire is a broadcast and is dropped.
    """

    def __init__(self):
        self._ws  = None
        self._url = None

    # -- connection ----------------------------------------------------------

    def resolve_url(self):
        url = os.environ.get("RMS_ROBOT_URL")
        if url:
            return url
        host = os.environ.get("RMS_ROBOT_HOST")
        if host:
            return f"ws://{host}:9000/control"
        return self._discover()

    def _discover(self, timeout=6.0):
        try:
            from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
        except ImportError:
            raise RobotError(
                "No RMS_ROBOT_URL or RMS_ROBOT_HOST set and zeroconf is not installed, "
                "so the robot cannot be found. Set RMS_ROBOT_HOST to the controller's IP "
                "in .mcp.json, or `pip install zeroconf` for mDNS discovery."
            )

        found = {}

        class Listener(ServiceListener):
            def add_service(self, zc, type_, name):
                info = zc.get_service_info(type_, name)
                if not info or not info.addresses:
                    return
                props = {k.decode(): v.decode() for k, v in info.properties.items()}
                found["url"] = "ws://{}:{}{}".format(
                    ".".join(map(str, info.addresses[0])),
                    info.port,
                    props.get("ControlEndpoint", "/control"),
                )

            def update_service(self, zc, type_, name):
                self.add_service(zc, type_, name)

            def remove_service(self, zc, type_, name):
                pass

        zc = Zeroconf()
        try:
            ServiceBrowser(zc, "_robot._tcp.local.", Listener())
            deadline = time.time() + timeout
            while time.time() < deadline and "url" not in found:
                time.sleep(0.1)
        finally:
            zc.close()

        if "url" not in found:
            raise RobotError(
                f"No robot advertised on mDNS (_robot._tcp) within {timeout:.0f}s. "
                "Is the controller running and on the same subnet? Otherwise set "
                "RMS_ROBOT_HOST to its IP."
            )
        return found["url"]

    def connect(self):
        if self._ws is not None:
            return
        try:
            import websocket
        except ImportError:
            raise RobotError("websocket-client is not installed. Run: pip install websocket-client")

        self._url = self.resolve_url()
        ws = websocket.WebSocket()
        try:
            ws.connect(self._url, timeout=ACK_TIMEOUT)
        except Exception as e:
            raise RobotError(f"Could not connect to {self._url}: {e}")
        ws.settimeout(ACK_TIMEOUT)
        self._ws = ws
        log(f"connected to {self._url}")

    def close(self):
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None

    # -- request/response ----------------------------------------------------

    def send(self, command, params=None, expect_reply=True):
        """Send one command and return its ACK payload (minus the envelope keys)."""
        self.connect()
        cid = uuid.uuid4().hex
        frame = {"type": "Command", "id": cid, "command": command, "params": params or {}}

        try:
            self._ws.send(json.dumps(frame))
        except Exception as e:
            # A stale socket usually surfaces on the first write; reopen once.
            self.close()
            self.connect()
            try:
                self._ws.send(json.dumps(frame))
            except Exception:
                raise RobotError(f"Send failed for {command}: {e}")

        if not expect_reply:
            return {}

        deadline = time.time() + ACK_TIMEOUT
        while time.time() < deadline:
            try:
                raw = self._ws.recv()
            except Exception as e:
                raise RobotError(f"No reply to {command}: {e}")
            if not raw:
                continue
            try:
                msg = json.loads(raw)
            except (ValueError, TypeError):
                continue
            if msg.get("type") == "ack" and msg.get("id") == cid:
                if msg.get("ok") is False:
                    raise RobotError(f"{command} rejected: {msg.get('error', 'no reason given')}")
                return {k: _maybe_json(v) for k, v in msg.items()
                        if k not in ("type", "command", "id", "ok")}
            # anything else is a broadcast to all clients - not ours

        raise RobotError(f"Timed out after {ACK_TIMEOUT:.0f}s waiting for an ACK to {command}.")


def _maybe_json(value):
    """
    Several commands return their payload as a JSON *string* (GetPoints, GetBuiltPrograms,
    GetGrids...). Decode those so callers see structured data either way.
    """
    if isinstance(value, str) and value[:1] in ("[", "{"):
        try:
            return json.loads(value)
        except ValueError:
            return value
    return value


ROBOT = RobotLink()


# ─────────────────────────────────────────────────────────────────────────────
# Program validation - offline fallback (robot_validate_program prefers the controller's
# ValidateBuiltProgram; robot_save_program still gates saves on these checks)
# ─────────────────────────────────────────────────────────────────────────────

def _walk_steps(steps, path="steps"):
    """Yield (jsonpath, step) for every step including those nested in loops/branches."""
    for i, step in enumerate(steps or []):
        if not isinstance(step, dict):
            continue
        here = f"{path}[{i}]"
        yield here, step
        for field in step_schema.NESTED_STEP_FIELDS:
            yield from _walk_steps(step.get(field), f"{here}.{field}")
        for j, branch in enumerate(step.get("elseIfBranches") or []):
            if isinstance(branch, dict):
                yield from _walk_steps(branch.get("steps"), f"{here}.elseIfBranches[{j}].steps")


def validate_program(prog, known_points=None, known_programs=None):
    """
    Static checks on a program before it is saved. Catches the mistakes that are
    invisible until run time: a misspelled step type the executor maps to Unknown,
    a $-less variable reference that silently evaluates to 0, a point that does
    not exist, a GoToLabel with no Label.

    The split matters: errors are things that are silently wrong or structurally
    broken, and block a save. Warnings are legal-but-suspect - an untaught point,
    an unconfigured step - and do not, because writing a program before teaching
    its points is a normal way to work and those failures are loud at run time.

    Returns (errors, warnings).
    """
    errors, warnings = [], []

    name = (prog.get("name") or "").strip()
    if not name:
        errors.append("Program has no name.")

    steps = prog.get("steps")
    if not isinstance(steps, list):
        errors.append("`steps` must be an array.")
        return errors, warnings

    variables = prog.get("variables") or []
    var_names, object_var_names = set(), set()
    for i, v in enumerate(variables):
        if not isinstance(v, dict):
            errors.append(f"variables[{i}] is not an object.")
            continue
        vn = (v.get("name") or "").strip()

        # A record list is `items` with elementType Record. `objects` is the legacy spelling,
        # still read so programs saved before the list types were unified keep validating.
        # An absent elementType reads as Record, matching the controller.
        items_field = "items" if v.get("items") is not None else \
                      "objects" if v.get("objects") is not None else None
        if vn and items_field:
            elem = v.get("elementType") if items_field == "items" else "Record"
            if elem is None:
                elem = "Record"
            if elem not in ("Number", "Boolean", "Point", "Record"):
                errors.append(f"variables[{i}] ('{vn}').elementType must be one of "
                              f"Number, Boolean, Point, Record - got {elem!r}.")
            if elem == "Record":
                object_var_names.add(vn)
            if not isinstance(v[items_field], list):
                errors.append(f"variables[{i}] ('{vn}').{items_field} must be an array.")
            else:
                for j, rec in enumerate(v[items_field]):
                    if not isinstance(rec, dict) or any(not isinstance(x, (int, float)) or isinstance(x, bool)
                                                        for x in rec.values()):
                        errors.append(f"variables[{i}].{items_field}[{j}] must be an object of number fields. "
                                      f"Expressions evaluate to numbers, so booleans go in as 0/1.")
        if not vn:
            errors.append(f"variables[{i}] has no name.")
        elif vn in var_names:
            errors.append(f"Duplicate variable name '{vn}'.")
        else:
            var_names.add(vn)
        if not v.get("id"):
            warnings.append(f"variables[{i}] ('{vn}') has no id; the app expects a unique one.")

    is_background = bool(prog.get("isBackground"))
    seen_ids, labels, goto_labels = set(), set(), []

    for path, step in _walk_steps(steps):
        stype = step.get("type")
        sid   = step.get("id")

        if not sid:
            errors.append(f"{path} has no id.")
        elif sid in seen_ids:
            errors.append(f"{path} reuses id '{sid}'; ids must be unique across the program.")
        else:
            seen_ids.add(sid)

        if stype not in step_schema.STEPS:
            errors.append(
                f"{path} has unknown type '{stype}'. The executor stores these as Unknown and "
                f"skips them. Valid types: {', '.join(sorted(step_schema.STEPS))}"
            )
            continue

        if is_background and stype in step_schema.BACKGROUND_FORBIDDEN:
            errors.append(f"{path} is a {stype}; background programs skip motion/tool/homing steps.")

        # Point targets. A move with no target is legal - the executor falls back to the
        # current TCP - so it is a warning, not an error. Likewise a point that is not
        # taught yet: the run fails loudly rather than moving somewhere wrong, and
        # authoring a program before teaching its points is a normal way to work.
        if stype in ("MoveL", "MoveJ", "JumpL", "JumpJ"):
            has_target = any(step.get(k) for k in
                             ("pointName", "pointNameExpr", "varPointName", "gridPoint", "stackPoint"))
            if not has_target and not any(
                step.get(f"override{a}") is not None for a in ("X", "Y", "Z", "RX", "RY", "RZ")
            ):
                warnings.append(f"{path} is a {stype} with no target; it falls back to the current "
                                f"TCP, so it will not move. Probably an unconfigured step.")
            pn = step.get("pointName")
            if pn and known_points is not None and pn not in known_points:
                warnings.append(f"{path} targets point '{pn}', which is not taught on the robot. "
                                f"The step will fault at run time unless it is taught first.")

        if stype == "CallRoutine":
            rn = step.get("routineName")
            if not rn:
                warnings.append(f"{path} is a CallRoutine with no routineName; it does nothing.")
            elif known_programs is not None and rn not in known_programs:
                warnings.append(f"{path} calls routine '{rn}', which does not exist on the robot yet.")

        if stype == "SetVariable":
            vn = step.get("variableName")
            if not vn:
                warnings.append(f"{path} is a SetVariable with no variableName; it does nothing.")
            elif var_names and vn not in var_names:
                errors.append(f"{path} assigns '{vn}', which is not declared in `variables`. "
                              f"Reading it later throws UnknownVariable at run time.")

        # A cellsVar holds a list of records, not a number, so it has to be declared as a
        # Record list - the executor would happily create it at run time, but a variable
        # the app cannot see is one the user cannot pick or inspect.
        if stype == "RunVision":
            for j, out in enumerate(step.get("colorOutputs") or []):
                cv = isinstance(out, dict) and out.get("cellsVar")
                if cv and cv not in object_var_names:
                    (errors if cv in var_names else warnings).append(
                        f"{path}.colorOutputs[{j}].cellsVar = '{cv}' is "
                        + ("not a Record list; it holds one record per grid cell, so declare "
                           'it with items: [] and elementType: "Record".' if cv in var_names else
                           'not declared in `variables`. Add it with items: [] and '
                           'elementType: "Record".')
                    )

        if stype == "Label":
            ln = step.get("labelName")
            if ln:
                labels.add(ln)
        if stype == "GoToLabel":
            ln = step.get("labelName")
            if ln:
                goto_labels.append((path, ln))

        if stype == "Loop":
            mode = step.get("loopMode", "count")
            if mode == "forEach" and not step.get("forEachVariableName"):
                errors.append(f"{path} is a forEach Loop with no forEachVariableName.")
            if mode == "while" and not step.get("loopWhileCondition"):
                errors.append(f"{path} is a while Loop with no loopWhileCondition.")
            if mode == "count" and step.get("loopCount") == 0:
                warnings.append(f"{path} loops 0 times, which means forever.")
            if not step.get("loopSteps"):
                warnings.append(f"{path} is a Loop with an empty body.")

        if stype == "SetOutput":
            if step.get("outputNumber") is None:
                errors.append(f"{path} is a SetOutput with no outputNumber.")
            if step.get("outputValue") is None:
                warnings.append(f"{path} is a SetOutput with no outputValue; it defaults to off.")

        if stype == "Wait":
            if step.get("waitMode") == "condition" and not step.get("waitCondition"):
                errors.append(f"{path} waits on a condition but none is set.")
            elif step.get("waitMode") in (None, "duration") and not step.get("waitMs"):
                warnings.append(f"{path} is a Wait with no waitMs.")

        # $-sigil checks. A bare name is not a lookup - the evaluator returns 0 for it,
        # so a forgotten $ is a plausible-looking wrong answer rather than an error.
        for field in ("variableExpr", "pointNameExpr", "statusMessage", "saveImagePath"):
            val = step.get(field)
            if isinstance(val, str):
                bad = _brace_missing_sigil(val)
                if bad:
                    errors.append(f"{path}.{field}: {bad} has no $ inside the braces. "
                                  f"It is left as written, not substituted.")
        for key, expr in (step.get("expressions") or {}).items():
            if isinstance(expr, str) and _bare_words(expr):
                errors.append(
                    f"{path}.expressions.{key} = '{expr}' references a name without $. "
                    f"Bare words evaluate to 0. Write '$name'."
                )

    for path, ln in goto_labels:
        if ln not in labels:
            errors.append(f"{path} jumps to label '{ln}', which no Label step defines.")

    return errors, warnings


# Words the evaluator understands on their own. Without the operators here, "$a and $b"
# would be reported as "and" missing its $.
_EXPR_KEYWORDS = ("true", "false", "and", "or", "not")

# The controller's expression functions (docs/expressions-and-variables.md section 2). A name
# from this list followed by "(" is a call, not a forgotten $.
_EXPR_FUNCTIONS = (
    "abs", "sign", "sqrt", "pow", "min", "max", "clamp", "round", "floor", "ceil", "trunc", "mod",
    "sin", "cos", "tan", "asin", "acos", "atan", "atan2", "deg", "rad", "hypot", "dist", "dist3",
    "if", "len", "sum", "avg", "minof", "maxof", "rand", "map", "lerp",
)


def _bare_words(expr):
    """Identifiers in an expression that are not $-prefixed, not .components, not keywords,
    and not function calls."""
    import re
    stripped = re.sub(r"\$\w+", " ", expr)
    stripped = re.sub(r"\.\w+", " ", stripped)
    stripped = re.sub(r"\b(" + "|".join(_EXPR_FUNCTIONS) + r")\s*\(", "(", stripped, flags=re.IGNORECASE)
    stripped = re.sub(r"\d+(\.\d*)?[eE][+-]?\d+", " ", stripped)  # 1e3 is a number
    return [w for w in re.findall(r"[A-Za-z_]\w*", stripped) if w.lower() not in _EXPR_KEYWORDS]


def _brace_missing_sigil(text):
    """First {...} group whose body references a name without $, or None. Mirrors the controller."""
    import re
    for m in re.findall(r"\{[^{}]*\}", text):
        body = m[1:-1].strip()
        if body and _bare_words(body):
            return m
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Tools
# ─────────────────────────────────────────────────────────────────────────────

def _ok(obj):
    return json.dumps(obj, indent=2, default=str)


# Commands safe to run through robot_raw_command without the motion gate.
READ_ONLY_COMMANDS = {
    "GetStatus", "GetRobotInfo", "GetRobotConfig", "GetPoints", "GetTools", "GetLocals",
    "GetGrids", "GetStacks", "GetBuiltPrograms", "GetProgramVariables", "GetProgramLogs",
    "GetProgramImages", "GetProgramVariableImage", "GetCameras", "GetVisionPrograms",
    "GetNanoDevices", "GetRelays",
    "GetAuxDevices", "GetAuxAxisConfig", "GetCameraResolutions",
    "GetCameraCalibration", "CalibrationPredict",
}

STATUS_FIELDS = [
    "moving", "wasHomed", "isHoming", "homingState", "driverConnected", "driverOk",
    "x", "y", "z", "rx", "ry", "rz",
    "speedS", "accelS", "decelS", "speedJ", "accelJ", "decelJ",
    "speedOverridePercent", "activeTool", "activeLocal",
    "faulted", "faultJoint", "faultDirection", "faultMessage", "limitBypass",
    "jointLimitsEnabled", "robotType", "version", "isLinux",
    "input1", "input2", "input3", "input4", "output1", "output2", "output3", "output4",
    "programs", "backgroundPrograms",
]


def tool_robot_status(args):
    data = ROBOT.send("GetStatus")
    full = args.get("full", False)
    if not full:
        data = {k: v for k, v in data.items() if k in STATUS_FIELDS}
    data["_endpoint"] = ROBOT._url
    data["_motionGate"] = "open" if ALLOW_MOTION else "closed (RMS_MCP_ALLOW_MOTION is not 1)"
    return _ok(data)


def tool_robot_points(args):
    data = ROBOT.send("GetPoints")
    points = data.get("points", [])
    name = args.get("name")
    if name:
        hit = next((p for p in points if str(p.get("name", "")).lower() == name.lower()), None)
        if hit is None:
            return _ok({"error": f"No point named '{name}'.",
                        "available": sorted(str(p.get("name", "")) for p in points)})
        return _ok(hit)
    if args.get("coordinates"):
        return _ok(points)
    return _ok({"count": len(points),
                "names": sorted(str(p.get("name", "")) for p in points)})


def tool_robot_programs(args):
    data = ROBOT.send("GetBuiltPrograms")
    progs = data.get("programs", [])
    return _ok([
        {
            "name":        p.get("name"),
            "description": p.get("description"),
            "steps":       len(p.get("steps") or []),
            "variables":   [v.get("name") for v in (p.get("variables") or [])],
            "isRoutine":    p.get("isRoutine", False),
            "isBackground": p.get("isBackground", False),
        }
        for p in progs
    ])


def tool_robot_get_program(args):
    name = args["name"]
    data = ROBOT.send("GetBuiltPrograms")
    progs = data.get("programs", [])
    hit = next((p for p in progs if str(p.get("name", "")).lower() == name.lower()), None)
    if hit is None:
        return _ok({"error": f"No program named '{name}'.",
                    "available": [p.get("name") for p in progs]})
    return _ok(hit)


def tool_robot_save_program(args):
    prog = args["program"]
    if isinstance(prog, str):
        prog = json.loads(prog)

    known_points = known_programs = None
    try:
        known_points = {str(p.get("name")) for p in ROBOT.send("GetPoints").get("points", [])}
        known_programs = {str(p.get("name")) for p in ROBOT.send("GetBuiltPrograms").get("programs", [])}
    except RobotError:
        pass  # validate offline rather than refuse to save

    errors, warnings = validate_program(prog, known_points, known_programs)
    if errors and not args.get("force"):
        return _ok({
            "saved": False,
            "errors": errors,
            "warnings": warnings,
            "hint": "Fix these and call again. Pass force=true only if you are certain a "
                    "check is wrong - saving a broken program is how a move ends up somewhere "
                    "unexpected.",
        })

    prog.setdefault("id", uuid.uuid4().hex)
    params = {
        "id":                   prog["id"],
        "name":                 prog.get("name", ""),
        "description":          prog.get("description", ""),
        "steps":                prog.get("steps", []),
        "variables":            prog.get("variables"),
        "isRoutine":            bool(prog.get("isRoutine", False)),
        "isBackground":         bool(prog.get("isBackground", False)),
        "killBackgroundOnStop": bool(prog.get("killBackgroundOnStop", True)),
    }
    ROBOT.send("SaveBuiltProgram", params)
    return _ok({"saved": True, "name": params["name"], "id": params["id"],
                "steps": len(params["steps"]), "warnings": warnings,
                "note": "Saved but not run. Use robot_run_program to execute it."})


def tool_robot_validate_program(args):
    """
    Validate on the controller (ValidateBuiltProgram): it knows the expression language,
    the property and function tables, routine bodies and every point/tool/local/grid/stack/
    vision program on the robot. The local Python checks in validate_program() are only a
    fallback for when the controller cannot be reached (or predates the command), or when
    check_robot=false asks for an offline structural check.
    """
    prog = args["program"]
    if isinstance(prog, str):
        prog = json.loads(prog)

    controller_error = None
    if args.get("check_robot", True):
        try:
            data = ROBOT.send("ValidateBuiltProgram", {"program": prog})
            problems = data.get("problems") or []
            errors = [p for p in problems if p.get("severity") == "error"]
            warnings = [p for p in problems if p.get("severity") != "error"]
            return _ok({
                "valid": not errors,
                "source": "controller",
                "errorCount": len(errors),
                "warningCount": len(warnings),
                "problems": problems,
            })
        except RobotError as e:
            controller_error = str(e)

    errors, warnings = validate_program(prog)
    result = {"valid": not errors, "source": "offline", "errors": errors, "warnings": warnings}
    if controller_error:
        result["note"] = (f"Controller validation unavailable ({controller_error}); ran the offline "
                          f"structural checks only - expressions, point names and routines were not "
                          f"verified.")
    return _ok(result)


def tool_robot_step_schema(args):
    types = args.get("types")
    if not types:
        return _ok({
            "stepTypes": {k: v["what"] for k, v in step_schema.STEPS.items()},
            "hint": "Call again with types=['MoveL','Loop'] for the full field list of specific types.",
            "common": step_schema.COMMON,
            "programShape": {
                "id": "string, generated if omitted",
                "name": "string, required - this is the key programs are looked up by",
                "description": "string",
                "steps": "array of ProgramStep",
                "variables": "array of ProgramVariable",
                "isRoutine": "bool - hidden from the program list, callable via CallRoutine",
                "isBackground": "bool - runs in parallel; motion steps are skipped",
                "killBackgroundOnStop": "bool, default true",
            },
            "variableFields": step_schema.VARIABLE_FIELDS,
        })
    out = {}
    for t in types:
        match = next((k for k in step_schema.STEPS if k.lower() == str(t).lower()), None)
        if match is None:
            out[t] = {"error": f"Unknown step type. Valid: {', '.join(sorted(step_schema.STEPS))}"}
        else:
            out[match] = {"what": step_schema.STEPS[match]["what"],
                          "fields": {**step_schema.COMMON, **step_schema.STEPS[match]["fields"]}}
    return _ok(out)


def tool_robot_teach_point(args):
    ROBOT.send("TeachPoint", {"name": args["name"]})
    return _ok({"taught": args["name"],
                "note": "Saved at the robot's current position, in the base frame."})


# ── Gated tools ──────────────────────────────────────────────────────────────

def tool_robot_move(args):
    cmd = "MoveJ" if args.get("joint") else "MoveL"
    params = {}
    if args.get("name"):
        params["name"] = args["name"]
    else:
        for axis in ("x", "y", "z", "rx", "ry", "rz"):
            if args.get(axis) is not None:
                params[axis] = args[axis]
        if not params:
            return _ok({"error": "Give either name= (a saved point) or some of x/y/z/rx/ry/rz."})
    for k in ("speed", "accel", "decel"):
        if args.get(k) is not None:
            params[k] = args[k]
    ROBOT.send(cmd, params)
    return _ok({"sent": cmd, "params": params,
                "note": "Queued on the motion thread. Poll robot_status until moving=false."})


def tool_robot_home(args):
    ROBOT.send("Home")
    return _ok({"sent": "Home", "note": "Homing started. Poll robot_status for wasHomed/isHoming."})


def tool_robot_run_program(args):
    name = args["name"]
    progs = ROBOT.send("GetBuiltPrograms").get("programs", [])
    if not any(str(p.get("name", "")).lower() == name.lower() for p in progs):
        return _ok({"error": f"No program named '{name}'.",
                    "available": [p.get("name") for p in progs]})
    ROBOT.send("ExecuteBuiltProgram", {"name": name})
    return _ok({"started": name,
                "note": "Running. robot_status.programs carries progress; robot_stop halts it."})


def tool_robot_set_output(args):
    # "SetOutput" is a program *step* type, not a WebSocket command. Each IO card
    # has its own command (see docs/websocket-api.md, "IO" section).
    card   = (args.get("card") or "stb").lower()
    number = int(args["number"])
    value  = bool(args["value"])
    if card == "stb":
        cmd, params = "SetSTBOutput", {"pin": number, "value": value}
    elif card == "nano":
        nano_id = args.get("nanoId")
        if not nano_id:
            return _ok({"error": "nanoId is required when card is 'nano' (see robot_status.io.nanos)."})
        cmd, params = "SetNanoOutput", {"nanoId": nano_id, "pin": number, "value": value}
    elif card == "relay":
        cmd, params = "SetRelay", {"relay": number, "value": value}
    else:
        return _ok({"error": f"Unknown card '{card}'. Use 'stb', 'nano' or 'relay'."})
    ROBOT.send(cmd, params)
    return _ok({"sent": cmd, "params": params})


def tool_robot_raw_command(args):
    cmd = args["command"]
    if cmd not in READ_ONLY_COMMANDS and not ALLOW_MOTION:
        return _ok({"error": GATE_MESSAGE, "command": cmd,
                    "allowedWithoutGate": sorted(READ_ONLY_COMMANDS)})
    return _ok(ROBOT.send(cmd, args.get("params") or {}))


def tool_robot_stop(args):
    """Never gated - a stop must always be available."""
    results = {}
    for cmd in ("StopBuiltProgram", "HardStop", "StopJog"):
        try:
            ROBOT.send(cmd)
            results[cmd] = "sent"
        except RobotError as e:
            results[cmd] = f"failed: {e}"
    return _ok(results)


# ── Registry ─────────────────────────────────────────────────────────────────

def _t(name, description, properties, required=(), handler=None, gated=False):
    return {
        "name": name,
        "description": description,
        "inputSchema": {"type": "object", "properties": properties, "required": list(required)},
        "_handler": handler,
        "_gated": gated,
    }


TOOLS = [
    _t("robot_status",
       "Current robot state: pose, homed/faulted flags, motion defaults, IO, running programs. "
       "Start here - it also reports which endpoint is in use and whether the motion gate is open.",
       {"full": {"type": "boolean",
                 "description": "Return every field instead of the curated subset. Default false."}},
       handler=tool_robot_status),

    _t("robot_points",
       "Saved (taught) points. Names only by default - these are what a move step's pointName "
       "must match, so check here before writing one.",
       {"name": {"type": "string", "description": "Return just this point, with coordinates."},
        "coordinates": {"type": "boolean", "description": "Return all points with coordinates."}},
       handler=tool_robot_points),

    _t("robot_programs",
       "Summary of every built program on the robot: name, step count, variables, routine/background flags.",
       {}, handler=tool_robot_programs),

    _t("robot_get_program",
       "Full JSON of one built program. Use this to read an existing program before editing it - "
       "save replaces the whole program, so edit a fetched copy rather than writing one from scratch.",
       {"name": {"type": "string", "description": "Program name."}},
       required=["name"], handler=tool_robot_get_program),

    _t("robot_step_schema",
       "Field reference for program steps. Call with no arguments for the list of step types and the "
       "program/variable shape; call with types=['MoveL','Loop'] for full field docs on specific types. "
       "Read this before authoring a program rather than guessing field names.",
       {"types": {"type": "array", "items": {"type": "string"},
                  "description": "Step type names to expand, e.g. ['MoveL','IfCondition']."}},
       handler=tool_robot_step_schema),

    _t("robot_validate_program",
       "Validate a program on the controller (ValidateBuiltProgram): expression syntax, unknown "
       "functions/variables/properties, read-only property writes, missing points/tools/locals/"
       "grids/stacks/vision programs/routines, routine recursion, labels, empty loops/branches, "
       "missing fields, plus disabled/unreachable steps and unused variables as warnings. Returns "
       "problems [{stepId, stepPath, field, severity, code, message}]. Falls back to offline "
       "structural checks if the robot is unreachable. Saves and runs nothing.",
       {"program": {"type": "object", "description": "The program object to check."},
        "check_robot": {"type": "boolean",
                        "description": "Validate on the robot (default true). false = offline structural checks only."}},
       required=["program"], handler=tool_robot_validate_program),

    _t("robot_save_program",
       "Create or replace a built program. Validates first and refuses to save on errors. Saving does "
       "NOT run the program. Replaces the program with the given name wholesale - fetch it with "
       "robot_get_program and modify that if you are editing rather than creating.",
       {"program": {"type": "object",
                    "description": "Program object: {name, description?, steps[], variables?, "
                                   "isRoutine?, isBackground?}. See robot_step_schema."},
        "force": {"type": "boolean", "description": "Save despite validation errors. Default false."}},
       required=["program"], handler=tool_robot_save_program),

    _t("robot_teach_point",
       "Save the robot's CURRENT position under a name. The robot must already be where you want the "
       "point - this records, it does not move.",
       {"name": {"type": "string", "description": "Point name. Reusing a name overwrites it."}},
       required=["name"], handler=tool_robot_teach_point),

    _t("robot_stop",
       "Stop everything now: halts the running program, hard-stops motion, cancels any jog. "
       "Always available, even when the motion gate is closed.",
       {}, handler=tool_robot_stop),

    # ── Gated ────────────────────────────────────────────────────────────────
    _t("robot_move",
       "MOVES THE ROBOT. Single linear (default) or joint move to a saved point or explicit "
       "coordinates. The robot must be homed. Check robot_status first and confirm with the operator "
       "before moving to coordinates that were not taught.",
       {"name":  {"type": "string", "description": "Saved point to move to."},
        "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
        "rx": {"type": "number"}, "ry": {"type": "number"}, "rz": {"type": "number"},
        "joint": {"type": "boolean", "description": "Use MoveJ instead of MoveL. Default false."},
        "speed": {"type": "number", "description": "mm/s linear, deg/s joint."},
        "accel": {"type": "number"}, "decel": {"type": "number"}},
       handler=tool_robot_move, gated=True),

    _t("robot_home",
       "MOVES THE ROBOT. Clears any latched fault and runs the full homing sequence.",
       {}, handler=tool_robot_home, gated=True),

    _t("robot_run_program",
       "MOVES THE ROBOT. Executes a saved built program. Read the program first so you know what it "
       "does; robot_stop halts it.",
       {"name": {"type": "string", "description": "Program name."}},
       required=["name"], handler=tool_robot_run_program, gated=True),

    _t("robot_set_output",
       "DRIVES HARDWARE. Sets a digital output - grippers, valves, actuators.",
       {"number": {"type": "integer", "description": "Output index: STB pin 1-4, Nano pin number, or relay 1-4."},
        "value":  {"type": "boolean", "description": "Target state."},
        "card":   {"type": "string", "description": 'IO device: "stb" (default), "nano" or "relay".'},
        "nanoId": {"type": "string", "description": 'Nano device id, required when card is "nano".'}},
       required=["number", "value"], handler=tool_robot_set_output, gated=True),

    _t("robot_raw_command",
       "Escape hatch for any WebSocket command not covered above (tools, locals, grids, stacks, aux "
       "axis, cameras, config). Read-only Get* commands work ungated; anything else needs the motion "
       "gate. See docs/websocket-api.md for the command list.",
       {"command": {"type": "string", "description": "Command name, e.g. 'GetGrids'."},
        "params":  {"type": "object", "description": "Command parameters."}},
       required=["command"], handler=tool_robot_raw_command),
]

TOOLS_BY_NAME = {t["name"]: t for t in TOOLS}


def call_tool(name, args):
    tool = TOOLS_BY_NAME.get(name)
    if tool is None:
        return f"Unknown tool '{name}'.", True
    if tool["_gated"] and not ALLOW_MOTION:
        return GATE_MESSAGE, True
    try:
        return tool["_handler"](args or {}), False
    except RobotError as e:
        return f"Robot error: {e}", True
    except KeyError as e:
        return f"Missing required argument: {e}", True
    except Exception as e:
        log(f"{name} failed: {type(e).__name__}: {e}")
        return f"{type(e).__name__}: {e}", True


# ─────────────────────────────────────────────────────────────────────────────
# JSON-RPC over stdio
# ─────────────────────────────────────────────────────────────────────────────

def respond(msg_id, result=None, error=None):
    msg = {"jsonrpc": "2.0", "id": msg_id}
    if error is not None:
        msg["error"] = error
    else:
        msg["result"] = result
    sys.stdout.write(json.dumps(msg) + "\n")
    sys.stdout.flush()


def handle(msg):
    method = msg.get("method")
    msg_id = msg.get("id")
    params = msg.get("params") or {}

    # Notifications carry no id and must not be answered.
    if msg_id is None:
        return

    if method == "initialize":
        respond(msg_id, {
            "protocolVersion": params.get("protocolVersion", PROTOCOL),
            "capabilities": {"tools": {}},
            "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
        })
    elif method == "tools/list":
        respond(msg_id, {"tools": [
            {k: v for k, v in t.items() if not k.startswith("_")} for t in TOOLS
        ]})
    elif method == "tools/call":
        text, is_error = call_tool(params.get("name"), params.get("arguments"))
        respond(msg_id, {"content": [{"type": "text", "text": text}], "isError": is_error})
    elif method == "ping":
        respond(msg_id, {})
    else:
        respond(msg_id, error={"code": -32601, "message": f"Method not found: {method}"})


def main():
    log(f"ready - motion gate {'OPEN' if ALLOW_MOTION else 'closed'}")
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        try:
            handle(msg)
        except Exception as e:
            log(f"handler crashed: {type(e).__name__}: {e}")
            if msg.get("id") is not None:
                respond(msg["id"], error={"code": -32603, "message": str(e)})
    ROBOT.close()


if __name__ == "__main__":
    main()
