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

    def send(self, command, params=None, expect_reply=True, keep_ok=False):
        """Send one command and return its ACK payload (minus the envelope keys). keep_ok: the
        command's own result has an `ok` field (ParseProgramText) - return it instead of treating
        ok:false as a rejection; only an ack carrying `error` is one then."""
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
                if msg.get("ok") is False and (not keep_ok or "error" in msg):
                    raise RobotError(f"{command} rejected: {msg.get('error', 'no reason given')}")
                return {k: _maybe_json(v) for k, v in msg.items()
                        if k not in ("type", "command", "id") and (keep_ok or k != "ok")}
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

        # cells / items / lines / pose outputs hold a list of records, not a number, so the
        # variable has to be declared as a Record list - the executor would happily create
        # it at run time, but a variable the app cannot see is one the user cannot pick or
        # inspect.
        if stype == "RunVision":
            for j, out in enumerate(step.get("toolOutputs") or []):
                if not isinstance(out, dict):
                    errors.append(f"{path}.toolOutputs[{j}] is not an object.")
                    continue
                cv, kind = out.get("variable"), out.get("value")
                if not out.get("toolId") or not kind or not cv:
                    errors.append(f"{path}.toolOutputs[{j}] needs toolId, value and variable.")
                    continue
                if kind in ("cells", "items", "lines", "pose") and cv not in object_var_names:
                    (errors if cv in var_names else warnings).append(
                        f"{path}.toolOutputs[{j}].variable = '{cv}' is "
                        + (f"not a Record list; {kind} is a list of records, so declare "
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
    "GetIO", "GetRelayState",
    "GetIoHistoryConfig", "GetIoHistoryWindow", "ListIoRecordings", "GetIoRecordingWindow",
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


# Controller error codes that only say "this refers to something not on the robot yet" - a
# point, grid, routine... Authoring before teaching is a normal way to work, so with
# allow_untaught=true these are reported as pending instead of blocking the save.
PENDING_CODES = {"unknownPoint", "unknownTool", "unknownLocal", "unknownGrid", "unknownStack",
                 "unknownRoutine", "unknownVisionProgram"}


def _controller_problems(prog):
    """The controller's ValidateBuiltProgram problems, or None when it cannot be asked."""
    try:
        return ROBOT.send("ValidateBuiltProgram", {"program": prog}).get("problems") or []
    except RobotError:
        return None


def _problem_key(p):
    return (p.get("code"), p.get("stepId"), p.get("field"), p.get("message"))


def _problem_diff(before, after):
    """Problems a change introduced and the ones it fixed (matched on code/step/field/message)."""
    kb = {_problem_key(p) for p in before}
    ka = {_problem_key(p) for p in after}
    return ([p for p in after if _problem_key(p) not in kb],
            [p for p in before if _problem_key(p) not in ka])


def _check_program(prog, allow_untaught=False):
    """
    Everything known about a program before it is saved. The controller's validator is the
    authority (it knows the expression language and every point/grid/routine on the robot);
    the offline structural checks run too and catch what it cannot see (missing/duplicate
    ids, unknown step types, $-less names). Returns `blocking` problems (errors that stop a
    save), `pending` (missing references, when allow_untaught) and `warnings`.
    """
    known_points = known_programs = None
    try:
        known_points = {str(p.get("name") or p.get("Name")) for p in ROBOT.send("GetPoints").get("points", [])}
        known_programs = {str(p.get("name")) for p in ROBOT.send("GetBuiltPrograms").get("programs", [])}
    except RobotError:
        pass
    offline_errors, offline_warnings = validate_program(prog, known_points, known_programs)
    problems = _controller_problems(prog)
    blocking = [{"source": "offline", "severity": "error", "message": m} for m in offline_errors]
    pending = []
    warnings = [{"source": "offline", "severity": "warning", "message": m} for m in offline_warnings]
    if problems is not None:
        # The controller knows its own step types (and reports unknownStepType itself), so an
        # offline "unknown type" only means this server's schema is behind - don't block on it.
        blocking = [b for b in blocking if "has unknown type" not in b["message"]]
        # The controller reports missing points/routines itself, with step ids; drop the offline copies.
        warnings = [w for w in warnings if "not taught on the robot" not in w["message"]
                    and "does not exist on the robot yet" not in w["message"]]
        for p in problems:
            if p.get("severity") != "error":
                warnings.append(p)
            elif allow_untaught and p.get("code") in PENDING_CODES:
                pending.append(p)
            else:
                blocking.append(p)
    return {"blocking": blocking, "pending": pending, "warnings": warnings,
            "controllerChecked": problems is not None, "problems": problems or []}


def _dropped_fields(sent, back, path):
    """
    Fields sent but missing (or nulled) in the controller's read-back. The controller
    deserialises into typed models, so a misspelt or unknown field is silently dropped -
    the program saves "fine" and the setting just isn't there.
    """
    out = []
    if not (isinstance(sent, dict) and isinstance(back, dict)):
        return out
    for k, v in sent.items():
        if v is None or k == "lastUpdatedUnixMs":
            continue
        if k not in back:
            out.append(f"{path}.{k} (unknown to the controller - misspelt?)")
        elif back[k] is None and v not in ([], {}, ""):
            out.append(f"{path}.{k} (sent {json.dumps(v)[:60]}, read back null - wrong type?)")
        elif k in ("steps", "loopSteps", "ifSteps", "elseSteps") and isinstance(v, list):
            by_id = {s.get("id"): s for s in back[k] or [] if isinstance(s, dict)}
            for i, st in enumerate(v):
                if isinstance(st, dict) and st.get("id") in by_id:
                    out += _dropped_fields(st, by_id[st["id"]], f"{path}.{k}[{i}]")
                elif isinstance(st, dict):
                    out.append(f"{path}.{k}[{i}] (step id {st.get('id')!r} missing after save)")
        elif k == "elseIfBranches" and isinstance(v, list) and isinstance(back[k], list):
            for i, (bs, bb) in enumerate(zip(v, back[k])):
                out += _dropped_fields(bs, bb, f"{path}.{k}[{i}]")
        elif k == "variables" and isinstance(v, list):
            by_name = {x.get("name"): x for x in back[k] or [] if isinstance(x, dict)}
            for i, var in enumerate(v):
                if isinstance(var, dict) and var.get("name") in by_name:
                    out += _dropped_fields(var, by_name[var["name"]], f"{path}.variables[{i}]")
    return out


def _save_checked(prog, args, previous=None):
    """Validate, save, read back. Shared by robot_save_program and robot_modify_program."""
    allow = bool(args.get("allow_untaught"))
    check = _check_program(prog, allow)
    diff = {}
    if previous is not None and check["controllerChecked"]:
        before = _controller_problems(previous) or []
        new, fixed = _problem_diff(before, check["problems"])
        diff = {"newProblems": new, "fixedProblems": fixed}
    common = {"errors": check["blocking"], "pending": check["pending"], "warnings": check["warnings"],
              **diff, "controllerChecked": check["controllerChecked"]}
    if args.get("dry_run"):
        return {"saved": False, "dryRun": True, "wouldSave": not check["blocking"] or bool(args.get("force")), **common}
    if check["blocking"] and not args.get("force"):
        return {
            "saved": False, **common,
            "hint": "Fix the errors and call again. Errors whose code is a missing reference "
                    f"({', '.join(sorted(PENDING_CODES))}) can be saved with allow_untaught=true "
                    "when the point/grid/routine will be taught later. force=true skips every check - "
                    "only when a check is provably wrong.",
        }

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
    # Keep a display page and the like when an edited copy is saved back.
    for k in ("displayPageMode", "displayPageLayout", "displayPageHtml"):
        if prog.get(k) is not None:
            params[k] = prog[k]
    ROBOT.send("SaveBuiltProgram", params)

    back = _get_program(params["name"])
    dropped = _dropped_fields(params, back, "program") if back else ["program (not found after save)"]
    note = "Saved but not run. Use robot_run_program to execute it."
    if dropped:
        note += " Some fields did not survive the save - see droppedFields."
    if check["pending"]:
        note += " Pending items must be taught/created before it runs."
    return {"saved": True, "name": params["name"], "id": params["id"], "steps": len(params["steps"]),
            "forced": bool(check["blocking"]), **common, "droppedFields": dropped, "note": note}


def _get_program(name):
    progs = ROBOT.send("GetBuiltPrograms").get("programs", [])
    return next((p for p in progs if str(p.get("name", "")).lower() == str(name).lower()), None)


def tool_robot_save_program(args):
    prog = args["program"]
    if isinstance(prog, str):
        prog = json.loads(prog)
    previous = None
    try:
        previous = _get_program(prog.get("name", ""))
    except RobotError:
        pass
    if previous is not None and not prog.get("id"):
        prog["id"] = previous.get("id")   # replacing keeps its id, so CallRoutine links survive
    result = _save_checked(prog, args, previous)
    result["replaced"] = previous is not None
    return _ok(result)


# ── Targeted edits ────────────────────────────────────────────────────────────

_CHILD_LISTS = ("loopSteps", "ifSteps", "elseSteps")


def _locate(steps, sid):
    """(containing list, index) of the step with this id anywhere in the tree, or None."""
    for i, st in enumerate(steps or []):
        if not isinstance(st, dict):
            continue
        if st.get("id") == sid:
            return steps, i
        for f in _CHILD_LISTS:
            hit = _locate(st.get(f), sid)
            if hit:
                return hit
        for br in st.get("elseIfBranches") or []:
            hit = _locate(br.get("steps"), sid)
            if hit:
                return hit
    return None


def _need(steps, sid):
    hit = _locate(steps, sid)
    if not hit:
        raise ValueError(f"no step with id {sid!r}")
    return hit


def apply_edit(prog, e):
    """Apply one robot_modify_program edit to a program dict in place; returns a summary line."""
    op = e.get("op")
    steps = prog.setdefault("steps", [])
    if op == "set":                       # merge fields into a step; null removes a field
        lst, i = _need(steps, e["stepId"])
        fields = e.get("fields") or {}
        if "id" in fields:
            raise ValueError("a step's id cannot be changed with set")
        for k, v in fields.items():
            if v is None:
                lst[i].pop(k, None)
            else:
                lst[i][k] = v
        return f"set {sorted(fields)} on {e['stepId']}"
    if op == "replace":                   # swap a step for one or more steps
        lst, i = _need(steps, e["stepId"])
        new = e.get("steps") or ([e["step"]] if e.get("step") else [])
        lst[i:i + 1] = new
        return f"replaced {e['stepId']} with {len(new)} step(s)"
    if op == "insert":                    # after/before a step, or into a parent's list at an index
        new = e.get("steps") or [e["step"]]
        ref = e.get("after") or e.get("before")
        if ref:
            lst, i = _need(steps, ref)
            at = i + 1 if e.get("after") else i
        else:
            if e.get("parentId"):
                pl, pi = _need(steps, e["parentId"])
                field = e.get("list") or "loopSteps"
                if field not in _CHILD_LISTS:
                    raise ValueError(f"list must be one of {', '.join(_CHILD_LISTS)}")
                lst = pl[pi].setdefault(field, [])
            else:
                lst = steps
            at = e.get("index", len(lst))
        lst[at:at] = new
        return f"inserted {len(new)} step(s)"
    if op == "remove":
        lst, i = _need(steps, e["stepId"])
        del lst[i]
        return f"removed {e['stepId']}"
    if op == "setVariable":               # upsert by name (merges fields into an existing one)
        v = e["variable"]
        vars_ = prog.setdefault("variables", [])
        cur = next((x for x in vars_ if x.get("name") == v.get("name")), None)
        if cur:
            cur.update(v)
            return f"updated variable {v.get('name')}"
        vars_.append({"id": uuid.uuid4().hex, **v})
        return f"added variable {v.get('name')}"
    if op == "removeVariable":
        vars_ = prog.get("variables") or []
        kept = [x for x in vars_ if x.get("name") != e["name"]]
        if len(kept) == len(vars_):
            raise ValueError(f"no variable named {e['name']!r}")
        prog["variables"] = kept
        return f"removed variable {e['name']}"
    if op == "setProgram":                # description / isRoutine / isBackground ...
        fields = e.get("fields") or {}
        for k, v in fields.items():
            if k in ("id", "steps", "variables", "name"):
                raise ValueError(f"setProgram cannot change {k}")
            prog[k] = v
        return f"set program {sorted(fields)}"
    raise ValueError(f"unknown op {op!r} (set, replace, insert, remove, setVariable, removeVariable, setProgram)")


def tool_robot_modify_program(args):
    """Apply targeted edits to a saved program, validate the result on the controller, report
    which problems the edit introduced or fixed, and save it (or not, with dry_run)."""
    original = _get_program(args["name"])
    if original is None:
        return _ok({"error": f"No program named '{args['name']}'."})
    prog = json.loads(json.dumps(original))
    applied = []
    for n, e in enumerate(args.get("edits") or []):
        try:
            applied.append(apply_edit(prog, e))
        except (KeyError, ValueError, TypeError) as ex:
            return _ok({"saved": False, "error": f"edits[{n}]: {ex}", "applied": applied,
                        "note": "Nothing was saved."})
    result = _save_checked(prog, args, original)
    result["applied"] = applied
    return _ok(result)


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

    if args.get("check_robot", True):
        # The same gate robot_save_program applies, so "valid" here means "would save".
        check = _check_program(prog, bool(args.get("allow_untaught")))
        if check["controllerChecked"]:
            return _ok({
                "valid": not check["blocking"],
                "source": "controller",
                "errorCount": len(check["blocking"]),
                "warningCount": len(check["warnings"]),
                "errors": check["blocking"],
                "pending": check["pending"],
                "warnings": check["warnings"],
                "problems": check["problems"],
            })
        controller_error = "the controller could not be reached"
    else:
        controller_error = None

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


# ── Config / IO setup ──────────────────────────────────────────────────────
# These set channels up - a Nano pin's role, an HX711 load cell, its calibration.
# They persist to nano_config.json and apply to the device when it is connected,
# but they move nothing and drive no load, so they are ungated like teaching a
# point. Actuating an output is a different thing and stays on robot_set_output.

PIN_TYPES       = ("Input", "Output", "Neopixel", "Unconfigured")
LOAD_CELL_UNITS = ("g", "kg", "oz", "lb", "N")


def _slim_pin(p):
    """One pin, trimmed to what matters for its role (full state is behind full=true)."""
    out = {"pin": p.get("pin"), "type": p.get("type"), "name": p.get("name")}
    t = p.get("type")
    if t == "LoadCell":
        out.update({"sckPin": p.get("sckPin"), "unit": p.get("unit"),
                    "grams": p.get("grams"), "raw": p.get("raw"),
                    "tareOffset": p.get("tareOffset"), "countsPerGram": p.get("countsPerGram"),
                    "sampleRateHz": p.get("sampleRateHz")})
    elif t == "Neopixel":
        out["pixelCount"] = p.get("pixelCount")
    else:
        out["value"] = p.get("value")
    return out


def tool_robot_io(args):
    data  = ROBOT.send("GetIO")
    nanos = data.get("ioDevices") or data.get("nanos") or []
    relay = data.get("relay") or {}

    nano_id = args.get("deviceId") or args.get("nanoId")
    if nano_id:
        match = [n for n in nanos if str(n.get("id")) == nano_id]
        if not match:
            return _ok({"error": f"No Nano with id '{nano_id}'.",
                        "available": [n.get("id") for n in nanos]})
        nanos = match

    if not args.get("full"):
        nanos = [{"id": n.get("id"), "name": n.get("name"), "connected": n.get("connected"),
                  "pins": [_slim_pin(p) for p in (n.get("pins") or [])]}
                 for n in nanos]
    return _ok({"nanos": nanos, "relay": relay})


def tool_robot_configure_pin(args):
    nano_id = args.get("deviceId") or args["nanoId"]
    pin     = int(args["pin"])
    done    = {}

    ptype = args.get("type")
    if ptype is not None:
        if ptype not in PIN_TYPES:
            return _ok({"error": f"type must be one of {', '.join(PIN_TYPES)} (got {ptype!r}). "
                                 f"For an HX711 load cell use robot_load_cell action='configure'."})
        p = {"deviceId": nano_id, "pin": pin, "type": ptype}
        if args.get("pixelCount") is not None:
            p["pixelCount"] = int(args["pixelCount"])
        ROBOT.send("ConfigureIoPin", p)   # sets the pin first, so a rename below finds it
        done["type"] = ptype

    name = args.get("name")
    if name is not None:
        ROBOT.send("RenameIoPin", {"deviceId": nano_id, "pin": pin, "name": name})
        done["name"] = name

    if not done:
        return _ok({"error": "Nothing to do - pass type and/or name."})
    return _ok({"configured": {"nanoId": nano_id, "pin": pin, **done},
                "note": "Saved to nano_config.json and applied to the device. ConfigureNanoPin "
                        "needs the Nano connected; it errors if it is offline."})


def tool_robot_load_cell(args):
    action  = (args.get("action") or "").lower()
    nano_id = args.get("deviceId") or args["nanoId"]

    if action == "configure":
        p = {"deviceId": nano_id, "dtPin": int(args["dtPin"]), "sckPin": int(args["sckPin"])}
        unit = args.get("unit")
        if unit:
            if unit not in LOAD_CELL_UNITS:
                return _ok({"error": f"unit must be one of {', '.join(LOAD_CELL_UNITS)}."})
            p["unit"] = unit
        ROBOT.send("ConfigureLoadCell", p)
        return _ok({"configured": {"nanoId": nano_id, "dtPin": p["dtPin"], "sckPin": p["sckPin"],
                                   "unit": p.get("unit", "g")},
                    "note": "HX711 set up: DT is the data pin, SCK the clock. Saved to nano_config.json "
                            "and CFG'd on the device (queued until it connects). It streams raw counts "
                            "until you action='tare' then action='calibrate' to read grams."})

    if action in ("tare", "calibrate", "unit"):
        pin = int(args["pin"])  # the DT pin
        if action == "tare":
            r = ROBOT.send("TareLoadCell", {"deviceId": nano_id, "pin": pin})
            return _ok({"tared": {"nanoId": nano_id, "pin": pin}, "tareOffset": r.get("tareOffset"),
                        "note": "Zeroed at the current reading. Put a known weight on, then "
                                "action='calibrate' with knownGrams."})
        if action == "calibrate":
            if args.get("knownGrams") is None:
                return _ok({"error": "calibrate needs knownGrams - the weight now on the cell, in grams."})
            r = ROBOT.send("CalibrateLoadCell",
                           {"deviceId": nano_id, "pin": pin, "knownGrams": float(args["knownGrams"])})
            return _ok({"calibrated": {"nanoId": nano_id, "pin": pin},
                        "countsPerGram": r.get("countsPerGram"),
                        "note": "Scale set. Readings now report grams, shown in the display unit."})
        # unit
        unit = args.get("unit")
        if unit not in LOAD_CELL_UNITS:
            return _ok({"error": f"unit must be one of {', '.join(LOAD_CELL_UNITS)}."})
        ROBOT.send("SetLoadCellUnit", {"deviceId": nano_id, "pin": pin, "unit": unit})
        return _ok({"unit": unit, "pin": pin})

    return _ok({"error": "action must be one of: configure, tare, calibrate, unit.",
                "actions": {
                    "configure": "Create/update the HX711 channel. Needs dtPin, sckPin; optional unit. "
                                 "Works offline (persists, applies on connect).",
                    "tare":      "Zero the cell. Needs pin (the DT pin). Nano must be connected.",
                    "calibrate": "Set the scale from a known weight. Needs pin + knownGrams. Nano must be connected.",
                    "unit":      "Change the display unit. Needs pin + unit."}})


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
    elif card in ("nano", "io"):
        nano_id = args.get("deviceId") or args.get("nanoId")
        if not nano_id:
            return _ok({"error": "nanoId is required when card is 'nano' (see robot_status.io.nanos)."})
        cmd, params = "SetIoOutput", {"deviceId": nano_id, "pin": number, "value": value}
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

# ── Text view (Simple Robot Script, docs/program-text.md) ─────────────────────

def _problem_lines(problems):
    """Parse/validation problems as 'line:col severity code: message' strings, plus the raw list."""
    return [f"{p.get('line')}:{p.get('col')} {p.get('severity')} {p.get('code')}: {p.get('message')}"
            for p in problems]


def tool_robot_program_text_get(args):
    name = args["name"]
    prog = _get_program(name)
    if prog is None:
        progs = ROBOT.send("GetBuiltPrograms").get("programs", [])
        return _ok({"error": f"No program named '{name}'.", "available": [p.get("name") for p in progs]})
    res = ROBOT.send("FormatProgramText", {"program": prog,
                                           "includeIds": bool(args.get("include_ids")),
                                           "includeHeader": bool(args.get("header"))})
    return _ok({"name": prog.get("name"), "languageVersion": res.get("languageVersion"), "text": res.get("text", ""),
                "note": "Edit the text and pass it to robot_program_text_set. Step ids are kept by matching; "
                        "@with {...} lines carry fields the text has no words for - leave them as they are."})


def tool_robot_program_text_set(args):
    name = args["name"]
    text = args["text"]
    previous = None
    try:
        previous = _get_program(name)
    except RobotError:
        pass
    parsed = ROBOT.send("ParseProgramText", {"text": text, "base": previous,
                                             "validate": bool(args.get("validate", True))}, keep_ok=True)
    problems = parsed.get("problems") or []
    parse_errors = [p for p in problems if p.get("source") == "parse" and p.get("severity") == "error"]
    if not parsed.get("ok") or parsed.get("program") is None:
        return _ok({"saved": False, "parsed": False, "errors": _problem_lines(parse_errors),
                    "problems": problems,
                    "hint": "Fix the lines above and call again. Lines and columns are 1-based. Nothing was saved."})
    prog = parsed["program"]
    prog["name"] = name
    if not prog.get("id"):
        prog["id"] = (previous or {}).get("id") or uuid.uuid4().hex
    if previous is None:
        for flag in ("isRoutine", "isBackground"):
            arg = "is_routine" if flag == "isRoutine" else "is_background"
            if arg in args:
                prog[flag] = bool(args[arg])
    result = _save_checked(prog, args, previous)
    result["textProblems"] = _problem_lines([p for p in problems if p.get("source") == "parse"])
    result["created"] = previous is None
    return _ok(result)


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
                        "description": "Validate on the robot (default true). false = offline structural checks only."},
        "allow_untaught": {"type": "boolean",
                           "description": "Report missing points/grids/routines as pending rather than errors (as robot_save_program)."}},
       required=["program"], handler=tool_robot_validate_program),

    _t("robot_save_program",
       "Create or replace a built program. Validates on the controller first (its ValidateBuiltProgram, "
       "plus offline structural checks) and refuses to save on errors; missing references "
       "(unknownPoint/Grid/Stack/Tool/Local/Routine/VisionProgram) can be saved with allow_untaught=true "
       "when they will be taught later. After saving it reads the program back and lists droppedFields - "
       "fields the controller did not keep (misspelt or wrong type). When replacing, newProblems/"
       "fixedProblems compare with the old version. Saving does NOT run the program. Replaces the whole "
       "program - to change part of an existing one use robot_modify_program.",
       {"program": {"type": "object",
                    "description": "Program object: {name, description?, steps[], variables?, "
                                   "isRoutine?, isBackground?}. See robot_step_schema."},
        "allow_untaught": {"type": "boolean",
                           "description": "Save even if points/grids/routines it uses do not exist yet (reported as pending)."},
        "dry_run": {"type": "boolean", "description": "Check only; do not save."},
        "force": {"type": "boolean", "description": "Save despite validation errors. Default false."}},
       required=["program"], handler=tool_robot_save_program),
    _t("robot_modify_program",
       "Edit part of a saved program without rewriting it: edits are applied in order, then the result "
       "is validated on the controller like robot_save_program (newProblems/fixedProblems show what the "
       "edit changed) and saved - or not, with dry_run. Edits: "
       "{op:'set', stepId, fields:{...}} (null removes a field); {op:'replace', stepId, steps:[...]}; "
       "{op:'insert', steps:[...], after|before: stepId} or {op:'insert', steps, parentId?, "
       "list?: 'loopSteps'|'ifSteps'|'elseSteps', index?}; {op:'remove', stepId}; "
       "{op:'setVariable', variable:{name,...}} (upsert); {op:'removeVariable', name}; "
       "{op:'setProgram', fields:{description,...}}. Step ids come from robot_get_program.",
       {"name": {"type": "string", "description": "Program to edit."},
        "edits": {"type": "array", "items": {"type": "object"}, "description": "Edits, applied in order."},
        "allow_untaught": {"type": "boolean", "description": "As robot_save_program."},
        "dry_run": {"type": "boolean", "description": "Validate and diff only; do not save."},
        "force": {"type": "boolean", "description": "Save despite validation errors."}},
       required=["name", "edits"], handler=tool_robot_modify_program),
    _t("robot_program_text_get",
       "A built program as Simple Robot Script text (.srs) - the same program the Blocks editor shows, "
       "one statement per step: MoveL Pick1, speed=200 / if $n > 3 ... end / $count = $count + 1. "
       "Names (points, tools, routines, devices) appear instead of ids. Read this, edit the text, and "
       "save it with robot_program_text_set. Reference: docs/program-text.md.",
       {"name": {"type": "string", "description": "Program name."},
        "include_ids": {"type": "boolean", "description": "Write @id \"...\" above every step (default false)."},
        "header": {"type": "boolean", "description": "Add the @program {...} header line (for exporting a file)."}},
       required=["name"], handler=tool_robot_program_text_get),
    _t("robot_program_text_set",
       "Create or replace a built program from Simple Robot Script text. The controller parses it "
       "(problems come back with line:col), reconciles step ids against the saved program so unchanged "
       "steps keep their ids, then the result is validated and saved like robot_save_program "
       "(allow_untaught / dry_run / force behave the same). Saving does NOT run the program. "
       "Write the text the way robot_program_text_get prints it; see docs/program-text.md.",
       {"name": {"type": "string", "description": "Program name (created when it does not exist)."},
        "text": {"type": "string", "description": "The whole program as .srs text."},
        "validate": {"type": "boolean", "description": "Also run the validator and map its problems onto lines (default true)."},
        "is_routine": {"type": "boolean", "description": "New programs only: make it a routine."},
        "is_background": {"type": "boolean", "description": "New programs only: make it a background program."},
        "allow_untaught": {"type": "boolean", "description": "As robot_save_program."},
        "dry_run": {"type": "boolean", "description": "Parse and validate only; do not save."},
        "force": {"type": "boolean", "description": "Save despite validation errors (never despite parse errors)."}},
       required=["name", "text"], handler=tool_robot_program_text_set),
    _t("robot_teach_point",
       "Save the robot's CURRENT position under a name. The robot must already be where you want the "
       "point - this records, it does not move.",
       {"name": {"type": "string", "description": "Point name. Reusing a name overwrites it."}},
       required=["name"], handler=tool_robot_teach_point),

    _t("robot_stop",
       "Stop everything now: halts the running program, hard-stops motion, cancels any jog. "
       "Always available, even when the motion gate is closed.",
       {}, handler=tool_robot_stop),

    # ── Config / IO setup (ungated - sets channels up, moves nothing) ──────────
    _t("robot_io",
       "Read the IO config + live state: every Nano (id, connected, pins) and the USB relay board. "
       "Start here before configuring - it gives the nanoId and current pin roles you configure "
       "against. Trimmed per pin role by default; full=true for every field.",
       {"nanoId": {"type": "string", "description": "Only this Nano (default: all)."},
        "full":   {"type": "boolean", "description": "Every pin field instead of the role-trimmed set."}},
       handler=tool_robot_io),

    _t("robot_configure_pin",
       "Set a Nano pin's role and/or name. type is Input, Output, Neopixel or Unconfigured "
       "(for an HX711 load cell use robot_load_cell instead). Persists to nano_config.json and applies "
       "to the device - the Nano must be connected or it errors. Does NOT actuate the pin; "
       "robot_set_output drives an output's value.",
       {"nanoId":     {"type": "string"},
        "pin":        {"type": "integer", "description": "Nano pin number (Arduino D-pin)."},
        "type":       {"type": "string", "description": "Input | Output | Neopixel | Unconfigured."},
        "name":       {"type": "string", "description": "Friendly label for the pin."},
        "pixelCount": {"type": "integer", "description": "Neopixel only: number of pixels."}},
       required=["nanoId", "pin"], handler=tool_robot_configure_pin),

    _t("robot_load_cell",
       "Set up or calibrate an HX711 load cell on a Nano. action='configure' creates the channel "
       "(dtPin = data, sckPin = clock; works offline and applies on connect); action='tare' zeroes it; "
       "action='calibrate' sets the scale from a known weight (knownGrams); action='unit' changes the "
       "display unit. tare/calibrate need the Nano connected and streaming.",
       {"action":     {"type": "string", "description": "configure | tare | calibrate | unit."},
        "nanoId":     {"type": "string"},
        "dtPin":      {"type": "integer", "description": "configure: HX711 DT (data) pin."},
        "sckPin":     {"type": "integer", "description": "configure: HX711 SCK (clock) pin."},
        "pin":        {"type": "integer", "description": "tare/calibrate/unit: the DT pin of the cell."},
        "knownGrams": {"type": "number",  "description": "calibrate: weight currently on the cell, in grams."},
        "unit":       {"type": "string",  "description": "Display unit: g, kg, oz, lb or N."}},
       required=["action", "nanoId"], handler=tool_robot_load_cell),

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
