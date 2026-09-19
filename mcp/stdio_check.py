"""
Drives robot_mcp.py as a subprocess over stdio, exactly the way an MCP client
does, and exercises the whole ungated tool surface against a live controller.

This is the end-to-end test: calling the tool functions in-process skips the
JSON-RPC framing, the subprocess env, and the argument marshalling, which is
where a client-visible break would actually live.

    python stdio_check.py [host]        # default localhost

Safety: this never moves the robot. The gated tools are checked for refusal with
the gate CLOSED. The gate-OPEN check calls robot_move with no arguments, which
returns an argument error from inside the handler - proving the gate opened
without a motion command ever reaching the controller.
"""
import json
import os
import subprocess
import sys

HERE   = os.path.dirname(os.path.abspath(__file__))
SERVER = os.path.join(HERE, "robot_mcp.py")


class Client:
    """Minimal MCP client: spawn the server, speak JSON-RPC over its stdio."""

    def __init__(self, env_extra):
        env = dict(os.environ)
        env.update(env_extra)
        env["PYTHONIOENCODING"] = "utf-8"
        self.proc = subprocess.Popen(
            [sys.executable, SERVER],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, encoding="utf-8", bufsize=1, env=env,
        )
        self._id = 0

    def rpc(self, method, params=None):
        self._id += 1
        self.proc.stdin.write(json.dumps(
            {"jsonrpc": "2.0", "id": self._id, "method": method, "params": params or {}}) + "\n")
        self.proc.stdin.flush()
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("server closed the pipe")
            msg = json.loads(line)
            if msg.get("id") == self._id:
                if "error" in msg:
                    raise RuntimeError(msg["error"])
                return msg["result"]

    # Arguments go in as a dict rather than **kwargs: several tools take an
    # argument called `name`, which would collide with the parameter.
    def call(self, tool, args=None):
        r = self.rpc("tools/call", {"name": tool, "arguments": args or {}})
        return r["content"][0]["text"], r.get("isError", False)

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()


RESULTS = []


def check(label, ok, detail=""):
    RESULTS.append((label, ok))
    print(f"  {'ok  ' if ok else 'FAIL'} {label}{('  — ' + detail) if detail else ''}")
    return ok


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    env = {"RMS_ROBOT_HOST": host, "RMS_MCP_ALLOW_MOTION": "0"}
    print(f"== server against {host}, motion gate CLOSED ==")

    c = Client(env)
    try:
        init = c.rpc("initialize", {"protocolVersion": "2024-11-05", "capabilities": {}})
        check("initialize", init["serverInfo"]["name"] == "simple-robot-controller")

        tools = c.rpc("tools/list")["tools"]
        check("tools/list", len(tools) == 14, f"{len(tools)} tools")
        check("no private keys leak", not any(k.startswith("_") for t in tools for k in t))

        # ── reads ────────────────────────────────────────────────────────────
        text, err = c.call("robot_status")
        status = json.loads(text)
        check("robot_status", not err and "wasHomed" in status)
        check("endpoint is the host we asked for",
              status.get("_endpoint") == f"ws://{host}:9000/control", status.get("_endpoint", ""))
        check("gate reported closed", "closed" in status.get("_motionGate", ""))

        text, err = c.call("robot_points")
        pts = json.loads(text)
        check("robot_points", not err and "names" in pts, f"{pts.get('count')} points")

        text, err = c.call("robot_programs")
        progs = json.loads(text)
        check("robot_programs", not err and isinstance(progs, list), f"{len(progs)} programs")

        if progs:
            name = progs[0]["name"]
            text, err = c.call("robot_get_program", {"name": name})
            full = json.loads(text)
            check("robot_get_program", not err and full.get("name") == name, name)

        text, err = c.call("robot_step_schema")
        check("robot_step_schema (index)", not err and "stepTypes" in json.loads(text))
        text, err = c.call("robot_step_schema", {"types": ["MoveL", "Loop"]})
        check("robot_step_schema (types)", not err and "pointNameExpr" in json.loads(text)["MoveL"]["fields"])

        text, err = c.call("robot_raw_command", {"command": "GetRobotInfo"})
        check("robot_raw_command (Get* ungated)", not err, text.replace("\n", " ")[:70])

        # ── validation ───────────────────────────────────────────────────────
        text, err = c.call("robot_validate_program", {"program": {
            "name": "lint me",
            "steps": [{"id": "x", "type": "MoveL", "expressions": {"speed": "baseSpeed * 2"}}],
        }})
        v = json.loads(text)
        check("validator flags a missing $", not v["valid"] and
              any("without $" in e for e in v["errors"]))
        check("validator warns, not errors, on an untargeted move",
              any("no target" in w for w in v["warnings"]))

        # ── write round trip ─────────────────────────────────────────────────
        prog = {
            "name": "_mcp_stdio_check",
            "variables": [{"id": "v1", "name": "i", "value": 0}],
            "steps": [
                {"id": "s1", "type": "StatusUpdate", "statusMessage": "at {$i + 1}"},
                {"id": "s2", "type": "Loop", "loopMode": "count", "loopCount": 2, "loopSteps": [
                    {"id": "s3", "type": "Wait", "waitMs": 50},
                    {"id": "s4", "type": "SetVariable", "variableName": "i", "variableExpr": "$i + 1"},
                ]},
            ],
        }
        text, err = c.call("robot_save_program", {"program": prog})
        check("robot_save_program", not err and json.loads(text).get("saved") is True)

        text, err = c.call("robot_get_program", {"name": "_mcp_stdio_check"})
        back = json.loads(text)
        check("round trip: nesting", [s["type"] for s in back["steps"][1]["loopSteps"]]
              == ["Wait", "SetVariable"])
        check("round trip: interpolation survives", back["steps"][0]["statusMessage"] == "at {$i + 1}")

        # ── the gate ─────────────────────────────────────────────────────────
        for tool, args in [("robot_move", {"name": "Home"}), ("robot_home", {}),
                           ("robot_run_program", {"name": "_mcp_stdio_check"}),
                           ("robot_set_output", {"number": 1, "value": True})]:
            text, err = c.call(tool, args)
            check(f"{tool} refused by the gate", err and "Blocked" in text)

        text, err = c.call("robot_raw_command", {"command": "SetSpeedOverride", "params": {"percent": 50}})
        check("robot_raw_command refuses a non-Get", "Blocked" in text)

        # robot_stop must work regardless of the gate. The robot is idle, so this is a no-op.
        text, err = c.call("robot_stop")
        stop = json.loads(text)
        check("robot_stop works with the gate closed",
              not err and all(v == "sent" for v in stop.values()), str(stop))
    finally:
        c.close()

    # ── gate OPEN, without moving anything ───────────────────────────────────
    print("\n== motion gate OPEN (no motion issued) ==")
    c = Client({**env, "RMS_MCP_ALLOW_MOTION": "1"})
    try:
        c.rpc("initialize", {"protocolVersion": "2024-11-05", "capabilities": {}})
        text, err = c.call("robot_status")
        check("gate reported open", "open" == json.loads(text).get("_motionGate"))
        # No target given: the handler returns an argument error before it sends anything,
        # which proves the gate let the call through without commanding a move.
        text, err = c.call("robot_move")
        check("robot_move passes the gate, then rejects empty args",
              "Blocked" not in text and "Give either name=" in text)

        # Cleanup needs a non-Get command, hence the open gate.
        c.call("robot_raw_command", {"command": "DeleteBuiltProgram", "params": {"name": "_mcp_stdio_check"}})
        text, _ = c.call("robot_programs")
        check("cleaned up",
              "_mcp_stdio_check" not in [p["name"] for p in json.loads(text)])
    finally:
        c.close()

    failed = [label for label, ok in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
