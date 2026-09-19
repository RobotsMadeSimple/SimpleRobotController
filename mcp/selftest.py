"""
Round-trip check against a live controller: save a throwaway program, read it
back, confirm nesting and interpolation survived, then delete it again.

Writes one program named _mcp_selftest and removes it. Moves nothing, and does
not need the motion gate.
"""
import json
import sys

import robot_mcp as r

NAME = "_mcp_selftest"

PROGRAM = {
    "name": NAME,
    "description": "temporary round-trip check",
    "variables": [{"id": "v1", "name": "i", "value": 0, "displayOnMonitor": True}],
    "steps": [
        {"id": "a1", "type": "StatusUpdate", "statusMessage": "Cycle {$i + 1} starting"},
        {"id": "a2", "type": "Loop", "loopMode": "count", "loopCount": 2, "loopSteps": [
            {"id": "a3", "type": "Wait", "waitMs": 100},
            {"id": "a4", "type": "SetVariable", "variableName": "i", "variableExpr": "$i + 1"},
        ]},
    ],
}


def main():
    failures = []

    saved = json.loads(r.call_tool("robot_save_program", {"program": PROGRAM})[0])
    print("save ->", json.dumps(saved))
    if not saved.get("saved"):
        print("FAIL: save refused")
        return 1

    back = json.loads(r.call_tool("robot_get_program", {"name": NAME})[0])
    checks = [
        ("name",         back.get("name") == NAME),
        ("step count",   len(back.get("steps", [])) == 2),
        ("variables",    [v["name"] for v in back.get("variables") or []] == ["i"]),
        ("nested loop",  [s["type"] for s in back["steps"][1].get("loopSteps") or []]
                         == ["Wait", "SetVariable"]),
        ("interpolation", back["steps"][0].get("statusMessage") == "Cycle {$i + 1} starting"),
        ("expr",         back["steps"][1]["loopSteps"][1].get("variableExpr") == "$i + 1"),
    ]
    for label, ok in checks:
        print(f"  {'ok  ' if ok else 'FAIL'} {label}")
        if not ok:
            failures.append(label)

    # Cleanup goes straight down the link rather than through robot_raw_command,
    # so the test does not need the motion gate open just to tidy up after itself.
    r.ROBOT.send("DeleteBuiltProgram", {"name": NAME})
    remaining = [p.get("name") for p in r.ROBOT.send("GetBuiltPrograms")["programs"]]
    cleaned = NAME not in remaining
    print(f"  {'ok  ' if cleaned else 'FAIL'} cleaned up")
    if not cleaned:
        failures.append("cleanup")

    print("FAILURES:", failures or "none")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
