"""
Live check of the program-authoring gate against a running controller (moves nothing):
robot_save_program blocks on controller errors, saves missing references with
allow_untaught, reports fields the controller dropped, and robot_modify_program applies
targeted edits with a before/after problem diff. Writes and deletes `_mcp_program_check`.

    RMS_ROBOT_URL=ws://127.0.0.1:9000/control python program_tools_check.py
"""
import json
import sys

import robot_mcp as r

NAME = "_mcp_program_check"
failures = []


def check(cond, what):
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        failures.append(what)


def call(tool, **args):
    return json.loads(getattr(r, tool)(args))


def base():
    return {
        "name": NAME,
        "variables": [{"id": "v1", "name": "n", "value": 0}],
        "steps": [
            {"id": "s1", "type": "SetVariable", "variableName": "n", "variableExpr": "$n + 1"},
            {"id": "s2", "type": "Loop", "loopMode": "count", "loopCount": 2, "loopSteps": [
                {"id": "s3", "type": "Wait", "waitMs": 10},
            ]},
        ],
    }


def main():
    # The offline schema must know every step type the controller does.
    import os, re
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "RobotControl", "Models", "ProgramSteps.cs"),
               encoding="utf8").read()
    enum = [t.strip() for t in re.search(r"public enum StepType \{([^}]*)\}", src).group(1).split(",")]
    missing = [t for t in enum if t and t != "Unknown" and t not in r.step_schema.STEPS]
    check(not missing, f"step_schema covers every StepType (missing: {missing})")

    # Offline edit engine.
    p = base()
    r.apply_edit(p, {"op": "set", "stepId": "s3", "fields": {"waitMs": 50}})
    r.apply_edit(p, {"op": "insert", "after": "s1", "steps": [{"id": "s4", "type": "Wait", "waitMs": 5}]})
    r.apply_edit(p, {"op": "insert", "parentId": "s2", "list": "loopSteps", "index": 0,
                     "steps": [{"id": "s5", "type": "Wait", "waitMs": 1}]})
    r.apply_edit(p, {"op": "remove", "stepId": "s4"})
    r.apply_edit(p, {"op": "setVariable", "variable": {"name": "m", "value": 2}})
    check([s["id"] for s in p["steps"]] == ["s1", "s2"] and [s["id"] for s in p["steps"][1]["loopSteps"]] == ["s5", "s3"]
          and p["steps"][1]["loopSteps"][1]["waitMs"] == 50 and any(v["name"] == "m" for v in p["variables"]),
          "apply_edit: set / insert after / insert into a loop / remove / setVariable")
    try:
        r.apply_edit(p, {"op": "remove", "stepId": "nope"})
        check(False, "apply_edit rejects an unknown step id")
    except ValueError:
        check(True, "apply_edit rejects an unknown step id")

    # A controller-detected error blocks the save.
    bad = base()
    bad["steps"][0]["variableExpr"] = "$n +* 1"
    res = call("tool_robot_save_program", program=bad)
    check(res["saved"] is False and res["controllerChecked"] and any(e.get("code") == "expressionSyntax" for e in res["errors"]),
          "save blocked by a controller expressionSyntax error")

    # A missing point blocks, unless allow_untaught - then it saves as pending.
    pt = base()
    pt["steps"].append({"id": "m1", "type": "MoveL", "pointName": "_mcp_no_such_point"})
    res = call("tool_robot_save_program", program=pt)
    check(res["saved"] is False and any(e.get("code") == "unknownPoint" for e in res["errors"]), "missing point blocks by default")
    res = call("tool_robot_save_program", program=pt, allow_untaught=True)
    check(res["saved"] is True and any(p.get("code") == "unknownPoint" for p in res["pending"]), "allow_untaught saves it as pending")

    # A misspelt field is saved "fine" by the controller but dropped - reported.
    typo = base()
    typo["steps"][1]["loopSteps"][0]["waitMss"] = 99
    res = call("tool_robot_save_program", program=typo)
    check(res["saved"] is True and any("waitMss" in d for d in res["droppedFields"]), "dropped misspelt field reported")
    check(res["replaced"] is True, "replacing reports replaced=true")

    # Modify: dry run shows the problem an edit introduces; a fixing edit saves.
    res = call("tool_robot_modify_program", name=NAME, dry_run=True,
               edits=[{"op": "set", "stepId": "s1", "fields": {"variableExpr": "$missing + 1"}}])
    check(res.get("dryRun") and any(p.get("code") == "unknownVariable" for p in res.get("newProblems", [])),
          "modify dry_run lists the new unknownVariable problem")
    res = call("tool_robot_modify_program", name=NAME,
               edits=[{"op": "set", "stepId": "s3", "fields": {"waitMss": None, "waitMs": 20}},
                      {"op": "insert", "after": "s1", "steps": [{"id": "s6", "type": "StatusUpdate", "statusMessage": "n={$n}"}]}])
    back = r._get_program(NAME)
    ids = [s["id"] for s in back["steps"]]
    check(res["saved"] is True and ids[:2] == ["s1", "s6"] and back["steps"][2]["loopSteps"][0]["waitMs"] == 20,
          "modify saves targeted edits")
    res = call("tool_robot_modify_program", name=NAME, edits=[{"op": "remove", "stepId": "zzz"}])
    check(res["saved"] is False and "edits[0]" in res.get("error", ""), "modify with a bad step id saves nothing")

    r.ROBOT.send("DeleteBuiltProgram", {"name": NAME})
    print("\n" + ("ALL PASSED" if not failures else f"{len(failures)} FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
