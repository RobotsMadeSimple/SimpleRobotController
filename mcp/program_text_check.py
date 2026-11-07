"""
Live check of the text tools against a running controller (moves nothing):
robot_program_text_get prints every program and robot_program_text_set reads it back
unchanged (dry run), an edit made in the text keeps the step ids, and parse problems come
back with line:col. Writes and deletes `_mcp_text_check`.

    RMS_ROBOT_URL=ws://127.0.0.1:9000/control python program_text_check.py
"""
import json
import sys

import robot_mcp as r

NAME = "_mcp_text_check"
failures = []

TEXT = """#!srs 1
var n = 0

## Count up
repeat 3 as $i
  $n = $n + 1
  Wait 10ms
end
status "n = {$n}"
"""


def check(cond, what):
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        failures.append(what)


def call(tool, **args):
    return json.loads(getattr(r, tool)(args))


def main():
    # Every existing program prints, and reads back without a parse error.
    progs = call("tool_robot_programs")
    bad = []
    for p in progs:
        t = call("tool_robot_program_text_get", name=p["name"])
        res = call("tool_robot_program_text_set", name=p["name"], text=t["text"], dry_run=True, allow_untaught=True)
        if res.get("parsed") is False:
            bad.append((p["name"], res.get("errors")))
    check(not bad, f"all {len(progs)} programs format and parse back" + (f": {bad[:2]}" if bad else ""))

    try:
        res = call("tool_robot_program_text_set", name=NAME, text=TEXT)
        check(res.get("saved") is True and res.get("created") is True, "a new program saves from text")
        saved = r._get_program(NAME)
        check(saved is not None and len(saved["steps"]) == 3, "it has the note, the loop and the status")

        text = call("tool_robot_program_text_get", name=NAME)["text"]
        check("repeat 3 as $i" in text and '## Count up' in text, "it prints back as written")

        res = call("tool_robot_program_text_set", name=NAME,
                   text=text.replace("repeat 3", "repeat 5").replace('status "n = {$n}"', 'status "n = {$n}"\npause'))
        after = r._get_program(NAME)
        check(res.get("saved") is True, "an edit saves")
        check([s["id"] for s in after["steps"][:3]] == [s["id"] for s in saved["steps"]], "the edit keeps the step ids")
        check(after["steps"][1]["loopSteps"][0]["id"] == saved["steps"][1]["loopSteps"][0]["id"], "nested ids are kept too")
        check(after["steps"][1].get("loopCount") == 5, "the edit is applied")

        res = call("tool_robot_program_text_set", name=NAME, text="MoveL here, sped=3\nrepeat 2\n")
        check(res.get("saved") is False and any(e.startswith("1:13 error unknownArgument") for e in res.get("errors", [])),
              "a parse error comes back with its position and nothing is saved")
    finally:
        try:
            r.ROBOT.send("DeleteBuiltProgram", {"name": NAME})
        except r.RobotError:
            pass

    print(f"\n{len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
