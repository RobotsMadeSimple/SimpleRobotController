"""A small ``plugin.json`` validator mirroring docs/plugins.md section 2 (the controller is authoritative)."""
from __future__ import annotations

import re
from typing import Any, Dict, List

RESERVED_IDS = {"robot", "program", "time", "aux", "camera", "stb", "relay", "nano", "plugin", "global", "local", "list", "time_ms"}

_ID = re.compile(r"^[a-z][a-z0-9_]{1,31}$")
_STEP_ID = re.compile(r"^[a-z][A-Za-z0-9_]{0,31}$")
_NAME = re.compile(r"^[a-z][A-Za-z0-9_]{0,31}$")
_RUNTIMES = ("python", "dotnet", "exe", "external")
_CONFIG_TYPES = ("string", "password", "number", "boolean", "enum")
_PARAM_TYPES = ("number", "boolean", "string", "enum", "point", "list", "image", "variable")
_OUTPUT_TYPES = ("number", "boolean", "string", "point", "list", "image")


def validate_manifest(m: Dict[str, Any]) -> List[str]:
    """Return a list of problems (empty when valid)."""
    problems: List[str] = []
    pid = m.get("id")
    if not isinstance(pid, str) or not _ID.match(pid):
        problems.append("id must match ^[a-z][a-z0-9_]{1,31}$")
    elif pid in RESERVED_IDS:
        problems.append(f"id '{pid}' is a reserved expression root")
    if m.get("protocolVersion") != 1:
        problems.append("protocolVersion must be 1")
    runtime = m.get("runtime")
    if runtime not in _RUNTIMES:
        problems.append(f"runtime must be one of {_RUNTIMES}")
    elif runtime != "external" and not m.get("entry"):
        problems.append("entry is required unless runtime is external")
    entry = m.get("entry")
    if isinstance(entry, str) and (entry.startswith(("/", "\\")) or ".." in entry.replace("\\", "/").split("/")):
        problems.append("entry must stay inside the plugin folder")
    for key in ("name", "version"):
        if not isinstance(m.get(key), str) or not m.get(key):
            problems.append(f"{key} is required")

    for i, f in enumerate(m.get("configSchema") or []):
        if f.get("type") not in _CONFIG_TYPES:
            problems.append(f"configSchema[{i}].type invalid")
        if not f.get("key"):
            problems.append(f"configSchema[{i}].key missing")
        if f.get("type") == "enum" and not f.get("options"):
            problems.append(f"configSchema[{i}] enum needs options")

    seen = set()
    for i, s in enumerate(m.get("steps") or []):
        sid = s.get("id")
        if not isinstance(sid, str) or not _STEP_ID.match(sid):
            problems.append(f"steps[{i}].id invalid")
        elif sid in seen:
            problems.append(f"steps[{i}].id '{sid}' duplicated")
        seen.add(sid)
        for j, p in enumerate(s.get("params") or []):
            if p.get("type") not in _PARAM_TYPES:
                problems.append(f"steps[{i}].params[{j}].type invalid")
            if p.get("type") == "enum" and not p.get("options"):
                problems.append(f"steps[{i}].params[{j}] enum needs options")
        for j, o in enumerate(s.get("outputs") or []):
            if o.get("type") not in _OUTPUT_TYPES:
                problems.append(f"steps[{i}].outputs[{j}].type invalid")

    for i, fn in enumerate(m.get("functions") or []):
        if not isinstance(fn.get("name"), str) or not _NAME.match(fn["name"]):
            problems.append(f"functions[{i}].name invalid")
        t = fn.get("timeoutMs")
        if t is not None and not (0 < t <= 5000):
            problems.append(f"functions[{i}].timeoutMs must be 1..5000")
    for i, p in enumerate(m.get("properties") or []):
        if not isinstance(p.get("name"), str) or not _NAME.match(p["name"]):
            problems.append(f"properties[{i}].name invalid")
        if p.get("type") not in ("number", "boolean"):
            problems.append(f"properties[{i}].type must be number or boolean")
    return problems
