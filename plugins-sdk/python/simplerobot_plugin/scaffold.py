"""``python -m simplerobot_plugin new <id>``: create a starter plugin folder."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Dict

from .manifest import RESERVED_IDS, validate_manifest

_MAIN = '''"""{name}: a SimpleRobotController plugin."""
from simplerobot_plugin import Plugin, StepError

plugin = Plugin()  # reads SRC_PLUGIN_* from the environment


@plugin.on_ready
async def ready(ctx):
    ctx.log(f"{{ctx.plugin_id}} ready (controller {{ctx.controller_version}})")


@plugin.step("hello")
async def hello(ctx, params):
    who = params.get("who") or "world"
    if who == "fail":
        raise StepError("asked to fail", code="helloFailed")
    ctx.progress("greeting", percent=50)
    return {{"greeting": f"Hello, {{who}}!"}}


if __name__ == "__main__":
    plugin.run()
'''

_README = '''# {name}

A SimpleRobotController plugin created with `python -m simplerobot_plugin new {id}`.

## Develop

1. In the app, open Plugins, install nothing yet. Instead set `"runtime": "external"` in `plugin.json`
   and copy the plugin's connect token and URL from the plugin detail page.
2. `pip install -r requirements.txt`
3. `SRC_PLUGIN_ID={id} SRC_PLUGIN_URL=ws://<controller>:<port>/plugin SRC_PLUGIN_TOKEN=<token> python main.py`

## Package

Zip the folder (without `.venv`) so `plugin.json` is at the root or in one top-level folder, then
install it from the app or `POST /plugins/install` (`Content-Type: application/zip`).
Restore `"runtime": "python"` before packaging.
'''


def scaffold(plugin_id: str, directory: Path) -> Path:
    """Create ``<directory>/<plugin_id>/`` and return its path. Raises ``ValueError`` on bad input."""
    if plugin_id in RESERVED_IDS:
        raise ValueError(f"'{plugin_id}' is a reserved expression root")
    name = plugin_id.replace("_", " ").title()
    manifest: Dict[str, object] = {
        "id": plugin_id,
        "name": name,
        "version": "0.1.0",
        "description": f"{name} plugin",
        "author": "",
        "protocolVersion": 1,
        "runtime": "python",
        "entry": "main.py",
        "python": {"minVersion": "3.9", "requirements": "requirements.txt"},
        "autoStart": True,
        "restart": {"mode": "always", "maxRestarts": 5, "backoffMs": 2000},
        "steps": [
            {
                "id": "hello",
                "label": "Say hello",
                "description": "Example step: builds a greeting.",
                "params": [{"key": "who", "label": "Who", "type": "string", "default": "world"}],
                "outputs": [{"key": "greeting", "label": "Greeting", "type": "string"}],
                "cancellable": True,
            }
        ],
        "functions": [],
        "properties": [],
    }
    problems = validate_manifest(manifest)
    if problems:
        raise ValueError("; ".join(problems))
    target = directory / plugin_id
    if target.exists() and any(target.iterdir()):
        raise ValueError(f"{target} already exists and is not empty")
    target.mkdir(parents=True, exist_ok=True)
    (target / "plugin.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (target / "main.py").write_text(_MAIN.format(name=name), encoding="utf-8")
    (target / "requirements.txt").write_text(
        "# Pin to a release, or use a path install while developing, e.g.\n"
        "#   simplerobot-plugin @ file:///absolute/path/to/plugins-sdk/python\n"
        "simplerobot-plugin>=0.1.0\n",
        encoding="utf-8",
    )
    (target / "README.md").write_text(_README.format(name=name, id=plugin_id), encoding="utf-8")
    return target
