# simplerobot-plugin

Python SDK for SimpleRobotController plugins. Contract: `docs/plugins.md` (this SDK is section 8.1).
Python 3.9+, asyncio, one dependency (`websockets>=11`).

## Install

```
pip install ./plugins-sdk/python          # from a repo checkout
pip install -e "./plugins-sdk/python[test]"   # editable + pytest
```

In a plugin's `requirements.txt` use a pin (`simplerobot-plugin==0.1.0`) or a path/URL install
(`simplerobot-plugin @ file:///abs/path/to/plugins-sdk/python`). The controller creates the venv and runs
`pip install -r requirements.txt` for you.

Scaffold a new plugin: `python -m simplerobot_plugin new my_plugin [--dir .]` creates `plugin.json`,
`main.py`, `requirements.txt` and `README.md`.

## Hello world

`plugin.json` declares what you contribute (see `docs/plugins.md` section 2); `main.py` implements it:

```python
from simplerobot_plugin import Plugin, StepError

plugin = Plugin()                     # reads SRC_PLUGIN_ID / _URL / _TOKEN / _DIR, SRC_DATA_DIR, SRC_CONTROLLER_VERSION

@plugin.step("hello")
async def hello(ctx, params):
    return {"greeting": f"Hello {params['who']}"}

plugin.run()
```

`plugin.run()` blocks: connects, sends `plugin.ready` first (with the token and `{sdk:{name,version}}`), serves
requests concurrently (every inbound request runs in its own task), reconnects after socket loss with exponential
backoff 1 s to 30 s, and returns after replying to `shutdown`. It exits non-zero when the controller rejects the token
(close 4401, not retried; exit code 1), when another connection replaced it (4409, exit 3) or when the URL/token are
missing (exit 2). When the controller process disappears (it was force-killed; the SDK watches
`SRC_PARENT_PID` every 2 s) it cancels its background tasks and exits with code 4 ("parent exited");
pass `watch_parent=False` to disable or `parent_pid=` to override. `await plugin.run_async()` returns the exit code instead of exiting.

Handlers may be plain functions or coroutines. A plain function that blocks stalls the event loop, so use
`await asyncio.to_thread(...)` for blocking I/O. Exceptions in event/background/ready handlers are logged (`ctx.log`
level `error`) and do not stop the plugin.

## Decorators

| Decorator | Handler | Notes |
|---|---|---|
| `@plugin.on_ready` | `fn(ctx)` | After every successful handshake (also after reconnects). Properties you set earlier are re-published automatically. |
| `@plugin.step("id")` | `fn(ctx, params) -> dict \| None` | `ctx` is a `StepContext`. Return outputs (`None` is `{}`). `raise StepError(message, code="stepFailed")` fails the step: `ok:false`, `error=code`. Any other exception: `ok:false, error:"exception", message=repr(e)`. |
| `@plugin.function("name")` | `fn(ctx, *args) -> number \| bool` | Called from expressions as `<id>.name(...)`; keep it fast (manifest `timeoutMs`, default 250 ms). Bools become 1/0. |
| `@plugin.on("program.started")` | `fn(ctx, event)` | `event` is a dict of the event data with `event.name` set. `*` suffix globs work (`"robot.*"`). Events are delivered in order, one at a time. |
| `@plugin.background` | `async fn(ctx)` | Started after ready, cancelled on disconnect/shutdown, restarted after reconnect. A crash is logged, not restarted. |
| `@plugin.on_config_changed` | `fn(ctx, config)` | After the user saves the config form. `ctx.config` is updated first. |

Constructor: `Plugin(url=None, token=None, plugin_id=None, plugin_dir=None, data_dir=None, controller_version=None,
manifest_override=None, echo_logs=False, reconnect_min=1.0, reconnect_max=30.0, ready_timeout=15.0)`.
Keyword arguments override the environment. `manifest_override` (dict, or path to a JSON file) is sent in
`plugin.ready` and replaces the manifest's `steps/functions/properties` while developing.

### `ctx`

| Member | |
|---|---|
| `config`, `controller_version`, `plugin_id`, `plugin_dir`, `data_dir` | read-only (`plugin_dir` / `data_dir` are `pathlib.Path`) |
| `log(message, level="info")` | `debug`/`info`/`warn`/`error`; goes to the plugin log (stderr while disconnected) |
| `set_properties(weight=1.2)` / `set_properties({"weight": 1.2})` | live `$<id>.weight`; numbers/bools only are kept by the controller |
| `clear_properties()` | all properties become unknown |
| `set_status("ok"\|"degraded"\|"error", message=None)` | shown on the plugin card |
| `progress(message=None, percent=None)` | steps only (carries the `invocationId`); elsewhere raises `RuntimeError` |
| `await command("SetSTBOutput", pin=1, value=True)` | any command of `docs/websocket-api.md`; returns the ack dict or raises `CommandError(code, message)` |
| `await get_variables(program=None)` | `{"variables": {...}, "lists": {...}, "strings": {...}}` |
| `await set_variables({"x": 1}, program=None)` | computed variables raise `CommandError("computedVariable")` |
| `await subscribe("status", position_interval_ms=50, status_interval_ms=None, io_interval_ms=None)` | merged with the events used in `@plugin.on`; remembered across reconnects |
| `is_cancelled`, `await cancelled()`, `cancel_reason`, `invocation_id` | `StepContext` only; set by `step.cancel` (`stopped`, `reset`, `timeout`, `shutdown`) |

Events used in `@plugin.on(...)` are subscribed automatically at ready. Cancellation does not interrupt your
coroutine: check `ctx.is_cancelled` or `await ctx.cancelled()` (e.g. `asyncio.wait`) and return early; the
controller discards a late reply.

## Values as the plugin sees them (docs/plugins.md section 6)

Step `params` (already resolved by the controller; absent keys use the manifest default):

| Param type | Python value |
|---|---|
| `number` | `int`/`float` |
| `boolean` | `bool` |
| `string`, `enum` | `str` |
| `point` | `{"x","y","z","rx","ry","rz"}` dict |
| `list` | list of numbers, point dicts or record dicts |
| `image` | base64 JPEG `str` (`base64.b64decode`) |
| `variable` | the raw variable name (`str`); use `ctx.get_variables` / `ctx.set_variables` |

Step outputs you return (`{key: value}`; missing keys are skipped silently; type mismatch fails the program):

| Output type | Return |
|---|---|
| `number` | number |
| `boolean` | `bool` or number (written as 0/1) |
| `string` | `str` |
| `point` | `{"x","y","z","rx","ry","rz"}` or a list of 6 numbers |
| `list` | list (numbers, point dicts or record dicts) |
| `image` | base64 JPEG `str` |

Properties are numbers (booleans become 0/1). Function arguments are numbers and the result is a number or bool.

## Development mode

Set `"runtime": "external"` in `plugin.json` (nothing is launched), install the folder (or just create it) and copy
the token and URL from the plugin detail page, then run it yourself:

```
SRC_PLUGIN_ID=my_plugin SRC_PLUGIN_URL=ws://127.0.0.1:<port>/plugin SRC_PLUGIN_TOKEN=<token> python main.py
```

or, without the environment, `Plugin(url="ws://...", token="...", plugin_id="my_plugin", echo_logs=True)`.
`Plugin(manifest_override={...})` lets you iterate on steps/functions/properties without reinstalling.
Restore `"runtime": "python"` before packaging.

## Packaging

Zip the plugin folder (without `.venv`, `__pycache__`, `plugin.log`) so that `plugin.json` is at the zip root or inside
one top-level folder, pin the SDK in `requirements.txt`, then:

```
curl -X POST -H "Content-Type: application/zip" --data-binary @my_plugin.zip \
     "http://<controller>:<port>/plugins/install?replace=true"
```

(or use **Install** on the app's Plugins page). The response is `200 {"id", "plugin"}` or
`400 {"error": "badZip"|"badManifest"|"idExists", "message"}`.

## Tests

```
pip install -e "plugins-sdk/python[test]"
python -m pytest plugins-sdk/python
```

The tests run an in-process fake controller built on `websockets.serve` (`tests/fake_controller.py`).
