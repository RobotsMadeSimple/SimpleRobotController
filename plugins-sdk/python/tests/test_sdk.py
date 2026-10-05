from __future__ import annotations

import asyncio
import contextlib
import json
import sys
from pathlib import Path
from typing import Any, AsyncIterator, Awaitable, Callable, Dict, Optional

import pytest

from fake_controller import TOKEN, FakeController, wait_until
from simplerobot_plugin import CommandError, Plugin, StepError
from simplerobot_plugin.__main__ import main as cli_main
from simplerobot_plugin.manifest import validate_manifest


def make_plugin(fc: FakeController, **kw: Any) -> Plugin:
    kw.setdefault("reconnect_min", 0.05)
    kw.setdefault("reconnect_max", 0.2)
    return Plugin(url=fc.url, token=kw.pop("token", TOKEN), plugin_id="t", **kw)


@contextlib.asynccontextmanager
async def running(plugin: Plugin, fc: Optional[FakeController] = None, wait_ready: bool = True) -> AsyncIterator["asyncio.Task[int]"]:
    own = fc is None
    task = asyncio.create_task(plugin.run_async())
    try:
        if fc is not None and wait_ready:
            await wait_until(lambda: fc.ws is not None and plugin._session is not None and plugin._session.ready)
        yield task
    finally:
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)


def scenario(fn: Callable[[FakeController], Awaitable[None]]) -> Callable[[], None]:
    def wrapper() -> None:
        async def go() -> None:
            fc = FakeController()
            await fc.start()
            try:
                await asyncio.wait_for(fn(fc), 20)
            finally:
                await fc.stop()

        asyncio.run(go())

    wrapper.__name__ = fn.__name__
    return wrapper


# ---------------------------------------------------------------- handshake
@scenario
async def test_ready_is_first_message_with_token_and_sdk(fc: FakeController) -> None:
    p = make_plugin(fc, manifest_override={"steps": []})
    seen: Dict[str, Any] = {}

    @p.on_ready
    async def ready(ctx: Any) -> None:
        seen["config"] = ctx.config
        seen["cv"] = ctx.controller_version
        seen["pid"] = ctx.plugin_id

    async with running(p, fc):
        await wait_until(lambda: seen)
    first = fc.received[0]
    assert first["t"] == "req" and first["method"] == "plugin.ready" and isinstance(first["id"], str)
    assert first["params"]["token"] == TOKEN
    assert first["params"]["sdk"]["name"] and first["params"]["sdk"]["version"] == "0.1.0"
    assert first["params"]["manifest"] == {"steps": []}
    assert seen == {"config": {"port": "COM3"}, "cv": "9.9.9", "pid": "t"}


@scenario
async def test_bad_token_exits_nonzero(fc: FakeController) -> None:
    p = make_plugin(fc, token="wrong")
    code = await asyncio.wait_for(p.run_async(), 5)
    assert code != 0
    assert fc.connections == 1  # no reconnect storm


def test_missing_env_exits_nonzero(monkeypatch: Any) -> None:
    for k in ("SRC_PLUGIN_URL", "SRC_PLUGIN_TOKEN"):
        monkeypatch.delenv(k, raising=False)
    assert asyncio.run(Plugin().run_async()) == 2


def test_environment_is_read(monkeypatch: Any, tmp_path: Path) -> None:
    monkeypatch.setenv("SRC_PLUGIN_ID", "abc")
    monkeypatch.setenv("SRC_PLUGIN_URL", "ws://x/plugin")
    monkeypatch.setenv("SRC_PLUGIN_TOKEN", "tok")
    monkeypatch.setenv("SRC_PLUGIN_DIR", str(tmp_path))
    monkeypatch.setenv("SRC_DATA_DIR", str(tmp_path / "d"))
    monkeypatch.setenv("SRC_CONTROLLER_VERSION", "1.2")
    p = Plugin()
    assert (p.plugin_id, p.url, p.token, p.ctx.controller_version) == ("abc", "ws://x/plugin", "tok", "1.2")
    assert p.ctx.plugin_dir == tmp_path and p.ctx.data_dir == tmp_path / "d"
    assert Plugin(url="u", token="t2").token == "t2"


# ---------------------------------------------------------------- controller -> plugin requests
@scenario
async def test_step_execute_outputs_and_errors(fc: FakeController) -> None:
    p = make_plugin(fc)

    @p.step("ok")
    def ok(ctx: Any, params: Dict[str, Any]) -> Dict[str, Any]:  # sync handler
        return {"x": params["a"] + 1}

    @p.step("none")
    async def none(ctx: Any, params: Any) -> None:
        return None

    @p.step("fail")
    async def fail(ctx: Any, params: Any) -> None:
        raise StepError("scale timeout", code="scaleTimeout")

    @p.step("boom")
    async def boom(ctx: Any, params: Any) -> None:
        raise ValueError("bad")

    @p.step("badret")
    async def badret(ctx: Any, params: Any) -> Any:
        return 5

    async with running(p, fc):
        def ex(step: str, **params: Any) -> Any:
            return fc.request("step.execute", {"invocationId": "i1", "stepId": step, "programName": "P", "params": params, "isBackground": False})

        r = await ex("ok", a=1)
        assert r["ok"] and r["result"] == {"outputs": {"x": 2}} and r["id"].startswith("c")
        assert (await ex("none"))["result"] == {"outputs": {}}
        r = await ex("fail")
        assert (r["ok"], r["error"], r["message"]) == (False, "scaleTimeout", "scale timeout")
        r = await ex("boom")
        assert r["ok"] is False and r["error"] == "exception" and "ValueError" in r["message"]
        assert (await ex("badret"))["error"] == "badResult"
        assert (await ex("nope"))["error"] == "unknownStep"
        r = await fc.request("bogus.method")
        assert r["error"] == "unknownMethod"


@scenario
async def test_function_call(fc: FakeController) -> None:
    p = make_plugin(fc)

    @p.function("double")
    def double(ctx: Any, x: float) -> float:
        return x * 2

    @p.function("flag")
    async def flag(ctx: Any) -> bool:
        return True

    @p.function("bad")
    async def bad(ctx: Any) -> str:
        return "x"

    async with running(p, fc):
        assert (await fc.request("function.call", {"name": "double", "args": [4]}))["result"] == {"value": 8.0}
        assert (await fc.request("function.call", {"name": "flag", "args": []}))["result"] == {"value": 1.0}
        assert (await fc.request("function.call", {"name": "bad", "args": []}))["ok"] is False
        assert (await fc.request("function.call", {"name": "nope", "args": []}))["error"] == "unknownFunction"


@scenario
async def test_cancel_progress_and_concurrency(fc: FakeController) -> None:
    p = make_plugin(fc)
    reasons: Dict[str, Any] = {}
    started = asyncio.Event()

    @p.step("wait")
    async def wait(ctx: Any, params: Any) -> Dict[str, Any]:
        assert not ctx.is_cancelled
        ctx.progress("working", percent=10)
        started.set()
        reasons["r"] = await ctx.cancelled()
        assert ctx.is_cancelled
        return {"cancelled": True}

    @p.step("quick")
    async def quick(ctx: Any, params: Any) -> Dict[str, Any]:
        return {"q": 1}

    @p.on_ready
    async def ready(ctx: Any) -> None:
        with pytest.raises(RuntimeError):
            ctx.progress("not in a step")

    async with running(p, fc):
        slow = asyncio.create_task(fc.request("step.execute", {"invocationId": "slow", "stepId": "wait", "params": {}}))
        await started.wait()
        # served concurrently while the slow step is outstanding
        assert (await fc.request("step.execute", {"invocationId": "q", "stepId": "quick", "params": {}}))["result"] == {"outputs": {"q": 1}}
        prog = fc.frames(t="evt", event="step.progress")[0]
        assert prog["data"] == {"invocationId": "slow", "message": "working", "percent": 10}
        assert (await fc.request("step.cancel", {"invocationId": "slow", "reason": "stopped"}))["result"] == {}
        assert (await slow)["result"] == {"outputs": {"cancelled": True}}
        assert reasons["r"] == "stopped"
        # cancel for an unknown invocation is harmless
        assert (await fc.request("step.cancel", {"invocationId": "zzz", "reason": "reset"}))["ok"]


@scenario
async def test_config_changed(fc: FakeController) -> None:
    p = make_plugin(fc)
    got = []

    @p.on_config_changed
    def changed(ctx: Any, config: Dict[str, Any]) -> None:
        got.append((config, ctx.config))

    async with running(p, fc):
        r = await fc.request("config.changed", {"config": {"port": "COM9"}})
        assert r["ok"] and r["result"] == {}
    assert got == [({"port": "COM9"}, {"port": "COM9"})]


@scenario
async def test_shutdown_replies_then_exits_cleanly(fc: FakeController) -> None:
    p = make_plugin(fc)
    async with running(p, fc) as task:
        r = await fc.request("shutdown", {"reason": "stop"})
        assert r["ok"] and r["result"] == {}
        assert await asyncio.wait_for(task, 5) == 0
    assert fc.connections == 1


# ---------------------------------------------------------------- plugin -> controller
@scenario
async def test_events_dispatch_and_autosubscribe(fc: FakeController) -> None:
    p = make_plugin(fc)
    got = []

    @p.on("program.started")
    async def started(ctx: Any, event: Any) -> None:
        got.append(("exact", event.name, dict(event)))

    @p.on("robot.*")
    def robot(ctx: Any, event: Any) -> None:
        got.append(("glob", event.name))

    @p.on("io.changed")
    async def bad(ctx: Any, event: Any) -> None:
        raise RuntimeError("handler failure must not kill the loop")

    async with running(p, fc):
        sub = await wait_until(lambda: fc.frames(method="events.subscribe"))
        assert len(sub) == 1 and sorted(sub[0]["params"]["events"]) == ["io.changed", "program.started", "robot.*"]
        await fc.event("program.started", {"programName": "P"})
        await fc.event("robot.homed", {})
        await fc.event("program.stopped", {})  # no handler
        await fc.event("io.changed", {"changes": {}})
        await fc.event("robot.fault", {})
        await wait_until(lambda: len(got) == 3)
        await wait_until(lambda: any(f["event"] == "log" and f["data"]["level"] == "error" for f in fc.frames(t="evt")))
    assert got == [("exact", "program.started", {"programName": "P"}), ("glob", "robot.homed"), ("glob", "robot.fault")]


@scenario
async def test_no_subscribe_without_handlers_and_explicit_subscribe(fc: FakeController) -> None:
    p = make_plugin(fc)
    async with running(p, fc):
        assert not fc.frames(method="events.subscribe")
        await p.ctx.subscribe("status", position_interval_ms=50, io_interval_ms=10)
        sub = fc.frames(method="events.subscribe")[0]["params"]
        assert sub == {"events": ["status"], "positionIntervalMs": 50, "ioIntervalMs": 10}


@scenario
async def test_ctx_notifications_and_requests(fc: FakeController) -> None:
    p = make_plugin(fc)
    fc.command_results["Nope"] = {"ok": False, "error": "unknownCommand", "message": "no such command"}
    async with running(p, fc):
        ctx = p.ctx
        ctx.log("hello", level="warn")
        ctx.set_properties(weight=1.5, stable=True)
        ctx.set_properties({"a": 2})
        ctx.set_status("degraded", "no scale")
        ctx.clear_properties()
        await wait_until(lambda: fc.frames(event="properties.clear"))
        evts = [(f["event"], f["data"]) for f in fc.frames(t="evt")]
        assert evts == [
            ("log", {"level": "warn", "message": "hello"}),
            ("properties.set", {"values": {"weight": 1.5, "stable": True}}),
            ("properties.set", {"values": {"a": 2}}),
            ("status", {"state": "degraded", "message": "no scale"}),
            ("properties.clear", {}),
        ]
        assert await ctx.command("SetSTBOutput", pin=1, value=True) == {"echo": {"pin": 1, "value": True}}
        assert fc.frames(method="controller.command")[0]["params"] == {"command": "SetSTBOutput", "params": {"pin": 1, "value": True}}
        with pytest.raises(CommandError) as ei:
            await ctx.command("Nope")
        assert ei.value.code == "unknownCommand"
        assert (await ctx.get_variables())["variables"] == {"a": 1}
        assert fc.frames(method="variables.get")[0]["params"] == {}
        await ctx.set_variables({"x": 3}, program="P")
        assert fc.frames(method="variables.set")[0]["params"] == {"programName": "P", "values": {"x": 3}}
        with pytest.raises(ValueError):
            ctx.set_status("bogus")


@scenario
async def test_background_runs_and_stops_on_shutdown(fc: FakeController) -> None:
    p = make_plugin(fc)
    state = {"ticks": 0, "cancelled": False}

    @p.background
    async def poll(ctx: Any) -> None:
        try:
            while True:
                state["ticks"] += 1
                ctx.set_properties(total=state["ticks"])
                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            state["cancelled"] = True
            raise

    async with running(p, fc) as task:
        await wait_until(lambda: len(fc.frames(event="properties.set")) >= 2)
        await fc.request("shutdown")
        assert await asyncio.wait_for(task, 5) == 0
    assert state["cancelled"]


def test_background_requires_async() -> None:
    with pytest.raises(TypeError):
        Plugin(url="u", token="t").background(lambda ctx: None)  # type: ignore[arg-type]


# ---------------------------------------------------------------- reconnect
@scenario
async def test_reconnect_after_drop(fc: FakeController) -> None:
    p = make_plugin(fc)
    readies = []

    @p.on_ready
    async def ready(ctx: Any) -> None:
        readies.append(1)
        ctx.set_properties(total=7)  # replayed automatically too

    @p.on("program.started")
    def noop(ctx: Any, event: Any) -> None: ...

    async with running(p, fc):
        await wait_until(lambda: len(readies) == 1)
        await fc.drop()
        await wait_until(lambda: fc.connections == 2 and len(readies) == 2)
        assert [f["method"] for f in fc.frames(method="plugin.ready")] == ["plugin.ready"] * 2
        assert len(fc.frames(method="events.subscribe")) == 2
        # still serving after reconnect
        p.ctx.log("back")
        await wait_until(lambda: any(f["data"].get("message") == "back" for f in fc.frames(event="log")))


def test_reconnect_backoff_grows_and_caps(monkeypatch: Any) -> None:
    import simplerobot_plugin.plugin as pm

    def refuse(*a: Any, **k: Any) -> Any:
        raise ConnectionRefusedError("down")

    monkeypatch.setattr(pm, "connect", refuse)

    async def go() -> None:
        p = Plugin(url="ws://127.0.0.1:1/plugin", token="t", plugin_id="t", reconnect_min=1.0, reconnect_max=8.0)
        sleeps = []
        real_sleep = asyncio.sleep

        async def fake_sleep(d: float) -> None:
            sleeps.append(d)
            if len(sleeps) >= 6:
                raise asyncio.CancelledError
            await real_sleep(0)

        monkeypatch.setattr(pm.asyncio, "sleep", fake_sleep)
        with pytest.raises(asyncio.CancelledError):
            await p.run_async()
        assert sleeps == [1.0, 2.0, 4.0, 8.0, 8.0, 8.0]

    asyncio.run(go())


# ---------------------------------------------------------------- misc
def test_default_backoff_is_1_to_30() -> None:
    p = Plugin(url="u", token="t")
    assert (p.reconnect_min, p.reconnect_max) == (1.0, 30.0)


def test_scaffold_and_manifest(tmp_path: Path) -> None:
    assert cli_main(["new", "my_plugin", "--dir", str(tmp_path)]) == 0
    d = tmp_path / "my_plugin"
    for f in ("plugin.json", "main.py", "requirements.txt", "README.md"):
        assert (d / f).is_file()
    assert validate_manifest(json.loads((d / "plugin.json").read_text())) == []
    compile((d / "main.py").read_text(), "main.py", "exec")
    assert cli_main(["new", "my_plugin", "--dir", str(tmp_path)]) == 1  # not empty
    assert cli_main(["new", "Bad-Id", "--dir", str(tmp_path)]) == 1
    assert cli_main(["new", "robot", "--dir", str(tmp_path)]) == 1


def test_demo_counter_manifest_valid() -> None:
    root = Path(__file__).resolve().parents[3] / "examples" / "python" / "demo_counter"
    m = json.loads((root / "plugin.json").read_text())
    assert validate_manifest(m) == []
    assert m["runtime"] == "python" and m["entry"] == "main.py"
    assert (root / "requirements.txt").is_file()


@scenario
async def test_demo_counter_end_to_end(fc: FakeController) -> None:
    import importlib.util

    root = Path(__file__).resolve().parents[3] / "examples" / "python" / "demo_counter"
    spec = importlib.util.spec_from_file_location("demo_counter_main", root / "main.py")
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    p = mod.plugin
    p.url, p.token, p.plugin_id = fc.url, TOKEN, "demo_counter"
    p.reconnect_min = 0.05
    async with running(p, fc):
        r = await fc.request("step.execute", {"invocationId": "1", "stepId": "count", "params": {"by": 2}})
        assert r["result"] == {"outputs": {"total": 2.0}}
        r = await fc.request("step.execute", {"invocationId": "2", "stepId": "count", "params": {}})
        assert r["result"] == {"outputs": {"total": 3.0}}
        assert (await fc.request("function.call", {"name": "double", "args": [21]}))["result"] == {"value": 42.0}
        await fc.event("program.started", {"programName": "P"})
        await wait_until(lambda: any("program.started: P" in f["data"].get("message", "") for f in fc.frames(event="log")))
