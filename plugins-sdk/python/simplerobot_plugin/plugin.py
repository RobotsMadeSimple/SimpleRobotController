"""The :class:`Plugin` class: decorators, connection management and protocol framing."""
from __future__ import annotations

import asyncio
import inspect
import itertools
import json
import os
import sys
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Optional, Set, Tuple, TypeVar, Union

from websockets.exceptions import WebSocketException

try:  # websockets >= 13
    from websockets.asyncio.client import connect
except ImportError:  # websockets 11/12
    from websockets.client import connect  # type: ignore[no-redef]

from .context import Context, StepContext
from .errors import CommandError, StepError

SDK_NAME = "simplerobot-plugin-python"
SDK_VERSION = "0.1.0"

_MAX_MESSAGE = 32 * 1024 * 1024  # controller limit is 16 MB; leave headroom
_LOG_LEVELS = ("debug", "info", "warn", "error")

F = TypeVar("F", bound=Callable[..., Any])

# Exit codes returned by Plugin.run() / run_async()
EXIT_OK = 0
EXIT_REJECTED = 1  # controller refused us (bad token, close 4401 / ready error)
EXIT_BAD_ENV = 2  # url or token missing
EXIT_REPLACED = 3  # another connection replaced ours (close 4409)


class Event(dict):  # type: ignore[type-arg]
    """Event payload (a plain dict) with the event name attached as ``.name``."""

    name: str = ""


class _Reject(Exception):
    """Internal: reply ``ok:false`` with a given code."""

    def __init__(self, code: str, message: str = "") -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class _Session:
    """One live websocket: outbound queue, request correlation, inbound reader."""

    def __init__(self, ws: Any) -> None:
        self.ws = ws
        self.out: "asyncio.Queue[str]" = asyncio.Queue()
        self.events: "asyncio.Queue[Tuple[str, Dict[str, Any]]]" = asyncio.Queue()
        self.pending: Dict[str, "asyncio.Future[Any]"] = {}
        self.ids = itertools.count(1)
        self.done = asyncio.Event()
        self.shutdown = asyncio.Event()
        self.close_code: Optional[int] = None
        self.request_tasks: Set["asyncio.Task[None]"] = set()
        self.ready = False

    def send_obj(self, obj: Dict[str, Any]) -> None:
        text = json.dumps(obj, separators=(",", ":"))  # raises on unserialisable data
        self.out.put_nowait(text)

    async def request(self, method: str, params: Dict[str, Any]) -> Any:
        if self.done.is_set():
            raise ConnectionError("not connected")
        rid = str(next(self.ids))
        fut: "asyncio.Future[Any]" = asyncio.get_running_loop().create_future()
        self.pending[rid] = fut
        try:
            self.send_obj({"t": "req", "id": rid, "method": method, "params": params})
            return await fut
        finally:
            self.pending.pop(rid, None)

    async def writer(self) -> None:
        try:
            while True:
                text = await self.out.get()
                try:
                    await self.ws.send(text)
                finally:
                    self.out.task_done()
        except asyncio.CancelledError:
            raise
        except Exception:
            self._finish()

    def _finish(self) -> None:
        if self.close_code is None:
            self.close_code = getattr(self.ws, "close_code", None)
        self.done.set()
        for fut in list(self.pending.values()):
            if not fut.done():
                fut.set_exception(ConnectionError("connection closed"))
        # unblock a pending shutdown flush
        while not self.out.empty():
            self.out.get_nowait()
            self.out.task_done()


class Plugin:
    """A SimpleRobotController plugin.

    Environment (set by the controller): ``SRC_PLUGIN_ID``, ``SRC_PLUGIN_URL``,
    ``SRC_PLUGIN_TOKEN``, ``SRC_PLUGIN_DIR``, ``SRC_DATA_DIR``, ``SRC_CONTROLLER_VERSION``.
    Constructor keyword arguments override them (development mode).
    """

    def __init__(
        self,
        *,
        url: Optional[str] = None,
        token: Optional[str] = None,
        plugin_id: Optional[str] = None,
        plugin_dir: Union[str, Path, None] = None,
        data_dir: Union[str, Path, None] = None,
        controller_version: Optional[str] = None,
        manifest_override: Union[Dict[str, Any], str, Path, None] = None,
        echo_logs: bool = False,
        reconnect_min: float = 1.0,
        reconnect_max: float = 30.0,
        ready_timeout: float = 15.0,
    ) -> None:
        env = os.environ
        self.url: Optional[str] = url or env.get("SRC_PLUGIN_URL")
        self.token: Optional[str] = token or env.get("SRC_PLUGIN_TOKEN")
        self.plugin_id: str = plugin_id or env.get("SRC_PLUGIN_ID", "")
        self.plugin_dir: Path = Path(plugin_dir or env.get("SRC_PLUGIN_DIR") or os.getcwd())
        self.data_dir: Path = Path(data_dir or env.get("SRC_DATA_DIR") or self.plugin_dir)
        self._controller_version: str = controller_version or env.get("SRC_CONTROLLER_VERSION", "")
        if isinstance(manifest_override, (str, Path)):
            manifest_override = json.loads(Path(manifest_override).read_text(encoding="utf-8"))
        self.manifest_override: Optional[Dict[str, Any]] = manifest_override
        self.echo_logs = echo_logs
        self.reconnect_min = reconnect_min
        self.reconnect_max = reconnect_max
        self.ready_timeout = ready_timeout

        self._steps: Dict[str, Callable[..., Any]] = {}
        self._functions: Dict[str, Callable[..., Any]] = {}
        self._event_handlers: List[Tuple[str, Callable[..., Any]]] = []
        self._ready_handlers: List[Callable[..., Any]] = []
        self._config_handlers: List[Callable[..., Any]] = []
        self._background: List[Callable[..., Any]] = []

        self._config: Dict[str, Any] = {}
        self._props: Dict[str, Any] = {}
        self._extra_events: Set[str] = set()
        self._intervals: Dict[str, int] = {}
        self._session: Optional[_Session] = None
        self._invocations: Dict[str, StepContext] = {}
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self.ctx = Context(self)

    # ------------------------------------------------------------------ decorators
    def on_ready(self, fn: F) -> F:
        """Run ``fn(ctx)`` after every successful handshake (including reconnects)."""
        self._ready_handlers.append(fn)
        return fn

    def step(self, step_id: str) -> Callable[[F], F]:
        """Register ``fn(ctx, params) -> dict | None`` as the handler for a manifest step."""

        def deco(fn: F) -> F:
            if step_id in self._steps:
                raise ValueError(f"step '{step_id}' registered twice")
            self._steps[step_id] = fn
            return fn

        return deco

    def function(self, name: str) -> Callable[[F], F]:
        """Register ``fn(ctx, *args) -> number | bool`` as an expression function."""

        def deco(fn: F) -> F:
            if name in self._functions:
                raise ValueError(f"function '{name}' registered twice")
            self._functions[name] = fn
            return fn

        return deco

    def on(self, event: str) -> Callable[[F], F]:
        """Register ``fn(ctx, event)`` for a controller event (``*`` suffix globs allowed)."""

        def deco(fn: F) -> F:
            self._event_handlers.append((event, fn))
            return fn

        return deco

    def background(self, fn: F) -> F:
        """Register an ``async fn(ctx)`` started after ready and cancelled on disconnect/shutdown."""
        if not inspect.iscoroutinefunction(fn):
            raise TypeError("@plugin.background requires an async function")
        self._background.append(fn)
        return fn

    def on_config_changed(self, fn: F) -> F:
        """Run ``fn(ctx, config)`` when the user saves a new config."""
        self._config_handlers.append(fn)
        return fn

    # ------------------------------------------------------------------ public run API
    def run(self) -> None:
        """Blocking entry point. Exits the process with a non-zero code when rejected."""
        try:
            code = asyncio.run(self.run_async())
        except KeyboardInterrupt:
            code = EXIT_OK
        if code != EXIT_OK:
            sys.exit(code)

    async def run_async(self) -> int:
        """Connect, serve, reconnect with exponential backoff. Returns an exit code."""
        self._loop = asyncio.get_running_loop()
        if not self.url or not self.token:
            self._stderr("SRC_PLUGIN_URL / SRC_PLUGIN_TOKEN not set (or pass url= and token=)")
            return EXIT_BAD_ENV
        backoff = self.reconnect_min
        while True:
            try:
                outcome, was_ready = await self._connect_once()
            except (OSError, WebSocketException, asyncio.TimeoutError) as e:
                self._stderr(f"connection failed: {e!r}")
                outcome, was_ready = "dropped", False
            finally:
                self._session = None
            if outcome == "shutdown":
                return EXIT_OK
            if outcome == "rejected":
                return EXIT_REJECTED
            if outcome == "replaced":
                return EXIT_REPLACED
            if was_ready:
                backoff = self.reconnect_min
            self._stderr(f"connection lost; reconnecting in {backoff:.1f}s")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, self.reconnect_max)

    # ------------------------------------------------------------------ outbound helpers (used by Context)
    def _stderr(self, message: str) -> None:
        print(f"[{self.plugin_id or 'plugin'}] {message}", file=sys.stderr, flush=True)

    def _log(self, message: str, level: str = "info") -> None:
        if level not in _LOG_LEVELS:
            level = "info"
        if self._session is None or not self._session.ready or self.echo_logs:
            self._stderr(f"{level}: {message}")
        if self._session is not None and self._session.ready:
            self._emit_evt("log", {"level": level, "message": str(message)})

    def _emit_evt(self, event: str, data: Dict[str, Any]) -> None:
        session = self._session
        if session is None or self._loop is None or session.done.is_set():
            return  # not connected: dropped (properties are replayed on reconnect)
        text = json.dumps({"t": "evt", "event": event, "data": data}, separators=(",", ":"))
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is self._loop:
            session.out.put_nowait(text)
        else:
            self._loop.call_soon_threadsafe(session.out.put_nowait, text)

    async def _request(self, method: str, params: Dict[str, Any]) -> Any:
        session = self._session
        if session is None or not session.ready:
            raise ConnectionError("not connected to the controller")
        return await session.request(method, params)

    async def _subscribe(
        self, events: List[str], position: Optional[int], status: Optional[int], io: Optional[int]
    ) -> None:
        self._extra_events.update(events)
        for key, value in (("positionIntervalMs", position), ("statusIntervalMs", status), ("ioIntervalMs", io)):
            if value is not None:
                self._intervals[key] = int(value)
        if self._session is not None and self._session.ready:
            await self._send_subscription()

    async def _send_subscription(self) -> None:
        names = sorted({e for e, _ in self._event_handlers} | self._extra_events)
        if not names:
            return
        params: Dict[str, Any] = {"events": names}
        params.update(self._intervals)
        await self._request("events.subscribe", params)

    # ------------------------------------------------------------------ connection
    async def _connect_once(self) -> Tuple[str, bool]:
        assert self.url is not None and self.token is not None
        async with connect(self.url, max_size=_MAX_MESSAGE, open_timeout=10) as ws:
            session = _Session(ws)
            self._session = session
            tasks: List["asyncio.Task[Any]"] = [
                asyncio.create_task(self._reader(session)),
                asyncio.create_task(session.writer()),
            ]
            bg: List["asyncio.Task[Any]"] = []
            try:
                params: Dict[str, Any] = {"token": self.token, "sdk": {"name": SDK_NAME, "version": SDK_VERSION}}
                if self.manifest_override is not None:
                    params["manifest"] = self.manifest_override
                try:
                    res = await asyncio.wait_for(session.request("plugin.ready", params), self.ready_timeout)
                except (ConnectionError, asyncio.TimeoutError) as e:
                    return self._classify_close(session, repr(e)), False
                except CommandError as e:
                    self._stderr(f"controller rejected plugin.ready: {e}")
                    return "rejected", False
                session.ready = True
                res = res if isinstance(res, dict) else {}
                self._config = dict(res.get("config") or {})
                if res.get("controllerVersion"):
                    self._controller_version = str(res["controllerVersion"])
                if res.get("pluginId") and not self.plugin_id:
                    self.plugin_id = str(res["pluginId"])
                if res.get("dataDir") and not os.environ.get("SRC_DATA_DIR"):
                    self.data_dir = Path(str(res["dataDir"]))
                tasks.append(asyncio.create_task(self._event_worker(session)))

                try:
                    await self._send_subscription()
                    if self._props:
                        self._emit_evt("properties.set", {"values": dict(self._props)})
                    for handler in self._ready_handlers:
                        await self._safe_call("on_ready", handler, self.ctx)
                except ConnectionError:
                    return self._classify_close(session, "lost during startup"), True
                for fn in self._background:
                    bg.append(asyncio.create_task(self._run_background(fn)))

                waiters = [asyncio.create_task(session.done.wait()), asyncio.create_task(session.shutdown.wait())]
                try:
                    await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
                finally:
                    for w in waiters:
                        w.cancel()
                if session.shutdown.is_set():
                    return "shutdown", True
                return self._classify_close(session, "socket closed"), True
            finally:
                session.ready = False
                for t in bg + list(session.request_tasks) + tasks:
                    t.cancel()
                await asyncio.gather(*bg, *list(session.request_tasks), *tasks, return_exceptions=True)
                self._invocations.clear()

    def _classify_close(self, session: _Session, detail: str) -> str:
        code = session.close_code
        if code == 4401:
            self._stderr("controller closed the connection with 4401 (bad token); not retrying")
            return "rejected"
        if code == 4409:
            self._stderr("connection replaced by another instance (4409); exiting")
            return "replaced"
        self._stderr(f"disconnected ({detail}, close code {code})")
        return "dropped"

    async def _reader(self, session: _Session) -> None:
        try:
            async for raw in session.ws:
                try:
                    msg = json.loads(raw)
                    if not isinstance(msg, dict):
                        raise ValueError("frame is not a JSON object")
                except ValueError as e:
                    self._log(f"protocol error: bad frame ({e})", "error")
                    continue
                self._on_message(session, msg)
        except asyncio.CancelledError:
            raise
        except WebSocketException as e:
            rcvd = getattr(e, "rcvd", None)
            if rcvd is not None:
                session.close_code = rcvd.code
        except Exception as e:  # defensive: never let the reader die silently
            self._stderr(f"reader failed: {e!r}")
        finally:
            session._finish()

    def _on_message(self, session: _Session, msg: Dict[str, Any]) -> None:
        kind = msg.get("t")
        if kind == "res":
            fut = session.pending.get(str(msg.get("id")))
            if fut is None or fut.done():
                self._log(f"protocol error: unmatched response id {msg.get('id')!r}", "warn")
            elif msg.get("ok"):
                fut.set_result(msg.get("result") or {})
            else:
                fut.set_exception(CommandError(str(msg.get("error", "error")), str(msg.get("message", ""))))
        elif kind == "req":
            task = asyncio.create_task(self._handle_request(session, msg))
            session.request_tasks.add(task)
            task.add_done_callback(session.request_tasks.discard)
        elif kind == "evt":
            data = msg.get("data")
            session.events.put_nowait((str(msg.get("event", "")), data if isinstance(data, dict) else {}))
        else:
            self._log(f"protocol error: unknown frame type {kind!r}", "warn")

    # ------------------------------------------------------------------ inbound requests
    async def _handle_request(self, session: _Session, msg: Dict[str, Any]) -> None:
        rid = str(msg.get("id"))
        method = msg.get("method")
        out: Dict[str, Any]
        try:
            params = msg.get("params")
            if params is None:
                params = {}
            if not isinstance(params, dict):
                raise _Reject("badParams", "params must be an object")
            result = await self._dispatch(session, str(method), params)
            out = {"t": "res", "id": rid, "ok": True, "result": result}
            text = json.dumps(out, separators=(",", ":"))
        except _Reject as r:
            out = {"t": "res", "id": rid, "ok": False, "error": r.code, "message": r.message}
            text = json.dumps(out)
        except StepError as e:
            out = {"t": "res", "id": rid, "ok": False, "error": e.code, "message": e.message}
            text = json.dumps(out)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._log(f"{method} handler raised {e!r}", "error")
            out = {"t": "res", "id": rid, "ok": False, "error": "exception", "message": repr(e)}
            text = json.dumps(out)
        if session.done.is_set():
            return
        session.out.put_nowait(text)
        if method == "shutdown":
            await session.out.join()  # reply is on the wire before we close
            session.shutdown.set()

    async def _dispatch(self, session: _Session, method: str, p: Dict[str, Any]) -> Dict[str, Any]:
        if method == "step.execute":
            step_id = str(p.get("stepId", ""))
            inv = str(p.get("invocationId", ""))
            handler = self._steps.get(step_id)
            if handler is None:
                raise _Reject("unknownStep", f"no handler registered for step '{step_id}'")
            ctx = StepContext(self, inv, step_id, str(p.get("programName", "")))
            self._invocations[inv] = ctx
            try:
                outputs = await self._call(handler, ctx, p.get("params") or {})
            finally:
                self._invocations.pop(inv, None)
            if outputs is None:
                outputs = {}
            if not isinstance(outputs, dict):
                raise _Reject("badResult", f"step handler must return a dict or None, got {type(outputs).__name__}")
            return {"outputs": outputs}
        if method == "step.cancel":
            ctx2 = self._invocations.get(str(p.get("invocationId", "")))
            if ctx2 is not None:
                ctx2._cancel(str(p.get("reason", "")))
            return {}
        if method == "function.call":
            name = str(p.get("name", ""))
            fn = self._functions.get(name)
            if fn is None:
                raise _Reject("unknownFunction", f"no function '{name}'")
            args = p.get("args") or []
            value = await self._call(fn, self.ctx, *args)
            if isinstance(value, bool):
                value = 1.0 if value else 0.0
            elif isinstance(value, (int, float)):
                value = float(value)
            else:
                raise _Reject("badResult", f"function must return a number or bool, got {type(value).__name__}")
            return {"value": value}
        if method == "config.changed":
            cfg = p.get("config")
            self._config = dict(cfg) if isinstance(cfg, dict) else {}
            for h in self._config_handlers:
                await self._call(h, self.ctx, self._config)
            return {}
        if method == "shutdown":
            self._log(f"shutdown requested ({p.get('reason', '')})", "info")
            return {}
        raise _Reject("unknownMethod", f"unknown method '{method}'")

    # ------------------------------------------------------------------ events & background
    async def _event_worker(self, session: _Session) -> None:
        while True:
            name, data = await session.events.get()
            for pattern, handler in list(self._event_handlers):
                if _matches(pattern, name):
                    ev = Event(data)
                    ev.name = name
                    await self._safe_call(f"event '{name}'", handler, self.ctx, ev)

    async def _run_background(self, fn: Callable[..., Any]) -> None:
        try:
            await fn(self.ctx)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._log(f"background task {getattr(fn, '__name__', fn)} failed: {e!r}", "error")

    async def _call(self, fn: Callable[..., Any], *args: Any) -> Any:
        result = fn(*args)
        if inspect.isawaitable(result):
            result = await result
        return result

    async def _safe_call(self, what: str, fn: Callable[..., Any], *args: Any) -> None:
        try:
            await self._call(fn, *args)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._log(f"{what} handler raised {e!r}", "error")


def _matches(pattern: str, name: str) -> bool:
    if pattern == "*":
        return True
    if pattern.endswith("*"):
        return name.startswith(pattern[:-1])
    return pattern == name
