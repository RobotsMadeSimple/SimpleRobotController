"""In-process fake controller built on websockets.serve."""
from __future__ import annotations

import asyncio
import json
from typing import Any, Callable, Dict, List, Optional

import websockets

TOKEN = "good-token"


async def wait_until(cond: Callable[[], Any], timeout: float = 5.0) -> Any:
    end = asyncio.get_running_loop().time() + timeout
    while True:
        v = cond()
        if v:
            return v
        if asyncio.get_running_loop().time() > end:
            raise AssertionError("condition not met in time")
        await asyncio.sleep(0.01)


class FakeController:
    def __init__(self, token: str = TOKEN, config: Optional[Dict[str, Any]] = None) -> None:
        self.token = token
        self.config = config or {"port": "COM3"}
        self.received: List[Dict[str, Any]] = []  # every frame from the plugin, in order
        self.connections = 0
        self.ws: Any = None
        self.command_results: Dict[str, Dict[str, Any]] = {}
        self.variables: Dict[str, Any] = {"variables": {"a": 1}, "lists": {}, "strings": {}}
        self._ids = 0
        self._waiters: Dict[str, "asyncio.Future[Dict[str, Any]]"] = {}
        self.server: Any = None
        self.port = 0

    @property
    def url(self) -> str:
        return f"ws://127.0.0.1:{self.port}/plugin"

    async def start(self, port: int = 0) -> None:
        self.server = await websockets.serve(self._handler, "127.0.0.1", port)
        self.port = self.server.sockets[0].getsockname()[1]

    async def stop(self) -> None:
        self.server.close()
        await self.server.wait_closed()

    async def _handler(self, ws: Any) -> None:
        self.connections += 1
        first = json.loads(await ws.recv())
        self.received.append(first)
        params = first.get("params") or {}
        if first.get("method") != "plugin.ready" or params.get("token") != self.token:
            await ws.close(4401, "bad token")
            return
        self.ws = ws
        await self._send(ws, {"t": "res", "id": first["id"], "ok": True, "result": {
            "controllerVersion": "9.9.9", "pluginId": "t", "config": self.config, "dataDir": "/data"}})
        try:
            async for raw in ws:
                msg = json.loads(raw)
                self.received.append(msg)
                if msg["t"] == "res":
                    fut = self._waiters.pop(msg["id"], None)
                    if fut:
                        fut.set_result(msg)
                elif msg["t"] == "req":
                    await self._answer(ws, msg)
        except websockets.ConnectionClosed:
            pass

    async def _send(self, ws: Any, obj: Dict[str, Any]) -> None:
        await ws.send(json.dumps(obj))

    async def _answer(self, ws: Any, msg: Dict[str, Any]) -> None:
        m, p = msg["method"], msg.get("params") or {}
        if m == "events.subscribe":
            res: Dict[str, Any] = {"ok": True, "result": {"subscribed": p["events"]}}
        elif m == "controller.command":
            res = self.command_results.get(p["command"], {"ok": True, "result": {"echo": p.get("params")}})
        elif m == "variables.get":
            res = {"ok": True, "result": self.variables}
        elif m == "variables.set":
            res = {"ok": True, "result": {}}
        else:
            res = {"ok": False, "error": "unknownMethod", "message": m}
        await self._send(ws, {"t": "res", "id": msg["id"], **res})

    # ---- test-side helpers
    async def request(self, method: str, params: Optional[Dict[str, Any]] = None, timeout: float = 5.0) -> Dict[str, Any]:
        self._ids += 1
        rid = f"c{self._ids}"
        fut: "asyncio.Future[Dict[str, Any]]" = asyncio.get_running_loop().create_future()
        self._waiters[rid] = fut
        await self._send(self.ws, {"t": "req", "id": rid, "method": method, "params": params or {}})
        return await asyncio.wait_for(fut, timeout)

    async def event(self, name: str, data: Dict[str, Any]) -> None:
        await self._send(self.ws, {"t": "evt", "event": name, "data": data})

    async def drop(self) -> None:
        await self.ws.close(1011)

    def frames(self, **match: Any) -> List[Dict[str, Any]]:
        return [f for f in self.received if all(f.get(k) == v for k, v in match.items())]
