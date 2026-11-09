"""The ``ctx`` object handed to every handler."""
from __future__ import annotations

import asyncio
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Mapping, Optional

from .errors import CommandError

if TYPE_CHECKING:  # pragma: no cover
    from .plugin import Plugin

_STATES = ("ok", "degraded", "error")


class Context:
    """Per-plugin context. Step handlers receive a :class:`StepContext` (a subclass)."""

    def __init__(self, plugin: "Plugin") -> None:
        self._plugin = plugin

    # -- read-only information ------------------------------------------------
    @property
    def config(self) -> Dict[str, Any]:
        """Current config (schema defaults merged in). Updated by ``config.changed``."""
        return self._plugin._config

    @property
    def controller_version(self) -> str:
        return self._plugin._controller_version

    @property
    def plugin_id(self) -> str:
        return self._plugin.plugin_id

    @property
    def plugin_dir(self) -> Path:
        return self._plugin.plugin_dir

    @property
    def data_dir(self) -> Path:
        return self._plugin.data_dir

    # -- fire-and-forget notifications ---------------------------------------
    def log(self, message: str, level: str = "info") -> None:
        """Append a line to the plugin log (level: debug, info, warn, error)."""
        self._plugin._log(message, level)

    def set_properties(self, *values: Mapping[str, Any], **kwvalues: Any) -> None:
        """Publish live property values: ``set_properties(weight=1.5)`` or ``set_properties({"weight": 1.5})``."""
        merged: Dict[str, Any] = {}
        for v in values:
            merged.update(v)
        merged.update(kwvalues)
        if not merged:
            return
        self._plugin._props.update(merged)
        self._plugin._emit_evt("properties.set", {"values": merged})

    def clear_properties(self) -> None:
        """Make all of this plugin's properties unknown."""
        self._plugin._props.clear()
        self._plugin._emit_evt("properties.clear", {})

    def set_status(self, state: str, message: Optional[str] = None) -> None:
        """Report ``ok`` / ``degraded`` / ``error`` (shown on the plugin card)."""
        if state not in _STATES:
            raise ValueError(f"state must be one of {_STATES}")
        data: Dict[str, Any] = {"state": state}
        if message is not None:
            data["message"] = message
        self._plugin._emit_evt("status", data)

    def progress(self, message: Optional[str] = None, percent: Optional[float] = None) -> None:
        raise RuntimeError("ctx.progress() is only valid inside a step handler")

    # -- requests -------------------------------------------------------------
    async def command(self, name: str, /, **params: Any) -> Dict[str, Any]:
        """Run any WebSocket-API command. Returns the ack payload or raises :class:`CommandError`."""
        result = await self._plugin._request("controller.command", {"command": name, "params": params})
        if isinstance(result, dict) and result.get("ok") is False:
            raise CommandError(str(result.get("error", "commandFailed")), str(result.get("message", "")))
        return result if isinstance(result, dict) else {}

    async def get_variables(self, program: Optional[str] = None) -> Dict[str, Any]:
        """``{"variables": {...}, "lists": {...}, "strings": {...}}`` (globals when ``program`` is None)."""
        params: Dict[str, Any] = {}
        if program is not None:
            params["programName"] = program
        return await self._plugin._request("variables.get", params)

    async def set_variables(self, values: Mapping[str, Any], program: Optional[str] = None) -> None:
        params: Dict[str, Any] = {"values": dict(values)}
        if program is not None:
            params["programName"] = program
        await self._plugin._request("variables.set", params)

    async def subscribe(
        self,
        *events: str,
        position_interval_ms: Optional[int] = None,
        status_interval_ms: Optional[int] = None,
        io_interval_ms: Optional[int] = None,
    ) -> None:
        """Subscribe to controller events (globs ending in ``*`` allowed).

        Events used in ``@plugin.on(...)`` are always included. The subscription
        is remembered and re-sent after a reconnect.
        """
        await self._plugin._subscribe(list(events), position_interval_ms, status_interval_ms, io_interval_ms)


class StepContext(Context):
    """Context for one step invocation: adds progress and cancellation."""

    def __init__(self, plugin: "Plugin", invocation_id: str, step_id: str = "", program_name: str = "") -> None:
        super().__init__(plugin)
        self.invocation_id = invocation_id
        self.step_id = step_id
        self.program_name = program_name
        self._cancel_event = asyncio.Event()
        self._cancel_reason: Optional[str] = None

    @property
    def is_cancelled(self) -> bool:
        return self._cancel_event.is_set()

    @property
    def cancel_reason(self) -> Optional[str]:
        """``stopped`` / ``reset`` / ``timeout`` / ``shutdown`` once cancelled."""
        return self._cancel_reason

    async def cancelled(self) -> str:
        """Wait until the controller cancels this step; returns the reason."""
        await self._cancel_event.wait()
        return self._cancel_reason or ""

    def _cancel(self, reason: str) -> None:
        self._cancel_reason = reason
        self._cancel_event.set()

    def progress(self, message: Optional[str] = None, percent: Optional[float] = None) -> None:
        data: Dict[str, Any] = {"invocationId": self.invocation_id}
        if message is not None:
            data["message"] = message
        if percent is not None:
            data["percent"] = percent
        self._plugin._emit_evt("step.progress", data)
