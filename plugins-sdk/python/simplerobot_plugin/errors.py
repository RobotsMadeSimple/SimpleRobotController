"""Exceptions raised by / understood by the SDK."""
from __future__ import annotations


class StepError(Exception):
    """Raise from a step (or function) handler to fail it with a controller-visible code.

    The reply becomes ``ok:false, error=<code>, message=<message>``; for a step the
    controller fails the running program with ``message``.
    """

    def __init__(self, message: str, code: str = "stepFailed") -> None:
        super().__init__(message)
        self.message = message
        self.code = code


class CommandError(Exception):
    """A ``ctx.command(...)`` (or other controller request) answered ``ok:false``."""

    def __init__(self, code: str, message: str = "") -> None:
        super().__init__(f"{code}: {message}" if message else code)
        self.code = code
        self.message = message
