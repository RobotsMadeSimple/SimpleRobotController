"""SimpleRobotController plugin SDK (see docs/plugins.md, section 8.1)."""
from .context import Context, StepContext
from .errors import CommandError, StepError
from .plugin import Event, Plugin

__version__ = "0.1.0"
__all__ = ["Plugin", "Context", "StepContext", "Event", "StepError", "CommandError", "__version__"]
