"""``python -m simplerobot_plugin new <id> [--dir .]``"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from .scaffold import scaffold


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m simplerobot_plugin")
    sub = parser.add_subparsers(dest="cmd", required=True)
    new = sub.add_parser("new", help="scaffold a plugin folder")
    new.add_argument("id", help="plugin id, ^[a-z][a-z0-9_]{1,31}$")
    new.add_argument("--dir", default=".", help="parent directory (default: current)")
    args = parser.parse_args(argv)
    try:
        target = scaffold(args.id, Path(args.dir))
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"created {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
