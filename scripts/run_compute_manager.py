#!/usr/bin/env python3

"""Windows desktop entry point for the SWING Compute Worker manager."""

from pathlib import Path
import sys


APP_ROOT = str(Path(__file__).resolve().parent.parent)
if APP_ROOT not in sys.path:
    sys.path.insert(0, APP_ROOT)

from swing_compute.windows_manager import main


if __name__ == "__main__":
    raise SystemExit(main())
