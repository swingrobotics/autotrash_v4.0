#!/usr/bin/env python3

"""Entry point that also works from a copied DEV installation tree."""

import os
from pathlib import Path
import sys


def _force_utf8_stdio():
    """Keep third-party ML progress output safe on non-UTF-8 Windows locales.

    torch.onnx's dynamo exporter emits Unicode status glyphs (for example a
    check mark). A Korean Windows Python process commonly inherits cp949 for
    stdout/stderr, which can make a successful export fail only while printing
    progress. Configure the current process streams and any child Python
    processes for UTF-8 before importing the training stack.
    """

    os.environ.setdefault("PYTHONUTF8", "1")
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            try:
                reconfigure(encoding="utf-8", errors="replace")
            except (OSError, ValueError):
                pass


_force_utf8_stdio()

# When executed as app/scripts/run_compute_worker.py, Python puts only the
# scripts directory at sys.path[0]. Add the application root explicitly so the
# sibling swing_compute/ and autonomous_car/ packages can always be imported.
APP_ROOT = str(Path(__file__).resolve().parent.parent)
if APP_ROOT not in sys.path:
    sys.path.insert(0, APP_ROOT)

from swing_compute.fast_capabilities import install_fast_capabilities
from swing_compute.record_worker_extensions import install_record_worker_extensions
from swing_compute.gps_worker_extensions import install_gps_worker_extensions
from swing_compute.record_preview_worker_extensions import (
    install_record_preview_worker_extensions,
)

# /api/v1/status must stay responsive even when the first torch import on a
# CPU-only Windows host takes several seconds. GPU probing runs once in a
# background thread and later status polls receive the cached result.
install_fast_capabilities()

# Extend the existing safe job queue with USB import, standalone rover sync,
# JPEG-first training, H.264 replay generation and optional MCAP export.
install_record_worker_extensions()

# AUTO_GPS is a separate route-conditioned policy. Install this after the
# RECORD extension so it reuses recursive JPEG sync while keeping its own
# dataset/trainer/evaluator/exporter contract.
install_gps_worker_extensions()

# Model preview is installed last so it can reuse both recursive RECORD sync
# and AUTO_GPS route transfer while remaining diagnostic-only.
install_record_preview_worker_extensions()

# v0.3 keeps the existing RECORD/training worker and adds a dedicated,
# non-queued live UFLD endpoint. Motor/steering authority remains on the rover.
from swing_compute.ufld_live import main


if __name__ == "__main__":
    raise SystemExit(main())
