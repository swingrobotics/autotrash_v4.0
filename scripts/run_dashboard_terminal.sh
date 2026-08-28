#!/usr/bin/env bash
set -euo pipefail

TTYD_BIN="${SWING_TERMINAL_TTYD_BIN:-/usr/bin/ttyd}"
INTERFACE="${SWING_TERMINAL_INTERFACE:-eth0}"
PORT="${SWING_TERMINAL_PORT:-7681}"
CREDENTIAL="${SWING_TERMINAL_CREDENTIAL:-}"
CWD="${SWING_TERMINAL_CWD:-/home/gnss/camera-stream}"
MAX_CLIENTS="${SWING_TERMINAL_MAX_CLIENTS:-2}"

if [[ ! -x "$TTYD_BIN" ]]; then
  echo "SWING terminal: ttyd not found at $TTYD_BIN" >&2
  exit 127
fi
if [[ -z "$CREDENTIAL" || "$CREDENTIAL" != *:* ]]; then
  echo "SWING terminal: SWING_TERMINAL_CREDENTIAL must be user:password" >&2
  exit 2
fi
if ! ip link show "$INTERFACE" >/dev/null 2>&1; then
  echo "SWING terminal: interface not found: $INTERFACE" >&2
  exit 3
fi
if [[ ! -d "$CWD" ]]; then
  echo "SWING terminal: working directory not found: $CWD" >&2
  exit 4
fi

exec "$TTYD_BIN" \
  --interface "$INTERFACE" \
  --port "$PORT" \
  --credential "$CREDENTIAL" \
  --writable \
  --check-origin \
  --max-clients "$MAX_CLIENTS" \
  --cwd "$CWD" \
  --terminal-type xterm-256color \
  --client-option fontSize=14 \
  --client-option disableLeaveAlert=true \
  /bin/bash -l
