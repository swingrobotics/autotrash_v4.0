#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run with sudo: sudo bash scripts/install_dashboard_terminal.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TARGET_USER="${SWING_TERMINAL_USER:-${SUDO_USER:-gnss}}"
TARGET_GROUP="${SWING_TERMINAL_GROUP:-$TARGET_USER}"
PORT="${SWING_TERMINAL_PORT:-7681}"
AUTH_USER="${SWING_TERMINAL_AUTH_USER:-swing}"
ENV_FILE="/etc/default/swing-terminal"
UNIT_FILE="/etc/systemd/system/swing-dashboard-terminal.service"
RUNNER="$PROJECT_ROOT/scripts/run_dashboard_terminal.sh"

if ! id "$TARGET_USER" >/dev/null 2>&1; then
  echo "SWING terminal user does not exist: $TARGET_USER" >&2
  exit 2
fi

if [[ -n "${SWING_TERMINAL_INTERFACE:-}" ]]; then
  INTERFACE="$SWING_TERMINAL_INTERFACE"
else
  INTERFACE="$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.137\./ {print $2; exit}')"
  INTERFACE="${INTERFACE:-eth0}"
fi
if ! ip link show "$INTERFACE" >/dev/null 2>&1; then
  echo "Network interface not found: $INTERFACE" >&2
  ip -brief link >&2 || true
  exit 3
fi

if ! command -v ttyd >/dev/null 2>&1; then
  echo "Installing ttyd..."
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y ttyd
fi
TTYD_BIN="$(command -v ttyd)"

EXISTING_CREDENTIAL=""
if [[ -f "$ENV_FILE" ]]; then
  EXISTING_CREDENTIAL="$(sed -n 's/^SWING_TERMINAL_CREDENTIAL="\(.*\)"$/\1/p' "$ENV_FILE" | head -1)"
fi
if [[ -n "$EXISTING_CREDENTIAL" ]]; then
  CREDENTIAL="$EXISTING_CREDENTIAL"
else
  if command -v openssl >/dev/null 2>&1; then
    PASSWORD="$(openssl rand -base64 18 | tr -d '\n=')"
  else
    PASSWORD="$(python3 - <<'PY'
import secrets
import string
alphabet = string.ascii_letters + string.digits
print(''.join(secrets.choice(alphabet) for _ in range(24)))
PY
)"
  fi
  CREDENTIAL="${AUTH_USER}:${PASSWORD}"
fi

install -d -m 0755 /etc/default
cat >"$ENV_FILE" <<EOF
SWING_TERMINAL_TTYD_BIN="$TTYD_BIN"
SWING_TERMINAL_INTERFACE="$INTERFACE"
SWING_TERMINAL_PORT="$PORT"
SWING_TERMINAL_CREDENTIAL="$CREDENTIAL"
SWING_TERMINAL_CWD="$PROJECT_ROOT"
SWING_TERMINAL_MAX_CLIENTS="2"
EOF
chmod 0600 "$ENV_FILE"
chown root:root "$ENV_FILE"
chmod 0755 "$RUNNER"

cat >"$UNIT_FILE" <<EOF
[Unit]
Description=SWING Rover private-LAN dashboard terminal
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$TARGET_USER
Group=$TARGET_GROUP
WorkingDirectory=$PROJECT_ROOT
EnvironmentFile=$ENV_FILE
ExecStart=$RUNNER
Restart=on-failure
RestartSec=2
KillSignal=SIGTERM
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
EOF
chmod 0644 "$UNIT_FILE"

systemctl daemon-reload
systemctl enable --now swing-dashboard-terminal.service
sleep 1

if ! systemctl is-active --quiet swing-dashboard-terminal.service; then
  echo "SWING terminal service failed to start." >&2
  systemctl --no-pager --full status swing-dashboard-terminal.service >&2 || true
  exit 4
fi

IP_ADDRESS="$(ip -o -4 addr show dev "$INTERFACE" 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}')"
IP_ADDRESS="${IP_ADDRESS:-192.168.137.2}"
AUTH_NAME="${CREDENTIAL%%:*}"
AUTH_PASSWORD="${CREDENTIAL#*:}"

cat <<EOF

SWING dashboard terminal installed.
  service   : swing-dashboard-terminal.service
  user      : $TARGET_USER
  interface : $INTERFACE
  url       : http://$IP_ADDRESS:$PORT/
  login     : $AUTH_NAME
  password  : $AUTH_PASSWORD

The credential is stored root-only in $ENV_FILE.
The terminal runs as Linux user '$TARGET_USER'; sudo still requires that user's normal sudo password.
EOF
