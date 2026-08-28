#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

section() { printf '\n=== %s ===\n' "$1"; }
run() {
  printf '$ %s\n' "$*"
  "$@" 2>&1 || true
}

section "Repository"
run git status --short --branch
run git rev-parse HEAD
run git branch --show-current

section "Platform"
run uname -a
run /usr/bin/python3 --version
run id
if id gnss >/dev/null 2>&1; then
  run id gnss
else
  echo "gnss user: NOT FOUND"
fi

section "Installed systemd service"
run systemctl cat camera-stream
run systemctl show camera-stream \
  -p LoadState -p ActiveState -p SubState \
  -p User -p Group -p SupplementaryGroups \
  -p MainPID -p ExecMainStatus -p Restart \
  -p KillMode -p KillSignal -p TimeoutStopUSec

section "Persistent serial identities"
run ls -la /dev/serial/by-id/
run readlink -f /dev/ttyACM0
run readlink -f /dev/ttyACM1

section "Configured device resolution"
if [[ -x "$ROOT/.venv/bin/python3" ]]; then
  "$ROOT/.venv/bin/python3" - <<'PY' 2>&1 || true
from camera_stream import config
print("CAMERA_DEVICE=", config.CAMERA_DEVICE)
print("GPS_DEVICE=", config.GPS_DEVICE)
print("ARDUINO_DEVICE=", config.ARDUINO_DEVICE)
print("LIDAR_DEVICE=", config.LIDAR_DEVICE)
PY
else
  echo ".venv python not found"
fi

section "AI runtime dependencies and selected model"
if [[ -x "$ROOT/.venv/bin/python3" ]]; then
  SWING_AUDIT_ROOT="$ROOT" "$ROOT/.venv/bin/python3" - <<'PY' 2>&1 || true
import json
import os

root = os.environ["SWING_AUDIT_ROOT"]
models_root = os.environ.get("AUTONOMY_MODELS_PATH", os.path.join(root, "models"))

try:
    import cv2
    import numpy
    import onnxruntime as ort
    print("cv2=", cv2.__version__)
    print("numpy=", numpy.__version__)
    print("onnxruntime=", ort.__version__)
    print("providers=", ort.get_available_providers())
except Exception as error:
    print("AI_DEPENDENCY_ERROR=", f"{type(error).__name__}: {error}")
    raise

from autonomous_car.ai import AutoAiRuntime, ModelRegistry

selection_path = os.path.join(models_root, "selected-model.json")
if not os.path.isfile(selection_path):
    print("selected AUTO_AI model: NONE")
else:
    with open(selection_path, "r", encoding="utf-8") as file:
        selected = json.load(file)
    model_id = str(selected.get("model_id") or "").strip()
    print("selected_model_id=", model_id or "NONE")
    if model_id:
        registry = ModelRegistry(models_root)
        model = registry.get(model_id)
        model_path = os.path.join(models_root, os.path.basename(model["model_file"]))
        manifest_path = os.path.join(models_root, os.path.basename(model["manifest_file"]))
        print("model_path=", model_path)
        print("manifest_path=", manifest_path)
        with open(manifest_path, "r", encoding="utf-8") as file:
            manifest = json.load(file)
        export = manifest.get("export") or {}
        print("manifest_external_data=", export.get("external_data"))
        print("manifest_self_contained=", export.get("self_contained"))
        runtime = AutoAiRuntime(model_path, manifest_path)
        print("AUTO_AI_RUNTIME_LOAD=PASS")
        print(json.dumps(runtime.snapshot(), ensure_ascii=False, indent=2))
PY
else
  echo ".venv python not found"
fi

section "Device permissions"
for path in /dev/video0 /dev/gpiomem /dev/i2c-* /dev/ttyACM* /dev/serial0 /run/gpsd-control.sock; do
  for item in $path; do
    [[ -e "$item" ]] && run ls -l "$item"
  done
done

section "Listening sockets"
run ss -ltnp

section "Local service status"
if command -v curl >/dev/null 2>&1; then
  run curl --fail --silent --show-error --max-time 3 http://127.0.0.1:8080/api/v2/status
  run curl --fail --silent --show-error --max-time 3 http://127.0.0.1:8080/api/v2/performance
else
  echo "curl not installed"
fi

section "Runtime storage"
for path in \
  "$ROOT/recordings" \
  "$ROOT/models" \
  "$ROOT/gps-routes" \
  "$ROOT/local-maps"; do
  if [[ -e "$path" ]]; then
    run df -h "$path"
    run ls -ld "$path"
  fi
done

section "Validation reminder"
echo "This script is read-only. It does not start motors, steering, AUTO, mapping, reboot, or poweroff."
echo "Do not migrate camera-stream.service to User=gnss until the device/network permissions above are verified."
