#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="${SWING_PRETRAINED_ROAD_MODEL:-$ROOT/models/pretrained/ultrafast_lane_tusimple_288x800.onnx}"
PRIMARY_URL="${SWING_PRETRAINED_ROAD_URL:-https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/140_Ultra-Fast-Lane-Detection/resources_tusimple.tar.gz}"
SECONDARY_URL="https://ap-northeast-2.wasabisys.com/pinto-model-zoo/140_Ultra-Fast-Lane-Detection/resources_tusimple.tar.gz"

# Raspberry Pi OS commonly mounts /tmp as a small RAM-backed tmpfs. The upstream
# PINTO bundle is ~6.5 GB, so default to disk-backed project storage unless the
# operator explicitly provides TMPDIR/SWING_UFLD_TMPDIR.
WORK_BASE="${SWING_UFLD_TMPDIR:-${TMPDIR:-$ROOT/.cache/ufld}}"
mkdir -p "$(dirname "$TARGET")" "$WORK_BASE"

# The archive plus extracted ONNX coexist during validation. Fail early with a
# useful message instead of letting curl terminate with error 23 mid-download.
AVAILABLE_KB="$(df -Pk "$WORK_BASE" | awk 'NR==2 {print $4}')"
MINIMUM_KB=$((8 * 1024 * 1024))
if [ "${AVAILABLE_KB:-0}" -lt "$MINIMUM_KB" ]; then
  echo "Insufficient free space for UFLD installation in: $WORK_BASE" >&2
  echo "Need at least 8 GiB free; available: $(( ${AVAILABLE_KB:-0} / 1024 )) MiB" >&2
  echo "Set SWING_UFLD_TMPDIR to a disk-backed path with enough free space." >&2
  exit 1
fi

WORK="$(mktemp -d "$WORK_BASE/swing-ufld.XXXXXX")"
ARCHIVE="$WORK/resources_tusimple.tar.gz"
EXTRACTED="$WORK/ultrafast_lane_tusimple_288x800.onnx"
trap 'rm -rf "$WORK"' EXIT

echo "Installing external pretrained lane model"
echo "  detector: Ultra-Fast-Lane-Detection TuSimple ResNet18"
echo "  ONNX contract: 1x3x288x800 -> 1x101x56x4"
echo "  target: $TARGET"
echo "  work dir: $WORK_BASE"

downloaded=0
for URL in "$PRIMARY_URL" "$SECONDARY_URL"; do
  [ -n "$URL" ] || continue
  echo "  source: $URL"
  if curl --fail --location --retry 4 --retry-delay 2 --connect-timeout 15 \
      --output "$ARCHIVE" "$URL"; then
    downloaded=1
    break
  fi
  rm -f "$ARCHIVE"
done
if [ "$downloaded" -ne 1 ]; then
  echo "Could not download the UFLD TuSimple model archive" >&2
  exit 1
fi

python3 - "$ARCHIVE" "$EXTRACTED" <<'PY'
import os
import sys
import tarfile

archive, output = sys.argv[1:]
with tarfile.open(archive, "r:gz") as bundle:
    files = [member for member in bundle.getmembers() if member.isfile()]
    onnx = [member for member in files if member.name.lower().endswith(".onnx")]
    preferred = [
        member for member in onnx
        if "tusimple" in member.name.lower()
        and ("288x800" in member.name.lower() or "800x288" in member.name.lower())
    ]
    if len(preferred) != 1:
        preferred = [
            member for member in onnx
            if "288x800" in member.name.lower() or "800x288" in member.name.lower()
        ]
    if len(preferred) != 1 and len(onnx) == 1:
        preferred = onnx
    if len(preferred) != 1:
        names = ", ".join(member.name for member in onnx)
        raise SystemExit(f"UFLD 288x800 ONNX member not uniquely found: {names}")
    member = preferred[0]
    source = bundle.extractfile(member)
    if source is None:
        raise SystemExit(f"Could not read archive member: {member.name}")
    with source, open(output, "wb") as target:
        while True:
            chunk = source.read(1024 * 1024)
            if not chunk:
                break
            target.write(chunk)
print(f"extracted_member={member.name}")
print(f"extracted_bytes={os.path.getsize(output)}")
PY

# Validate the exact external UFLD inference contract before replacing the
# installed model. The PINTO TuSimple artifact is currently ~245 MB; allow
# reasonable packaging/export variation while still rejecting implausible files.
python3 - "$EXTRACTED" <<'PY'
import hashlib
import os
import sys

import numpy as np
import onnxruntime as ort

path = sys.argv[1]
size = os.path.getsize(path)
if not 100_000_000 <= size <= 500_000_000:
    raise SystemExit(f"unexpected UFLD model size: {size} bytes")
with open(path, "rb") as file:
    sha256 = hashlib.sha256(file.read()).hexdigest()

options = ort.SessionOptions()
options.intra_op_num_threads = 2
options.inter_op_num_threads = 1
options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
try:
    options.add_session_config_entry("session.intra_op.allow_spinning", "0")
    options.add_session_config_entry("session.inter_op.allow_spinning", "0")
except Exception:
    pass
session = ort.InferenceSession(path, sess_options=options, providers=["CPUExecutionProvider"])
inputs = session.get_inputs()
outputs = session.get_outputs()
if len(inputs) != 1 or len(outputs) != 1:
    raise SystemExit(f"expected 1 UFLD input/output, found {len(inputs)}/{len(outputs)}")
input_shape = list(inputs[0].shape)
output_shape = list(outputs[0].shape)
if input_shape != [1, 3, 288, 800]:
    raise SystemExit(f"unexpected UFLD input shape: {input_shape}")
if output_shape != [1, 101, 56, 4]:
    raise SystemExit(f"unexpected UFLD output shape: {output_shape}")
values = session.run(
    [outputs[0].name],
    {inputs[0].name: np.zeros((1, 3, 288, 800), dtype=np.float32)},
)[0]
if values.shape != (1, 101, 56, 4) or not np.all(np.isfinite(values)):
    raise SystemExit(f"UFLD ONNX smoke inference invalid: {values.shape}")
print(f"verified bytes={size} sha256={sha256}")
print(f"input={inputs[0].name}:{input_shape}")
print(f"output={outputs[0].name}:{output_shape}")
PY

chmod 0644 "$EXTRACTED"
mv -f "$EXTRACTED" "$TARGET"
python3 - "$TARGET" <<'PY'
import hashlib
import pathlib
import sys
path = pathlib.Path(sys.argv[1])
digest = hashlib.sha256(path.read_bytes()).hexdigest()
path.with_suffix(path.suffix + ".sha256").write_text(digest + "\n", encoding="ascii")
print(f"installed_sha256={digest}")
PY
chmod 0644 "${TARGET}.sha256"
trap - EXIT
rm -rf "$WORK"

echo "Installed: $TARGET"
echo "License: Ultra-Fast-Lane-Detection/PINTO artifact is MIT; see THIRD_PARTY_MODELS.md"
