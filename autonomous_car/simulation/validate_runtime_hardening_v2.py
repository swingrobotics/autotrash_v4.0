import ast
import json
from pathlib import Path

from autonomous_car.recording.record_manager import RecordManager


class CountingWriter:
    def __init__(self):
        self.rows = 0

    def writerow(self, row):
        self.rows += 1


class CountingFile:
    def __init__(self):
        self.flushes = 0

    def flush(self):
        self.flushes += 1


def _lock_contains_call(source, call_name):
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.With):
            continue
        uses_self_lock = any(
            isinstance(item.context_expr, ast.Attribute)
            and isinstance(item.context_expr.value, ast.Name)
            and item.context_expr.value.id == "self"
            and item.context_expr.attr == "lock"
            for item in node.items
        )
        if not uses_self_lock:
            continue
        if any(
            isinstance(child, ast.Call)
            and isinstance(child.func, ast.Attribute)
            and child.func.attr == call_name
            for child in ast.walk(node)
        ):
            return True
    return False


def _stop_signals_before_lock(source):
    marker = "def stop(self, reason=\"operator_stop\"):"
    if marker not in source:
        return False
    body = source.split(marker, 1)[1].split("\n    def ", 1)[0]
    event_pos = body.find(".set()")
    lock_pos = body.find("with self.lock:")
    return event_pos >= 0 and lock_pos >= 0 and event_pos < lock_pos


def validate():
    ai_source = Path("server_v2_ai.py").read_text(encoding="utf-8")
    gps_source = Path("server_v2_gps_ai.py").read_text(encoding="utf-8")
    ort_source = Path("autonomous_car/ai/ort_session.py").read_text(encoding="utf-8")

    # Heavy JPEG preprocessing + ONNX inference must not execute while the
    # controller lifecycle lock is held. Only final state validation/actuation
    # should use that short critical section.
    assert not _lock_contains_call(ai_source, "infer_jpeg")
    assert not _lock_contains_call(gps_source, "infer_jpeg")

    # STOP must wake the active run before waiting for the short output lock.
    assert _stop_signals_before_lock(ai_source)
    assert _stop_signals_before_lock(gps_source)
    assert "args=(generation, stop_event, runtime)" in ai_source
    assert "args=(generation, stop_event, runtime, extractor)" in gps_source
    assert '"control_loop_seconds"' in ai_source
    assert '"control_loop_seconds"' in gps_source

    # RecordManager already owns a dedicated writer thread. Rows should stay in
    # Python's buffered file layer and only flush on the periodic batch flush or
    # final close/fsync, not once per high-rate sensor sample.
    manager = RecordManager(
        "/tmp/record-hardening-test",
        sample_provider=lambda: {},
        camera_provider=lambda: (None, -1, None, None),
    )
    writer = CountingWriter()
    file = CountingFile()
    manager._writers["control"] = writer
    manager._files["control"] = file
    manager._write_row("control", {"monotonic": 1.0})
    assert writer.rows == 1
    assert file.flushes == 0
    manager._flush_streams(sync=False)
    assert file.flushes == 1
    assert manager.flush_interval_seconds >= 0.05

    # Keep ORT defaults conservative but expose the official tuning controls for
    # target-Pi benchmarking without changing the model/runtime contract.
    assert "AUTONOMY_ORT_INTRA_OP_THREADS" in ort_source
    assert "AUTONOMY_ORT_ALLOW_SPINNING" in ort_source
    assert "ORT_SEQUENTIAL" in ort_source

    return {
        "auto_ai_inference_outside_lifecycle_lock": "PASS",
        "auto_gps_inference_outside_lifecycle_lock": "PASS",
        "per_run_stop_event_and_generation": "PASS",
        "record_periodic_buffered_flush": "PASS",
        "onnx_threading_is_pi_tunable": "PASS",
        "control_loop_latency_is_exposed": "PASS",
    }


def main():
    result = validate()
    print("Autonomy V2 runtime hardening: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
