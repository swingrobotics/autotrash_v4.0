import os


def _default_intra_op_threads():
    """Leave CPU headroom for camera/sensor/control threads on the rover."""

    count = os.cpu_count() or 1
    if count <= 2:
        return 1
    return 2


def build_cpu_session_options(ort):
    """Create deterministic, Pi-friendly ONNX Runtime CPU session options.

    The vehicle runs inference beside camera capture, LiDAR, IMU, HTTP and motor
    control threads. Letting ONNX Runtime occupy every core and spin while idle
    can increase scheduler jitter even when the ONNX call itself is fast. Keep
    the default inference pool small and disable spinning; operators can still
    override both settings after benchmarking their target hardware.

    AUTONOMY_ORT_INTRA_OP_THREADS=0 explicitly restores ONNX Runtime's automatic
    physical-core selection. AUTONOMY_ORT_ALLOW_SPINNING=1 restores spinning.
    """

    options = ort.SessionOptions()
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

    default_threads = _default_intra_op_threads()
    raw_threads = os.environ.get(
        "AUTONOMY_ORT_INTRA_OP_THREADS",
        str(default_threads),
    )
    try:
        intra_threads = int(raw_threads)
    except (TypeError, ValueError):
        intra_threads = default_threads
    intra_threads = max(0, min(64, intra_threads))
    options.intra_op_num_threads = intra_threads

    allow_spinning = str(
        os.environ.get("AUTONOMY_ORT_ALLOW_SPINNING", "0")
    ).strip().lower() not in {"0", "false", "no", "off"}
    options.add_session_config_entry(
        "session.intra_op.allow_spinning",
        "1" if allow_spinning else "0",
    )

    return options, {
        "intra_op_num_threads": intra_threads,
        "default_intra_op_num_threads": default_threads,
        "execution_mode": "ORT_SEQUENTIAL",
        "allow_spinning": allow_spinning,
    }


__all__ = ["build_cpu_session_options"]
