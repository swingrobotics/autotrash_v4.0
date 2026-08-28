import math


class OnnxParityError(RuntimeError):
    pass


def verify_onnx_parity(
    model,
    model_path,
    named_torch_inputs,
    *,
    torch,
    absolute_tolerance=1e-4,
    relative_tolerance=1e-4,
):
    """Compare PyTorch and CPU ONNX Runtime outputs on the same tensors.

    torch.onnx.export(verify=True) validates the exporter path, but the vehicle
    executes the generated artifact through ONNX Runtime. This check records an
    explicit numerical contract for that final runtime boundary.
    """
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as error:
        raise OnnxParityError(
            "ONNX parity verification requires NumPy and ONNX Runtime"
        ) from error

    model.eval()
    ordered_inputs = tuple(named_torch_inputs.values())
    with torch.no_grad():
        torch_output = model(*ordered_inputs).detach().cpu().numpy()

    session = ort.InferenceSession(
        model_path,
        providers=["CPUExecutionProvider"],
    )
    ort_inputs = {
        name: tensor.detach().cpu().numpy()
        for name, tensor in named_torch_inputs.items()
    }
    outputs = session.run(["control"], ort_inputs)
    if not outputs:
        raise OnnxParityError("ONNX Runtime returned no control output")
    ort_output = np.asarray(outputs[0])
    torch_output = np.asarray(torch_output)
    if ort_output.shape != torch_output.shape:
        raise OnnxParityError(
            f"ONNX parity shape mismatch: torch={torch_output.shape}, ort={ort_output.shape}"
        )
    if not np.isfinite(torch_output).all() or not np.isfinite(ort_output).all():
        raise OnnxParityError("ONNX parity output contains non-finite values")

    difference = np.abs(torch_output - ort_output)
    max_abs = float(difference.max()) if difference.size else 0.0
    mean_abs = float(difference.mean()) if difference.size else 0.0
    if not math.isfinite(max_abs) or not math.isfinite(mean_abs):
        raise OnnxParityError("ONNX parity error metric is non-finite")

    passed = bool(
        np.allclose(
            torch_output,
            ort_output,
            atol=float(absolute_tolerance),
            rtol=float(relative_tolerance),
        )
    )
    result = {
        "passed": passed,
        "absolute_tolerance": float(absolute_tolerance),
        "relative_tolerance": float(relative_tolerance),
        "maximum_absolute_error": max_abs,
        "mean_absolute_error": mean_abs,
        "torch_output": torch_output.reshape(-1).astype(float).tolist(),
        "onnxruntime_output": ort_output.reshape(-1).astype(float).tolist(),
        "provider": "CPUExecutionProvider",
    }
    if not passed:
        raise OnnxParityError(
            "PyTorch/ONNX Runtime parity failed: "
            f"max_abs={max_abs:.8g}, atol={absolute_tolerance}, rtol={relative_tolerance}"
        )
    return result


__all__ = ["OnnxParityError", "verify_onnx_parity"]
