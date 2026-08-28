from __future__ import annotations

import threading

from . import worker as worker_module


_PATCHED = False


def install_fast_capabilities():
    """Make /status cheap while expensive torch/CUDA probing runs once in background."""
    global _PATCHED
    if _PATCHED:
        return

    def capabilities(self):
        lock = getattr(self, "_capability_probe_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._capability_probe_lock = lock

        with lock:
            cache = getattr(self, "_capability_probe_cache", None)
            if cache is None:
                torch_available = self._module_available("torch")
                cache = {
                    "local_cpu_training": torch_available,
                    "onnx_runtime": self._module_available("onnxruntime"),
                    "openvino": self._module_available("openvino"),
                    "opencv": self._module_available("cv2"),
                    "remote_gpu_training": False,
                    "record_cache": True,
                    "cuda": False,
                    "mps": False,
                    "gpu_name": None,
                    "probe_pending": bool(torch_available),
                    "probe_error": None,
                }
                self._capability_probe_cache = cache

            if cache.get("probe_pending") and not getattr(
                self, "_capability_probe_started", False
            ):
                self._capability_probe_started = True
                threading.Thread(
                    target=_probe_torch,
                    args=(self,),
                    name="swing-capability-probe",
                    daemon=True,
                ).start()
            return dict(cache)

    def _probe_torch(self):
        error_text = None
        cuda = False
        mps = False
        gpu_name = None
        try:
            import torch

            cuda = bool(torch.cuda.is_available())
            mps = bool(
                getattr(torch.backends, "mps", None)
                and torch.backends.mps.is_available()
            )
            if cuda:
                gpu_name = str(torch.cuda.get_device_name(0))
        except Exception as error:  # capability probing must never break status
            error_text = f"{type(error).__name__}: {error}"

        lock = getattr(self, "_capability_probe_lock", threading.RLock())
        with lock:
            cache = dict(getattr(self, "_capability_probe_cache", {}) or {})
            cache.update(
                {
                    "cuda": cuda,
                    "mps": mps,
                    "gpu_name": gpu_name,
                    "probe_pending": False,
                    "probe_error": error_text,
                }
            )
            self._capability_probe_cache = cache

    worker_module.ComputeWorker.capabilities = capabilities
    worker_module.ComputeWorker._probe_torch_capabilities = _probe_torch
    _PATCHED = True


__all__ = ["install_fast_capabilities"]
