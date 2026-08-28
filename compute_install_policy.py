"""Policy wrapper for Compute Worker candidate installation.

This is separate from binary transfer mechanics so policy can remain explicit:
model IDs are immutable, and QUICK candidates that regress on the fixed held-out
set cannot be installed even if a private-LAN client calls the API directly.
"""

import json
from urllib.parse import urlparse

import server_v2_release as release
from autonomous_car.ai import ModelRegistryError
from compute_rover_api import _worker_get_json, _worker_url_allowed


_INSTALLED = False


def install_compute_candidate_policy():
    global _INSTALLED
    if _INSTALLED:
        return True
    original = release.ReleaseHandler

    class ComputeCandidatePolicyHandler(original):
        def do_POST(self):
            if urlparse(self.path).path != "/api/v2/compute/model/install":
                super().do_POST()
                return
            try:
                payload = self._read_json()
                model_id = release.full.ai.MODEL_REGISTRY._normalize_id(
                    payload.get("model_id")
                )
                try:
                    release.full.ai.MODEL_REGISTRY.get(model_id)
                except ModelRegistryError:
                    pass
                else:
                    raise ValueError("MODEL_ID_ALREADY_EXISTS")

                urls = [
                    str(value).rstrip("/")
                    for value in payload.get("worker_urls") or []
                    if _worker_url_allowed(value)
                ]
                if not urls:
                    raise ValueError("NO_PRIVATE_WORKER_URL")
                job_id = str(payload.get("job_id") or "").strip()
                token = str(payload.get("artifact_token") or "").strip()
                job = None
                last_error = None
                for url in urls:
                    try:
                        job = _worker_get_json(
                            url, f"/api/v1/jobs/{job_id}", token
                        )
                        break
                    except Exception as error:
                        last_error = error
                if job is None:
                    raise ValueError(f"WORKER_JOB_UNREACHABLE:{last_error}")
                result = job.get("result") or {}
                if str(result.get("model_id") or "") != model_id:
                    raise ValueError("WORKER_MODEL_ID_MISMATCH")
                if str(result.get("mode") or "").upper() == "QUICK":
                    comparison = result.get("regression_comparison") or {}
                    if comparison.get("regression_guard_passed") is not True:
                        raise ValueError("QUICK_REGRESSION_GUARD_FAILED")

                # Lower handler must consume exactly the same JSON body. The
                # final service _read_json() honors this cache on keep-alive.
                self._cached_json_payload = payload
                super().do_POST()
            except (ValueError, OSError, TypeError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error)}, 409)

    release.ReleaseHandler = ComputeCandidatePolicyHandler
    _INSTALLED = True
    return True


__all__ = ["install_compute_candidate_policy"]
