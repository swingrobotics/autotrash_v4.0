#!/usr/bin/env python3
"""Final guarded Autonomy V2 service entrypoint."""

import hmac
import ipaddress
import json
import os
import signal
import socket
import threading
import time

import server_v2_release as release
from server_v2_gps_ai import install_gps_ai
from unified_dashboard import UNIFIED_DASHBOARD_HTML
from unified_dashboard_extras import UNIFIED_DASHBOARD_EXTRAS
from unified_dashboard_data_tools import UNIFIED_DASHBOARD_DATA_TOOLS
from v2_option_panel import V2_OPTION_PANEL
from vehicle_settings_panel import VEHICLE_SETTINGS_PANEL
from camera_calibration_panel import CAMERA_CALIBRATION_PANEL
from operator_hmi_style import OPERATOR_HMI_STYLE
from lane_dashboard_overlay import LANE_DASHBOARD_OVERLAY
from vehicle_runtime_settings import VehicleRuntimeSettings, VehicleSettingsError
from autonomous_car.ai import ModelRegistryError
from autonomous_car.full_runtime_hardening import install_full_runtime_hardening
from autonomous_car.perception.camera_calibration_session import (
    CameraCalibrationSession,
    CameraCalibrationSessionError,
)
from autonomous_car.production_guard import ProductionRuntimeGuard, shutdown_runtime
from autonomous_car.runtime_guard import install_manual_takeover_guards
from autonomous_car.runtime_metrics import collect_runtime_metrics
from autonomous_car.status_cache import install_gps_status_cache
from autonomous_car.web_requests import normalize_drive_mode_request


def main():
    gps_ai = install_gps_ai(release)

    # These replacements happen before release.main starts sensors/HTTP and
    # before any mode can be selected. All existing full-server functions refer
    # to the module globals dynamically, so they see the hardened instances.
    install_full_runtime_hardening(release.full, gps_ai=gps_ai)
    install_gps_status_cache(gps_ai)
    vehicle_settings = VehicleRuntimeSettings(release.full.legacy)
    camera_calibration_session = CameraCalibrationSession(
        release.full.legacy.camera_calibration,
        os.environ.get(
            "CAMERA_CHARUCO_SAMPLES_PATH",
            "/home/gnss/camera-stream/calibration/charuco-samples",
        ),
    )
    production_guard = ProductionRuntimeGuard(release, gps_ai=gps_ai)

    def calibration_snapshot(snapshot=None):
        result = dict(
            camera_calibration_session.snapshot()
            if snapshot is None
            else snapshot
        )
        editing = vehicle_settings.snapshot()
        result["editable"] = bool(editing.get("editable"))
        result["edit_block_reason"] = editing.get("edit_block_reason")
        return result

    def require_calibration_editable():
        editing = vehicle_settings.snapshot()
        if not editing.get("editable"):
            raise CameraCalibrationSessionError(
                editing.get("edit_block_reason")
                or "Stop the vehicle before changing camera calibration"
            )

    # Optional bearer/header protection for deployments exposed beyond the
    # trusted vehicle LAN. Leave AUTONOMY_API_TOKEN unset for the existing
    # same-origin dashboard workflow; tokenless writes are then limited to
    # loopback/private/link-local client addresses.
    api_token = str(os.environ.get("AUTONOMY_API_TOKEN") or "").strip()
    maximum_json_body_bytes = max(
        1024,
        int(os.environ.get("AUTONOMY_MAX_JSON_BODY_BYTES", str(256 * 1024))),
    )
    connection_timeout_seconds = max(
        1.0,
        float(os.environ.get("AUTONOMY_HTTP_CONNECTION_TIMEOUT_SECONDS", "10")),
    )
    maximum_connections = max(
        4,
        int(os.environ.get("AUTONOMY_HTTP_MAX_CONNECTIONS", "24")),
    )

    # ThreadingHTTPServer creates one worker per connection. Keep request workers
    # daemonized, bound the accept backlog and cap concurrently alive workers.
    # Per-connection socket timeouts are installed by FinalReleaseHandler.setup.
    base_http_server = release.full.legacy.ThreadingHTTPServer

    class HardenedThreadingHTTPServer(base_http_server):
        daemon_threads = True
        block_on_close = False
        allow_reuse_address = True
        request_queue_size = 32

        def __init__(self, *args, **kwargs):
            self._connection_slots = threading.BoundedSemaphore(maximum_connections)
            super().__init__(*args, **kwargs)

        def process_request(self, request, client_address):
            if not self._connection_slots.acquire(blocking=False):
                try:
                    request.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                request.close()
                return
            try:
                return super().process_request(request, client_address)
            except BaseException:
                self._connection_slots.release()
                raise

        def process_request_thread(self, request, client_address):
            try:
                return super().process_request_thread(request, client_address)
            finally:
                self._connection_slots.release()

    release.full.legacy.ThreadingHTTPServer = HardenedThreadingHTTPServer

    # install_gps_ai replaces ReleaseHandler with the GPS-aware handler. Wrap
    # that final handler for cross-mode safety and V1-primary/V2-option UI.
    gps_handler = release.ReleaseHandler

    class FinalReleaseHandler(gps_handler):
        def setup(self):
            super().setup()
            self.connection.settimeout(connection_timeout_seconds)

        def handle_one_request(self):
            # BaseHTTPRequestHandler instances are reused for HTTP/1.1 keep-alive
            # requests. The legacy JSON reader caches the body on the handler,
            # therefore clear it at the start of *every* request before any V2
            # layer can inspect the next POST body.
            if hasattr(self, "_cached_json_payload"):
                del self._cached_json_payload
            return super().handle_one_request()

        def _redirect(self, location):
            self.send_response(302)
            self.send_header("Location", location)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", "0")
            self.end_headers()

        @staticmethod
        def _advanced_v2_body():
            return UNIFIED_DASHBOARD_HTML.replace(
                b"</body>",
                UNIFIED_DASHBOARD_EXTRAS
                + UNIFIED_DASHBOARD_DATA_TOOLS
                + b"</body>",
                1,
            )

        def _request_body_preflight(self):
            transfer_encoding = str(self.headers.get("Transfer-Encoding") or "").strip()
            if transfer_encoding and transfer_encoding.lower() != "identity":
                self.close_connection = True
                self._send_json({"error": "TRANSFER_ENCODING_NOT_SUPPORTED"}, 400)
                return False

            value = self.headers.get("Content-Length")
            if value is None:
                return True
            try:
                length = int(value)
            except (TypeError, ValueError):
                self.close_connection = True
                self._send_json({"error": "INVALID_CONTENT_LENGTH"}, 400)
                return False
            if length < 0:
                self.close_connection = True
                self._send_json({"error": "INVALID_CONTENT_LENGTH"}, 400)
                return False
            if length > maximum_json_body_bytes:
                # Do not leave an unread oversized body on a persistent
                # connection; close it after the 413 response.
                self.close_connection = True
                self._send_json(
                    {
                        "error": "REQUEST_BODY_TOO_LARGE",
                        "maximum_bytes": maximum_json_body_bytes,
                    },
                    413,
                )
                return False
            return True

        def _read_json(self):
            if hasattr(self, "_cached_json_payload"):
                return self._cached_json_payload
            value = self.headers.get("Content-Length")
            try:
                content_length = 0 if value is None else int(value)
            except (TypeError, ValueError) as error:
                raise ValueError("INVALID_CONTENT_LENGTH") from error
            if content_length < 0 or content_length > maximum_json_body_bytes:
                raise ValueError("REQUEST_BODY_LENGTH_REJECTED")
            try:
                raw = self.rfile.read(content_length) if content_length else b"{}"
            except (TimeoutError, socket.timeout) as error:
                self.close_connection = True
                raise ValueError("REQUEST_BODY_TIMEOUT") from error
            if content_length and len(raw) != content_length:
                self.close_connection = True
                raise ValueError("INCOMPLETE_REQUEST_BODY")
            try:
                payload = json.loads(raw.decode("utf-8"))
            except UnicodeDecodeError as error:
                raise ValueError("REQUEST_BODY_NOT_UTF8") from error
            self._cached_json_payload = payload
            return payload

        @staticmethod
        def _private_client_address(address):
            try:
                ip = ipaddress.ip_address(str(address).split("%", 1)[0])
            except ValueError:
                return False
            return bool(ip.is_loopback or ip.is_private or ip.is_link_local)

        def _write_request_allowed(self):
            # Reject browser cross-origin writes. Requests without Origin (for
            # example local curl/system tooling) are only accepted from the rover
            # private network when no API token is configured.
            origin = str(self.headers.get("Origin") or "").strip().rstrip("/")
            if origin:
                host = str(self.headers.get("Host") or "").strip()
                allowed = {f"http://{host}", f"https://{host}"}
                if not host or origin not in allowed:
                    self._send_json({"error": "CROSS_ORIGIN_WRITE_REJECTED"}, 403)
                    return False

            if api_token:
                supplied = str(self.headers.get("X-Autonomy-Token") or "").strip()
                authorization = str(self.headers.get("Authorization") or "").strip()
                if not supplied and authorization.lower().startswith("bearer "):
                    supplied = authorization[7:].strip()
                if not supplied or not hmac.compare_digest(supplied, api_token):
                    self._send_json({"error": "AUTONOMY_API_TOKEN_REQUIRED"}, 401)
                    return False
            elif not self._private_client_address(self.client_address[0]):
                self._send_json({"error": "PRIVATE_NETWORK_WRITE_REQUIRED"}, 403)
                return False
            return True

        def do_GET(self):
            if self.path == "/":
                # Keep the original V1 operator dashboard and inject V2 mode
                # controls plus vehicle/camera tuning popups into its existing
                # operator surface. The HMI style is deliberately last so it
                # only overrides presentation without changing runtime logic.
                body = release.full.legacy.INDEX_HTML.replace(
                    b"</body>",
                    V2_OPTION_PANEL
                    + VEHICLE_SETTINGS_PANEL
                    + CAMERA_CALIBRATION_PANEL
                    + LANE_DASHBOARD_OVERLAY
                    + OPERATOR_HMI_STYLE
                    + b"</body>",
                    1,
                )
                self._send_html(body)
                return

            if self.path == "/v2":
                self._send_html(self._advanced_v2_body())
                return

            # Compatibility URLs point into V1 primary / V2 advanced pages.
            if self.path == "/legacy":
                self._redirect("/")
                return
            if self.path == "/ai-data":
                self._redirect("/v2#data")
                return
            if self.path == "/gps-ai":
                self._redirect("/v2#gps")
                return

            if self.path == "/api/vehicle/settings":
                self._send_json(vehicle_settings.snapshot())
                return

            if self.path == "/api/camera/calibration":
                self._send_json(calibration_snapshot())
                return

            if self.path == "/api/camera/calibration/preview":
                try:
                    frame, sequence, frame_monotonic, _ = (
                        release.full.legacy.camera.snapshot_frame()
                    )
                    preview = camera_calibration_session.preview(frame, sequence)
                    preview["data_age"] = (
                        time.monotonic() - frame_monotonic
                        if frame_monotonic is not None
                        else None
                    )
                    self._send_json(preview)
                except Exception as error:
                    self._send_json(
                        {
                            "error": f"{type(error).__name__}: {error}",
                            "valid": False,
                        },
                        503,
                    )
                return

            if self.path == "/api/v2/performance":
                metrics = collect_runtime_metrics(
                    release.full.legacy,
                    release.full,
                    gps_ai=gps_ai,
                )
                metrics["production_guard"] = production_guard.snapshot()
                self._send_json(metrics)
                return

            if self.path == "/api/v2/status":
                status = release.full.full_status()
                # Route-bound GPS models are managed in the GPS AI section and
                # must not be presented as route-independent AUTO_AI choices.
                status["ai"]["models"] = release.full.ai.MODEL_REGISTRY.list_models("AUTO_AI")
                status["ai"]["datasets"] = release._list_datasets()
                status["ai"]["dataset_build"] = release.DATASET_BUILD_CONTROLLER.snapshot()
                status["camera_calibration"] = calibration_snapshot()
                status["production_guard"] = production_guard.snapshot()
                status.setdefault("security", {}).update(
                    {
                        "cross_origin_write_guard": True,
                        "api_token_required": bool(api_token),
                        "maximum_json_body_bytes": maximum_json_body_bytes,
                        "connection_timeout_seconds": connection_timeout_seconds,
                        "maximum_connections": maximum_connections,
                        "unauthenticated_write_scope": (
                            None if api_token else "private_network_only"
                        ),
                        "internet_exposure_supported": False,
                    }
                )
                self._send_json(status)
                return

            if self.path == "/api/v2/ai/models":
                self._send_json(
                    {
                        "models": release.full.ai.MODEL_REGISTRY.list_models("AUTO_AI"),
                        "ai": release.full.ai.ai_status(),
                    }
                )
                return

            super().do_GET()

        def do_POST(self):
            if not self._request_body_preflight():
                return
            if not self._write_request_allowed():
                return

            if self.path == "/api/safety/emergency-stop" and gps_ai.controller.active:
                gps_ai.controller.stop("emergency_stop")

            if self.path == "/api/vehicle/settings":
                try:
                    payload = self._read_json()
                    self._send_json(vehicle_settings.update(payload), 202)
                except (
                    VehicleSettingsError,
                    ValueError,
                    OSError,
                    TypeError,
                    json.JSONDecodeError,
                ) as error:
                    snapshot = vehicle_settings.snapshot()
                    self._send_json({"error": str(error), **snapshot}, 409)
                return

            calibration_paths = {
                "/api/camera/calibration/configure",
                "/api/camera/calibration/capture",
                "/api/camera/calibration/remove-last",
                "/api/camera/calibration/reset",
                "/api/camera/calibration/solve",
            }
            if self.path in calibration_paths:
                try:
                    require_calibration_editable()
                    if self.path == "/api/camera/calibration/configure":
                        result = camera_calibration_session.configure(self._read_json())
                    elif self.path == "/api/camera/calibration/capture":
                        # Consume the optional {} body so keep-alive framing stays
                        # synchronized, then capture the freshest server frame.
                        self._read_json()
                        frame, sequence, _, _ = (
                            release.full.legacy.camera.snapshot_frame()
                        )
                        result = camera_calibration_session.capture(frame, sequence)
                    elif self.path == "/api/camera/calibration/remove-last":
                        self._read_json()
                        result = camera_calibration_session.remove_last()
                    elif self.path == "/api/camera/calibration/reset":
                        self._read_json()
                        result = camera_calibration_session.reset_samples()
                    else:
                        self._read_json()
                        result = camera_calibration_session.solve()
                    self._send_json(calibration_snapshot(result), 202)
                except (
                    CameraCalibrationSessionError,
                    ValueError,
                    RuntimeError,
                    OSError,
                    TypeError,
                    json.JSONDecodeError,
                ) as error:
                    self._send_json(
                        {
                            "error": str(error),
                            **calibration_snapshot(),
                        },
                        409,
                    )
                return

            # Validate mode requests before they reach DriveMode(...). Keep the
            # normalized payload cached so the lower V2 handler reads the exact
            # same request body without consuming rfile twice.
            if self.path == "/api/v2/mode":
                try:
                    payload = self._read_json()
                    mode, record_gps = normalize_drive_mode_request(payload)
                    normalized = dict(payload)
                    normalized["mode"] = mode
                    normalized["record_gps"] = record_gps
                    self._cached_json_payload = normalized
                except (ValueError, TypeError, json.JSONDecodeError) as error:
                    self._send_json({"error": str(error)}, 400)
                    return
                super().do_POST()
                return

            # The GPS-aware lower handler inspects this endpoint to prevent a
            # GPS model from being selected as route-independent AUTO_AI. Do the
            # complete AUTO_AI selection here so the JSON body is consumed only
            # once and the legacy AI handler never has to read it a second time.
            if self.path == "/api/v2/ai/select":
                try:
                    payload = self._read_json()
                    if release.full.ai.AUTO_AI_CONTROLLER.active:
                        raise ValueError("Stop AUTO_AI before changing models")
                    model = release.full.ai.MODEL_REGISTRY.get(payload.get("model_id"))
                    if str(model.get("policy_type") or "AUTO_AI") != "AUTO_AI":
                        raise ValueError("Use GPS AI selection for AUTO_GPS models")
                    selected = release.full.ai.select_ai_model(model["model_id"])
                    self._send_json(
                        {"selected": selected, "ai": release.full.ai.ai_status()},
                        202,
                    )
                except (
                    ValueError,
                    OSError,
                    TypeError,
                    ModelRegistryError,
                    json.JSONDecodeError,
                ) as error:
                    self._send_json(
                        {"error": str(error), "ai": release.full.ai.ai_status()},
                        409,
                    )
                return

            super().do_POST()

    release.ReleaseHandler = FinalReleaseHandler

    install_manual_takeover_guards(
        release.full.legacy,
        auto_ai_controller=release.full.ai.AUTO_AI_CONTROLLER,
        auto_gps_controller=gps_ai.controller,
        auto_local_controller=release.full.AUTO_LOCAL_CONTROLLER,
        auto_orchestrator=release.full.AUTO_ORCHESTRATOR,
    )
    production_guard.start()

    def termination_signal(_signum, _frame):
        # camera-stream.service sends SIGTERM to the Python main process and
        # gives the service a grace period before killing remaining children.
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, termination_signal)
    signal.signal(signal.SIGINT, termination_signal)
    try:
        release.main()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_runtime(release, gps_ai=gps_ai, production_guard=production_guard)


if __name__ == "__main__":
    main()
