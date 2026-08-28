import json
import math
import os
from pathlib import Path
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from server import INDEX_HTML, LOGO_PATH


HOST = "127.0.0.1"
PORT = int(os.environ.get("PREVIEW_PORT", "8081"))


def steering_state():
    return {
        "connected": True,
        "port": "PREVIEW",
        "enabled": False,
        "throttle": 0.0,
        "encoder_connected": True,
        "encoder_raw": 2865,
        "encoder_zero_raw": 2865,
        "steering_angle_degrees": 12.0,
        "steer_right_raw_limit": 2629,
        "steer_left_raw_limit": 3164,
        "steer_right_reference_raw": 2679,
        "steer_center_raw": 2865,
        "steer_left_reference_raw": 3114,
        "steer_limit_allowance_raw": 50,
        "config_supported": False,
        "center_supported": False,
    }


def imu_state():
    return {
        "connected": True,
        "bus_online": True,
        "bus": "PREVIEW",
        "addresses": ["0x69"],
        "orientation": {
            "heading_degrees": 127.4,
            "global_heading_degrees": 127.4,
            "relative_yaw_degrees": 18.2,
            "yaw_rate_dps": 0.0,
            "turn_direction": "RIGHT",
            "roll_degrees": 0.8,
            "pitch_degrees": -1.3,
            "calibrated": True,
            "calibration": {
                "active": False,
                "progress": 1.0,
                "message": "보정 완료",
            },
        },
    }


def lidar_state():
    points = []
    for bearing in range(-90, 91, 2):
        distance = 2200 + 450 * math.sin(math.radians(bearing * 3))
        if -14 <= bearing <= 18:
            distance = 1050 + abs(bearing) * 8
        points.append(
            {
                "bearing_degrees": bearing,
                "distance_mm": round(distance),
                "confidence": 220,
            }
        )
    return {
        "connected": True,
        "device": "PREVIEW",
        "rotation_hz": 10.0,
        "point_count": len(points),
        "scan_point_count": len(points),
        "points": points,
        "last_update": time.time(),
        "error": None,
        "camera_yaw_degrees": 0.0,
        "camera_fov_degrees": 82.1,
        "max_overlay_distance_mm": 4000,
    }


def ntrip_state():
    return {
        "host": "caster.example.com",
        "port": 2101,
        "mountpoint": "VRS-RTCM32",
        "username": "preview",
        "tls": False,
        "enabled": False,
        "password_saved": False,
        "configured": True,
        "connected": False,
        "status": "STOPPED",
        "bytes_received": 0,
        "correction_messages": 0,
        "last_correction": None,
        "last_gga": None,
        "error": None,
    }


def wifi_scan_state():
    return {
        "wifi": {
            "connected": False,
            "state": "DISCONNECTED",
            "ssid": None,
            "signal": None,
            "ipv4": None,
        },
        "networks": [
            {"active": False, "ssid": "GNSS-LAB", "signal": 92, "security": "WPA2"},
            {"active": False, "ssid": "Vehicle-Hotspot", "signal": 74, "security": "WPA2"},
            {"active": False, "ssid": "OPEN-NET", "signal": 48, "security": "OPEN"},
        ],
        "error": None,
    }


def status_state():
    imu = imu_state()
    lidar = lidar_state()
    steering = steering_state()
    return {
        "system": {
            "hostname": "RASPBERRY-PI",
            "time": time.time(),
            "cpu_count": 4,
            "cpu_load_percent": 18.0,
            "temperature_c": 46.2,
            "memory_total_bytes": 4 * 1024 ** 3,
            "memory_used_bytes": 1.7 * 1024 ** 3,
            "disk_total_bytes": 64 * 1024 ** 3,
            "disk_used_bytes": 19 * 1024 ** 3,
            "uptime_seconds": 18342,
        },
        "camera": {
            "online": True,
            "device": "PREVIEW",
            "size": "1280x720",
            "framerate": 30,
        },
        "devices": {
            "gps": {
                "connected": True,
                "port": "PREVIEW",
                "mode": 3,
                "fix": "3D FIX",
                "latitude": 37.5665,
                "longitude": 126.9780,
                "altitude_m": 38.0,
                "satellites_used": 18,
                "hdop": 0.72,
            },
            "imu": imu,
            "lidar": lidar,
            "arduino": steering,
        },
        "network": {
            "wifi": {"connected": False, "ssid": ""},
            "ethernet": {"interface": "LOCAL", "ipv4": HOST},
            "internet": {"online": False, "latency_ms": None, "via": "local"},
            "client": {"ip": HOST, "ping_ok": True, "latency_ms": 0.1},
        },
        "navigation": {"mode": "standby", "motors_enabled": False, "throttle": 0.0},
        "ntrip": ntrip_state(),
    }


CAMERA_PLACEHOLDER = b"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
<rect width="1280" height="720" fill="#070c10"/>
<path d="M0 590L360 350L600 500L850 270L1280 570V720H0Z" fill="#13252d"/>
<path d="M590 720L625 410H655L690 720" fill="#41e4d2" opacity=".18"/>
</svg>"""


class PreviewHandler(BaseHTTPRequestHandler):
    def send_bytes(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, payload, status=200):
        self.send_bytes(
            json.dumps(payload).encode("utf-8"),
            "application/json; charset=utf-8",
            status,
        )

    def do_GET(self):
        routes = {
            "/api/status": status_state,
            "/api/imu": imu_state,
            "/api/lidar": lidar_state,
            "/api/steering": steering_state,
            "/api/ntrip": ntrip_state,
            "/api/network/wifi/scan": wifi_scan_state,
        }
        if self.path == "/":
            self.send_bytes(INDEX_HTML, "text/html; charset=utf-8")
        elif self.path == "/assets/swing-logo-white.png":
            with open(LOGO_PATH, "rb") as logo_file:
                self.send_bytes(logo_file.read(), "image/png")
        elif self.path == "/stream.mjpg":
            self.send_bytes(CAMERA_PLACEHOLDER, "image/svg+xml")
        elif self.path in routes:
            self.send_json(routes[self.path]())
        elif self.path == "/health":
            self.send_bytes(b"ok\n", "text/plain; charset=utf-8")
        else:
            self.send_error(404)

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length:
            self.rfile.read(content_length)
        if self.path == "/api/ntrip/config":
            state = ntrip_state()
            state.update(status="CONNECTING", enabled=True)
            self.send_json(state, 202)
        elif self.path == "/api/ntrip/stop":
            self.send_json(ntrip_state())
        elif self.path == "/api/network/wifi/connect":
            self.send_json(
                {
                    "connected": True,
                    "wifi": {
                        "connected": True,
                        "state": "CONNECTED",
                        "ssid": "GNSS-LAB",
                        "signal": -45,
                        "ipv4": "192.168.1.50",
                    },
                    "internet": {"online": True, "latency_ms": 12.0, "via": "wlan0"},
                },
                202,
            )
        elif self.path == "/api/network/wifi/disconnect":
            self.send_json({"connected": False, "wifi": wifi_scan_state()["wifi"]})
        elif self.path in {"/api/system/poweroff", "/api/system/reboot"}:
            self.send_json({"accepted": True, "preview": True}, 202)
        else:
            self.send_json({"error": "Controls are disabled in local preview."}, 403)

    def log_message(self, format_string, *args):
        return


if __name__ == "__main__":
    print(f"Local layout preview: http://{HOST}:{PORT}", flush=True)
    ThreadingHTTPServer((HOST, PORT), PreviewHandler).serve_forever()
