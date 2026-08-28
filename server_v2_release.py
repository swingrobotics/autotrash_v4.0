#!/usr/bin/env python3
"""Guarded final entrypoint for the Autonomy V2 software stack."""

import json
import os
import threading
import time

import server_v2_full as full
from autonomous_car import DriveMode
from autonomous_car.ai import DatasetBuilder, ModelRegistryError
from autonomous_car.localization import MapStoreError


PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
DATASETS_ROOT = os.environ.get(
    "AUTONOMY_DATASETS_PATH",
    os.path.join(PROJECT_ROOT, "datasets"),
)

AI_DATA_HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AUTO_AI DATA</title><style>
:root{color-scheme:dark;--bg:#0b0e0c;--p:#141915;--l:#303932;--t:#eef3ee;--m:#919d93;--a:#b8d89a}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--t);font-family:system-ui,sans-serif}main{max-width:1050px;margin:auto;padding:18px}.panel{background:var(--p);border:1px solid var(--l);border-radius:12px;padding:14px;margin-bottom:12px}button,input{background:#202820;color:var(--t);border:1px solid #465248;border-radius:8px;padding:9px}button{cursor:pointer;font-weight:700}a{color:var(--a)}table{width:100%;border-collapse:collapse;font-size:12px}td,th{border-bottom:1px solid var(--l);padding:7px;text-align:left}.muted{color:var(--m)}pre{white-space:pre-wrap;font-size:11px;color:var(--m)}</style></head><body><main><p><a href="/">← V2 주행 화면</a></p><div class="panel"><h2>AUTO_AI DATASET</h2><p class="muted">RECORD 세션을 선택해 timestamp 동기화된 Dataset을 만듭니다. AUTO/FAULT/E-STOP 로그는 자동 거부되고, 새 RECORD의 안정화 LiDAR safety_points를 학습 입력으로 사용합니다.</p><input id="dataset-id" placeholder="dataset 이름 (선택)"><button id="build">선택 세션으로 Dataset 생성</button></div><div class="panel"><h3>RECORD 세션</h3><table><thead><tr><th></th><th>세션</th><th>설명</th><th>크기</th><th>GPS 경로</th></tr></thead><tbody id="sessions"></tbody></table></div><div class="panel"><h3>생성된 Dataset</h3><pre id="datasets"></pre></div><div class="panel"><h3>Build 상태</h3><pre id="status">loading...</pre></div><script>
const $=id=>document.getElementById(id);async function api(p,o={}){const r=await fetch(p,{cache:'no-store',...o});let d={};try{d=await r.json()}catch{}if(!r.ok)throw new Error(d.error||`HTTP ${r.status}`);return d}function post(p,b){return api(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})}function bytes(v){if(!Number.isFinite(v))return '-';if(v>1073741824)return(v/1073741824).toFixed(1)+' GB';if(v>1048576)return(v/1048576).toFixed(1)+' MB';return Math.round(v/1024)+' KB'}
async function refresh(){try{const [r,d]=await Promise.all([api('/api/recordings'),api('/api/v2/ai/datasets')]);$('sessions').innerHTML=(r.sessions||[]).map(s=>`<tr><td><input class="session" type="checkbox" value="${s.session}" ${s.active?'disabled':''}></td><td>${s.session}</td><td>${s.label||''}</td><td>${bytes(s.size_bytes)}</td><td>${s.has_route?'YES':'NO'}</td></tr>`).join('');$('datasets').textContent=JSON.stringify(d.datasets,null,2);$('status').textContent=JSON.stringify(d.build,null,2);$('build').disabled=!!d.build.active}catch(e){$('status').textContent=e.message}}$('build').onclick=async()=>{const sessions=[...document.querySelectorAll('.session:checked')].map(x=>x.value);try{await post('/api/v2/ai/datasets/build',{sessions,dataset_id:$('dataset-id').value||null});await refresh()}catch(e){alert(e.message)}};refresh();setInterval(async()=>{try{const d=await api('/api/v2/ai/datasets');$('datasets').textContent=JSON.stringify(d.datasets,null,2);$('status').textContent=JSON.stringify(d.build,null,2);$('build').disabled=!!d.build.active}catch{}},1500);
</script></main></body></html>""".encode("utf-8")


def _recording_samples_v2():
    """Extend legacy RECORD rows without changing the historical base server.

    lidar_raw keeps the original raw points for diagnostics and also stores the
    same temporally stabilized `safety_points` representation consumed by the
    AUTO_AI runtime. The aligned DatasetBuilder prefers safety_points for new
    recordings and remains backward compatible with old runs.
    """
    samples = full.legacy.recording_samples()
    raw = samples.get("lidar_raw")
    if isinstance(raw, dict):
        lidar = full.legacy.lidar_monitor.snapshot()
        extended = dict(raw)
        extended["safety_points"] = list(lidar.get("safety_points") or [])
        samples = dict(samples)
        samples["lidar_raw"] = extended
    return samples


def _list_datasets():
    if not os.path.isdir(DATASETS_ROOT):
        return []
    result = []
    for name in sorted(os.listdir(DATASETS_ROOT)):
        path = os.path.join(DATASETS_ROOT, name, "dataset.json")
        if not os.path.isfile(path):
            continue
        try:
            with open(path, "r", encoding="utf-8") as file:
                document = json.load(file)
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        result.append(
            {
                "dataset_id": document.get("dataset_id", name),
                "created_at": document.get("created_at"),
                "accepted_samples": document.get("accepted_samples"),
                "rejected_samples": document.get("rejected_samples"),
                "split_counts": document.get("split_counts"),
                "scenario_counts": document.get("scenario_counts"),
                "sessions": [
                    item.get("session")
                    for item in document.get("sessions") or []
                    if item.get("session")
                ],
            }
        )
    return result


def _datasets_using_recording(session_name):
    session_name = os.path.basename(str(session_name or "").strip())
    if not session_name:
        return []
    return [
        str(dataset.get("dataset_id") or "")
        for dataset in _list_datasets()
        if session_name in (dataset.get("sessions") or [])
    ]


def _recordings_root_for_sessions(session_names):
    """Resolve the live USB/legacy root selected by the final storage bridge."""
    roots = set()
    resolver = getattr(full.legacy, "recording_session_path", None)
    for value in session_names or ():
        raw = str(value or "").strip()
        name = os.path.basename(raw)
        if not raw or raw != name or name in {".", ".."}:
            raise ValueError(f"Invalid RECORD session name: {value}")
        if callable(resolver):
            session_path = os.path.abspath(os.path.realpath(resolver(name)))
        else:
            session_path = os.path.abspath(
                os.path.join(full.legacy.RECORDINGS_PATH, name)
            )
        if not os.path.isdir(session_path):
            raise FileNotFoundError(f"RECORD session not found: {name}")
        roots.add(os.path.dirname(session_path))
    if not roots:
        raise ValueError("Select at least one RECORD session")
    if len(roots) != 1:
        raise ValueError(
            "Selected RECORD sessions span multiple storage roots. "
            "Build them separately or use the Compute Worker."
        )
    return roots.pop()


_LEGACY_DELETE_RECORDING_SESSION = full.legacy.delete_recording_session


def _delete_recording_session_guarded(session_name):
    """Do not remove source videos that an AUTO_AI dataset still references."""
    dependencies = _datasets_using_recording(session_name)
    if dependencies:
        raise ValueError(
            "RECORDING_IN_USE_BY_DATASET: " + ", ".join(dependencies)
        )
    return _LEGACY_DELETE_RECORDING_SESSION(session_name)


class DatasetBuildController:
    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.dataset_id = None
        self.sessions = []
        self.started_at = None
        self.finished_at = None
        self.result = None
        self.error = None
        self.thread = None

    def start(self, sessions, dataset_id=None):
        sessions = [str(item).strip() for item in (sessions or []) if str(item).strip()]
        if not sessions:
            raise ValueError("Select at least one RECORD session")
        with self.lock:
            if self.active:
                raise ValueError("A dataset build is already active")
            if full.legacy.record_manager.active:
                raise ValueError("Stop RECORD before building a dataset")
            if full.MAPPING_CONTROLLER.active:
                raise ValueError("Stop mapping before building a dataset")
            self.active = True
            self.dataset_id = dataset_id
            self.sessions = sessions
            self.started_at = time.time()
            self.finished_at = None
            self.result = None
            self.error = None
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "dataset_id": self.dataset_id,
                "sessions": list(self.sessions),
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "result": self.result,
                "error": self.error,
            }

    def _run(self):
        try:
            # The removable-storage bridge is installed after this module is
            # imported, so resolve the selected session root at execution time.
            recordings_root = _recordings_root_for_sessions(self.sessions)
            builder = DatasetBuilder(
                recordings_root,
                DATASETS_ROOT,
            )
            result = builder.build(self.sessions, self.dataset_id)
            with self.lock:
                self.dataset_id = result.get("dataset_id")
                self.result = result
        except Exception as error:
            with self.lock:
                self.error = f"{type(error).__name__}: {error}"
        finally:
            with self.lock:
                self.active = False
                self.finished_at = time.time()


DATASET_BUILD_CONTROLLER = DatasetBuildController()


class ReleaseHandler(full.FullHandler):
    def do_GET(self):
        if self.path == "/":
            body = full.FULL_HTML.replace(
                b'<a href="/legacy">',
                b'<a href="/ai-data">AI DATA</a> &nbsp; <a href="/legacy">',
            )
            self._send_html(body)
            return
        if self.path == "/ai-data":
            self._send_html(AI_DATA_HTML)
            return
        if self.path == "/api/v2/ai/datasets":
            self._send_json(
                {
                    "datasets": _list_datasets(),
                    "build": DATASET_BUILD_CONTROLLER.snapshot(),
                }
            )
            return
        if self.path == "/api/v2/status":
            status = full.full_status()
            status["ai"]["models"] = full.ai.MODEL_REGISTRY.list_models()
            status["ai"]["datasets"] = _list_datasets()
            status["ai"]["dataset_build"] = DATASET_BUILD_CONTROLLER.snapshot()
            self._send_json(status)
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/v2/ai/datasets/build":
            try:
                payload = self._read_json()
                result = DATASET_BUILD_CONTROLLER.start(
                    payload.get("sessions"),
                    payload.get("dataset_id"),
                )
                self._send_json(result, 202)
            except (ValueError, OSError, TypeError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error)}, 409)
            return

        if self.path == "/api/v2/mode":
            try:
                payload = self._read_json()
                target = DriveMode(str(payload.get("mode") or "").strip().upper())
                if DATASET_BUILD_CONTROLLER.active and target == DriveMode.RECORD:
                    raise ValueError("Dataset build is active. Wait until it finishes before RECORD.")
                if full.MAPPING_CONTROLLER.active and target != DriveMode.MANUAL:
                    if target == DriveMode.DISARMED:
                        full.MAPPING_CONTROLLER.stop(save=True)
                    else:
                        raise ValueError(
                            "Mapping is active. Save/stop mapping before RECORD or autonomous mode changes."
                        )
                result = full.full_select_mode(
                    target.value,
                    record_gps=payload.get("record_gps", True),
                )
                self._send_json(result, 202 if result.get("accepted", False) else 409)
            except (
                ValueError,
                OSError,
                TypeError,
                MapStoreError,
                ModelRegistryError,
                json.JSONDecodeError,
            ) as error:
                self._send_json({"error": str(error), "status": full.full_status()}, 409)
            return
        super().do_POST()


def main():
    # Replace only safe compatibility hooks; legacy hardware/telemetry APIs and
    # their request routing remain intact.
    full.legacy.record_manager.sample_provider = _recording_samples_v2
    full.legacy.delete_recording_session = _delete_recording_session_guarded

    full.legacy.camera.start()
    full.legacy.gps_monitor.start()
    full.legacy.ntrip_client.start()
    full.legacy.imu_monitor.start()
    full.legacy.lidar_monitor.start()
    full.legacy.motor_controller.start()
    full.legacy.perception_monitor.start()
    httpd = full.legacy.ThreadingHTTPServer(
        (full.legacy.HOST, full.legacy.PORT),
        ReleaseHandler,
    )
    print(
        f"GNSS Autonomy V2 RELEASE listening on http://{full.legacy.HOST}:{full.legacy.PORT} "
        f"(AI data: /ai-data, legacy dashboard: /legacy)",
        flush=True,
    )
    httpd.serve_forever()


if __name__ == "__main__":
    main()
