#!/usr/bin/env python3
"""Autonomy V2 dashboard/API entrypoint.

This module intentionally reuses the proven hardware/telemetry backend in
server.py while the runtime is migrated from legacy names to the V2 modes.
The user-facing API exposes canonical MANUAL/RECORD/AUTO_AI/AUTO_GPS/
AUTO_LOCAL/AUTO semantics without pretending unfinished AI/SLAM runtimes exist.
"""

import json

import server as legacy
from autonomous_car import DriveMode
from autonomous_car.modes import AutoCapabilities, AutoGpsPlanner, AutoModeSelector


MODE_ORDER = [
    DriveMode.MANUAL,
    DriveMode.RECORD,
    DriveMode.AUTO_AI,
    DriveMode.AUTO_GPS,
    DriveMode.AUTO_LOCAL,
    DriveMode.AUTO,
]

AUTO_SELECTOR = AutoModeSelector()


V2_HTML = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GNSS Autonomy V2</title>
<style>
:root{color-scheme:dark;--bg:#0b0e0c;--panel:#141915;--line:#2d352f;--text:#eef3ee;--muted:#8f9a91;--ok:#9dcc82;--warn:#d7ba77;--bad:#ff7887;--accent:#abc98a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Inter,system-ui,sans-serif}
header{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:14px 18px;border-bottom:1px solid var(--line);background:#090c0a;position:sticky;top:0}
h1{font-size:15px;margin:0;letter-spacing:.08em}.sub{font-size:11px;color:var(--muted);margin-top:4px}
main{max-width:1180px;margin:auto;padding:18px}.statusbar{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-bottom:16px}
.card,.mode{border:1px solid var(--line);background:var(--panel);border-radius:12px;padding:14px}.card span{display:block;color:var(--muted);font-size:10px;letter-spacing:.08em}.card strong{display:block;margin-top:6px;font:700 14px ui-monospace,monospace}
.modegrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.mode{display:flex;flex-direction:column;min-height:170px}.mode.active{border-color:var(--accent);box-shadow:0 0 0 1px #abc98a55 inset}.mode h2{margin:0;font-size:15px}.mode p{color:var(--muted);font-size:12px;line-height:1.5;flex:1}.mode small{color:var(--muted)}
button{min-height:42px;border:1px solid #425047;border-radius:9px;background:#202820;color:var(--text);font-weight:750;cursor:pointer;padding:8px 12px}button:hover:not(:disabled){border-color:var(--accent)}button:disabled{opacity:.42;cursor:not-allowed}.primary{background:#263421;border-color:#607d56}.danger{background:#35151b;border-color:#7a3340;color:#ffadb7}.warning{background:#322a17;border-color:#6b592d;color:#efd08a}
.actions{display:flex;flex-wrap:wrap;gap:8px;margin:16px 0}.notice{margin-top:14px;border:1px solid var(--line);border-radius:10px;padding:12px;font:12px ui-monospace,monospace;color:var(--muted);white-space:pre-wrap}.good{color:var(--ok)!important}.warn{color:var(--warn)!important}.bad{color:var(--bad)!important}
.record-option{display:flex;align-items:center;gap:9px;margin:8px 0 10px;color:var(--text);font-size:12px}.record-option input{width:18px;height:18px}
a{color:var(--accent)}
@media(max-width:800px){.statusbar{grid-template-columns:repeat(2,minmax(0,1fr))}.modegrid{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <div><h1>GNSS AUTONOMY V2</h1><div class="sub">MANUAL · RECORD · AUTO_AI · AUTO_GPS · AUTO_LOCAL · AUTO</div></div>
  <div><a href="/legacy">기존 상세 대시보드</a></div>
</header>
<main>
<section class="statusbar">
 <div class="card"><span>CANONICAL MODE</span><strong id="canonical-mode">-</strong></div>
 <div class="card"><span>RUNTIME MODE</span><strong id="runtime-mode">-</strong></div>
 <div class="card"><span>RECORDING</span><strong id="recording">-</strong></div>
 <div class="card"><span>GPS ROUTE</span><strong id="gps-route">-</strong></div>
</section>

<section class="modegrid">
 <article class="mode" data-card="MANUAL"><h2>1. MANUAL</h2><p>사람이 100% 직접 조작합니다. 차선보조·일반 장애물 회피·사람 자동정지는 주행에 개입하지 않습니다. E-STOP/Watchdog/하드웨어 제한은 유지합니다.</p><button data-mode="MANUAL" class="primary">MANUAL 선택</button></article>
 <article class="mode" data-card="RECORD"><h2>2. RECORD</h2><p>MANUAL과 동일하게 사람이 직접 운전하면서 학습/매핑용 센서 데이터를 동기화해 기록합니다.</p><label class="record-option"><input id="record-gps" type="checkbox" checked> GPS/RTK도 기록</label><button data-mode="RECORD" class="primary">RECORD 시작</button></article>
 <article class="mode" data-card="AUTO_AI"><h2>3. AUTO_AI</h2><p>선택한 학습 모델이 정상 주행과 일반 장애물 대응을 담당하고, 사람 위험만 외부에서 강제 STOP합니다.</p><small id="ai-note">학습/추론 Runtime 연결 전</small><button data-mode="AUTO_AI">AUTO_AI 선택</button></article>
 <article class="mode" data-card="AUTO_GPS"><h2>4. AUTO_GPS</h2><p>GPS/RTK 경로를 기본으로 주행하며, 차선이 보이면 보조하고 LiDAR Local Avoidance로 임시 우회 후 원 경로에 복귀합니다.</p><small id="gps-note">경로를 먼저 로드해야 합니다.</small><button data-mode="AUTO_GPS" class="primary">AUTO_GPS 시작</button></article>
 <article class="mode" data-card="AUTO_LOCAL"><h2>5. AUTO_LOCAL</h2><p>저장된 SLAM 지도를 선택하고 현재 위치를 Localization한 뒤 목적지까지 주행합니다.</p><small id="local-note">SLAM Runtime 연결 전</small><button data-mode="AUTO_LOCAL">AUTO_LOCAL 선택</button></article>
 <article class="mode" data-card="AUTO"><h2>6. AUTO</h2><p>환경을 판단해 GPS → LOCAL → 검증된 AI 순으로 안전하게 전략을 선택합니다. 현재는 구현된 Runtime만 선택합니다.</p><button data-mode="AUTO" class="primary">AUTO 시작</button></article>
</section>

<div class="actions">
 <button id="disarm" class="warning">STOP / DISARM</button>
 <button id="estop" class="danger">EMERGENCY STOP</button>
 <button id="reset">SAFETY RESET</button>
 <button id="refresh">상태 새로고침</button>
</div>
<div id="notice" class="notice">상태를 불러오는 중...</div>
</main>
<script>
const notice=document.getElementById('notice');
const recordGps=document.getElementById('record-gps');
let statusCache=null;
function setNotice(text,cls=''){notice.textContent=text;notice.className='notice '+cls}
async function api(path,options={}){const r=await fetch(path,{cache:'no-store',...options});let data={};try{data=await r.json()}catch{}if(!r.ok)throw new Error(data.error||data.message||`HTTP ${r.status}`);return data}
function post(path,body={}){return api(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})}
function render(s){statusCache=s;document.getElementById('canonical-mode').textContent=s.state.canonical_mode;document.getElementById('runtime-mode').textContent=s.state.mode;document.getElementById('recording').textContent=s.recording.active?'ACTIVE':'OFF';document.getElementById('gps-route').textContent=s.gps.route_loaded?(s.gps.preflight_ready?'READY':'LOADED'):'NOT LOADED';document.querySelectorAll('[data-card]').forEach(x=>x.classList.toggle('active',x.dataset.card===s.state.canonical_mode));
 const caps=s.capabilities;document.querySelector('[data-mode="AUTO_AI"]').disabled=!caps.AUTO_AI.implemented;document.querySelector('[data-mode="AUTO_LOCAL"]').disabled=!caps.AUTO_LOCAL.implemented;document.querySelector('[data-mode="AUTO_GPS"]').disabled=!caps.AUTO_GPS.implemented;document.querySelector('[data-mode="AUTO"]').disabled=!caps.AUTO.implemented;
 document.getElementById('gps-note').textContent=s.gps.route_loaded?(s.gps.preflight_ready?'GPS preflight READY':'경로 로드됨 · preflight 대기/실패'):'기존 상세 대시보드에서 GPS 경로를 로드하세요.';
 document.getElementById('ai-note').textContent=caps.AUTO_AI.reason;document.getElementById('local-note').textContent=caps.AUTO_LOCAL.reason;
}
async function refresh(){try{const s=await api('/api/v2/status');render(s);setNotice(`V2 server online\nmode=${s.state.canonical_mode}\n${s.message||''}`,'good')}catch(e){setNotice(e.message,'bad')}}
document.querySelectorAll('[data-mode]').forEach(btn=>btn.addEventListener('click',async()=>{btn.disabled=true;try{const mode=btn.dataset.mode;const result=await post('/api/v2/mode',{mode,record_gps:recordGps.checked});setNotice(JSON.stringify(result,null,2),'good');await refresh()}catch(e){setNotice(e.message,'bad')}finally{await refresh()}}));
document.getElementById('disarm').addEventListener('click',async()=>{try{await post('/api/v2/mode',{mode:'DISARMED'});await refresh()}catch(e){setNotice(e.message,'bad')}});
document.getElementById('estop').addEventListener('click',async()=>{if(!confirm('EMERGENCY STOP을 실행합니까?'))return;try{await post('/api/safety/emergency-stop',{});await refresh()}catch(e){setNotice(e.message,'bad')}});
document.getElementById('reset').addEventListener('click',async()=>{try{await post('/api/safety/reset',{});await refresh()}catch(e){setNotice(e.message,'bad')}});
document.getElementById('refresh').addEventListener('click',refresh);refresh();setInterval(refresh,1500);
</script>
</body></html>""".encode("utf-8")


def _lidar_safety_points():
    return legacy.lidar_monitor.snapshot().get("safety_points") or []


def _ensure_auto_gps_planner():
    planner = legacy.auto_route_runtime.planner
    if planner is None:
        raise ValueError("No processed GPS route is loaded")
    if isinstance(planner, AutoGpsPlanner):
        return planner
    wrapped = AutoGpsPlanner(
        planner.route,
        route_planner=planner,
        lidar_provider=_lidar_safety_points,
    )
    legacy.auto_route_runtime.planner = wrapped
    return wrapped


def _avoidance_snapshot():
    planner = legacy.auto_route_runtime.planner
    if isinstance(planner, AutoGpsPlanner):
        return planner.snapshot()
    return {"active": False, "side": None, "reason": None, "rejoin_index": None, "temporary_path": []}


def _route_preflight():
    if legacy.auto_route_runtime.planner is None:
        return {"route_loaded": False, "ready": False, "details": None}
    try:
        details = legacy.auto_route_runtime.preflight()
        return {
            "route_loaded": True,
            "ready": bool(details.get("ready")),
            "details": details,
        }
    except Exception as error:
        return {
            "route_loaded": True,
            "ready": False,
            "details": {"error": f"{type(error).__name__}: {error}"},
        }


def _canonical_state():
    snapshot = legacy.vehicle_state_machine.snapshot()
    snapshot.setdefault("canonical_mode", DriveMode(snapshot["mode"]).canonical.value)
    return snapshot


def _capabilities():
    preflight = _route_preflight()
    return {
        "MANUAL": {"implemented": True, "ready": True, "reason": "human_control"},
        "RECORD": {"implemented": True, "ready": True, "reason": "manual_recording"},
        "AUTO_AI": {
            "implemented": False,
            "ready": False,
            "reason": "AUTO_AI training/inference runtime not connected yet",
        },
        "AUTO_GPS": {
            "implemented": True,
            "ready": preflight["ready"],
            "reason": "gps_preflight_ready" if preflight["ready"] else "gps_route_or_preflight_not_ready",
        },
        "AUTO_LOCAL": {
            "implemented": False,
            "ready": False,
            "reason": "AUTO_LOCAL SLAM/localization runtime not connected yet",
        },
        "AUTO": {
            "implemented": True,
            "ready": preflight["ready"],
            "reason": "gps_strategy_available" if preflight["ready"] else "no_implemented_autonomous_strategy_ready",
        },
    }


def v2_status():
    preflight = _route_preflight()
    recording = legacy.record_manager.snapshot()
    return {
        "state": _canonical_state(),
        "recording": recording,
        "gps": {
            "route_loaded": preflight["route_loaded"],
            "preflight_ready": preflight["ready"],
            "preflight": preflight["details"],
            "runtime": legacy.auto_route_runtime.snapshot(),
            "avoidance": _avoidance_snapshot(),
        },
        "capabilities": _capabilities(),
        "message": "AUTO_AI and AUTO_LOCAL are visible but remain disabled until their runtimes are implemented.",
    }


def _stop_recording_for_transition(reason):
    if not legacy.record_manager.active:
        return
    legacy.record_manager.add_event("V2_MODE_TRANSITION", reason)
    legacy.record_manager.stop()
    if legacy.vehicle_state_machine.mode == DriveMode.RECORD:
        legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)


def _stop_navigation_for_transition(reason):
    if legacy.auto_route_runtime.active:
        legacy.auto_route_runtime.stop(reason)


def _ensure_manual_runtime(reason):
    mode = legacy.vehicle_state_machine.mode
    if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
        raise ValueError("Safety reset is required before selecting a driving mode")
    if mode == DriveMode.RECORD:
        _stop_recording_for_transition(reason)
        mode = legacy.vehicle_state_machine.mode
    if mode in {DriveMode.AUTO_ROUTE, DriveMode.AUTO_HYBRID} or legacy.auto_route_runtime.active:
        _stop_navigation_for_transition(reason)
        mode = legacy.vehicle_state_machine.mode
    if mode == DriveMode.DISARMED:
        legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
    elif mode not in {DriveMode.MANUAL, DriveMode.MANUAL_ASSIST}:
        legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
    return _canonical_state()


def select_mode(mode_name, record_gps=True):
    try:
        target = DriveMode(str(mode_name).strip().upper())
    except ValueError as error:
        raise ValueError(f"Unknown drive mode: {mode_name}") from error

    if target == DriveMode.DISARMED:
        _stop_recording_for_transition("v2_disarm")
        _stop_navigation_for_transition("v2_disarm")
        legacy.motor_controller.stop()
        mode = legacy.vehicle_state_machine.mode
        if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
            raise ValueError("Use SAFETY RESET to leave EMERGENCY_STOP/FAULT")
        if mode != DriveMode.DISARMED:
            legacy.vehicle_state_machine.transition(DriveMode.DISARMED, "v2_disarm")
        return {"accepted": True, "target": "DISARMED", "status": v2_status()}

    if target == DriveMode.MANUAL:
        _ensure_manual_runtime("v2_manual_selected")
        return {"accepted": True, "target": target.value, "status": v2_status()}

    if target == DriveMode.RECORD:
        _ensure_manual_runtime("v2_record_prepare")
        if legacy.record_manager.active:
            return {"accepted": True, "target": target.value, "status": v2_status()}
        metadata = dict(legacy.recording_metadata())
        metadata.update(
            purpose="RECORD",
            record_gps=bool(record_gps),
            autonomy_schema="v2",
        )
        result = legacy.record_manager.start(metadata)
        legacy.vehicle_state_machine.transition(DriveMode.RECORD, "v2_record_started")
        return {
            "accepted": True,
            "target": target.value,
            "record_gps": bool(record_gps),
            "recording": result,
            "status": v2_status(),
        }

    if target == DriveMode.AUTO_GPS:
        _stop_recording_for_transition("v2_auto_gps_prepare")
        mode = legacy.vehicle_state_machine.mode
        if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
            raise ValueError("Safety reset is required before AUTO_GPS")
        if mode not in {DriveMode.DISARMED, DriveMode.MANUAL_ASSIST}:
            legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, "v2_auto_gps_prepare")
        _ensure_auto_gps_planner()
        result = legacy.auto_route_runtime.start()
        if not result.get("active"):
            return {
                "accepted": False,
                "target": target.value,
                "error": "AUTO_GPS preflight did not pass",
                "runtime": result,
                "status": v2_status(),
            }
        lane_assist = False
        try:
            legacy.auto_route_runtime.enable_hybrid()
            lane_assist = True
        except ValueError:
            lane_assist = False
        return {
            "accepted": True,
            "target": target.value,
            "runtime_mode": legacy.vehicle_state_machine.mode.value,
            "lane_assist": lane_assist,
            "avoidance": _avoidance_snapshot(),
            "status": v2_status(),
        }

    if target in {DriveMode.AUTO_AI, DriveMode.AUTO_LOCAL}:
        raise NotImplementedError(
            f"{target.value} is part of the V2 interface but its runtime is not implemented yet"
        )

    if target == DriveMode.AUTO:
        preflight = _route_preflight()
        selection = AUTO_SELECTOR.select(
            AutoCapabilities(
                gps_ready=preflight["ready"],
                local_map_id=None,
                local_localization_ready=False,
                ai_model_id=None,
                ai_model_validated=False,
                ai_environment_match=False,
            )
        )
        if not selection.ready or selection.target_mode is None:
            raise ValueError("AUTO found no implemented and ready autonomous strategy")
        if selection.target_mode == DriveMode.AUTO_GPS:
            result = select_mode(DriveMode.AUTO_GPS.value, record_gps=record_gps)
            result["auto_selection"] = {
                "target_mode": selection.target_mode.value,
                "reason": selection.reason,
                "resource_id": selection.resource_id,
            }
            return result
        raise NotImplementedError(
            f"AUTO selected {selection.target_mode.value}, but that runtime is not connected yet"
        )

    raise ValueError(f"Mode is not user-selectable in V2: {target.value}")


class V2Handler(legacy.CameraHandler):
    def _send_html(self, body):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self._send_html(V2_HTML)
            return
        if self.path == "/legacy":
            self._send_html(legacy.INDEX_HTML)
            return
        if self.path == "/api/v2/status":
            self._send_json(v2_status())
            return
        if self.path == "/api/v2/modes":
            self._send_json(
                {
                    "modes": [mode.value for mode in MODE_ORDER],
                    "capabilities": _capabilities(),
                    "state": _canonical_state(),
                }
            )
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/v2/mode":
            try:
                payload = self._read_json()
                result = select_mode(
                    payload.get("mode"),
                    record_gps=payload.get("record_gps", True),
                )
                status = 202 if result.get("accepted", False) else 409
                self._send_json(result, status)
            except NotImplementedError as error:
                self._send_json({"error": str(error), "status": v2_status()}, 501)
            except (ValueError, OSError, TypeError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error), "status": v2_status()}, 409)
            return
        super().do_POST()


def main():
    legacy.camera.start()
    legacy.gps_monitor.start()
    legacy.ntrip_client.start()
    legacy.imu_monitor.start()
    legacy.lidar_monitor.start()
    legacy.motor_controller.start()
    legacy.perception_monitor.start()
    httpd = legacy.ThreadingHTTPServer((legacy.HOST, legacy.PORT), V2Handler)
    print(
        f"GNSS Autonomy V2 listening on http://{legacy.HOST}:{legacy.PORT} "
        f"(legacy dashboard: /legacy)",
        flush=True,
    )
    httpd.serve_forever()


if __name__ == "__main__":
    main()
