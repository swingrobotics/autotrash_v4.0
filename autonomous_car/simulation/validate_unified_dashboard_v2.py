import json
from pathlib import Path

from unified_dashboard import UNIFIED_DASHBOARD_HTML
from unified_dashboard_extras import UNIFIED_DASHBOARD_EXTRAS
from unified_dashboard_data_tools import UNIFIED_DASHBOARD_DATA_TOOLS
from v2_option_panel import V2_OPTION_PANEL
from vehicle_settings_panel import VEHICLE_SETTINGS_PANEL
from autonomous_car.web_requests import normalize_drive_mode_request


def _mode_request_checks():
    accepted = {}
    for mode in ["MANUAL", "RECORD", "AUTO_AI", "AUTO_GPS", "AUTO_LOCAL", "AUTO", "DISARMED"]:
        accepted[mode] = normalize_drive_mode_request(
            {"mode": f"  {mode.lower()}  ", "record_gps": False}
        ) == (mode, False)
    rejected = []
    for payload in [{}, {"mode": ""}, {"mode": "   "}, {"mode": "AUTO_ROUTE"}, []]:
        try:
            normalize_drive_mode_request(payload)
        except ValueError:
            rejected.append(True)
        else:
            rejected.append(False)
    return all(accepted.values()) and all(rejected)


def main():
    base_html = UNIFIED_DASHBOARD_HTML.decode("utf-8")
    extras_html = UNIFIED_DASHBOARD_EXTRAS.decode("utf-8")
    data_tools_html = UNIFIED_DASHBOARD_DATA_TOOLS.decode("utf-8")
    advanced_html = (UNIFIED_DASHBOARD_HTML + UNIFIED_DASHBOARD_EXTRAS + UNIFIED_DASHBOARD_DATA_TOOLS).decode("utf-8")
    option_html = V2_OPTION_PANEL.decode("utf-8")
    vehicle_html = VEHICLE_SETTINGS_PANEL.decode("utf-8")
    final_source = Path("server_v2_final.py").read_text(encoding="utf-8")
    legacy_source = Path("server.py").read_text(encoding="utf-8")
    operator_status_source = Path("operator_mode_status.py").read_text(encoding="utf-8")

    required_modes = ["MANUAL", "RECORD", "AUTO_AI", "AUTO_GPS", "AUTO_LOCAL", "AUTO"]

    checks = {
        "v1_is_primary_dashboard": (
            "release.full.legacy.INDEX_HTML.replace" in final_source
            and 'if self.path == "/":' in final_source
            and "V2_OPTION_PANEL" in final_source
        ),
        "existing_v1_drive_mode_button_is_reused": (
            'id="autonomy-open"' in legacy_source
            and "주행 모드" in legacy_source
            and "document.getElementById('autonomy-open')" in option_html
            and "v2-option-button" not in option_html
        ),
        "drive_mode_modal_is_user_facing_chooser": (
            "#autonomy-modal .autonomy-safety-actions" in option_html
            and "#autonomy-modal .settings-grid" in option_html
            and "display:none!important" in option_html
            and "주행 모드 선택" in option_html
            and "현재 주행" in option_html
            and "선택한 주행" in option_html
            and "실제 preflight" not in option_html
            and "AUTO_ALLOWED AI" not in option_html
        ),
        "six_v2_modes_are_selectable": (
            all(f'data-v2-select="{mode}"' in option_html for mode in required_modes)
            and 'href="/v2"' in option_html
        ),
        "chooser_groups_human_autonomy_and_auto": (
            "직접 운전" in option_html
            and "자율주행" in option_html
            and "자동 자율주행" in option_html
        ),
        "chooser_shows_friendly_readiness": (
            "data-v2-badge" in option_html
            and "data-v2-reason" in option_html
            and "function badgeState(mode)" in option_html
            and "capability(mode)" in option_html
            and "사용 가능" in option_html
            and "준비 필요" in option_html
            and "주행 중" in option_html
        ),
        "record_gps_option_is_scoped_to_record": (
            'id="v2-record-gps"' in option_html
            and "GPS 위치도 함께 저장" in option_html
            and "selectedMode==='RECORD'&&recordGps.checked" in option_html
        ),
        "selection_does_not_start_runtime": (
            "function selectOnly(mode)" in option_html
            and "localStorage.setItem(SELECTED_KEY,mode)" in option_html
            and "modal.hidden=true" in option_html
            and "post('/api/v2/mode'" not in option_html.split("function selectOnly(mode)", 1)[1].split("async function refresh", 1)[0]
        ),
        "main_dashboard_has_contextual_start_stop": (
            "id='v2-main-action'" in option_html
            and "modeStartLabel" in option_html
            and "modeStopLabel" in option_html
            and "주행 기록 시작" in option_html
            and "toggleSelected" in option_html
            and "await post('/api/v2/mode',{mode:selectedMode" in option_html
            and "await post('/api/v2/mode',{mode:'MANUAL'" in option_html
        ),
        "manual_activation_has_stop_disarm": (
            "running&&selectedMode==='MANUAL'" in option_html
            and "await post('/api/v2/mode',{mode:'DISARMED'" in option_html
            and "수동 운전 종료" in option_html
        ),
        "header_shows_selected_mode": (
            "openButton.textContent=`주행 모드 · ${modeName(selectedMode)}`" in option_html
            and "swing.v2.selectedMode" in option_html
        ),
        "auto_selector_shows_actual_strategy_without_internal_codes": (
            "function autoSelector()" in option_html
            and "function autoStrategy()" in option_html
            and "function autoIsRunning()" in option_html
            and "자동 자율주행이 현재 사용할 방식을 선택했습니다." in option_html
        ),
        "not_ready_auto_cannot_start": (
            "readiness(selectedMode)" in option_html
            and "mainAction.disabled=true" in option_html
            and "ready.checking" in option_html
        ),
        "manual_gamepad_bridge_has_no_hidden_button_click": (
            "#lidar-drive-button{display:none!important}" in option_html
            and "function humanDriveActive()" in option_html
            and "function syncLegacyManualArm()" in option_html
            and "typeof toggleDriveArmed!=='function'" in option_html
            and "toggleDriveArmed()" in option_html
            and "legacyDriveButton.click()" not in option_html
        ),
        "local_readiness_uses_controller_preflight": (
            "AUTO_LOCAL_CONTROLLER.preflight()" in operator_status_source
            and 'local["preflight_ready"] = local_ready' in operator_status_source
            and '"checking": local_checking' in operator_status_source
            and '"ready": bool(gps_ready or local_ready or compatible_ai)' in operator_status_source
        ),
        "main_dashboard_retains_emergency_stop_and_reset": (
            "id='v2-main-estop'" in option_html
            and "id='v2-main-reset'" in option_html
            and "긴급 정지" in option_html
            and "안전 상태 해제" in option_html
            and "/api/safety/emergency-stop" in option_html
            and "/api/safety/reset" in option_html
        ),
        "advanced_dashboard_hides_developer_views_and_raw_status": (
            'if self.path == "/v2":' in final_source
            and 'data-view="debug"' in base_html
            and '#nav button[data-view="drive"],#nav button[data-view="debug"]' in extras_html
            and '#view-data pre,#view-gps pre,#view-local pre,#view-hardware pre,#view-system pre' in extras_html
            and "주행 설정 및 데이터 관리" in extras_html
            and "학습 데이터" in extras_html
            and "GPS 주행" in extras_html
            and "지도 주행" in extras_html
            and "장치 설정" in extras_html
            and "연결 및 기기" in extras_html
            and "friendlyError" in extras_html
            and "UNIFIED_DASHBOARD_EXTRAS" in final_source
            and "UNIFIED_DASHBOARD_DATA_TOOLS" in final_source
        ),
        "advanced_workflows_are_user_guided": (
            "AI 주행 준비" in extras_html
            and "1 · 주행 기록 선택" in extras_html
            and "GPS 자율주행 준비" in extras_html
            and "1 · GPS 기록 선택" in extras_html
            and "지도 자율주행 준비" in extras_html
            and "1 · 지도 준비" in extras_html
            and "저장된 주행 기록 관리" in data_tools_html
        ),
        "advanced_connection_settings_are_forms_not_json": (
            'id="wifi-ssid"' in extras_html
            and 'id="ntrip-host"' in extras_html
            and 'id="ntrip-mountpoint"' in extras_html
            and "ntrip-config-json" not in extras_html.split("/* Regression compatibility tokens", 1)[0]
            and "throttle-points" not in extras_html.split("/* Regression compatibility tokens", 1)[0]
        ),
        "vehicle_settings_has_dirty_and_inline_validation": (
            'id="vehicle-settings-dirty"' in vehicle_html
            and "기본값 미리보기" in vehicle_html
            and "dirtyKeys()" in vehicle_html
            and "updateEditingState()" in vehicle_html
            and "STOP < CRAWL < SLOW" in vehicle_html
            and "조향 최소 PWM은 최대 PWM 이하여야" in vehicle_html
            and "저장하지 않은 변경사항" in vehicle_html
        ),
        "legacy_modal_dom_preserved_for_polling_compatibility": (
            "oldSafety=card.querySelector('.autonomy-safety-actions')" in option_html
            and "card.insertBefore(panel,oldSafety)" in option_html
            and "innerHTML=''" not in option_html
        ),
        "server_resets_json_cache_each_http_request": (
            "def handle_one_request(self):" in final_source
            and 'hasattr(self, "_cached_json_payload")' in final_source
            and "del self._cached_json_payload" in final_source
        ),
        "server_validates_v2_mode_before_enum": (
            "normalize_drive_mode_request(payload)" in final_source
            and 'if self.path == "/api/v2/mode":' in final_source
            and 'self._send_json({"error": str(error)}, 400)' in final_source
        ),
        "drive_mode_request_validation": _mode_request_checks(),
        "compatibility_urls_are_unambiguous": (
            'self.path == "/legacy"' in final_source
            and 'self._redirect("/")' in final_source
            and 'self.path == "/ai-data"' in final_source
            and 'self._redirect("/v2#data")' in final_source
            and 'self.path == "/gps-ai"' in final_source
            and 'self._redirect("/v2#gps")' in final_source
        ),
    }

    result = {"passed": all(checks.values()), "checks": checks}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
