"""Static policy regression for external-UFLD PRETRAINED_ROAD AUTO fallback."""


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _position(source, token):
    position = source.find(token)
    _require(position >= 0, f"missing policy token: {token}")
    return position


def main():
    runtime = open("autonomous_car/pretrained_auto_runtime.py", encoding="utf-8").read()
    hybrid = open("autonomous_car/control/hybrid_lane_controller.py", encoding="utf-8").read()
    perception = open("autonomous_car/perception/pretrained_road.py", encoding="utf-8").read()
    guard = open("autonomous_car/production_guard.py", encoding="utf-8").read()
    full = open("server_v2_full.py", encoding="utf-8").read()
    installer = open("scripts/install_pretrained_road_model.sh", encoding="utf-8").read()
    ui = open("v2_option_panel.py", encoding="utf-8").read()

    gps = _position(full, '# 1) GPS/RTK route')
    local = _position(full, '# 2) Saved LOCAL map')
    learned = _position(full, '# 3) Environment-compatible AUTO_ALLOWED learned model')
    _require(gps < local < learned, "existing AUTO strategy order changed")
    original = _position(runtime, "return original_auto_start()")
    pretrained = _position(runtime, "check = controller.preflight(probe_frame=True)")
    _require(original < pretrained, "PRETRAINED_ROAD bypasses higher-priority AUTO strategies")

    _require('source="pretrained_road_auto"' in runtime, "pretrained AUTO control source missing")
    _require("safety_supervisor.evaluate" in runtime, "pretrained AUTO bypasses SafetySupervisor")
    _require("current_delay" in runtime, "current-iteration stall is not sent to SafetySupervisor")
    _require("TARGET_SPEED_NEURAL_MPS = 0.20" in runtime, "conservative neural speed missing")
    _require("TARGET_SPEED_FALLBACK_MPS = 0.11" in runtime, "fallback speed reduction missing")
    _require("PRETRAINED_AUTO_LANE_LOST" in runtime, "lane-loss fail-safe missing")
    _require("PRETRAINED_AUTO_NEURAL_BACKEND_REQUIRED" in runtime, "classical-only preflight can still arm AUTO")
    _require("probe_neural_latency_jpeg" in runtime, "pre-motion neural latency probe missing")
    _require("SWING_PRETRAINED_ROAD_MAX_INFERENCE_MS" in runtime, "latency budget configuration missing")
    _require("NEURAL_INFERENCE_TOO_SLOW" in hybrid, "neural latency circuit breaker missing")
    _require('NEURAL_BACKEND = "UFLD_ONNX"' in hybrid, "external UFLD backend is not primary")
    _require("UFLD_TUSIMPLE_RES18_288X800" in perception, "UFLD TuSimple Res18 perception missing")
    _require("EXTERNAL_UFLD_TUSIMPLE" in perception, "external UFLD decoder contract missing")
    _require("from third_party.ufld import" in perception, "vendored UFLD decoder is not used by runtime")
    _require('"decoder_adapter": "third_party.ufld"' in perception, "vendored decoder identity missing")
    _require("install_pretrained_auto_runtime" in guard, "production runtime does not install pretrained AUTO")
    _require("PRETRAINED_AUTO_CONTROLLER" in guard, "production shutdown/fault guard omits pretrained AUTO")
    _require("140_Ultra-Fast-Lane-Detection" in installer, "UFLD PINTO artifact source missing")
    _require("sha256" in installer.lower(), "installed model SHA-256 is not recorded")
    _require("[1, 3, 288, 800]" in installer, "installer does not validate UFLD input contract")
    _require("[1, 101, 56, 4]" in installer, "installer does not validate UFLD output contract")

    _require("PRETRAINED_ROAD:'차선 추적 자율주행'" in ui, "operator UI has no pretrained lane label")
    _require(
        "['AUTO_AI','AUTO_GPS','AUTO_LOCAL','PRETRAINED_ROAD'].includes(autoStrategy())" in ui,
        "operator UI does not treat PRETRAINED_ROAD as active AUTO",
    )
    _require("pretrained_road_available:'UFLD 차선 추적 주행 사용 가능'" in ui, "operator readiness omits UFLD fallback")
    _require("UFLD 차선 추적 주행" in ui, "AUTO chooser does not explain lane fallback")

    print("Pretrained AUTO policy V2 regression: PASS")
    print(
        {
            "strategy_order": ["AUTO_GPS", "AUTO_LOCAL", "AUTO_AI", "PRETRAINED_ROAD"],
            "neural_backend": "UFLD_ONNX",
            "decoder_adapter": "third_party.ufld",
            "external_detector": "Ultra-Fast-Lane-Detection TuSimple ResNet18",
            "maximum_neural_inference_ms": 160.0,
            "neural_target_speed_mps": 0.20,
            "fallback_target_speed_mps": 0.11,
            "operator_auto_tracks_pretrained": True,
        }
    )


if __name__ == "__main__":
    main()
