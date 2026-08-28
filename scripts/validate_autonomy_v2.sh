#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-python3}"

"$PYTHON_BIN" -m compileall -q \
  autonomous_car camera_stream swing_compute \
  v2_option_panel.py vehicle_settings_panel.py vehicle_runtime_settings.py \
  lane_dashboard_overlay.py lane_neural_preview.py lane_record_observer.py \
  manual_control_hardening.py camera_calibration_panel.py dashboard_terminal_hmi.py \
  compute_gps_training_bridge.py compute_gps_training_hmi.py \
  unified_dashboard.py unified_dashboard_extras.py unified_dashboard_data_tools.py \
  server_v2.py server_v2_ai.py server_v2_full.py server_v2_release.py server_v2_gps_ai.py server_v2_final.py

"$PYTHON_BIN" -m autonomous_car.simulation.validate_autonomy_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_unified_dashboard_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_dashboard_terminal_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_manual_control_priority_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_record_low_latency_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_record_ufld_observer_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_geometry_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_candidate_priority_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_prior_tracking_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_calibration_projection_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_temporal_recovery_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_pair_recovery_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_pretrained_road_perception_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_hybrid_lane_controller_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_lane_neural_preview_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_pretrained_auto_policy_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_charuco_calibration_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_vehicle_settings_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_runtime_hardening_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_service_hardening_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_production_guard_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_gps_ai_route_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_gps_ai_dataset_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_compute_gps_training_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_auto_gps_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_auto_local_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_ai_dataset_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_auto_ai_environment_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_autonomous_timing_v2
"$PYTHON_BIN" -m autonomous_car.simulation.validate_runtime_guard_v2

echo "Autonomy V2 core software regression: PASS"
echo "For training/export/inference smoke, install requirements-ai-training.txt and run:"
echo "  $PYTHON_BIN -m autonomous_car.simulation.validate_ai_training_v2"
echo "  $PYTHON_BIN -m autonomous_car.simulation.validate_gps_ai_training_v2"
