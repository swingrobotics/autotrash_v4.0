import csv
import json
import os
import struct
import tempfile
import zlib

from autonomous_car.ai import GpsDatasetBuilder
from autonomous_car.routes import GpsRouteNormalizer


def _write_csv(path, fields, rows):
    with open(path, "w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_lidar(path, timestamps):
    with open(path, "wb") as file:
        for timestamp in timestamps:
            points = [
                {"bearing_degrees": 0.0, "distance_mm": 1800, "confidence": 100},
                {"bearing_degrees": -30.0, "distance_mm": 2600, "confidence": 100},
                {"bearing_degrees": 30.0, "distance_mm": 2400, "confidence": 100},
            ]
            row = {"monotonic": timestamp, "points": points, "safety_points": points}
            payload = zlib.compress(json.dumps(row).encode("utf-8"))
            file.write(struct.pack("<dI", timestamp, len(payload)))
            file.write(payload)


def _make_session(root, name, lateral_lon_offset, time_offset, camera_fix_quality=None):
    path = os.path.join(root, name)
    os.makedirs(path)
    with open(os.path.join(path, "metadata.json"), "w", encoding="utf-8") as file:
        json.dump({"purpose": "RECORD", "record_gps": True}, file)
    open(os.path.join(path, "camera.mp4"), "wb").close()

    camera_times = [time_offset + 1.0, time_offset + 1.1]
    _write_csv(
        os.path.join(path, "camera_timestamps.csv"),
        ["frame_number", "source_sequence", "monotonic", "wall_time", "filename",
         "steering_angle_degrees", "target_steering_angle_degrees",
         "requested_throttle", "final_throttle"],
        [
            {"frame_number": 1, "source_sequence": 10, "monotonic": camera_times[0], "wall_time": 0,
             "filename": "", "steering_angle_degrees": 0.2, "target_steering_angle_degrees": 0.0,
             "requested_throttle": 0.20, "final_throttle": 0.20},
            {"frame_number": 2, "source_sequence": 11, "monotonic": camera_times[1], "wall_time": 0,
             "filename": "", "steering_angle_degrees": 2.5, "target_steering_angle_degrees": 3.0,
             "requested_throttle": 0.18, "final_throttle": 0.18},
        ],
    )
    _write_csv(
        os.path.join(path, "imu.csv"),
        ["monotonic", "yaw_rate_dps", "yaw_degrees"],
        [
            {"monotonic": camera_times[0] + 0.01, "yaw_rate_dps": 0.5, "yaw_degrees": 90.0},
            {"monotonic": camera_times[1] + 0.01, "yaw_rate_dps": 0.7, "yaw_degrees": 90.0},
        ],
    )
    _write_csv(
        os.path.join(path, "control.csv"),
        ["monotonic", "target_speed_mps", "stop_reason"],
        [
            {"monotonic": camera_times[0], "target_speed_mps": 0.20, "stop_reason": ""},
            {"monotonic": camera_times[1], "target_speed_mps": 0.18, "stop_reason": ""},
        ],
    )
    _write_csv(
        os.path.join(path, "vehicle_state.csv"),
        ["monotonic", "mode"],
        [
            {"monotonic": camera_times[0], "mode": "RECORD"},
            {"monotonic": camera_times[1], "mode": "RECORD"},
        ],
    )

    camera_fix_quality = camera_fix_quality or {10: ("RTK FIXED", 0.7), 11: ("RTK FIXED", 0.7)}
    gnss_rows = []
    for index in range(35):
        timestamp = time_offset + index * 0.1
        status, hdop = camera_fix_quality.get(index, ("RTK FIXED", 0.7))
        gnss_rows.append(
            {"monotonic": timestamp, "latitude": 37.0,
             "longitude": 127.0 + lateral_lon_offset + index * 0.000001,
             "altitude_m": 10.0, "speed_mps": 0.2, "rtk_status": status,
             "hdop": "" if hdop is None else hdop, "is_valid": True}
        )
    _write_csv(
        os.path.join(path, "gnss.csv"),
        ["monotonic", "latitude", "longitude", "altitude_m", "speed_mps",
         "rtk_status", "hdop", "is_valid"],
        gnss_rows,
    )
    _write_lidar(os.path.join(path, "lidar_raw.bin"), camera_times)


def main():
    with tempfile.TemporaryDirectory() as directory:
        recordings = os.path.join(directory, "recordings")
        datasets = os.path.join(directory, "datasets")
        routes = os.path.join(directory, "routes")
        os.makedirs(recordings)
        os.makedirs(routes)
        sessions = ["run_a", "run_b", "run_c"]
        _make_session(recordings, "run_a", -0.0000001, 0.0)
        _make_session(recordings, "run_b", 0.0, 10.0,
                      {10: ("RTK FLOAT", 0.8), 11: ("DGNSS FIX", None)})
        _make_session(recordings, "run_c", 0.0000001, 20.0,
                      {10: ("RTK FLOAT", 3.0), 11: ("DGPS", None)})

        route_path = os.path.join(routes, "route_v1.json")
        route = GpsRouteNormalizer(minimum_fixed_samples=10).build(
            recordings, sessions, "route_v1", output_path=route_path
        )
        assert route.route_id == "route_v1"
        assert route.quality["reference_fix_policy"] == "RTK_FIXED_ONLY"
        assert route.quality["contains_dgps_fallback"] is False

        result = GpsDatasetBuilder(recordings, datasets, route_path).build(
            sessions, dataset_id="gps_dataset_v1"
        )
        assert result["policy_type"] == "AUTO_GPS"
        assert result["route"]["route_id"] == "route_v1"
        assert result["route"]["fix_policy"] == "RTK_FIXED_ONLY"
        assert result["accepted_samples"] == 5
        assert result["rejected_samples"] == 1
        assert result["gps_quality"]["policy"]["runtime_fix_policy"] == "RTK_FIXED_ONLY"
        assert result["gps_quality"]["accepted_by_status"]["RTK FIXED"] == 2
        assert result["gps_quality"]["accepted_by_status"]["RTK FLOAT"] == 1
        assert result["gps_quality"]["accepted_by_status"]["DGPS FIX"] == 2
        assert result["gps_quality"]["rejected_by_reason"]["CONDITIONAL_FIX_HDOP_TOO_HIGH"] == 1

        session_splits = {item["session"]: item["split"] for item in result["sessions"]}
        assert list(session_splits.values()).count("train") == 2
        assert list(session_splits.values()).count("validation") == 1
        assert list(session_splits.values()).count("test") == 0
        assert result["split_counts"]["test"] == 0
        assert result["gps_training_policy"]["three_session_split"] == "2_train_1_validation_curve_coverage_aware"
        temporal_policy = result["gps_training_policy"]["temporal_context"]
        assert temporal_policy["history_steps"] == 5
        assert temporal_policy["auxiliary_feature_size"] == 20
        assert temporal_policy["current_steering_excluded"] is True

        with open(os.path.join(datasets, "gps_dataset_v1", "samples.jsonl"), "r", encoding="utf-8") as file:
            samples = [json.loads(line) for line in file if line.strip()]
        assert len(samples) == 5
        statuses = {sample["evaluation_only"]["rtk_status"] for sample in samples}
        assert statuses == {"RTK FIXED", "RTK FLOAT", "DGPS FIX"}
        dgps_without_hdop = 0
        transition_samples = 0
        steering_groups = set()
        for sample in samples:
            route_features = sample["learned_features"]["route"]
            assert len(route_features["normalized"]) == 8
            assert route_features["feature_order"][0] == "cross_track_error"
            temporal = sample["learned_features"]["temporal"]
            assert temporal["history_steps"] == 5
            assert len(temporal["yaw_rate_history_dps"]) == 5
            assert len(temporal["previous_steering_history_degrees"]) == 5
            assert temporal["current_steering_excluded"] is True
            assert sample["labels"]["steering_degrees"] not in temporal["previous_steering_history_degrees"] or sample["labels"]["steering_degrees"] == 0.0
            assert sample["labels"]["throttle_label_source"] == "requested_throttle"
            assert sample["synchronization"]["gnss_skew_seconds"] <= 0.20
            context = sample["training_context"]
            assert context["steering_group"] in {"straight", "gentle", "sharp"}
            assert "route_recovery" in context
            steering_groups.add(context["steering_group"])
            if context["steering_transition"]:
                transition_samples += 1
            if sample["evaluation_only"]["rtk_status"] != "RTK FIXED":
                assert sample["evaluation_only"]["gps_quality_tier"] == "CONDITIONAL"
                assert sample["evaluation_only"]["route_deviation_limit_m"] == 1.5
            if sample["evaluation_only"]["rtk_status"] == "RTK FLOAT":
                assert sample["evaluation_only"]["gnss_hdop"] <= 1.5
            if sample["evaluation_only"]["rtk_status"] == "DGPS FIX":
                assert sample["evaluation_only"]["gnss_hdop"] is None
                dgps_without_hdop += 1
        assert dgps_without_hdop == 2
        assert steering_groups == {"straight", "gentle"}
        assert transition_samples >= 2

        dgps_all = {index: ("DGNSS FIX", None) for index in range(35)}
        _make_session(recordings, "run_dgps_a", -0.0000001, 40.0, dgps_all)
        _make_session(recordings, "run_dgps_b", 0.0000001, 50.0, dgps_all)
        dgps_route_path = os.path.join(routes, "route_dgps.json")
        dgps_route = GpsRouteNormalizer(minimum_fixed_samples=10).build(
            recordings, ["run_dgps_a", "run_dgps_b"], "route_dgps",
            output_path=dgps_route_path,
        )
        assert dgps_route.quality["contains_dgps_fallback"] is True
        assert dgps_route.quality["reference_fix_policy"] == "PREFER_RTK_FIXED_DGPS_FALLBACK"
        assert dgps_route.quality["source_fix_policies"] == {
            "run_dgps_a": "DGPS_FALLBACK", "run_dgps_b": "DGPS_FALLBACK",
        }

    print("GPS temporal split + curve transition + FIXED/DGPS quality validation: PASS")


if __name__ == "__main__":
    main()
