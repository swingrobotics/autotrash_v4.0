#!/usr/bin/env python3
"""End-to-end smoke validation for synchronized RECORD model preview.

The test builds a minimal real RECORD directory on disk (JPEG camera frames,
compressed lidar_raw.bin, IMU/GNSS CSV streams and human control labels), then
runs the production preview pipeline with deterministic fake model runtimes.
This validates synchronization, media rendering, CSV output, AUTO_GPS route
feature extraction, measured steering feedback and the no-control-authority
contract without requiring a large trained ONNX artifact in CI.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import zlib

import cv2
import numpy as np

from autonomous_car.ai import record_preview


TIMES = (10.0, 10.1, 10.2)


def _write_csv(path: Path, fieldnames, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_lidar(session: Path, timestamps):
    with open(session / "lidar_raw.bin", "wb") as handle:
        for timestamp in timestamps:
            document = {
                "safety_points": [
                    {"bearing_degrees": 0.0, "distance_mm": 1800, "confidence": 220},
                    {"bearing_degrees": -25.0, "distance_mm": 2200, "confidence": 210},
                    {"bearing_degrees": 25.0, "distance_mm": 2150, "confidence": 210},
                ]
            }
            payload = zlib.compress(
                json.dumps(document, separators=(",", ":")).encode("utf-8"),
                level=1,
            )
            handle.write(struct.pack("<dI", timestamp, len(payload)))
            handle.write(payload)


def _build_record(root: Path) -> Path:
    session = root / "run_preview_smoke"
    frames = session / "camera_frames" / "segment_0000"
    frames.mkdir(parents=True)
    (session / "metadata.json").write_text(
        json.dumps(
            {
                "session": session.name,
                "record_gps": True,
                "camera_storage": "segmented_jpeg_frames_v2",
                "lidar_raw_encoding": "zlib_json_frames_v1",
            }
        ),
        encoding="utf-8",
    )

    camera_rows = []
    for index, timestamp in enumerate(TIMES, start=1):
        relative = f"camera_frames/segment_0000/frame_{index:08d}.jpg"
        image = np.zeros((180, 320, 3), dtype=np.uint8)
        image[:, :, 1] = 30 + index * 20
        cv2.putText(
            image,
            f"FRAME {index}",
            (24, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if not cv2.imwrite(str(session / relative), image):
            raise AssertionError("failed to create RECORD JPEG")
        camera_rows.append(
            {
                "frame_number": index,
                "source_sequence": index,
                "monotonic": timestamp,
                "wall_time": 1000.0 + timestamp,
                "filename": relative,
                "steering_angle_degrees": 4.0 + index,
                "target_steering_angle_degrees": 5.0 + index,
                "requested_throttle": 0.20 + index * 0.01,
                "final_throttle": 0.18 + index * 0.01,
                "steering_monotonic": timestamp,
                "control_monotonic": timestamp,
                "steering_skew_seconds": 0.0,
                "control_skew_seconds": 0.0,
            }
        )
    _write_csv(
        session / "camera_timestamps.csv",
        list(camera_rows[0]),
        camera_rows,
    )

    imu_rows = [
        {
            "monotonic": timestamp,
            "wall_time": 1000.0 + timestamp,
            "yaw_degrees": 90.0,
            "pitch_degrees": 0.0,
            "roll_degrees": 0.0,
            "yaw_rate_dps": 1.5,
            "acceleration_x": 0.0,
            "acceleration_y": 0.0,
            "acceleration_z": 9.8,
            "imu_timestamp": timestamp,
            "is_valid": True,
            "data_age": 0.0,
            "error_code": "",
        }
        for timestamp in TIMES
    ]
    _write_csv(session / "imu.csv", list(imu_rows[0]), imu_rows)

    gnss_rows = [
        {
            "monotonic": timestamp,
            "wall_time": 1000.0 + timestamp,
            "latitude": 37.0,
            "longitude": 127.0,
            "altitude_m": 0.0,
            "rtk_status": "RTK FIXED",
            "satellites": 20,
            "hdop": 0.6,
            "speed_mps": 1.0,
            "course_degrees": 90.0,
            "gnss_timestamp": timestamp,
            "is_valid": True,
            "data_age": 0.0,
            "error_code": "",
        }
        for timestamp in TIMES
    ]
    _write_csv(session / "gnss.csv", list(gnss_rows[0]), gnss_rows)
    _write_lidar(session, TIMES)
    return session


def _build_route(path: Path, route_id="preview-route"):
    document = {
        "schema": "normalized_gps_route_v1",
        "route_id": route_id,
        "origin": {
            "origin_latitude": 37.0,
            "origin_longitude": 127.0,
            "origin_altitude": 0.0,
        },
        "source_sessions": ["run_preview_smoke"],
        "quality": {"reference_fix_policy": "RTK_FIXED_ONLY"},
        "points": [
            {"x": 0.0, "y": 0.0, "speed_mps": 1.0},
            {"x": 5.0, "y": 0.0, "speed_mps": 1.0},
            {"x": 10.0, "y": 0.0, "speed_mps": 1.0},
        ],
    }
    path.write_text(json.dumps(document), encoding="utf-8")


class _FakeAutoRuntime:
    maximum_steering_degrees = 30.0

    def __init__(self, model_path, manifest_path=None, providers=None):
        self.calls = 0

    def infer_jpeg(
        self,
        jpeg_bytes,
        lidar_points,
        imu_yaw_rate_dps=None,
        *,
        person_hazard=False,
    ):
        assert jpeg_bytes
        assert len(lidar_points) == 3
        assert abs(float(imu_yaw_rate_dps) - 1.5) < 1e-9
        assert person_hazard is False
        self.calls += 1
        normalized = 0.20
        return SimpleNamespace(
            steering_degrees=normalized * self.maximum_steering_degrees,
            throttle=0.25,
            normalized_steering=normalized,
            person_stop=False,
            inference_seconds=0.001,
            lidar_observed_sectors=3,
        )


class _FakeGpsRuntime(_FakeAutoRuntime):
    route_vectors = []
    measured_values = []
    reset_calls = 0
    requires_measured_steering = True

    def reset_temporal_state(self):
        self.__class__.reset_calls += 1

    def infer_jpeg(
        self,
        jpeg_bytes,
        lidar_points,
        imu_yaw_rate_dps,
        route_features,
        person_hazard=False,
        measured_steering_degrees=None,
    ):
        normalized = list(route_features.get("normalized") or [])
        assert len(normalized) == 8
        assert all(-1.000001 <= float(value) <= 1.000001 for value in normalized)
        self.__class__.route_vectors.append(tuple(normalized))
        self.__class__.measured_values.append(measured_steering_degrees)
        return super().infer_jpeg(
            jpeg_bytes,
            lidar_points,
            imu_yaw_rate_dps,
            person_hazard=person_hazard,
        )


def _read_csv(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _assert_common(summary, csv_path, *, policy):
    assert summary.policy_type == policy
    assert summary.source_frames == 3
    assert summary.inferred_frames == 3
    assert summary.skipped_frames == 0
    assert abs(summary.lidar_sync_ratio - 1.0) < 1e-9
    assert abs(summary.imu_sync_ratio - 1.0) < 1e-9
    assert summary.control_authority == "NONE"
    assert Path(summary.output_video).is_file()
    assert Path(summary.output_video).stat().st_size > 0
    assert Path(summary.output_csv).is_file()
    rows = _read_csv(csv_path)
    assert len(rows) == 3
    assert {row["status"] for row in rows} == {"OK"}
    assert {row["control_authority"] for row in rows} == {"NONE"}
    assert all(row["human_steering_degrees"] for row in rows)
    assert all(row["actual_steering_degrees"] for row in rows)
    assert all(row["model_steering_degrees"] for row in rows)
    assert all(row["steering_abs_error_degrees"] for row in rows)
    return rows


def main():
    original_auto = record_preview.AutoAiRuntime
    original_gps = record_preview.GpsAiRuntime
    original_lidar_skew = record_preview.MAXIMUM_LIDAR_SKEW_SECONDS
    record_preview.AutoAiRuntime = _FakeAutoRuntime
    record_preview.GpsAiRuntime = _FakeGpsRuntime
    try:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = _build_record(root)
            route_path = root / "preview-route.json"
            _build_route(route_path)

            # Path-safety and label-source contract checks.
            assert record_preview._safe_saved_frame(session, "../metadata.json") is None
            assert record_preview._safe_saved_frame(
                session, "camera_frames/segment_0000/frame_00000001.jpg"
            )
            first_camera = record_preview._camera_rows(session)[0]
            target, actual, throttle = record_preview._human_labels(first_camera)
            assert target == 6.0  # target steering remains the imitation/evaluation label
            assert actual == 5.0  # encoder steering remains independent measured state
            assert abs(throttle - 0.21) < 1e-9  # requested throttle preferred over final

            auto_manifest = root / "auto_manifest.json"
            auto_manifest.write_text(
                json.dumps({"policy_type": "AUTO_AI"}), encoding="utf-8"
            )
            auto_video = root / "auto-preview.mp4"
            auto_csv = root / "auto-preview.csv"
            progress = []
            auto_summary = record_preview.preview_record_session(
                session,
                root / "fake-auto.onnx",
                manifest_path=auto_manifest,
                output_video=auto_video,
                output_csv=auto_csv,
                progress_callback=lambda done, total: progress.append((done, total)),
            )
            _assert_common(auto_summary, auto_csv, policy="AUTO_AI")
            assert auto_summary.gnss_sync_ratio is None
            assert progress == [(1, 3), (2, 3), (3, 3)]

            # sample_every reuse is allowed only across otherwise valid synchronized frames.
            sampled_video = root / "auto-sampled.mp4"
            sampled_csv = root / "auto-sampled.csv"
            sampled = record_preview.preview_record_session(
                session,
                root / "fake-auto.onnx",
                manifest_path=auto_manifest,
                output_video=sampled_video,
                output_csv=sampled_csv,
                sample_every=2,
            )
            assert sampled.inferred_frames == 2
            sampled_rows = _read_csv(sampled_csv)
            assert [row["inference_reused"] for row in sampled_rows] == [
                "False",
                "True",
                "False",
            ]

            # A sensor sync gap invalidates the cached command. The following
            # valid frame must run a fresh inference even if sample_every would
            # normally reuse an older prediction.
            _write_lidar(session, (TIMES[0], TIMES[2]))
            record_preview.MAXIMUM_LIDAR_SKEW_SECONDS = 0.04
            gap_video = root / "auto-gap.mp4"
            gap_csv = root / "auto-gap.csv"
            gap = record_preview.preview_record_session(
                session,
                root / "fake-auto.onnx",
                manifest_path=auto_manifest,
                output_video=gap_video,
                output_csv=gap_csv,
                sample_every=3,
            )
            assert gap.inferred_frames == 2
            gap_rows = _read_csv(gap_csv)
            assert [row["status"] for row in gap_rows] == [
                "OK",
                "LIDAR_NOT_SYNCHRONIZED",
                "OK",
            ]
            assert gap_rows[2]["inference_reused"] == "False"
            record_preview.MAXIMUM_LIDAR_SKEW_SECONDS = original_lidar_skew
            _write_lidar(session, TIMES)

            # Cancellation is checked between rendered frames rather than only
            # after a full recording completes.
            cancel_state = {"stop": False}

            def cancel_progress(done, total):
                if done >= 1:
                    cancel_state["stop"] = True

            try:
                record_preview.preview_record_session(
                    session,
                    root / "fake-auto.onnx",
                    manifest_path=auto_manifest,
                    output_video=root / "cancel-preview.mp4",
                    output_csv=root / "cancel-preview.csv",
                    progress_callback=cancel_progress,
                    cancelled=lambda: cancel_state["stop"],
                )
            except RuntimeError as error:
                assert str(error) == "JOB_CANCELLED"
            else:
                raise AssertionError("RECORD preview cancellation must stop between frames")

            gps_manifest = root / "gps_manifest.json"
            gps_manifest.write_text(
                json.dumps({"policy_type": "AUTO_GPS", "route_id": "preview-route"}),
                encoding="utf-8",
            )
            gps_video = root / "gps-preview.mp4"
            gps_csv = root / "gps-preview.csv"
            _FakeGpsRuntime.route_vectors = []
            _FakeGpsRuntime.measured_values = []
            _FakeGpsRuntime.reset_calls = 0
            gps_summary = record_preview.preview_record_session(
                session,
                root / "fake-gps.onnx",
                manifest_path=gps_manifest,
                route_path=route_path,
                output_video=gps_video,
                output_csv=gps_csv,
                sample_every=5,
            )
            gps_rows = _assert_common(gps_summary, gps_csv, policy="AUTO_GPS")
            assert gps_summary.route_id == "preview-route"
            assert abs(gps_summary.gnss_sync_ratio - 1.0) < 1e-9
            assert len(_FakeGpsRuntime.route_vectors) == 3
            # Temporal measured-feedback models must ignore requested sampling
            # gaps and infer every frame so their history cadence stays correct.
            assert gps_summary.inferred_frames == 3
            assert _FakeGpsRuntime.measured_values == [5.0, 6.0, 7.0]
            assert [float(row["actual_steering_degrees"]) for row in gps_rows] == [5.0, 6.0, 7.0]

            mismatch_route = root / "wrong-route.json"
            _build_route(mismatch_route, route_id="wrong-route")
            try:
                record_preview.preview_record_session(
                    session,
                    root / "fake-gps.onnx",
                    manifest_path=gps_manifest,
                    route_path=mismatch_route,
                    output_video=root / "must-not-exist.mp4",
                    output_csv=root / "must-not-exist.csv",
                )
            except ValueError as error:
                assert "AUTO_GPS_RECORD_PREVIEW_ROUTE_MISMATCH" in str(error)
            else:
                raise AssertionError("AUTO_GPS route mismatch must fail closed")

        print("RECORD model preview measured steering smoke validation passed")
    finally:
        record_preview.AutoAiRuntime = original_auto
        record_preview.GpsAiRuntime = original_gps
        record_preview.MAXIMUM_LIDAR_SKEW_SECONDS = original_lidar_skew


if __name__ == "__main__":
    main()
