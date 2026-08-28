"""Preserve raw UFLD candidate lanes for offline RECORD diagnostics.

Offline replay uses the same SWING-oriented ego-pair scoring as the live Worker
UFLD path, while still storing every rejected raw candidate for diagnosis.  It
never owns steering, throttle or motor authority.
"""

from __future__ import annotations

import json
import math

import record_replay_ufld as offline
from autonomous_car.control.hybrid_lane_controller import HybridLaneController


class DiagnosticHybridLaneController(HybridLaneController):
    """UFLD replay controller retaining raw candidates and SWING pair scoring."""

    @staticmethod
    def _pair_score(left, right, center, width):
        confidence = left["confidence"] + right["confidence"]
        lane_center = (left["observed_bottom_x"] + right["observed_bottom_x"]) / 2.0
        center_error = abs(lane_center - center) / max(1.0, width)
        left_distance = max(0.0, center - left["observed_bottom_x"])
        right_distance = max(0.0, right["observed_bottom_x"] - center)
        symmetry_error = abs(left_distance - right_distance) / max(1.0, width)
        semantic_hint = (0.04 if left["lane_id"] == 1 else 0.0) + (
            0.04 if right["lane_id"] == 2 else 0.0
        )
        return confidence + semantic_hint - 0.75 * center_error - 0.30 * symmetry_error

    def _select_pair(self, candidates):
        width = self.processing_width
        center = width / 2.0
        guard = max(8.0, width * 0.012)
        lefts = [item for item in candidates if item["observed_bottom_x"] < center - guard]
        rights = [item for item in candidates if item["observed_bottom_x"] > center + guard]
        pairs = []
        for left in lefts:
            for right in rights:
                if left is right:
                    continue
                bottom_width = right["bottom_x"] - left["bottom_x"]
                top_width = right["top_x"] - left["top_x"]
                if bottom_width <= 18.0 or top_width <= 6.0:
                    continue
                ratio = bottom_width / max(1.0, top_width)
                if ratio < 1.005 or ratio > 14.0:
                    continue
                if not left["observed_bottom_x"] < center < right["observed_bottom_x"]:
                    continue
                center_bottom = (left["bottom_x"] + right["bottom_x"]) / 2.0
                center_error = abs(center_bottom - center) / max(1.0, width)
                if center_error > 0.42:
                    continue
                score = self._pair_score(left, right, center, width)
                score -= 0.08 * abs(math.log(max(1.0, ratio)) - math.log(2.5))
                pairs.append((score, left, right, ratio, center_error))
        if not pairs:
            return None, None
        pairs.sort(key=lambda item: item[0], reverse=True)
        score, left, right, ratio, center_error = pairs[0]
        if isinstance(self._last_preview, dict):
            self._last_preview.update(
                {
                    "pair_score": float(score),
                    "pair_perspective_ratio": float(ratio),
                    "pair_center_error_normalized": float(center_error),
                    "pair_candidates_evaluated": len(pairs),
                }
            )
        return left, right

    def analyze_neural_preview_jpeg(self, jpeg):
        capture = {}
        with self._lock:
            original_infer = self.pretrained.infer

            def infer_with_capture(image):
                result = original_infer(image)
                try:
                    height, width = image.shape[:2]
                    capture["image_size"] = (int(width), int(height))
                except Exception:
                    capture["image_size"] = None
                capture["neural"] = result
                return result

            self.pretrained.infer = infer_with_capture
            try:
                result = super().analyze_neural_preview_jpeg(jpeg)
            finally:
                self.pretrained.infer = original_infer

            neural = capture.get("neural") or {}
            image_size = capture.get("image_size")
            calibrated = self._calibration_usable(self.camera_calibration)
            candidates = []
            for lane in neural.get("lanes") or []:
                points = []
                for point in lane.get("points") or []:
                    if not isinstance(point, (list, tuple)) or len(point) < 2:
                        continue
                    try:
                        points.append([float(point[0]), float(point[1])])
                    except (TypeError, ValueError):
                        continue
                if len(points) < 2 or not image_size:
                    continue

                lane_id = lane.get("lane_id", lane.get("query_id", -1))
                try:
                    lane_id = int(lane_id)
                except (TypeError, ValueError):
                    lane_id = -1
                try:
                    confidence = float(lane.get("confidence") or 0.0)
                except (TypeError, ValueError):
                    confidence = 0.0

                document = {
                    "lane_id": lane_id,
                    "confidence": max(0.0, min(1.0, confidence)),
                    "points": points,
                    "coordinate_space": "UNDISTORTED" if calibrated else "RAW",
                }
                if calibrated:
                    document = self._project_line_to_raw(document, image_size)

                width, height = image_size
                normalized = []
                for point in document.get("points") or []:
                    try:
                        x = float(point[0]) / max(1.0, float(width))
                        y = float(point[1]) / max(1.0, float(height))
                    except (TypeError, ValueError):
                        continue
                    normalized.append(
                        [max(-0.25, min(1.25, x)), max(-0.25, min(1.25, y))]
                    )
                if len(normalized) < 2:
                    continue
                candidates.append(
                    {
                        "lane_id": lane_id,
                        "confidence": document["confidence"],
                        "normalized_points": normalized,
                        "coordinate_space": document.get("coordinate_space") or "RAW",
                    }
                )

            diagnostics = dict(self._last_preview or {})
            diagnostics["raw_candidate_lanes"] = candidates
            diagnostics["raw_candidate_count"] = len(candidates)
            self._last_preview = diagnostics
            return result


def _diagnostic_offline_controller():
    source = getattr(offline.release.full, "HYBRID_LANE_CONTROLLER", None)
    if source is None:
        raise RuntimeError("UFLD_RUNTIME_UNAVAILABLE")

    pretrained_source = source.pretrained
    pretrained_type = type(pretrained_source)
    pretrained = pretrained_type(
        pretrained_source.model_path,
        input_size=(pretrained_source.input_width, pretrained_source.input_height),
        threads=pretrained_source.threads,
        lane_probability_threshold=pretrained_source.lane_probability_threshold,
    )
    return DiagnosticHybridLaneController(
        pretrained,
        camera_calibration=source.camera_calibration,
        expected_lane_width_m=source.expected_lane_width_m,
        vehicle_width_m=source.vehicle_width_m,
        processing_width=source.processing_width,
        processing_height=source.processing_height,
        maximum_neural_inference_ms=source.maximum_neural_inference_ms,
    )


def install_candidate_diagnostics():
    if getattr(offline, "_ufld_candidate_diagnostics_installed", False):
        return True

    if "lane_candidates_json" not in offline._RESULT_FIELDS:
        offline._RESULT_FIELDS.append("lane_candidates_json")

    original_row_from_result = offline._row_from_result

    def row_from_result_with_candidates(offset_seconds, frame_index, lane, diagnostics):
        row = original_row_from_result(offset_seconds, frame_index, lane, diagnostics)
        row["lane_candidates_json"] = json.dumps(
            diagnostics.get("raw_candidate_lanes") or [],
            ensure_ascii=False,
            separators=(",", ":"),
        )
        return row

    offline._row_from_result = row_from_result_with_candidates
    offline._offline_controller = _diagnostic_offline_controller
    offline._ufld_candidate_diagnostics_installed = True
    return True


install_candidate_diagnostics()


__all__ = ["DiagnosticHybridLaneController", "install_candidate_diagnostics"]
