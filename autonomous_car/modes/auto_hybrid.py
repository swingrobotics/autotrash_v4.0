from dataclasses import dataclass


@dataclass(frozen=True)
class HybridFallbackDecision:
    fallback_reason: str | None
    lane_failure_count: int


class HybridFallbackGuard:
    def __init__(
        self,
        camera_timeout_seconds=0.3,
        maximum_lane_failures=5,
        minimum_lane_confidence=0.55,
    ):
        self.camera_timeout_seconds = float(camera_timeout_seconds)
        self.maximum_lane_failures = max(1, int(maximum_lane_failures))
        self.minimum_lane_confidence = max(
            0.0,
            min(1.0, float(minimum_lane_confidence)),
        )
        self.lane_failure_count = 0

    def reset(self):
        self.lane_failure_count = 0

    def evaluate(self, camera_age_seconds, new_frame=False, lane_result=None):
        if (
            camera_age_seconds is None
            or float(camera_age_seconds) > self.camera_timeout_seconds
        ):
            self.reset()
            return HybridFallbackDecision("CAMERA_TIMEOUT", self.lane_failure_count)
        if not new_frame:
            return HybridFallbackDecision(None, self.lane_failure_count)
        detected = bool(lane_result and lane_result.get("detected"))
        try:
            confidence = float((lane_result or {}).get("confidence") or 0.0)
        except (TypeError, ValueError):
            confidence = 0.0
        if detected and confidence >= self.minimum_lane_confidence:
            self.reset()
            return HybridFallbackDecision(None, self.lane_failure_count)
        self.lane_failure_count += 1
        if self.lane_failure_count >= self.maximum_lane_failures:
            self.reset()
            return HybridFallbackDecision(
                "LANE_CONFIDENCE_LOW" if detected else "LANE_NOT_DETECTED",
                self.lane_failure_count,
            )
        return HybridFallbackDecision(None, self.lane_failure_count)


class LaneContinuityFilter:
    def __init__(
        self,
        maximum_lateral_jump_m=0.35,
        maximum_heading_jump_degrees=12.0,
        correction_smoothing=0.35,
        maximum_lateral_jump_normalized=0.70,
    ):
        self.maximum_lateral_jump_m = abs(float(maximum_lateral_jump_m))
        self.maximum_lateral_jump_normalized = abs(
            float(maximum_lateral_jump_normalized)
        )
        self.maximum_heading_jump_degrees = abs(float(maximum_heading_jump_degrees))
        self.correction_smoothing = max(
            0.0,
            min(1.0, float(correction_smoothing)),
        )
        self.previous_lateral_error_m = None
        self.previous_lateral_error_normalized = None
        self.previous_heading_error_degrees = None
        self.previous_correction_degrees = None

    def reset(self):
        self.previous_lateral_error_m = None
        self.previous_lateral_error_normalized = None
        self.previous_heading_error_degrees = None
        self.previous_correction_degrees = None

    def filter(self, lane_result):
        result = dict(lane_result or {})
        if not result.get("detected"):
            return result
        try:
            lateral = float(result["lateral_error_m"])
            normalized_raw = result.get("lateral_error_normalized")
            normalized = (
                None
                if normalized_raw is None
                else float(normalized_raw)
            )
            heading = float(result["heading_error_degrees"])
            correction = float(
                result.get("correction_angle_degrees") or 0.0
            )
            confidence = max(
                0.0,
                min(1.0, float(result.get("confidence") or 0.0)),
            )
        except (KeyError, TypeError, ValueError):
            result.update(
                detected=False,
                confidence=0.0,
                correction_angle_degrees=0.0,
                error="LANE_RESULT_INVALID",
            )
            return result

        if self.previous_heading_error_degrees is not None:
            heading_jump = abs(
                heading - self.previous_heading_error_degrees
            )
            if (
                normalized is not None
                and self.previous_lateral_error_normalized is not None
            ):
                lateral_jump = abs(
                    normalized - self.previous_lateral_error_normalized
                )
                lateral_limit = max(
                    self.maximum_lateral_jump_normalized,
                    1e-6,
                )
                lateral_key = "lateral_jump_normalized"
            else:
                lateral_jump = abs(
                    lateral - self.previous_lateral_error_m
                )
                lateral_limit = max(
                    self.maximum_lateral_jump_m,
                    1e-6,
                )
                lateral_key = "lateral_jump_m"

            if (
                lateral_jump > lateral_limit
                or heading_jump > self.maximum_heading_jump_degrees
            ):
                result.update(
                    detected=False,
                    confidence=0.0,
                    correction_angle_degrees=0.0,
                    error="LANE_TEMPORAL_JUMP",
                    heading_jump_degrees=heading_jump,
                )
                result[lateral_key] = lateral_jump
                return result

            lateral_score = 1.0 - lateral_jump / lateral_limit
            heading_score = 1.0 - heading_jump / max(
                self.maximum_heading_jump_degrees,
                1e-6,
            )
            confidence *= 0.5 + 0.5 * min(
                lateral_score,
                heading_score,
            )

        if self.previous_correction_degrees is not None:
            smoothing = self.correction_smoothing
            correction = (
                smoothing * correction
                + (1.0 - smoothing) * self.previous_correction_degrees
            )

        self.previous_lateral_error_m = lateral
        self.previous_lateral_error_normalized = normalized
        self.previous_heading_error_degrees = heading
        self.previous_correction_degrees = correction
        result["confidence"] = confidence
        result["correction_angle_degrees"] = correction
        result["error"] = None
        return result
