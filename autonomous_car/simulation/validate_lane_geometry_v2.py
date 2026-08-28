"""Regression for indoor tape and outdoor road-lane geometry."""

from autonomous_car.control import LaneController


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _result(controller, image, label):
    result = controller.analyze_image(image)
    snapshot = result.as_dict()
    _require(result.detected, f"{label} lane was not detected: {snapshot}")
    _require(result.confidence >= 0.55, f"{label} confidence too low: {snapshot}")
    _require(result.left_line is not None, f"{label} left geometry missing: {snapshot}")
    _require(result.right_line is not None, f"{label} right geometry missing: {snapshot}")
    _require(result.center_line is not None, f"{label} center geometry missing: {snapshot}")
    return result


def main():
    controller = LaneController(expected_lane_width_m=1.0, vehicle_width_m=0.4826)
    _require(
        controller.available,
        "OpenCV/NumPy are required for lane geometry validation",
    )

    import cv2
    import numpy as np

    # Indoor: light concrete floor + black tape. Bottom width deliberately
    # exceeds the legacy 85%-of-frame rejection threshold.
    black = np.full((360, 640, 3), 182, dtype=np.uint8)
    cv2.line(black, (14, 350), (250, 180), (18, 18, 18), 10)
    cv2.line(black, (626, 350), (390, 180), (18, 18, 18), 10)
    black_result = _result(controller, black, "black tape")
    _require(
        black_result.marking == "BLACK",
        f"black marking classification failed: {black_result.as_dict()}",
    )
    _require(
        (black_result.lane_width_pixels or 0.0) > 640 * 0.85,
        f"black test lane does not regress old width cap: {black_result.as_dict()}",
    )

    # /api/lane and an autonomous controller can request the same JPEG. The
    # detector must return its cached LaneResult instead of running OpenCV twice.
    encoded, jpeg = cv2.imencode(".jpg", black)
    _require(bool(encoded), "failed to encode lane cache test frame")
    frame = jpeg.tobytes()
    controller.reset()
    cached_first = controller.analyze_jpeg(frame)
    cached_second = controller.analyze_jpeg(frame)
    _require(
        cached_first is cached_second,
        "identical camera frame did not reuse the lane result cache",
    )
    _require(cached_first.detected, f"cached black frame failed: {cached_first.as_dict()}")

    # Indoor/outdoor: yellow markings on a dark surface.
    controller.reset()
    yellow = np.full((360, 640, 3), 72, dtype=np.uint8)
    cv2.line(yellow, (90, 350), (260, 170), (0, 220, 255), 8)
    cv2.line(yellow, (550, 350), (380, 170), (0, 220, 255), 8)
    yellow_result = _result(controller, yellow, "yellow")
    _require(
        "YELLOW" in str(yellow_result.marking),
        f"yellow marking classification failed: {yellow_result.as_dict()}",
    )

    # Outdoor: white lane paint on asphalt.
    controller.reset()
    white = np.full((360, 640, 3), 78, dtype=np.uint8)
    cv2.line(white, (100, 350), (270, 170), (248, 248, 248), 8)
    cv2.line(white, (540, 350), (370, 170), (248, 248, 248), 8)
    white_result = _result(controller, white, "white road")
    _require(
        "WHITE" in str(white_result.marking),
        f"white marking classification failed: {white_result.as_dict()}",
    )

    # Outdoor: curved mixed white/yellow boundaries. A quadratic fit must retain
    # the curve instead of reducing it to two Hough straight lines.
    controller.reset()
    curved = np.full((360, 640, 3), 70, dtype=np.uint8)
    left_points = []
    right_points = []
    for y in range(150, 359):
        t = (y - 150) / 209.0
        center = 320.0 + 42.0 * t * t
        half_width = 60.0 + 180.0 * t
        left_points.append((int(center - half_width), y))
        right_points.append((int(center + half_width), y))
    cv2.polylines(
        curved,
        [np.asarray(left_points, dtype=np.int32)],
        False,
        (250, 250, 250),
        8,
    )
    cv2.polylines(
        curved,
        [np.asarray(right_points, dtype=np.int32)],
        False,
        (0, 220, 255),
        8,
    )
    curved_result = _result(controller, curved, "curved road")
    center_points = curved_result.center_line.get("points") or []
    _require(
        len(center_points) >= 10,
        f"curved center polyline too short: {curved_result.as_dict()}",
    )
    _require(
        abs(curved_result.heading_error_degrees or 0.0) > 0.5,
        f"curvature/heading not represented: {curved_result.as_dict()}",
    )

    # Dashed road paint must still produce enough samples for a curve fit.
    controller.reset()
    dashed = np.full((360, 640, 3), 74, dtype=np.uint8)
    for y0 in range(170, 350, 45):
        y1 = min(355, y0 + 25)
        left0 = int(270 - (y0 - 170) * 0.80)
        left1 = int(270 - (y1 - 170) * 0.80)
        right0 = int(370 + (y0 - 170) * 0.80)
        right1 = int(370 + (y1 - 170) * 0.80)
        cv2.line(
            dashed,
            (left0, y0),
            (left1, y1),
            (250, 250, 250),
            8,
        )
        cv2.line(
            dashed,
            (right0, y0),
            (right1, y1),
            (0, 220, 255),
            8,
        )
    dashed_result = _result(controller, dashed, "dashed road")

    # A short occlusion can infer one missing boundary from the most recent
    # two-sided width profile. This is intentionally confidence-limited.
    occluded = np.full((360, 640, 3), 74, dtype=np.uint8)
    cv2.line(occluded, (100, 350), (270, 170), (250, 250, 250), 8)
    occluded_result = controller.analyze_image(occluded)
    _require(
        occluded_result.detected,
        f"short one-sided occlusion was not bridged: {occluded_result.as_dict()}",
    )
    _require(
        occluded_result.inferred_right,
        f"missing right boundary was not marked inferred: {occluded_result.as_dict()}",
    )
    _require(
        occluded_result.confidence <= 0.64,
        f"inferred lane confidence is too permissive: {occluded_result.as_dict()}",
    )

    print("Lane geometry V2 regression: PASS")
    print(
        {
            "black": black_result.as_dict(),
            "yellow": yellow_result.as_dict(),
            "white": white_result.as_dict(),
            "curved": curved_result.as_dict(),
            "dashed": dashed_result.as_dict(),
            "occluded": occluded_result.as_dict(),
            "shared_frame_cache": True,
        }
    )


if __name__ == "__main__":
    main()
