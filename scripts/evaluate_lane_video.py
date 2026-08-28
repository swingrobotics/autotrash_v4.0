import argparse
import json
import os
import statistics
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from autonomous_car.control import LaneController
from autonomous_car.modes import LaneContinuityFilter
from autonomous_car.perception import CameraCalibration

try:
    import cv2
except ImportError:
    cv2 = None


def main():
    parser = argparse.ArgumentParser(
        description="Offline lane analysis for a recorded camera video",
    )
    parser.add_argument("video_path")
    parser.add_argument("--calibration")
    parser.add_argument("--sample-every", type=int, default=3)
    parser.add_argument("--maximum-frames", type=int, default=0)
    parser.add_argument("--minimum-detection-ratio", type=float, default=0.70)
    parser.add_argument("--minimum-mean-confidence", type=float, default=0.55)
    parser.add_argument("--maximum-correction-jump", type=float, default=3.0)
    arguments = parser.parse_args()
    if cv2 is None:
        raise RuntimeError("OpenCV is required")

    calibration = (
        CameraCalibration(arguments.calibration)
        if arguments.calibration
        else None
    )
    controller = LaneController(camera_calibration=calibration)
    continuity = LaneContinuityFilter()
    capture = cv2.VideoCapture(arguments.video_path)
    if not capture.isOpened():
        raise OSError(f"Could not open {arguments.video_path}")

    analyzed = 0
    detected = 0
    confidences = []
    corrections = []
    errors = {}
    source_index = 0
    while True:
        ok, image = capture.read()
        if not ok:
            break
        source_index += 1
        if source_index % max(1, arguments.sample_every):
            continue
        result = continuity.filter(controller.analyze_image(image).as_dict())
        analyzed += 1
        if result.get("detected"):
            detected += 1
            confidences.append(float(result.get("confidence") or 0.0))
            corrections.append(
                float(result.get("correction_angle_degrees") or 0.0)
            )
        elif result.get("error"):
            error = result["error"]
            errors[error] = errors.get(error, 0) + 1
        if arguments.maximum_frames and analyzed >= arguments.maximum_frames:
            break
    capture.release()

    correction_jumps = [
        abs(current - previous)
        for previous, current in zip(corrections, corrections[1:])
    ]
    detection_ratio = detected / analyzed if analyzed else 0.0
    mean_confidence = statistics.fmean(confidences) if confidences else 0.0
    maximum_jump = max(correction_jumps) if correction_jumps else 0.0
    failures = []
    if detection_ratio < arguments.minimum_detection_ratio:
        failures.append("DETECTION_RATIO")
    if mean_confidence < arguments.minimum_mean_confidence:
        failures.append("MEAN_CONFIDENCE")
    if maximum_jump > arguments.maximum_correction_jump:
        failures.append("CORRECTION_JUMP")
    document = {
        "passed": not failures,
        "failures": failures,
        "video_path": arguments.video_path,
        "calibration": calibration.snapshot() if calibration else None,
        "analyzed_frames": analyzed,
        "detected_frames": detected,
        "detection_ratio": detection_ratio,
        "mean_confidence": mean_confidence,
        "maximum_correction_jump_degrees": maximum_jump,
        "errors": errors,
    }
    print(json.dumps(document, ensure_ascii=False, indent=2))
    raise SystemExit(0 if document["passed"] else 1)


if __name__ == "__main__":
    main()
