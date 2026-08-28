import argparse
import glob
import json
import os
import sys
import urllib.request

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from autonomous_car.perception import CameraCalibration, calibrate_chessboard


def main():
    parser = argparse.ArgumentParser(
        description="Create camera intrinsics from chessboard images",
    )
    parser.add_argument("image_directory")
    parser.add_argument(
        "--output",
        default="/home/gnss/camera-stream/camera-calibration.json",
    )
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.025)
    parser.add_argument(
        "--reload-url",
        default="http://127.0.0.1:8080/api/camera/calibration/reload",
    )
    parser.add_argument("--no-reload", action="store_true")
    arguments = parser.parse_args()

    image_paths = []
    for pattern in ("*.jpg", "*.jpeg", "*.png"):
        image_paths.extend(
            glob.glob(os.path.join(arguments.image_directory, pattern))
        )
    image_paths.sort()
    document = calibrate_chessboard(
        image_paths,
        board_columns=arguments.columns,
        board_rows=arguments.rows,
        square_size_m=arguments.square_size_m,
    )
    calibration = CameraCalibration(arguments.output)
    snapshot = calibration.save(document)
    reload_result = None
    if not arguments.no_reload:
        try:
            request = urllib.request.Request(
                arguments.reload_url,
                method="POST",
                data=b"{}",
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(request, timeout=5) as response:
                reload_result = json.load(response)
        except Exception as error:
            reload_result = {"error": str(error)}
    print(
        json.dumps(
            {
                "calibration": snapshot,
                "runtime_reload": reload_result,
                "accepted_images": document["accepted_images"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
