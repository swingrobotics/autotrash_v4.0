import argparse
import hashlib
import os
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(
        description="Capture chessboard images from the dashboard camera",
    )
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:8080/snapshot.jpg",
    )
    parser.add_argument("--output-dir", default="camera-calibration-images")
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--interval", type=float, default=1.5)
    arguments = parser.parse_args()

    os.makedirs(arguments.output_dir, exist_ok=True)
    previous_digest = None
    captured = 0
    attempts = 0
    print(
        "체커보드가 화면 여러 위치와 각도에 보이도록 천천히 움직이세요.",
        flush=True,
    )
    while captured < max(1, arguments.count):
        attempts += 1
        with urllib.request.urlopen(arguments.url, timeout=5) as response:
            frame = response.read()
        if not frame.startswith(b"\xff\xd8") or not frame.endswith(b"\xff\xd9"):
            raise RuntimeError("Camera endpoint did not return a JPEG frame")
        digest = hashlib.sha256(frame).digest()
        if digest != previous_digest:
            captured += 1
            path = os.path.join(
                arguments.output_dir,
                f"calibration_{captured:03d}.jpg",
            )
            with open(path, "wb") as file:
                file.write(frame)
            print(f"[{captured}/{arguments.count}] {path}", flush=True)
            previous_digest = digest
        if captured < arguments.count:
            time.sleep(max(0.1, arguments.interval))
        if attempts > arguments.count * 10:
            raise RuntimeError("Too many duplicate camera frames")


if __name__ == "__main__":
    main()
