import argparse
import csv
import json
import statistics
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


def fetch_lidar(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url",
        default="http://192.168.137.2:8080/api/lidar",
    )
    parser.add_argument("--duration", type=float, default=1800.0)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output_path.with_suffix(".summary.json")
    started = time.monotonic()
    previous_connected = None
    disconnects = 0
    reconnects = 0
    samples = []

    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(
            output_file,
            fieldnames=[
                "timestamp_utc",
                "elapsed_seconds",
                "connected",
                "rotation_hz",
                "point_count",
                "scan_point_count",
                "last_update",
                "error",
            ],
        )
        writer.writeheader()

        while True:
            elapsed = time.monotonic() - started
            if elapsed > args.duration:
                break
            row = {
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_seconds": round(elapsed, 3),
                "connected": False,
                "rotation_hz": None,
                "point_count": 0,
                "scan_point_count": 0,
                "last_update": None,
                "error": None,
            }
            try:
                lidar = fetch_lidar(args.url, args.timeout)
                row.update(
                    {
                        "connected": bool(lidar.get("connected")),
                        "rotation_hz": lidar.get("rotation_hz"),
                        "point_count": lidar.get("point_count", 0),
                        "scan_point_count": lidar.get("scan_point_count", 0),
                        "last_update": lidar.get("last_update"),
                        "error": lidar.get("error"),
                    }
                )
            except Exception as error:
                row["error"] = str(error)

            connected = row["connected"]
            if previous_connected is True and not connected:
                disconnects += 1
            elif previous_connected is False and connected:
                reconnects += 1
            previous_connected = connected
            samples.append(row)
            writer.writerow(row)
            output_file.flush()
            time.sleep(args.interval)

    connected_rows = [row for row in samples if row["connected"]]
    rotation_values = [
        float(row["rotation_hz"])
        for row in connected_rows
        if row["rotation_hz"] is not None
    ]
    point_values = [int(row["point_count"]) for row in connected_rows]
    summary = {
        "url": args.url,
        "duration_seconds": round(time.monotonic() - started, 3),
        "sample_count": len(samples),
        "connected_samples": len(connected_rows),
        "availability_percent": (
            round(len(connected_rows) * 100.0 / len(samples), 3) if samples else 0.0
        ),
        "disconnect_count": disconnects,
        "reconnect_count": reconnects,
        "rotation_hz": {
            "minimum": min(rotation_values) if rotation_values else None,
            "mean": statistics.fmean(rotation_values) if rotation_values else None,
            "maximum": max(rotation_values) if rotation_values else None,
        },
        "point_count": {
            "minimum": min(point_values) if point_values else None,
            "mean": statistics.fmean(point_values) if point_values else None,
            "maximum": max(point_values) if point_values else None,
        },
        "errors": sorted(
            {
                str(row["error"])
                for row in samples
                if row["error"]
            }
        ),
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
