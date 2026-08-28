#!/usr/bin/env python3
import argparse
import csv
import json
import math
import statistics
import time
import urllib.request
from datetime import datetime
from pathlib import Path


def fetch_status(base_url, timeout):
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}/api/status",
        headers={"Cache-Control": "no-cache"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def finite_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def sample_from_status(status, started_at):
    gps = status.get("devices", {}).get("gps", {})
    navigation = status.get("navigation", {})
    safety = navigation.get("safety", {})
    return {
        "wall_time": time.time(),
        "elapsed_seconds": time.monotonic() - started_at,
        "mode": navigation.get("mode"),
        "motors_enabled": navigation.get("motors_enabled"),
        "motor_throttle": finite_number(navigation.get("throttle")),
        "requested_throttle": finite_number(safety.get("requested_throttle")),
        "final_throttle": finite_number(safety.get("final_throttle")),
        "allowed": safety.get("allowed"),
        "stop_reason": safety.get("stop_reason"),
        "obstacle_distance_m": finite_number(safety.get("obstacle_distance_m")),
        "gnss_fix": gps.get("fix"),
        "gnss_speed_mps": finite_number(gps.get("speed_mps")),
        "latitude": finite_number(gps.get("latitude")),
        "longitude": finite_number(gps.get("longitude")),
        "satellites_used": gps.get("satellites_used"),
        "hdop": finite_number(gps.get("hdop")),
        "error": "",
    }


def error_sample(started_at, error):
    return {
        "wall_time": time.time(),
        "elapsed_seconds": time.monotonic() - started_at,
        "mode": None,
        "motors_enabled": None,
        "motor_throttle": None,
        "requested_throttle": None,
        "final_throttle": None,
        "allowed": None,
        "stop_reason": None,
        "obstacle_distance_m": None,
        "gnss_fix": None,
        "gnss_speed_mps": None,
        "latitude": None,
        "longitude": None,
        "satellites_used": None,
        "hdop": None,
        "error": f"{type(error).__name__}: {error}",
    }


def active_segments(samples, minimum_duration, settle_seconds):
    segments = []
    current = []
    current_throttle = None

    def finish():
        nonlocal current
        if not current:
            return
        duration = current[-1]["elapsed_seconds"] - current[0]["elapsed_seconds"]
        if duration >= minimum_duration:
            usable_start = current[0]["elapsed_seconds"] + settle_seconds
            usable = [
                sample
                for sample in current
                if sample["elapsed_seconds"] >= usable_start
                and sample["gnss_speed_mps"] is not None
            ]
            if usable:
                speeds = [sample["gnss_speed_mps"] for sample in usable]
                segments.append(
                    {
                        "throttle": current_throttle,
                        "start_seconds": current[0]["elapsed_seconds"],
                        "duration_seconds": duration,
                        "samples": len(usable),
                        "mean_speed_mps": statistics.fmean(speeds),
                        "median_speed_mps": statistics.median(speeds),
                        "min_speed_mps": min(speeds),
                        "max_speed_mps": max(speeds),
                        "fixes": sorted(
                            {
                                sample["gnss_fix"]
                                for sample in usable
                                if sample["gnss_fix"]
                            }
                        ),
                    }
                )
        current = []

    for sample in samples:
        throttle = sample["final_throttle"]
        active = (
            sample["error"] == ""
            and sample["motors_enabled"] is True
            and sample["allowed"] is True
            and throttle is not None
            and abs(throttle) >= 0.08
        )
        rounded = round(abs(throttle), 2) if active else None
        if rounded != current_throttle:
            finish()
            current_throttle = rounded
        if active:
            current.append(sample)
    finish()
    return segments


def write_csv(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(samples[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Read-only throttle calibration logger. It only polls /api/status and "
            "never sends motor or steering commands."
        )
    )
    parser.add_argument(
        "--base-url",
        default="http://192.168.137.2:8080",
        help="Dashboard base URL",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=180.0,
        help="Recording duration in seconds",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Status polling rate in Hz",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="HTTP timeout in seconds",
    )
    parser.add_argument(
        "--minimum-segment",
        type=float,
        default=5.0,
        help="Minimum constant-throttle segment duration",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=2.0,
        help="Discard this time from each segment start",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output CSV path",
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.duration <= 0 or arguments.rate <= 0:
        raise SystemExit("duration and rate must be positive")

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    output = arguments.output or Path("field-tests") / f"throttle-{timestamp}.csv"
    interval = 1.0 / arguments.rate
    started_at = time.monotonic()
    deadline = started_at + arguments.duration
    samples = []

    print("READ-ONLY logger: no motor or steering commands will be sent.")
    print(f"Recording {arguments.base_url} for {arguments.duration:.1f}s -> {output}")
    while time.monotonic() < deadline:
        cycle_started = time.monotonic()
        try:
            status = fetch_status(arguments.base_url, arguments.timeout)
            sample = sample_from_status(status, started_at)
        except Exception as error:
            sample = error_sample(started_at, error)
        samples.append(sample)
        throttle = sample["final_throttle"]
        speed = sample["gnss_speed_mps"]
        print(
            "\r"
            f"{sample['elapsed_seconds']:6.1f}s "
            f"mode={sample['mode'] or '--':<13} "
            f"throttle={throttle if throttle is not None else 0:4.2f} "
            f"speed={speed if speed is not None else 0:5.2f}m/s "
            f"fix={sample['gnss_fix'] or '--':<10} "
            f"stop={sample['stop_reason'] or '--':<18}",
            end="",
            flush=True,
        )
        remaining = interval - (time.monotonic() - cycle_started)
        if remaining > 0:
            time.sleep(remaining)
    print()

    if not samples:
        raise SystemExit("No samples recorded")
    write_csv(output, samples)
    segments = active_segments(
        samples,
        minimum_duration=arguments.minimum_segment,
        settle_seconds=arguments.settle_seconds,
    )
    summary_path = output.with_suffix(".summary.json")
    summary_path.write_text(
        json.dumps(
            {
                "base_url": arguments.base_url,
                "duration_seconds": samples[-1]["elapsed_seconds"],
                "sample_count": len(samples),
                "error_count": sum(bool(sample["error"]) for sample in samples),
                "segments": segments,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"Saved CSV: {output}")
    print(f"Saved summary: {summary_path}")
    if not segments:
        print("No constant-throttle segment met the duration requirement.")
        return
    print("Stable segments:")
    for segment in segments:
        print(
            f"  {segment['throttle'] * 100:4.0f}% | "
            f"{segment['duration_seconds']:5.1f}s | "
            f"mean {segment['mean_speed_mps']:.3f}m/s "
            f"({segment['mean_speed_mps'] * 3.6:.2f}km/h) | "
            f"median {segment['median_speed_mps']:.3f}m/s | "
            f"{', '.join(segment['fixes']) or 'NO FIX'}"
        )


if __name__ == "__main__":
    main()
