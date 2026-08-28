"""Measured-steering temporal contract for new AUTO_GPS candidates.

The temporal GPS network shape stays compatible with the v2 20-value auxiliary
input, but the steering history semantics are corrected: training uses previous
encoder-measured wheel angle, and exported manifests require the runtime to feed
measured steering rather than previous model predictions.
"""

from __future__ import annotations

import json
import math
import os

from .temporal_gps import (
    GpsDatasetBuilder as _TemporalGpsDatasetBuilder,
    GpsOnnxExporter as _TemporalGpsOnnxExporter,
    TEMPORAL_HISTORY_STEPS,
    TEMPORAL_MAX_GAP_SECONDS,
)


STEERING_HISTORY_SOURCE = "MEASURED_ENCODER"


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _pad(values, size):
    values = list(values or [])[-size:]
    return [None] * max(0, size - len(values)) + values


def annotate_measured_steering_temporal_context(samples):
    """Attach yaw/current + previous measured-steering history to samples.

    For a sample at t, yaw history includes t while steering history ends at
    t-1. The current steering target and current encoder angle are therefore not
    leaked into the current prediction target.
    """

    for index, sample in enumerate(samples):
        current_time = _finite(sample.get("timestamp_monotonic"))
        chain = []
        previous_time = current_time
        cursor = index
        while cursor >= 0 and len(chain) < TEMPORAL_HISTORY_STEPS + 1:
            candidate = samples[cursor]
            timestamp = _finite(candidate.get("timestamp_monotonic"))
            if timestamp is None:
                break
            if previous_time is not None and previous_time - timestamp > TEMPORAL_MAX_GAP_SECONDS:
                break
            chain.append(candidate)
            previous_time = timestamp
            cursor -= 1
        chain.reverse()

        yaw_samples = chain[-TEMPORAL_HISTORY_STEPS:]
        measured_steering_samples = chain[:-1][-TEMPORAL_HISTORY_STEPS:]
        yaw_history = [
            _finite((item.get("learned_features") or {}).get("imu_yaw_rate_dps"))
            for item in yaw_samples
        ]
        measured_history = [
            _finite((item.get("labels") or {}).get("actual_steering_degrees"))
            for item in measured_steering_samples
        ]
        yaw_history = _pad(yaw_history, TEMPORAL_HISTORY_STEPS)
        measured_history = _pad(measured_history, TEMPORAL_HISTORY_STEPS)
        sample.setdefault("learned_features", {})["temporal"] = {
            "history_steps": TEMPORAL_HISTORY_STEPS,
            "yaw_rate_history_dps": yaw_history,
            # Keep the existing key so the 20-value tensor shape remains stable;
            # the dataset/manifest contract below makes the new semantics explicit.
            "previous_steering_history_degrees": measured_history,
            "steering_history_source": STEERING_HISTORY_SOURCE,
            "current_steering_excluded": True,
            "maximum_neighbor_gap_seconds": TEMPORAL_MAX_GAP_SECONDS,
        }


class GpsDatasetBuilder(_TemporalGpsDatasetBuilder):
    """Temporal GPS dataset using encoder-measured steering history."""

    _annotate_temporal_context = staticmethod(
        annotate_measured_steering_temporal_context
    )

    def build(self, session_names, dataset_id=None):
        document = super().build(session_names, dataset_id)
        temporal = document.setdefault("gps_training_policy", {}).setdefault(
            "temporal_context", {}
        )
        temporal.update(
            {
                "steering_history_source": STEERING_HISTORY_SOURCE,
                "current_steering_target_excluded": True,
                "current_measured_steering_excluded": True,
                "features_per_step": [
                    "imu_yaw_rate_normalized",
                    "imu_yaw_rate_present",
                    "previous_measured_steering_normalized",
                    "previous_measured_steering_present",
                ],
            }
        )
        document.setdefault("feature_contract", {})["temporal_steering"] = {
            "source": STEERING_HISTORY_SOURCE,
            "record_field": "camera_timestamps.csv:steering_angle_degrees",
            "target_field_not_used_as_history": "target_steering_angle_degrees",
            "current_frame_excluded": True,
            "missing_measurement": "zero_value_with_presence_bit_0",
        }
        output_path = os.path.join(
            self.output_root,
            document["dataset_id"],
            "dataset.json",
        )
        with open(output_path, "w", encoding="utf-8") as handle:
            json.dump(document, handle, ensure_ascii=False, indent=2)
        return document


class GpsOnnxExporter(_TemporalGpsOnnxExporter):
    """Export v3 manifest declaring measured steering feedback semantics."""

    def export(self, checkpoint_path, output_path, **kwargs):
        manifest = super().export(checkpoint_path, output_path, **kwargs)
        manifest["schema"] = "autonomy_gps_ai_onnx_manifest_v3"
        auxiliary = manifest.setdefault("inputs", {}).setdefault("auxiliary", {})
        auxiliary.update(
            {
                "steering_history_source": STEERING_HISTORY_SOURCE,
                "requires_measured_steering_feedback": True,
                "contract": (
                    f"{auxiliary.get('history_steps', TEMPORAL_HISTORY_STEPS)} chronological "
                    "steps: normalized IMU yaw rate + presence + previous encoder-measured "
                    "steering angle + presence; current steering is excluded"
                ),
            }
        )
        manifest["runtime_contract"] = {
            "steering_history_source": STEERING_HISTORY_SOURCE,
            "measured_steering_field": "steering_angle_degrees",
            "prediction_feedback_forbidden": True,
            "missing_measurement": "presence_bit_0",
        }
        with open(
            os.path.join(output_path, "model_manifest.json"),
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(manifest, handle, indent=2)
        return manifest


__all__ = [
    "GpsDatasetBuilder",
    "GpsOnnxExporter",
    "STEERING_HISTORY_SOURCE",
    "annotate_measured_steering_temporal_context",
]
