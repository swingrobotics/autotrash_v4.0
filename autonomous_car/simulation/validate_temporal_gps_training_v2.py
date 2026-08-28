#!/usr/bin/env python3
"""Fast contract checks for temporal/rare-curve AUTO_GPS training."""

from collections import Counter
import inspect

from autonomous_car.ai import GpsDatasetBuilder, GpsDrivingModelSpec, GpsTrainer
from autonomous_car.ai.gps_runtime import GpsAiRuntime
from autonomous_car.ai.measured_steering_gps import STEERING_HISTORY_SOURCE
from autonomous_car.ai.temporal_gps import (
    TEMPORAL_AUXILIARY_SIZE,
    TEMPORAL_HISTORY_STEPS,
    encode_temporal_auxiliary,
)
from autonomous_car.ai.video_preview import command_path_points


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def validate_preview_sign():
    width = 1000
    left = command_path_points(width, 600, +0.8)[-1][0]
    right = command_path_points(width, 600, -0.8)[-1][0]
    require(left < width * 0.5, "positive SWING steering must draw LEFT")
    require(right > width * 0.5, "negative SWING steering must draw RIGHT")


def validate_temporal_vector():
    spec = GpsDrivingModelSpec()
    require(spec.temporal_history_steps == TEMPORAL_HISTORY_STEPS == 5, "history must be five frames")
    require(spec.auxiliary_feature_size == TEMPORAL_AUXILIARY_SIZE == 20, "auxiliary must contain 20 values")
    sample = {
        "learned_features": {
            "imu_yaw_rate_dps": 30.0,
            "temporal": {
                "yaw_rate_history_dps": [-30.0, -15.0, 0.0, 15.0, 30.0],
                "previous_steering_history_degrees": [-10.0, -5.0, 0.0, 5.0, 10.0],
                "steering_history_source": STEERING_HISTORY_SOURCE,
            },
        }
    }
    values = encode_temporal_auxiliary(sample, spec)
    require(len(values) == 20, "temporal vector shape changed")
    require(values[0] < 0.0 and values[1] == 1.0, "negative yaw/presence lost")
    require(values[2] < 0.0 and values[3] == 1.0, "negative measured steering/presence lost")
    require(values[-4] > 0.0 and values[-3] == 1.0, "positive yaw/presence lost")
    require(values[-2] > 0.0 and values[-1] == 1.0, "positive measured steering/presence lost")


def validate_temporal_annotation():
    samples = []
    for index in range(6):
        samples.append(
            {
                "timestamp_monotonic": index * 0.1,
                "learned_features": {"imu_yaw_rate_dps": float(index)},
                "labels": {
                    # Deliberately make target and actual opposite so this test
                    # fails if imitation labels are accidentally reused as state.
                    "steering_degrees": float(50 + index),
                    "target_steering_degrees": float(50 + index),
                    "actual_steering_degrees": float(index * 2),
                },
            }
        )
    GpsDatasetBuilder._annotate_temporal_context(samples)
    temporal = samples[-1]["learned_features"]["temporal"]
    require(temporal["yaw_rate_history_dps"] == [1.0, 2.0, 3.0, 4.0, 5.0], "yaw history alignment changed")
    require(temporal["previous_steering_history_degrees"] == [0.0, 2.0, 4.0, 6.0, 8.0], "measured steering alignment changed")
    require(temporal["steering_history_source"] == "MEASURED_ENCODER", "measured steering source marker missing")
    require(10.0 not in temporal["previous_steering_history_degrees"], "current measured steering leaked into model input")
    require(50.0 not in temporal["previous_steering_history_degrees"], "human target steering leaked into temporal state")


def validate_curve_aware_split():
    class Probe(GpsDatasetBuilder):
        def __init__(self):
            self.values = {
                "only-right": Counter({"straight": 100, "sharp_right": 12}),
                "left-a": Counter({"straight": 100, "gentle_left": 15}),
                "left-b": Counter({"straight": 100, "gentle_left": 10}),
            }

        def _raw_steering_counts(self, session_name):
            return self.values[session_name]

    split = Probe()._assign_session_splits(["only-right", "left-a", "left-b"])
    require(split["only-right"] == "train", "unique right-turn demonstration was held out")
    require(list(split.values()).count("validation") == 1, "three-session split must hold out one run")
    require(list(split.values()).count("train") == 2, "three-session split must train on two runs")


def validate_recovery_weight():
    trainer = GpsTrainer()
    ordinary = {"scenario": "straight", "training_context": {}}
    recovery = {"scenario": "straight", "training_context": {"route_recovery": True}}
    require(trainer._weight(ordinary) == 1.0, "straight weight changed unexpectedly")
    require(trainer._weight(recovery) == 3.0, "route recovery must receive maximum steering weight")


def validate_runtime_contract():
    require(hasattr(GpsAiRuntime, "reset_temporal_state"), "runtime cannot clear stale temporal history")
    signature = inspect.signature(GpsAiRuntime.infer_jpeg)
    require("measured_steering_degrees" in signature.parameters, "runtime cannot accept encoder steering feedback")
    source = inspect.getsource(GpsAiRuntime.infer_jpeg)
    require("self.requires_measured_steering" in source, "runtime does not branch on measured steering contract")
    require("self._resolve_measured_steering" in source, "runtime does not resolve measured encoder steering")
    require(GpsAiRuntime.MEASURED_STEERING_SOURCE == "MEASURED_ENCODER", "runtime measured steering source changed")


def main():
    validate_preview_sign()
    validate_temporal_vector()
    validate_temporal_annotation()
    validate_curve_aware_split()
    validate_recovery_weight()
    validate_runtime_contract()
    print("TEMPORAL_GPS_MEASURED_STEERING_V3_PASS")


if __name__ == "__main__":
    main()
