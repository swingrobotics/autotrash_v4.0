from dataclasses import dataclass
import json
import os

from .training import (
    DrivingModelSpec,
    ManifestDataset,
    create_torch_model,
    require_training_dependencies,
)


@dataclass(frozen=True)
class EvaluationCriteria:
    maximum_steering_mae_degrees: float | None = None
    maximum_throttle_mae: float | None = None


class Evaluator:
    """Offline held-out evaluation. Never grants AUTO permission by itself."""

    def evaluate(
        self,
        dataset_path,
        checkpoint_path,
        *,
        split="test",
        output_path=None,
        recordings_root_override=None,
        criteria=None,
        device="auto",
    ):
        _, _, torch, _, DataLoader, _ = require_training_dependencies()
        resolved_device = self._device(torch, device)
        checkpoint = torch.load(
            checkpoint_path,
            map_location=resolved_device,
            weights_only=True,
        )
        model_spec = DrivingModelSpec(**checkpoint["model_spec"])
        model = create_torch_model(model_spec).to(resolved_device)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()

        dataset = ManifestDataset(
            dataset_path,
            split,
            model_spec,
            recordings_root_override,
        )
        if len(dataset) == 0:
            dataset.close()
            raise ValueError(f"Evaluation split contains no samples: {split}")
        loader = DataLoader(dataset, batch_size=64, shuffle=False, num_workers=0)

        steering_absolute_error = 0.0
        throttle_absolute_error = 0.0
        sample_count = 0
        scenario = {}
        with torch.no_grad():
            for image, lidar, auxiliary, target, sample_indices in loader:
                prediction = model(
                    image.to(resolved_device),
                    lidar.to(resolved_device),
                    auxiliary.to(resolved_device),
                ).cpu()
                target = target.cpu()
                for row_index in range(prediction.shape[0]):
                    steering_error = abs(
                        float(prediction[row_index, 0] - target[row_index, 0])
                    ) * model_spec.maximum_steering_degrees
                    throttle_error = abs(
                        float(prediction[row_index, 1] - target[row_index, 1])
                    )
                    steering_absolute_error += steering_error
                    throttle_absolute_error += throttle_error
                    sample_count += 1

                    source_index = int(sample_indices[row_index])
                    name = dataset.samples[source_index].get("scenario", "unknown")
                    bucket = scenario.setdefault(
                        name,
                        {
                            "samples": 0,
                            "steering_absolute_error_sum_degrees": 0.0,
                            "throttle_absolute_error_sum": 0.0,
                        },
                    )
                    bucket["samples"] += 1
                    bucket["steering_absolute_error_sum_degrees"] += steering_error
                    bucket["throttle_absolute_error_sum"] += throttle_error

        scenario_metrics = {}
        for name, bucket in scenario.items():
            count = max(1, bucket["samples"])
            scenario_metrics[name] = {
                "samples": bucket["samples"],
                "steering_mae_degrees": (
                    bucket["steering_absolute_error_sum_degrees"] / count
                ),
                "throttle_mae": bucket["throttle_absolute_error_sum"] / count,
            }

        steering_mae = steering_absolute_error / max(1, sample_count)
        throttle_mae = throttle_absolute_error / max(1, sample_count)
        criteria = criteria or EvaluationCriteria()
        checks = {}
        if criteria.maximum_steering_mae_degrees is not None:
            checks["steering_mae"] = (
                steering_mae <= criteria.maximum_steering_mae_degrees
            )
        if criteria.maximum_throttle_mae is not None:
            checks["throttle_mae"] = throttle_mae <= criteria.maximum_throttle_mae

        result = {
            "schema": "autonomy_ai_evaluation_v1",
            "split": split,
            "samples": sample_count,
            "steering_mae_degrees": steering_mae,
            "throttle_mae": throttle_mae,
            "scenario_metrics": scenario_metrics,
            "criteria": {
                "maximum_steering_mae_degrees": criteria.maximum_steering_mae_degrees,
                "maximum_throttle_mae": criteria.maximum_throttle_mae,
            },
            # None means no approval thresholds were supplied. Deliberately do
            # not interpret a training result as permission to drive.
            "criteria_passed": all(checks.values()) if checks else None,
            "checks": checks,
        }
        if output_path:
            os.makedirs(output_path, exist_ok=True)
            with open(
                os.path.join(output_path, "evaluation_metrics.json"),
                "w",
                encoding="utf-8",
            ) as file:
                json.dump(result, file, indent=2)
        dataset.close()
        return result

    @staticmethod
    def _device(torch, requested):
        if requested != "auto":
            return torch.device(requested)
        if torch.cuda.is_available():
            return torch.device("cuda")
        if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            return torch.device("mps")
        return torch.device("cpu")
