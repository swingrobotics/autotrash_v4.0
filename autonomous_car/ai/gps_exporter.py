from dataclasses import asdict
import json
import os

from autonomous_car.routes.gps_route import ROUTE_FEATURE_ORDER

from .exporter import _prepare_unicode_progress_output, _remove_stale_external_data
from .gps_training import (
    GpsDrivingModelSpec,
    create_gps_torch_model,
    require_training_dependencies,
)
from .onnx_parity import verify_onnx_parity


class GpsOnnxExporter:
    """Export GPS-conditioned driving and verify ONNX Runtime parity."""

    def export(
        self,
        checkpoint_path,
        output_path,
        verify=True,
        model_filename="gps_drive_model.onnx",
        parity_absolute_tolerance=1e-4,
        parity_relative_tolerance=1e-4,
    ):
        _, _, torch, _, _, _ = require_training_dependencies()
        os.makedirs(output_path, exist_ok=True)
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        if checkpoint.get("policy_type") != "AUTO_GPS":
            raise ValueError("GPS ONNX exporter requires an AUTO_GPS checkpoint")
        spec = GpsDrivingModelSpec(**checkpoint["model_spec"])
        if spec.route_feature_size != len(ROUTE_FEATURE_ORDER):
            raise ValueError("AUTO_GPS route feature size does not match runtime contract")
        model = create_gps_torch_model(spec)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()

        generator = torch.Generator(device="cpu")
        generator.manual_seed(20260825)
        image = torch.rand(
            (1, 3, spec.image_height, spec.image_width),
            dtype=torch.float32,
            generator=generator,
        )
        lidar = torch.rand((1, 14), dtype=torch.float32, generator=generator)
        auxiliary = torch.tensor([[-0.35, 1.0]], dtype=torch.float32)
        route = torch.linspace(
            -0.75,
            0.75,
            steps=spec.route_feature_size,
            dtype=torch.float32,
        ).reshape(1, spec.route_feature_size)
        path = os.path.join(output_path, model_filename)
        sidecar_path = _remove_stale_external_data(path)

        # Keep GPS AI artifacts self-contained for the same Pi install contract
        # as route-independent AUTO_AI models.
        _prepare_unicode_progress_output()
        torch.onnx.export(
            model,
            (image, lidar, auxiliary, route),
            f=path,
            input_names=["image", "lidar", "auxiliary", "route"],
            output_names=["control"],
            dynamo=True,
            verify=bool(verify),
            external_data=False,
        )
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            raise OSError("AUTO_GPS ONNX export failed")
        if os.path.exists(sidecar_path):
            raise OSError("AUTO_GPS ONNX exporter created unexpected external data sidecar")

        parity = None
        if verify:
            parity = verify_onnx_parity(
                model,
                path,
                {
                    "image": image,
                    "lidar": lidar,
                    "auxiliary": auxiliary,
                    "route": route,
                },
                torch=torch,
                absolute_tolerance=parity_absolute_tolerance,
                relative_tolerance=parity_relative_tolerance,
            )

        manifest = {
            "schema": "autonomy_gps_ai_onnx_manifest_v1",
            "policy_type": "AUTO_GPS",
            "route_id": checkpoint.get("route_id"),
            "model_file": os.path.basename(path),
            "model_spec": asdict(spec),
            "inputs": {
                "image": {
                    "shape": [1, 3, spec.image_height, spec.image_width],
                    "dtype": "float32",
                    "normalization": "RGB / 255.0",
                },
                "lidar": {
                    "shape": [1, 14],
                    "dtype": "float32",
                    "contract": "7 normalized sector distances followed by 7 observed-mask values",
                    "maximum_distance_m": spec.lidar_maximum_distance_m,
                },
                "auxiliary": {
                    "shape": [1, 2],
                    "dtype": "float32",
                    "contract": "normalized IMU yaw rate followed by presence bit",
                    "maximum_abs_yaw_rate_dps": spec.maximum_abs_yaw_rate_dps,
                },
                "route": {
                    "shape": [1, spec.route_feature_size],
                    "dtype": "float32",
                    "feature_order": list(ROUTE_FEATURE_ORDER),
                },
            },
            "output": {
                "control": {
                    "shape": [1, 2],
                    "dtype": "float32",
                    "index_0": "normalized steering",
                    "index_1": "throttle",
                }
            },
            "export": {
                "backend": "torch.onnx.export",
                "dynamo": True,
                "verify": bool(verify),
                "external_data": False,
                "self_contained": True,
                "onnxruntime_parity": parity,
            },
        }
        with open(
            os.path.join(output_path, "model_manifest.json"),
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(manifest, file, indent=2)
        return manifest


__all__ = ["GpsOnnxExporter"]
