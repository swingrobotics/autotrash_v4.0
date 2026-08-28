from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
import sys

from .onnx_parity import verify_onnx_parity
from .training import DrivingModelSpec, create_torch_model, require_training_dependencies


@dataclass(frozen=True)
class OnnxExportConfig:
    verify: bool = True
    model_filename: str = "drive_model.onnx"
    parity_absolute_tolerance: float = 1e-4
    parity_relative_tolerance: float = 1e-4


def _prepare_unicode_progress_output():
    """Prevent exporter progress glyphs from failing on cp949 Windows hosts."""

    os.environ.setdefault("PYTHONUTF8", "1")
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            try:
                reconfigure(encoding="utf-8", errors="replace")
            except (OSError, ValueError):
                pass


def _remove_stale_external_data(model_path):
    sidecar = model_path + ".data"
    if os.path.isfile(sidecar):
        os.remove(sidecar)
    return sidecar


class OnnxExporter:
    """Export a trained checkpoint and verify the vehicle runtime boundary."""

    def export(self, checkpoint_path, output_path, config=None):
        config = config or OnnxExportConfig()
        _, _, torch, _, _, _ = require_training_dependencies()
        os.makedirs(output_path, exist_ok=True)
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        model_spec = DrivingModelSpec(**checkpoint["model_spec"])
        model = create_torch_model(model_spec)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()

        # Non-zero deterministic probes exercise all input branches. Zero-only
        # verification can miss preprocessing/feature wiring mistakes that still
        # happen to map both runtimes to the same bias output.
        generator = torch.Generator(device="cpu")
        generator.manual_seed(20260825)
        image = torch.rand(
            (1, 3, model_spec.image_height, model_spec.image_width),
            dtype=torch.float32,
            generator=generator,
        )
        lidar = torch.rand((1, 14), dtype=torch.float32, generator=generator)
        auxiliary = torch.tensor([[0.25, 1.0]], dtype=torch.float32)
        model_path = os.path.join(output_path, config.model_filename)
        sidecar_path = _remove_stale_external_data(model_path)

        # Vehicle installation transfers one ONNX artifact plus its manifest.
        # Keep all weights inside the ONNX file so the Pi never depends on a
        # separate <model>.onnx.data sidecar that can be omitted during install.
        # torch.onnx's dynamo exporter also prints Unicode status symbols. On
        # Windows systems using the Korean cp949 console codec, printing those
        # symbols can otherwise raise UnicodeEncodeError after training succeeds.
        _prepare_unicode_progress_output()
        torch.onnx.export(
            model,
            (image, lidar, auxiliary),
            f=model_path,
            input_names=["image", "lidar", "auxiliary"],
            output_names=["control"],
            dynamo=True,
            verify=bool(config.verify),
            external_data=False,
        )
        if not os.path.isfile(model_path) or os.path.getsize(model_path) == 0:
            raise OSError("ONNX exporter did not create a model file")
        if os.path.exists(sidecar_path):
            raise OSError("ONNX exporter created unexpected external data sidecar")

        parity = None
        if config.verify:
            parity = verify_onnx_parity(
                model,
                model_path,
                {
                    "image": image,
                    "lidar": lidar,
                    "auxiliary": auxiliary,
                },
                torch=torch,
                absolute_tolerance=config.parity_absolute_tolerance,
                relative_tolerance=config.parity_relative_tolerance,
            )

        manifest = {
            "schema": "autonomy_ai_onnx_manifest_v1",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "model_file": os.path.basename(model_path),
            "model_spec": asdict(model_spec),
            "inputs": {
                "image": {
                    "shape": [1, 3, model_spec.image_height, model_spec.image_width],
                    "dtype": "float32",
                    "normalization": "RGB / 255.0",
                },
                "lidar": {
                    "shape": [1, 14],
                    "dtype": "float32",
                    "contract": "7 normalized sector distances followed by 7 observed-mask values",
                    "maximum_distance_m": model_spec.lidar_maximum_distance_m,
                },
                "auxiliary": {
                    "shape": [1, 2],
                    "dtype": "float32",
                    "contract": "normalized IMU yaw rate followed by presence bit",
                    "maximum_abs_yaw_rate_dps": model_spec.maximum_abs_yaw_rate_dps,
                },
            },
            "output": {
                "control": {
                    "shape": [1, 2],
                    "dtype": "float32",
                    "index_0": "steering normalized to [-1,1]; multiply by maximum_steering_degrees",
                    "index_1": "throttle normalized to [-1,1]",
                }
            },
            "export": {
                "backend": "torch.onnx.export",
                "dynamo": True,
                "verify": bool(config.verify),
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
