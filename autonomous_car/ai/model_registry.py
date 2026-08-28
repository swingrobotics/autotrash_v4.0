import json
import os
import re
import time
from pathlib import Path


class ModelRegistryError(ValueError):
    pass


MODEL_LIFECYCLE = (
    "TRAINED",
    "OFFLINE_VALIDATED",
    "CLOSED_AREA_VALIDATED",
    "AUTO_ALLOWED",
)
MODEL_POLICY_TYPES = ("AUTO_AI", "AUTO_GPS")


class ModelRegistry:
    """Metadata registry for learned driving models with explicit policy type."""

    def __init__(self, root_path):
        self.root_path = Path(root_path)

    def list_models(self, policy_type=None):
        expected = self._normalize_policy_type(policy_type) if policy_type else None
        if not self.root_path.exists():
            return []
        models = []
        for path in sorted(self.root_path.glob("*.json")):
            try:
                document = self._read_json(path)
            except (OSError, json.JSONDecodeError):
                continue
            # Selection/status JSON files may also contain model_id. A registry
            # entry is a model only when it carries an installed model_file.
            if "model_id" not in document or not document.get("model_file"):
                continue
            document.setdefault("policy_type", "AUTO_AI")
            if expected and self._normalize_policy_type(document.get("policy_type")) != expected:
                continue
            models.append(document)
        return models

    def register(self, model_id, model_file, *, metadata=None, validated=False,
                 auto_allowed=False, environments=None, validation_stage=None,
                 policy_type=None):
        model_id = self._normalize_id(model_id)
        model_file = os.path.basename(str(model_file or ""))
        if not model_file:
            raise ModelRegistryError("Model filename is required")
        metadata = dict(metadata or {})
        policy = self._normalize_policy_type(policy_type or metadata.get("policy_type") or "AUTO_AI")
        stage = self._normalize_stage(
            validation_stage or (
                "AUTO_ALLOWED" if auto_allowed and validated
                else "CLOSED_AREA_VALIDATED" if validated
                else "TRAINED"
            )
        )
        self.root_path.mkdir(parents=True, exist_ok=True)
        now = time.time()
        document = {
            "model_id": model_id,
            "model_file": model_file,
            "policy_type": policy,
            "created_at": now,
            "updated_at": now,
            "validation_stage": stage,
            "validated": self._vehicle_validated(stage),
            "auto_allowed": stage == "AUTO_ALLOWED",
            "environments": self._normalize_environments(environments),
            "training": {}, "input": {}, "output": {}, "metrics": {},
        }
        document.update(metadata)
        document["model_id"] = model_id
        document["model_file"] = model_file
        document["policy_type"] = self._normalize_policy_type(document.get("policy_type", policy))
        document["validation_stage"] = self._normalize_stage(document.get("validation_stage", stage))
        document["validated"] = self._vehicle_validated(document["validation_stage"])
        document["auto_allowed"] = document["validation_stage"] == "AUTO_ALLOWED"
        document["environments"] = self._normalize_environments(document.get("environments"))
        document["updated_at"] = now
        self._write_json(self._path(model_id), document)
        return document

    def get(self, model_id):
        path = self._path(model_id)
        if not path.exists():
            raise ModelRegistryError(f"Unknown model: {model_id}")
        document = self._read_json(path)
        if not document.get("model_file"):
            raise ModelRegistryError(f"Registry entry is not an installed model: {model_id}")
        document.setdefault("policy_type", "AUTO_AI")
        return document

    def update_lifecycle(self, model_id, stage, *, metrics=None):
        document = self.get(model_id)
        stage = self._normalize_stage(stage)
        current = self._normalize_stage(document.get("validation_stage") or "TRAINED")
        current_index = MODEL_LIFECYCLE.index(current)
        target_index = MODEL_LIFECYCLE.index(stage)
        # Promotion is evidence-bearing: do not let an API/UI call jump over a
        # validation gate. Demotion is always allowed because it only removes
        # permission. Registration may still set an initial verified stage when
        # importing an externally validated model and its evidence.
        if target_index > current_index + 1:
            expected = MODEL_LIFECYCLE[current_index + 1]
            raise ModelRegistryError(
                f"Model lifecycle promotion must be sequential: {current} -> {expected} before {stage}"
            )
        document["validation_stage"] = stage
        document["validated"] = self._vehicle_validated(stage)
        document["auto_allowed"] = stage == "AUTO_ALLOWED"
        if metrics is not None:
            document["metrics"] = dict(metrics)
        document["updated_at"] = time.time()
        self._write_json(self._path(model_id), document)
        return document

    def update_validation(self, model_id, *, validated, auto_allowed=None, metrics=None):
        if not validated:
            stage = "TRAINED"
        elif auto_allowed:
            stage = "AUTO_ALLOWED"
        else:
            stage = "CLOSED_AREA_VALIDATED"
        return self.update_lifecycle(model_id, stage, metrics=metrics)

    def compatible_for_auto(self, environment_tags, policy_type="AUTO_AI"):
        required = set(self._normalize_environments(environment_tags))
        if not required:
            return []
        policy = self._normalize_policy_type(policy_type)
        compatible = []
        for model in self.list_models(policy):
            if model.get("validation_stage") != "AUTO_ALLOWED":
                continue
            if not model.get("validated") or not model.get("auto_allowed"):
                continue
            supported = set(self._normalize_environments(model.get("environments")))
            if required.issubset(supported):
                compatible.append(model)
        return compatible

    def compatible_for_gps_route(self, route_id, *, auto_only=False):
        route_id = str(route_id or "").strip()
        if not route_id:
            return []
        result = []
        for model in self.list_models("AUTO_GPS"):
            if str(model.get("route_id") or "") != route_id:
                continue
            if auto_only and model.get("validation_stage") != "AUTO_ALLOWED":
                continue
            if not auto_only and model.get("validation_stage") not in {"CLOSED_AREA_VALIDATED", "AUTO_ALLOWED"}:
                continue
            result.append(model)
        return result

    def _path(self, model_id):
        return self.root_path / f"{self._normalize_id(model_id)}.json"

    @staticmethod
    def _normalize_id(value):
        value = re.sub(r"[^a-zA-Z0-9._-]+", "_", str(value or "").strip())
        value = value.strip("._-").lower()
        if not value:
            raise ModelRegistryError("Model ID is required")
        return value

    @staticmethod
    def _normalize_stage(value):
        stage = str(value or "").strip().upper()
        if stage not in MODEL_LIFECYCLE:
            raise ModelRegistryError(f"Unknown model lifecycle stage: {value}; expected one of {MODEL_LIFECYCLE}")
        return stage

    @staticmethod
    def _normalize_policy_type(value):
        policy = str(value or "").strip().upper()
        if policy not in MODEL_POLICY_TYPES:
            raise ModelRegistryError(f"Unknown model policy type: {value}; expected one of {MODEL_POLICY_TYPES}")
        return policy

    @staticmethod
    def _normalize_environments(values):
        if isinstance(values, str):
            values = [values]
        return sorted({str(value).strip().lower() for value in (values or []) if str(value).strip()})

    @staticmethod
    def _vehicle_validated(stage):
        return stage in {"CLOSED_AREA_VALIDATED", "AUTO_ALLOWED"}

    @staticmethod
    def _read_json(path):
        with open(path, "r", encoding="utf-8") as file:
            return json.load(file)

    @staticmethod
    def _fsync_parent(path):
        parent = os.path.dirname(os.path.abspath(str(path))) or "."
        try:
            descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        except OSError:
            return
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @classmethod
    def _write_json(cls, path, document):
        temporary = Path(str(path) + ".tmp")
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        cls._fsync_parent(path)
