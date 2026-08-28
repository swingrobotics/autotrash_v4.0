import json
import tempfile

from autonomous_car.ai import ModelRegistry


def validate():
    with tempfile.TemporaryDirectory() as directory:
        registry = ModelRegistry(directory)
        model = registry.register(
            "warehouse_v1",
            "warehouse_v1.onnx",
            validation_stage="AUTO_ALLOWED",
            environments=["Indoor", "Warehouse"],
        )
        assert model["environments"] == ["indoor", "warehouse"]
        assert registry.compatible_for_auto([]) == []
        assert registry.compatible_for_auto(None) == []
        assert len(registry.compatible_for_auto(["indoor"])) == 1
        assert len(registry.compatible_for_auto(["INDOOR", "WAREHOUSE"])) == 1
        assert registry.compatible_for_auto(["outdoor"]) == []
        return {
            "empty_environment_refuses_ai_fallback": "PASS",
            "case_normalized_environment_match": "PASS",
            "mismatch_refused": "PASS",
        }


def main():
    result = validate()
    print("AUTO_AI environment selection guard: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
