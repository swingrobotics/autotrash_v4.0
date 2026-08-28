#!/usr/bin/env python3

import argparse
import json
import os
import shutil

from autonomous_car.ai import ModelRegistry


def _load_json(path):
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as file:
        value = json.load(file)
    return value if isinstance(value, dict) else {}


def main():
    parser = argparse.ArgumentParser(description="Install an exported AUTO_AI ONNX model into the vehicle model registry")
    parser.add_argument("model_id"); parser.add_argument("onnx_path")
    parser.add_argument("--manifest", default=None); parser.add_argument("--evaluation", default=None)
    parser.add_argument("--models-root", default="/home/gnss/camera-stream/models")
    parser.add_argument("--environment", action="append", default=[])
    parser.add_argument("--stage", choices=["TRAINED","OFFLINE_VALIDATED","CLOSED_AREA_VALIDATED","AUTO_ALLOWED"], default="TRAINED")
    args = parser.parse_args()
    source_model = os.path.abspath(args.onnx_path)
    if not os.path.isfile(source_model): raise FileNotFoundError(source_model)
    models_root = os.path.abspath(args.models_root); os.makedirs(models_root, exist_ok=True)
    safe_id = ModelRegistry._normalize_id(args.model_id)
    installed_model=f"{safe_id}.onnx"; installed_manifest=f"{safe_id}.manifest.json"
    shutil.copy2(source_model, os.path.join(models_root, installed_model))
    manifest=_load_json(args.manifest)
    if manifest.get("policy_type") == "AUTO_GPS":
        raise ValueError("Use register_gps_ai_model.py for AUTO_GPS models")
    if args.manifest: shutil.copy2(os.path.abspath(args.manifest), os.path.join(models_root, installed_manifest))
    evaluation=_load_json(args.evaluation)
    metadata={"policy_type":"AUTO_AI","manifest_file":installed_manifest if args.manifest else None,
              "training":manifest.get("training",{}),"input":manifest.get("inputs",{}),"output":manifest.get("output",{}),"metrics":evaluation}
    result=ModelRegistry(models_root).register(safe_id,installed_model,metadata=metadata,environments=args.environment,
                                               validation_stage=args.stage,policy_type="AUTO_AI")
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__ == "__main__": main()
