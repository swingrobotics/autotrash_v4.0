#!/usr/bin/env python3
import argparse
import json
import os
import shutil
from autonomous_car.ai import ModelRegistry


def _load(path):
    if not path: return {}
    with open(path,"r",encoding="utf-8") as file:
        value=json.load(file)
    return value if isinstance(value,dict) else {}


def main():
    p=argparse.ArgumentParser(description="Install a route-bound AUTO_GPS ONNX model")
    p.add_argument("model_id"); p.add_argument("onnx_path"); p.add_argument("--manifest",required=True)
    p.add_argument("--evaluation",default=None); p.add_argument("--models-root",default="/home/gnss/camera-stream/models")
    p.add_argument("--stage",choices=["TRAINED","OFFLINE_VALIDATED","CLOSED_AREA_VALIDATED","AUTO_ALLOWED"],default="TRAINED")
    a=p.parse_args(); manifest=_load(a.manifest)
    if manifest.get("policy_type")!="AUTO_GPS" or not manifest.get("route_id"):
        raise ValueError("GPS model manifest must contain policy_type=AUTO_GPS and route_id")
    root=os.path.abspath(a.models_root); os.makedirs(root,exist_ok=True)
    safe=ModelRegistry._normalize_id(a.model_id); model_file=f"{safe}.onnx"; manifest_file=f"{safe}.manifest.json"
    shutil.copy2(os.path.abspath(a.onnx_path),os.path.join(root,model_file)); shutil.copy2(os.path.abspath(a.manifest),os.path.join(root,manifest_file))
    evaluation=_load(a.evaluation)
    metadata={"policy_type":"AUTO_GPS","route_id":manifest["route_id"],"manifest_file":manifest_file,
              "input":manifest.get("inputs",{}),"output":manifest.get("output",{}),"metrics":evaluation}
    result=ModelRegistry(root).register(safe,model_file,metadata=metadata,validation_stage=a.stage,policy_type="AUTO_GPS")
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=="__main__": main()
