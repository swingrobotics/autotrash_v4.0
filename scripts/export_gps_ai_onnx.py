#!/usr/bin/env python3
import argparse
import json
import os
from autonomous_car.ai import GpsOnnxExporter

def main():
    p=argparse.ArgumentParser(description="Export GPS-conditioned AUTO_GPS checkpoint to ONNX")
    p.add_argument("checkpoint_path"); p.add_argument("output_path"); p.add_argument("--skip-verify",action="store_true")
    a=p.parse_args(); r=GpsOnnxExporter().export(os.path.abspath(a.checkpoint_path),os.path.abspath(a.output_path),verify=not a.skip_verify)
    print(json.dumps(r,indent=2))
if __name__=="__main__": main()
