#!/usr/bin/env python3
import argparse
import json
import os
from autonomous_car.ai import GpsEvaluator

def main():
    p=argparse.ArgumentParser(description="Evaluate held-out GPS-conditioned AUTO_GPS model")
    p.add_argument("dataset_path"); p.add_argument("checkpoint_path"); p.add_argument("--recordings-root",default=None)
    p.add_argument("--split",default="test"); p.add_argument("--output-path",default=None)
    p.add_argument("--max-steering-mae-deg",type=float,default=None); p.add_argument("--max-throttle-mae",type=float,default=None)
    p.add_argument("--device",default="auto"); a=p.parse_args()
    r=GpsEvaluator().evaluate(os.path.abspath(a.dataset_path),os.path.abspath(a.checkpoint_path),split=a.split,
        recordings_root_override=os.path.abspath(a.recordings_root) if a.recordings_root else None,
        output_path=os.path.abspath(a.output_path) if a.output_path else None,
        maximum_steering_mae_degrees=a.max_steering_mae_deg,maximum_throttle_mae=a.max_throttle_mae,device=a.device)
    print(json.dumps(r,indent=2))
if __name__=="__main__": main()
