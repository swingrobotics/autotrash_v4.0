#!/usr/bin/env python3
import argparse
import json
import os

from autonomous_car.ai import GpsDatasetBuilder


def main():
    parser=argparse.ArgumentParser(description="Build AUTO_GPS training dataset from GPS-ON RECORD sessions")
    parser.add_argument("route_path")
    parser.add_argument("sessions",nargs="+")
    parser.add_argument("--recordings-root",default="/home/gnss/camera-stream/recordings")
    parser.add_argument("--output-root",default="/home/gnss/camera-stream/datasets")
    parser.add_argument("--dataset-id",default=None)
    args=parser.parse_args()
    result=GpsDatasetBuilder(os.path.abspath(args.recordings_root),os.path.abspath(args.output_root),
                             os.path.abspath(args.route_path)).build(args.sessions,args.dataset_id)
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=="__main__": main()
