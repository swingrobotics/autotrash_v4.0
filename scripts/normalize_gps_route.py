#!/usr/bin/env python3
import argparse
import json
import os

from autonomous_car.routes import GpsRouteNormalizer


def main():
    parser=argparse.ArgumentParser(description="Fuse multiple GPS-ON RECORD sessions into one normalized RTK route")
    parser.add_argument("route_id")
    parser.add_argument("sessions", nargs="+")
    parser.add_argument("--recordings-root", default="/home/gnss/camera-stream/recordings")
    parser.add_argument("--routes-root", default="/home/gnss/camera-stream/gps-routes")
    parser.add_argument("--spacing-m", type=float, default=0.20)
    args=parser.parse_args()
    os.makedirs(args.routes_root,exist_ok=True)
    output=os.path.join(os.path.abspath(args.routes_root),f"{args.route_id}.json")
    route=GpsRouteNormalizer(spacing_m=args.spacing_m).build(
        os.path.abspath(args.recordings_root),args.sessions,args.route_id,output_path=output)
    print(json.dumps(route.as_dict(),ensure_ascii=False,indent=2))

if __name__=="__main__": main()
