import csv
import json
import os
import tempfile

from autonomous_car.ai import ModelRegistry
from autonomous_car.mode_policy import policy_for
from autonomous_car.routes import GpsRouteFeatureExtractor, GpsRouteNormalizer
from autonomous_car.state import DriveMode


def _session(root,name,offset_lon=0.0,reverse=False):
    path=os.path.join(root,name); os.makedirs(path)
    with open(os.path.join(path,"metadata.json"),"w",encoding="utf-8") as file:
        json.dump({"purpose":"RECORD","record_gps":True},file)
    rows=[]
    for i in range(35):
        rows.append({"monotonic":i*0.1,"latitude":37.0,"longitude":127.0+offset_lon+i*0.000001,
                     "altitude_m":10.0,"speed_mps":0.3,"rtk_status":"RTK FIXED"})
    if reverse: rows=list(reversed(rows))
    with open(os.path.join(path,"gnss.csv"),"w",encoding="utf-8",newline="") as file:
        writer=csv.DictWriter(file,fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)


def main():
    with tempfile.TemporaryDirectory() as d:
        recordings=os.path.join(d,"recordings"); os.makedirs(recordings)
        _session(recordings,"run_a",0.0); _session(recordings,"run_b",0.0000002); _session(recordings,"run_c",-0.0000002,True)
        route=GpsRouteNormalizer(minimum_fixed_samples=10).build(recordings,["run_a","run_b","run_c"],"test_route")
        assert len(route.points)>5
        assert route.quality["source_run_count"]==3
        assert "run_c" in route.quality["reversed_sessions"]
        extractor=GpsRouteFeatureExtractor(route)
        feature=extractor.extract(37.0,127.0,90.0)
        assert abs(feature.cross_track_error_m)<0.5
        assert abs(feature.heading_error_degrees)<20.0
        assert len(feature.normalized)==8

        registry=ModelRegistry(os.path.join(d,"models"))
        registry.register("plain","plain.onnx",metadata={"policy_type":"AUTO_AI"},validated=True,auto_allowed=True,environments=["outdoor"])
        registry.register("gps","gps.onnx",metadata={"policy_type":"AUTO_GPS","route_id":"test_route"},validated=True,auto_allowed=True)
        assert [m["model_id"] for m in registry.compatible_for_auto(["outdoor"])]==["plain"]
        assert [m["model_id"] for m in registry.compatible_for_gps_route("test_route",auto_only=True)]==["gps"]

        policy=policy_for(DriveMode.AUTO_GPS)
        assert policy.learned_driving and policy.gps_navigation and policy.person_stop and policy.require_lidar
        assert not policy.local_avoidance and not policy.obstacle_stop_fallback and not policy.lane_assist
    print("GPS-conditioned AUTO_GPS route validation: PASS")

if __name__=="__main__": main()
