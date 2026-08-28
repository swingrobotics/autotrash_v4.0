import json
import os
import tempfile

from autonomous_car.ai import (
    GpsAiRuntime,
    GpsEvaluator,
    GpsOnnxExporter,
    GpsTrainer,
    GpsTrainingPolicy,
    TrainingConfig,
)


SECTORS=("far_right","right","front_right","front","front_left","left","far_left")


def main():
    import cv2
    import numpy as np
    with tempfile.TemporaryDirectory() as root:
        recordings=os.path.join(root,"recordings"); dataset=os.path.join(root,"dataset"); output=os.path.join(root,"train")
        os.makedirs(recordings); os.makedirs(dataset)
        samples=[]; splits=["train","train","validation","validation","test","test"]
        groups=["straight","gentle","sharp","gentle","straight","sharp"]
        transitions=[False,True,True,True,False,True]
        for i,split in enumerate(splits):
            image=np.full((90,160,3),40+i*20,dtype=np.uint8)
            path=os.path.join(recordings,f"frame_{i}.jpg"); cv2.imwrite(path,image)
            route=[0.0,0.0,0.1,0.25,0.1,0.65,0.8,i/10.0]
            scenario={"straight":"straight","gentle":"gentle_left","sharp":"sharp_left"}[groups[i]]
            previous=[None]*max(0,5-i)+[float(value-2) for value in range(max(0,i-5),i)]
            previous=previous[-5:]
            yaw=[None]*max(0,5-(i+1))+[float(value+1) for value in range(max(0,i-4),i+1)]
            yaw=yaw[-5:]
            samples.append({"schema":"autonomy_gps_ai_sample_v1","session":f"run_{i//2}","split":split,
                "camera":{"saved_frame_path":f"frame_{i}.jpg","video_path":"unused.mp4","video_frame_index":0},
                "learned_features":{"lidar":{"distances_m":{name:2.0 for name in SECTORS},"observed":{name:True for name in SECTORS}},
                                    "imu_yaw_rate_dps":1.0,"route":{"normalized":route},
                                    "temporal":{"history_steps":5,"yaw_rate_history_dps":yaw,
                                                "previous_steering_history_degrees":previous,
                                                "current_steering_excluded":True}},
                "labels":{"steering_degrees":float(i-2),"throttle":0.15},"scenario":scenario,
                "training_context":{"steering_group":groups[i],"steering_transition":transitions[i],
                                    "route_recovery": i==2}})
        with open(os.path.join(dataset,"samples.jsonl"),"w",encoding="utf-8") as file:
            for sample in samples: file.write(json.dumps(sample)+"\n")
        with open(os.path.join(dataset,"dataset.json"),"w",encoding="utf-8") as file:
            json.dump({"schema":"autonomy_gps_ai_dataset_v1","policy_type":"AUTO_GPS","recordings_root":recordings,
                       "sample_manifest":"samples.jsonl","route":{"route_id":"smoke_route"}},file)

        # A deliberately huge min_delta makes the second validation epoch count
        # as stale. Patience=1 verifies best-checkpoint restore and early stop.
        policy=GpsTrainingPolicy(early_stopping_patience=1,early_stopping_min_delta=1e9)
        trainer=GpsTrainer(config=TrainingConfig(epochs=5,batch_size=2,num_workers=0,device="cpu"),policy=policy)
        assert trainer._weight(samples[0])==1.0
        assert trainer._weight(samples[1])==2.0
        assert trainer._weight(samples[2])==3.0  # recovery outranks sharp/transition
        metrics=trainer.train(dataset,output)
        assert metrics["route_id"]=="smoke_route"
        assert metrics["curve_aware_steering_loss"]["loss"]=="weighted_mse"
        assert metrics["temporal_auxiliary"]["history_steps"]==5
        assert metrics["temporal_auxiliary"]["feature_size"]==20
        assert metrics["early_stopping"]["stopped_early"] is True
        assert metrics["epochs_completed"]==2
        assert metrics["best_epoch"]==1
        assert "sharp" in metrics["best_validation"]["steering_group_metrics"]
        assert metrics["best_validation"]["transition_metrics"]["samples"]>=1
        assert metrics["best_validation"]["recovery_metrics"]["samples"]==1

        evaluation=GpsEvaluator().evaluate(dataset,os.path.join(output,"checkpoint.pt"),split="test",device="cpu")
        assert evaluation["samples"]==2
        assert set(evaluation["steering_group_metrics"])=={"straight","sharp"}
        assert evaluation["transition_metrics"]["samples"]==1
        export=os.path.join(root,"export"); manifest=GpsOnnxExporter().export(os.path.join(output,"checkpoint.pt"),export,verify=True)
        assert manifest["policy_type"]=="AUTO_GPS" and manifest["route_id"]=="smoke_route"
        assert manifest["inputs"]["auxiliary"]["shape"]==[1,20]
        assert manifest["inputs"]["auxiliary"]["history_steps"]==5
        runtime=GpsAiRuntime(os.path.join(export,"gps_drive_model.onnx"),os.path.join(export,"model_manifest.json"))
        assert runtime.temporal_auxiliary and runtime.auxiliary_feature_size==20
        ok,jpeg=cv2.imencode(".jpg",np.full((90,160,3),100,dtype=np.uint8)); assert ok
        points=[{"bearing_degrees":0.0,"distance_mm":2000,"confidence":100}]
        result=runtime.infer_jpeg(jpeg.tobytes(),points,0.0,{"normalized":[0.0,0.0,0.1,0.25,0.1,0.65,0.8,0.1]})
        assert -20.0<=result.steering_degrees<=20.0 and -1.0<=result.throttle<=1.0
        assert runtime.snapshot()["temporal_history_filled"]["yaw"]==1
        assert runtime.snapshot()["temporal_history_filled"]["steering"]==1
        stopped=runtime.infer_jpeg(jpeg.tobytes(),points,0.0,{"normalized":[0.0]*8},person_hazard=True)
        assert stopped.person_stop and stopped.throttle==0.0
        assert runtime.snapshot()["temporal_history_filled"]=={"yaw":0,"steering":0}
    print("GPS temporal curve/recovery train/evaluate/ONNX/runtime smoke: PASS")

if __name__=="__main__": main()
