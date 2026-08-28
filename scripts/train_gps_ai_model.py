#!/usr/bin/env python3
import argparse
import json
import os

from autonomous_car.ai import GpsTrainer, TrainingConfig


def main():
    parser=argparse.ArgumentParser(description="Train GPS-conditioned AUTO_GPS model")
    parser.add_argument("dataset_path"); parser.add_argument("output_path")
    parser.add_argument("--recordings-root",default=None); parser.add_argument("--epochs",type=int,default=30)
    parser.add_argument("--batch-size",type=int,default=32); parser.add_argument("--learning-rate",type=float,default=1e-3)
    parser.add_argument("--device",default="auto")
    args=parser.parse_args()
    trainer=GpsTrainer(config=TrainingConfig(epochs=max(1,args.epochs),batch_size=max(1,args.batch_size),
                                              learning_rate=max(1e-7,args.learning_rate),device=args.device))
    result=trainer.train(os.path.abspath(args.dataset_path),os.path.abspath(args.output_path),
                         recordings_root_override=os.path.abspath(args.recordings_root) if args.recordings_root else None)
    print(json.dumps(result,indent=2))

if __name__=="__main__": main()
