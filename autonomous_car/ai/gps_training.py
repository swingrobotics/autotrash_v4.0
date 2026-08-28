from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os

from .training import ManifestDataset, TrainingConfig, require_training_dependencies
from autonomous_car.routes.gps_route import ROUTE_FEATURE_ORDER


@dataclass(frozen=True)
class GpsDrivingModelSpec:
    image_width: int = 160
    image_height: int = 90
    maximum_steering_degrees: float = 20.0
    maximum_abs_yaw_rate_dps: float = 90.0
    lidar_maximum_distance_m: float = 8.0
    route_feature_size: int = len(ROUTE_FEATURE_ORDER)
    output_size: int = 2


@dataclass(frozen=True)
class GpsTrainingPolicy:
    """GPS-specific optimization priorities layered on top of TrainingConfig.

    Continuous steering remains an MSE regression target, matching the existing
    trainer and common end-to-end driving practice. The per-sample weight makes
    mistakes on under-represented curves and curve-entry/exit context matter
    more without changing the model output contract.
    """

    straight_steering_loss_weight: float = 1.0
    gentle_steering_loss_weight: float = 1.5
    sharp_steering_loss_weight: float = 2.5
    transition_steering_loss_weight: float = 2.0
    maximum_steering_loss_weight: float = 3.0
    early_stopping_patience: int = 5
    early_stopping_min_delta: float = 5e-4


def create_gps_torch_model(spec=None):
    _, _, torch, nn, _, _ = require_training_dependencies()
    spec = spec or GpsDrivingModelSpec()

    class GpsDrivingNetwork(nn.Module):
        def __init__(self):
            super().__init__()
            self.image_encoder = nn.Sequential(
                nn.Conv2d(3,16,5,2,2), nn.ReLU(inplace=True),
                nn.Conv2d(16,24,5,2,2), nn.ReLU(inplace=True),
                nn.Conv2d(24,32,3,2,1), nn.ReLU(inplace=True),
                nn.Conv2d(32,48,3,2,1), nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((3,5)), nn.Flatten(),
                nn.Linear(48*3*5,128), nn.ReLU(inplace=True),
            )
            self.lidar_encoder = nn.Sequential(
                nn.Linear(14,32), nn.ReLU(inplace=True),
                nn.Linear(32,32), nn.ReLU(inplace=True),
            )
            self.aux_encoder = nn.Sequential(nn.Linear(2,16), nn.ReLU(inplace=True))
            self.route_encoder = nn.Sequential(
                nn.Linear(spec.route_feature_size,32), nn.ReLU(inplace=True),
                nn.Linear(32,32), nn.ReLU(inplace=True),
            )
            self.control_head = nn.Sequential(
                nn.Linear(128+32+16+32,128), nn.ReLU(inplace=True),
                nn.Dropout(0.10), nn.Linear(128,64), nn.ReLU(inplace=True),
                nn.Linear(64,spec.output_size), nn.Tanh(),
            )

        def forward(self, image, lidar, auxiliary, route):
            return self.control_head(torch.cat((
                self.image_encoder(image),
                self.lidar_encoder(lidar),
                self.aux_encoder(auxiliary),
                self.route_encoder(route),
            ), dim=1))
    return GpsDrivingNetwork()


def GpsManifestDataset(dataset_path, split, model_spec=None, recordings_root_override=None):
    _, np, torch, _, _, Dataset = require_training_dependencies()
    spec = model_spec or GpsDrivingModelSpec()
    base = ManifestDataset(dataset_path, split, spec, recordings_root_override)
    with open(os.path.join(os.path.abspath(dataset_path), "dataset.json"), "r", encoding="utf-8") as file:
        document = json.load(file)
    if document.get("policy_type") != "AUTO_GPS":
        base.close()
        raise ValueError("GPS trainer requires an AUTO_GPS dataset")

    class _GpsDataset(Dataset):
        def __init__(self):
            self.base = base
            self.samples = base.samples
        def __len__(self):
            return len(self.base)
        def __getitem__(self, index):
            image, lidar, auxiliary, target, source_index = self.base[index]
            route = self.samples[index]["learned_features"]["route"]["normalized"]
            route = np.asarray(route, dtype=np.float32)
            if route.shape != (spec.route_feature_size,):
                raise ValueError(f"Route feature shape mismatch: {route.shape}")
            return image, lidar, auxiliary, torch.from_numpy(route), target, source_index
        def scenario_counts(self):
            return self.base.scenario_counts()
        def close(self):
            self.base.close()
    return _GpsDataset()


class GpsTrainer:
    def __init__(self, model_spec=None, config=None, policy=None):
        self.model_spec = model_spec or GpsDrivingModelSpec()
        self.config = config or TrainingConfig()
        self.policy = policy or GpsTrainingPolicy()

    @staticmethod
    def _steering_group(sample):
        context = sample.get("training_context") or {}
        group = str(context.get("steering_group") or "").strip().lower()
        if group in {"straight", "gentle", "sharp"}:
            return group
        scenario = str(sample.get("scenario") or "").lower()
        if "sharp" in scenario:
            return "sharp"
        if "gentle" in scenario:
            return "gentle"
        return "straight"

    @staticmethod
    def _is_transition(sample):
        return bool((sample.get("training_context") or {}).get("steering_transition"))

    def _sample_steering_weight(self, sample):
        group = self._steering_group(sample)
        if group == "sharp":
            weight = float(self.policy.sharp_steering_loss_weight)
        elif group == "gentle":
            weight = float(self.policy.gentle_steering_loss_weight)
        else:
            weight = float(self.policy.straight_steering_loss_weight)
        if self._is_transition(sample):
            weight = max(weight, float(self.policy.transition_steering_loss_weight))
        return min(
            max(1e-6, float(self.policy.maximum_steering_loss_weight)),
            max(1e-6, weight),
        )

    def _batch_steering_weights(self, torch, dataset, indices, device):
        values = [
            self._sample_steering_weight(dataset.samples[int(index)])
            for index in indices.detach().cpu().tolist()
        ]
        return torch.tensor(values, dtype=torch.float32, device=device)

    def _weighted_steering_mse(self, prediction, target, weights, torch):
        errors = torch.square(prediction - target)
        denominator = torch.clamp(weights.sum(), min=1e-6)
        return (errors * weights).sum() / denominator

    def train(self, dataset_path, output_path, recordings_root_override=None):
        _, _, torch, _, DataLoader, _ = require_training_dependencies()
        os.makedirs(output_path, exist_ok=True)
        torch.manual_seed(self.config.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self.config.seed)
        device = self._device(torch)
        train_dataset = GpsManifestDataset(dataset_path, "train", self.model_spec, recordings_root_override)
        val_dataset = GpsManifestDataset(dataset_path, "validation", self.model_spec, recordings_root_override)
        if len(train_dataset) == 0:
            train_dataset.close(); val_dataset.close()
            raise ValueError("Training split contains no samples")
        counts = train_dataset.scenario_counts()
        sampler = self._scenario_sampler(torch, train_dataset, counts)
        train_loader = DataLoader(train_dataset, batch_size=self.config.batch_size,
                                  shuffle=sampler is None, sampler=sampler,
                                  num_workers=self.config.num_workers)
        val_loader = DataLoader(val_dataset, batch_size=self.config.batch_size,
                                shuffle=False, num_workers=self.config.num_workers) if len(val_dataset) else None
        model = create_gps_torch_model(self.model_spec).to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=self.config.learning_rate,
                                      weight_decay=self.config.weight_decay)
        history=[]; best=math.inf; best_state=None; best_epoch=None; best_validation=None
        stale_epochs=0; stopped_early=False
        patience=max(0,int(self.policy.early_stopping_patience))
        minimum_delta=max(0.0,float(self.policy.early_stopping_min_delta))

        for epoch in range(1,self.config.epochs+1):
            model.train(); total=0.0; batches=0
            for image,lidar,auxiliary,route,target,indices in train_loader:
                image,lidar,auxiliary,route,target = [x.to(device) for x in (image,lidar,auxiliary,route,target)]
                weights=self._batch_steering_weights(torch,train_dataset,indices,device)
                optimizer.zero_grad(set_to_none=True)
                pred=model(image,lidar,auxiliary,route)
                steer=self._weighted_steering_mse(pred[:,0],target[:,0],weights,torch)
                throttle=torch.square(pred[:,1]-target[:,1]).mean()
                loss=self.config.steering_loss_weight*steer+self.config.throttle_loss_weight*throttle
                loss.backward(); optimizer.step()
                total += float(loss.detach().cpu()); batches += 1

            validation=None
            if val_loader is not None:
                validation=self._validation_metrics(model,val_loader,val_dataset,device,torch)
                val_loss=float(validation["loss"])
                if val_loss < best - minimum_delta:
                    best=val_loss; best_epoch=epoch; stale_epochs=0
                    best_validation=validation
                    best_state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
                else:
                    stale_epochs += 1

            history.append({
                "epoch":epoch,
                "train_loss":total/max(1,batches),
                "validation_loss":None if validation is None else validation["loss"],
                "validation":validation,
            })
            if val_loader is not None and patience and stale_epochs >= patience:
                stopped_early=True
                break

        if best_state is not None:
            model.load_state_dict(best_state)
        with open(os.path.join(dataset_path,"dataset.json"),"r",encoding="utf-8") as file:
            dataset_document=json.load(file)
        checkpoint=os.path.join(output_path,"checkpoint.pt")
        torch.save({
            "schema":"autonomy_gps_ai_checkpoint_v1",
            "policy_type":"AUTO_GPS",
            "route_id":(dataset_document.get("route") or {}).get("route_id"),
            "model_spec":asdict(self.model_spec),
            "training_config":asdict(self.config),
            "gps_training_policy":asdict(self.policy),
            "best_epoch":best_epoch,
            "model_state_dict":model.state_dict(),
            "created_at":datetime.now(timezone.utc).isoformat(),
        }, checkpoint)
        metrics={
            "schema":"autonomy_gps_ai_training_metrics_v1",
            "policy_type":"AUTO_GPS",
            "route_id":(dataset_document.get("route") or {}).get("route_id"),
            "device":str(device),
            "train_samples":len(train_dataset),
            "validation_samples":len(val_dataset),
            "best_validation_loss":None if best==math.inf else best,
            "best_epoch":best_epoch,
            "best_validation":best_validation,
            "epochs_completed":len(history),
            "early_stopping":{
                "enabled":bool(val_loader is not None and patience>0),
                "patience":patience,
                "minimum_delta":minimum_delta,
                "stopped_early":stopped_early,
            },
            "curve_aware_steering_loss":{
                "loss":"weighted_mse",
                **asdict(self.policy),
                "transition_combination":"max(group_weight, transition_weight)",
            },
            "scenario_balancing":{
                "enabled":bool(self.config.balance_scenarios),
                "scenario_counts":counts,
                "exponent":self.config.scenario_balance_exponent,
                "maximum_weight_ratio":self.config.maximum_scenario_weight_ratio,
            },
            "history":history,
            "checkpoint":os.path.basename(checkpoint),
        }
        with open(os.path.join(output_path,"training_metrics.json"),"w",encoding="utf-8") as file:
            json.dump(metrics,file,indent=2)
        train_dataset.close(); val_dataset.close()
        return metrics

    def _scenario_sampler(self, torch, dataset, counts):
        if not self.config.balance_scenarios or len(counts)<=1:
            return None
        maximum=max(counts.values()); weights=[]
        exponent=max(0.0,float(self.config.scenario_balance_exponent))
        limit=max(1.0,float(self.config.maximum_scenario_weight_ratio))
        for sample in dataset.samples:
            count=max(1,counts.get(str(sample.get("scenario") or "unknown"),1))
            weights.append(min(limit,max(1.0,(maximum/count)**exponent)))
        generator=torch.Generator(); generator.manual_seed(self.config.seed)
        return torch.utils.data.WeightedRandomSampler(weights,len(weights),replacement=True,generator=generator)

    def _validation_metrics(self, model, loader, dataset, device, torch):
        model.eval(); count=0; steering_weighted_sum=0.0; steering_weight_sum=0.0; throttle_squared_sum=0.0
        steering_abs_sum=0.0; groups={}; transitions={"samples":0,"steering_sum":0.0,"throttle_sum":0.0}
        with torch.no_grad():
            for image,lidar,auxiliary,route,target,indices in loader:
                image,lidar,auxiliary,route,target=[x.to(device) for x in (image,lidar,auxiliary,route,target)]
                pred=model(image,lidar,auxiliary,route)
                weights=self._batch_steering_weights(torch,dataset,indices,device)
                steering_squared=torch.square(pred[:,0]-target[:,0])
                throttle_squared=torch.square(pred[:,1]-target[:,1])
                steering_weighted_sum += float((steering_squared*weights).sum().cpu())
                steering_weight_sum += float(weights.sum().cpu())
                throttle_squared_sum += float(throttle_squared.sum().cpu())
                indices_cpu=indices.detach().cpu().tolist(); pred_cpu=pred.detach().cpu(); target_cpu=target.detach().cpu()
                for row,index in enumerate(indices_cpu):
                    sample=dataset.samples[int(index)]
                    steering_error=abs(float(pred_cpu[row,0]-target_cpu[row,0]))*self.model_spec.maximum_steering_degrees
                    throttle_error=abs(float(pred_cpu[row,1]-target_cpu[row,1]))
                    steering_abs_sum += steering_error; count += 1
                    group=self._steering_group(sample)
                    bucket=groups.setdefault(group,{"samples":0,"steering_sum":0.0,"throttle_sum":0.0})
                    bucket["samples"]+=1; bucket["steering_sum"]+=steering_error; bucket["throttle_sum"]+=throttle_error
                    if self._is_transition(sample):
                        transitions["samples"]+=1; transitions["steering_sum"]+=steering_error; transitions["throttle_sum"]+=throttle_error
        steering_mse=steering_weighted_sum/max(1e-6,steering_weight_sum)
        throttle_mse=throttle_squared_sum/max(1,count)
        group_metrics={name:{"samples":value["samples"],
                             "steering_mae_degrees":value["steering_sum"]/value["samples"],
                             "throttle_mae":value["throttle_sum"]/value["samples"]}
                       for name,value in groups.items() if value["samples"]}
        transition_metrics=None
        if transitions["samples"]:
            transition_metrics={"samples":transitions["samples"],
                                "steering_mae_degrees":transitions["steering_sum"]/transitions["samples"],
                                "throttle_mae":transitions["throttle_sum"]/transitions["samples"]}
        return {
            "loss":self.config.steering_loss_weight*steering_mse+self.config.throttle_loss_weight*throttle_mse,
            "weighted_steering_mse":steering_mse,
            "throttle_mse":throttle_mse,
            "steering_mae_degrees":steering_abs_sum/max(1,count),
            "steering_group_metrics":group_metrics,
            "transition_metrics":transition_metrics,
        }

    def _device(self, torch):
        if self.config.device!="auto": return torch.device(self.config.device)
        if torch.cuda.is_available(): return torch.device("cuda")
        if getattr(torch.backends,"mps",None) and torch.backends.mps.is_available(): return torch.device("mps")
        return torch.device("cpu")


class GpsEvaluator:
    @staticmethod
    def _steering_group(sample):
        return GpsTrainer._steering_group(sample)

    @staticmethod
    def _is_transition(sample):
        return GpsTrainer._is_transition(sample)

    def evaluate(self, dataset_path, checkpoint_path, split="test", recordings_root_override=None,
                 output_path=None, maximum_steering_mae_degrees=None, maximum_throttle_mae=None,
                 device="auto"):
        _, _, torch, _, DataLoader, _ = require_training_dependencies()
        resolved = torch.device("cuda" if device=="auto" and torch.cuda.is_available() else ("cpu" if device=="auto" else device))
        checkpoint=torch.load(checkpoint_path,map_location=resolved,weights_only=True)
        spec=GpsDrivingModelSpec(**checkpoint["model_spec"])
        model=create_gps_torch_model(spec).to(resolved)
        model.load_state_dict(checkpoint["model_state_dict"]); model.eval()
        dataset=GpsManifestDataset(dataset_path,split,spec,recordings_root_override)
        if len(dataset)==0:
            dataset.close(); raise ValueError(f"Evaluation split contains no samples: {split}")
        loader=DataLoader(dataset,batch_size=64,shuffle=False,num_workers=0)
        steering=0.0; throttle=0.0; count=0; scenarios={}; groups={}
        transitions={"samples":0,"steering_sum":0.0,"throttle_sum":0.0}
        with torch.no_grad():
            for image,lidar,auxiliary,route,target,indices in loader:
                pred=model(image.to(resolved),lidar.to(resolved),auxiliary.to(resolved),route.to(resolved)).cpu()
                for row in range(pred.shape[0]):
                    se=abs(float(pred[row,0]-target[row,0]))*spec.maximum_steering_degrees
                    te=abs(float(pred[row,1]-target[row,1]))
                    steering+=se; throttle+=te; count+=1
                    sample=dataset.samples[int(indices[row])]
                    name=sample.get("scenario","unknown")
                    bucket=scenarios.setdefault(name,{"samples":0,"steering_sum":0.0,"throttle_sum":0.0})
                    bucket["samples"]+=1; bucket["steering_sum"]+=se; bucket["throttle_sum"]+=te
                    group=self._steering_group(sample)
                    group_bucket=groups.setdefault(group,{"samples":0,"steering_sum":0.0,"throttle_sum":0.0})
                    group_bucket["samples"]+=1; group_bucket["steering_sum"]+=se; group_bucket["throttle_sum"]+=te
                    if self._is_transition(sample):
                        transitions["samples"]+=1; transitions["steering_sum"]+=se; transitions["throttle_sum"]+=te
        scenario_metrics={name:{"samples":b["samples"],"steering_mae_degrees":b["steering_sum"]/b["samples"],
                                "throttle_mae":b["throttle_sum"]/b["samples"]} for name,b in scenarios.items()}
        group_metrics={name:{"samples":b["samples"],"steering_mae_degrees":b["steering_sum"]/b["samples"],
                             "throttle_mae":b["throttle_sum"]/b["samples"]} for name,b in groups.items()}
        transition_metrics=None
        if transitions["samples"]:
            transition_metrics={"samples":transitions["samples"],
                                "steering_mae_degrees":transitions["steering_sum"]/transitions["samples"],
                                "throttle_mae":transitions["throttle_sum"]/transitions["samples"]}
        steering_mae=steering/count; throttle_mae=throttle/count
        checks={}
        if maximum_steering_mae_degrees is not None: checks["steering_mae"]=steering_mae<=maximum_steering_mae_degrees
        if maximum_throttle_mae is not None: checks["throttle_mae"]=throttle_mae<=maximum_throttle_mae
        result={"schema":"autonomy_gps_ai_evaluation_v1","policy_type":"AUTO_GPS","route_id":checkpoint.get("route_id"),
                "split":split,"samples":count,"steering_mae_degrees":steering_mae,"throttle_mae":throttle_mae,
                "scenario_metrics":scenario_metrics,"steering_group_metrics":group_metrics,
                "transition_metrics":transition_metrics,
                "criteria_passed":all(checks.values()) if checks else None,"checks":checks}
        if output_path:
            os.makedirs(output_path,exist_ok=True)
            with open(os.path.join(output_path,"evaluation_metrics.json"),"w",encoding="utf-8") as file: json.dump(result,file,indent=2)
        dataset.close(); return result


class GpsOnnxExporter:
    def export(self, checkpoint_path, output_path, verify=True, model_filename="gps_drive_model.onnx"):
        _, _, torch, _, _, _ = require_training_dependencies()
        os.makedirs(output_path,exist_ok=True)
        checkpoint=torch.load(checkpoint_path,map_location="cpu",weights_only=True)
        spec=GpsDrivingModelSpec(**checkpoint["model_spec"])
        model=create_gps_torch_model(spec); model.load_state_dict(checkpoint["model_state_dict"]); model.eval()
        image=torch.zeros((1,3,spec.image_height,spec.image_width),dtype=torch.float32)
        lidar=torch.zeros((1,14),dtype=torch.float32)
        auxiliary=torch.zeros((1,2),dtype=torch.float32)
        route=torch.zeros((1,spec.route_feature_size),dtype=torch.float32)
        path=os.path.join(output_path,model_filename)
        torch.onnx.export(model,(image,lidar,auxiliary,route),f=path,
                          input_names=["image","lidar","auxiliary","route"],output_names=["control"],
                          dynamo=True,verify=bool(verify))
        if not os.path.isfile(path) or os.path.getsize(path)==0: raise OSError("ONNX export failed")
        manifest={"schema":"autonomy_gps_ai_onnx_manifest_v1","policy_type":"AUTO_GPS",
                  "route_id":checkpoint.get("route_id"),"model_file":os.path.basename(path),
                  "model_spec":asdict(spec),
                  "inputs":{"image":{"shape":[1,3,spec.image_height,spec.image_width]},
                            "lidar":{"shape":[1,14]},"auxiliary":{"shape":[1,2]},
                            "route":{"shape":[1,spec.route_feature_size],"feature_order":list(ROUTE_FEATURE_ORDER)}},
                  "output":{"control":{"shape":[1,2],"index_0":"normalized steering","index_1":"throttle"}},
                  "export":{"backend":"torch.onnx.export","dynamo":True,"verify":bool(verify)}}
        with open(os.path.join(output_path,"model_manifest.json"),"w",encoding="utf-8") as file: json.dump(manifest,file,indent=2)
        return manifest
