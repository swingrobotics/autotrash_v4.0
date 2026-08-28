from dataclasses import dataclass, field
from datetime import datetime, timezone
import csv
import json
import math
import os
import statistics

from autonomous_car.control import PathPoint
from autonomous_car.localization import LocalENUConverter

ROUTE_FEATURE_ORDER = (
    "cross_track_error",
    "heading_error",
    "near_bearing_error",
    "near_distance",
    "far_bearing_error",
    "far_distance",
    "remaining_distance",
    "route_progress",
)


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, float(value)))


def _wrap_radians(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _safe_id(value):
    value = str(value or "").strip()
    if not value or os.path.basename(value) != value or value in {".", ".."}:
        raise ValueError("Route ID is required")
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")
    if any(character not in allowed for character in value):
        raise ValueError("Route ID may contain only letters, numbers, -, _, and .")
    return value


def _fsync_parent(path):
    directory = os.path.dirname(os.path.abspath(path)) or "."
    try:
        descriptor = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    except OSError:
        return
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


@dataclass(frozen=True)
class NormalizedGpsRoute:
    route_id: str
    origin: dict
    points: list[PathPoint]
    source_sessions: list[str] = field(default_factory=list)
    quality: dict = field(default_factory=dict)
    created_at: str | None = None

    def as_dict(self):
        return {
            "schema": "normalized_gps_route_v1",
            "route_id": self.route_id,
            "created_at": self.created_at or datetime.now(timezone.utc).isoformat(),
            "origin": dict(self.origin),
            "source_sessions": list(self.source_sessions),
            "quality": dict(self.quality),
            "points": [
                {"x": point.x, "y": point.y, "speed_mps": point.speed_mps}
                for point in self.points
            ],
        }

    def save(self, path):
        directory = os.path.dirname(os.path.abspath(path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = str(path) + ".tmp"
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(self.as_dict(), file, ensure_ascii=False, indent=2)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        _fsync_parent(path)
        return self.as_dict()

    @staticmethod
    def load(path):
        with open(path, "r", encoding="utf-8") as file:
            document = json.load(file)
        if document.get("schema") not in {None, "normalized_gps_route_v1"}:
            raise ValueError(f"Unsupported GPS route schema: {document.get('schema')}")
        points = [
            PathPoint(float(item["x"]), float(item["y"]), item.get("speed_mps"))
            for item in document.get("points") or []
        ]
        if len(points) < 2:
            raise ValueError("Normalized GPS route requires at least two points")
        return NormalizedGpsRoute(
            route_id=_safe_id(document.get("route_id") or os.path.splitext(os.path.basename(path))[0]),
            origin=document["origin"],
            points=points,
            source_sessions=list(document.get("source_sessions") or []),
            quality=dict(document.get("quality") or {}),
            created_at=document.get("created_at"),
        )


class GpsRouteNormalizer:
    """Fuse multiple GPS-ON human RECORD runs into one repeatable RTK route."""

    def __init__(self, spacing_m=0.20, maximum_jump_m=2.0, smoothing_window=5,
                 outlier_distance_m=1.25, minimum_fixed_samples=20):
        self.spacing_m = max(0.05, float(spacing_m))
        self.maximum_jump_m = max(0.20, float(maximum_jump_m))
        self.smoothing_window = max(1, int(smoothing_window))
        self.outlier_distance_m = max(0.10, float(outlier_distance_m))
        self.minimum_fixed_samples = max(2, int(minimum_fixed_samples))

    def build(self, recordings_root, session_names, route_id, output_path=None):
        recordings_root = os.path.abspath(recordings_root)
        sessions = self._validate_sessions(recordings_root, session_names)
        route_id = _safe_id(route_id)
        raw_runs = []
        first_geodetic = None
        for session in sessions:
            samples = self._read_session(recordings_root, session)
            if first_geodetic is None:
                first_geodetic = samples[0][:3]
            raw_runs.append((session, samples))
        converter = LocalENUConverter(*first_geodetic)
        runs = []
        for session, samples in raw_runs:
            points = self._smooth(self._to_enu(samples, converter))
            if len(points) < 2:
                raise ValueError(f"{session}: insufficient route geometry after filtering")
            runs.append((session, points))
        reference = runs[0][1]
        aligned = [(runs[0][0], reference)]
        reversed_sessions = []
        for session, points in runs[1:]:
            same = self._endpoint_score(reference, points)
            reversed_points = list(reversed(points))
            reverse = self._endpoint_score(reference, reversed_points)
            if reverse < same:
                points = reversed_points
                reversed_sessions.append(session)
            aligned.append((session, points))
        lengths = [self._length(points) for _, points in aligned]
        target_length = statistics.median(lengths)
        station_count = max(2, int(round(target_length / self.spacing_m)) + 1)
        station_runs = [
            (session, self._resample_fraction(points, station_count))
            for session, points in aligned
        ]
        fused, dispersions, used_counts = [], [], []
        for station in range(station_count):
            candidates = [points[station] for _, points in station_runs]
            median_x = statistics.median(point.x for point in candidates)
            median_y = statistics.median(point.y for point in candidates)
            deviations = [math.hypot(point.x-median_x, point.y-median_y) for point in candidates]
            kept = [p for p,d in zip(candidates,deviations) if d <= self.outlier_distance_m] or candidates
            x = statistics.median(point.x for point in kept)
            y = statistics.median(point.y for point in kept)
            speeds = [float(p.speed_mps) for p in kept if p.speed_mps is not None and math.isfinite(float(p.speed_mps))]
            fused.append(PathPoint(x, y, statistics.median(speeds) if speeds else None))
            dispersions.append(sum(math.hypot(p.x-x,p.y-y) for p in kept)/len(kept))
            used_counts.append(len(kept))
        fused = self._resample_spacing(self._smooth(fused))
        quality = {
            "source_run_count": len(sessions),
            "source_lengths_m": lengths,
            "median_source_length_m": target_length,
            "normalized_length_m": self._length(fused),
            "spacing_m": self.spacing_m,
            "outlier_distance_m": self.outlier_distance_m,
            "mean_cross_run_dispersion_m": sum(dispersions)/len(dispersions),
            "max_cross_run_dispersion_m": max(dispersions),
            "minimum_runs_used_per_station": min(used_counts),
            "reversed_sessions": reversed_sessions,
            "fusion": "distance-progress median with per-station outlier rejection",
        }
        route = NormalizedGpsRoute(route_id, converter.to_dict(), fused, sessions, quality,
                                   datetime.now(timezone.utc).isoformat())
        if output_path:
            route.save(output_path)
        return route

    def _validate_sessions(self, recordings_root, session_names):
        result, seen = [], set()
        for raw in session_names or []:
            name = str(raw or "").strip()
            if not name or os.path.basename(name) != name or name in {".",".."}:
                raise ValueError(f"Invalid RECORD session name: {raw}")
            if name in seen:
                continue
            path = os.path.abspath(os.path.join(recordings_root, name))
            if os.path.commonpath([recordings_root, path]) != recordings_root:
                raise ValueError("RECORD session escapes recordings root")
            if not os.path.isdir(path):
                raise FileNotFoundError(path)
            result.append(name); seen.add(name)
        if len(result) < 2:
            raise ValueError("GPS route normalization requires at least two GPS-ON RECORD sessions")
        return result

    def _read_session(self, root, session):
        path = os.path.join(root, session)
        metadata_path = os.path.join(path, "metadata.json")
        if os.path.isfile(metadata_path):
            with open(metadata_path, "r", encoding="utf-8") as file:
                metadata = json.load(file)
            if metadata.get("record_gps") is False:
                raise ValueError(f"{session}: GPS recording was disabled")
            if str(metadata.get("purpose") or "RECORD").upper().startswith("AUTO"):
                raise ValueError(f"{session}: autonomous run cannot define a human reference route")
        gnss_path = os.path.join(path, "gnss.csv")
        if not os.path.isfile(gnss_path):
            raise FileNotFoundError(f"{session}: gnss.csv not found")
        result = []
        with open(gnss_path, "r", encoding="utf-8", newline="") as file:
            for row in csv.DictReader(file):
                if str(row.get("rtk_status") or row.get("fix") or "").upper() != "RTK FIXED":
                    continue
                try:
                    values = (float(row["latitude"]), float(row["longitude"]),
                              float(row.get("altitude_m") or 0.0), float(row.get("speed_mps") or 0.0))
                except (KeyError, TypeError, ValueError):
                    continue
                if all(math.isfinite(v) for v in values):
                    result.append(values)
        if len(result) < self.minimum_fixed_samples:
            raise ValueError(f"{session}: expected at least {self.minimum_fixed_samples} RTK FIXED samples, got {len(result)}")
        return result

    def _to_enu(self, samples, converter):
        result = []
        for lat, lon, alt, speed in samples:
            x,y,_ = converter.to_enu(lat,lon,alt)
            point = PathPoint(x,y,speed)
            if result and math.hypot(point.x-result[-1].x, point.y-result[-1].y) > self.maximum_jump_m:
                continue
            result.append(point)
        return result

    @staticmethod
    def _endpoint_score(reference, points):
        return math.hypot(reference[0].x-points[0].x, reference[0].y-points[0].y) + math.hypot(reference[-1].x-points[-1].x, reference[-1].y-points[-1].y)

    @staticmethod
    def _length(points):
        return sum(math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(points,points[1:]))

    def _smooth(self, points):
        if len(points) < self.smoothing_window:
            return list(points)
        radius = self.smoothing_window//2
        result=[]
        for index, point in enumerate(points):
            section=points[max(0,index-radius):min(len(points),index+radius+1)]
            speeds=[float(p.speed_mps) for p in section if p.speed_mps is not None and math.isfinite(float(p.speed_mps))]
            result.append(PathPoint(sum(p.x for p in section)/len(section),
                                    sum(p.y for p in section)/len(section),
                                    statistics.median(speeds) if speeds else point.speed_mps))
        return result

    def _resample_fraction(self, points, count):
        cumulative=[0.0]
        for a,b in zip(points,points[1:]):
            cumulative.append(cumulative[-1]+math.hypot(b.x-a.x,b.y-a.y))
        total=cumulative[-1]
        if total <= 1e-6:
            raise ValueError("Route length is zero")
        return [self._interpolate(points,cumulative,total*i/(count-1)) for i in range(count)]

    def _resample_spacing(self, points):
        total=self._length(points)
        count=max(2,int(math.floor(total/self.spacing_m))+1)
        result=self._resample_fraction(points,count)
        if math.hypot(result[-1].x-points[-1].x,result[-1].y-points[-1].y)>0.05:
            result.append(points[-1])
        return result

    @staticmethod
    def _interpolate(points,cumulative,target):
        if target<=0: return points[0]
        if target>=cumulative[-1]: return points[-1]
        low,high=0,len(cumulative)-1
        while low+1<high:
            mid=(low+high)//2
            if cumulative[mid]<target: low=mid
            else: high=mid
        a,b=points[low],points[high]
        span=max(1e-9,cumulative[high]-cumulative[low])
        r=(target-cumulative[low])/span
        sa=a.speed_mps if a.speed_mps is not None else b.speed_mps
        sb=b.speed_mps if b.speed_mps is not None else sa
        speed=None if sa is None or sb is None else float(sa)+(float(sb)-float(sa))*r
        return PathPoint(a.x+(b.x-a.x)*r,a.y+(b.y-a.y)*r,speed)


@dataclass(frozen=True)
class GpsRouteFeatures:
    nearest_index: int
    cross_track_error_m: float
    heading_error_degrees: float
    near_bearing_error_degrees: float
    near_distance_m: float
    far_bearing_error_degrees: float
    far_distance_m: float
    remaining_distance_m: float
    route_progress: float
    normalized: tuple[float, ...]

    def as_dict(self):
        return {
            "nearest_index": self.nearest_index,
            "cross_track_error_m": self.cross_track_error_m,
            "heading_error_degrees": self.heading_error_degrees,
            "near_bearing_error_degrees": self.near_bearing_error_degrees,
            "near_distance_m": self.near_distance_m,
            "far_bearing_error_degrees": self.far_bearing_error_degrees,
            "far_distance_m": self.far_distance_m,
            "remaining_distance_m": self.remaining_distance_m,
            "route_progress": self.route_progress,
            "feature_order": list(ROUTE_FEATURE_ORDER),
            "normalized": list(self.normalized),
        }


class GpsRouteFeatureExtractor:
    def __init__(self, route, near_lookahead_m=1.5, far_lookahead_m=4.0,
                 maximum_cross_track_m=2.0, maximum_heading_error_degrees=90.0,
                 maximum_bearing_error_degrees=90.0, maximum_lookahead_distance_m=6.0,
                 maximum_remaining_distance_m=60.0):
        self.route=route; self.points=route.points
        if len(self.points)<2: raise ValueError("GPS route requires at least two points")
        self.converter=LocalENUConverter(route.origin["origin_latitude"],route.origin["origin_longitude"],route.origin.get("origin_altitude",0.0))
        self.near_lookahead_m=float(near_lookahead_m); self.far_lookahead_m=float(far_lookahead_m)
        self.maximum_cross_track_m=float(maximum_cross_track_m)
        self.maximum_heading_error_degrees=float(maximum_heading_error_degrees)
        self.maximum_bearing_error_degrees=float(maximum_bearing_error_degrees)
        self.maximum_lookahead_distance_m=float(maximum_lookahead_distance_m)
        self.maximum_remaining_distance_m=float(maximum_remaining_distance_m)
        self.cumulative=[0.0]
        for a,b in zip(self.points,self.points[1:]):
            self.cumulative.append(self.cumulative[-1]+math.hypot(b.x-a.x,b.y-a.y))
        self.total_length=max(1e-6,self.cumulative[-1])

    @staticmethod
    def compass_to_enu_heading(compass_degrees):
        return math.radians((90.0-float(compass_degrees))%360.0)

    def extract(self, latitude, longitude, compass_heading_degrees, previous_index=None):
        x,y,_=self.converter.to_enu(float(latitude),float(longitude),0.0)
        segment_index,px,py,signed_cte=self._nearest_segment(x,y,previous_index)
        a,b=self.points[segment_index],self.points[segment_index+1]
        segment_heading=math.atan2(b.y-a.y,b.x-a.x)
        vehicle_heading=self.compass_to_enu_heading(compass_heading_degrees)
        heading_error=math.degrees(_wrap_radians(segment_heading-vehicle_heading))
        segment_length=max(1e-9,math.hypot(b.x-a.x,b.y-a.y))
        along=min(segment_length,math.hypot(px-a.x,py-a.y))
        current_distance=self.cumulative[segment_index]+along
        near=self._point_at_distance(min(self.total_length,current_distance+self.near_lookahead_m))
        far=self._point_at_distance(min(self.total_length,current_distance+self.far_lookahead_m))
        near_bearing,near_distance=self._target_geometry(x,y,vehicle_heading,near.x,near.y)
        far_bearing,far_distance=self._target_geometry(x,y,vehicle_heading,far.x,far.y)
        remaining=max(0.0,self.total_length-current_distance)
        progress=_clamp(current_distance/self.total_length,0.0,1.0)
        normalized=(
            _clamp(signed_cte/max(1e-6,self.maximum_cross_track_m),-1,1),
            _clamp(heading_error/max(1e-6,self.maximum_heading_error_degrees),-1,1),
            _clamp(near_bearing/max(1e-6,self.maximum_bearing_error_degrees),-1,1),
            _clamp(near_distance/max(1e-6,self.maximum_lookahead_distance_m),0,1),
            _clamp(far_bearing/max(1e-6,self.maximum_bearing_error_degrees),-1,1),
            _clamp(far_distance/max(1e-6,self.maximum_lookahead_distance_m),0,1),
            _clamp(remaining/max(1e-6,self.maximum_remaining_distance_m),0,1),
            progress,
        )
        return GpsRouteFeatures(segment_index,signed_cte,heading_error,near_bearing,near_distance,
                                far_bearing,far_distance,remaining,progress,normalized)

    def _nearest_segment(self,x,y,previous_index):
        if previous_index is None:
            start_index,end_index=0,len(self.points)-1
        else:
            center=max(0,min(int(previous_index),len(self.points)-2))
            start_index=max(0,center-15); end_index=min(len(self.points)-1,center+40)
        best=None
        for index in range(start_index,end_index):
            a,b=self.points[index],self.points[index+1]
            dx,dy=b.x-a.x,b.y-a.y
            length_sq=dx*dx+dy*dy
            if length_sq<=1e-12: continue
            t=_clamp(((x-a.x)*dx+(y-a.y)*dy)/length_sq,0,1)
            px,py=a.x+dx*t,a.y+dy*t
            distance=math.hypot(x-px,y-py)
            if best is None or distance<best[0]:
                cross=dx*(y-py)-dy*(x-px)
                best=(distance,index,px,py,distance if cross>=0 else -distance)
        if best is None: raise ValueError("Unable to project vehicle onto GPS route")
        return best[1],best[2],best[3],best[4]

    def _point_at_distance(self,target):
        if target<=0: return self.points[0]
        if target>=self.total_length: return self.points[-1]
        low,high=0,len(self.cumulative)-1
        while low+1<high:
            mid=(low+high)//2
            if self.cumulative[mid]<target: low=mid
            else: high=mid
        a,b=self.points[low],self.points[high]
        span=max(1e-9,self.cumulative[high]-self.cumulative[low])
        r=(target-self.cumulative[low])/span
        return PathPoint(a.x+(b.x-a.x)*r,a.y+(b.y-a.y)*r,None)

    @staticmethod
    def _target_geometry(x,y,vehicle_heading,target_x,target_y):
        dx,dy=target_x-x,target_y-y
        distance=math.hypot(dx,dy)
        bearing=math.atan2(dy,dx)
        return math.degrees(_wrap_radians(bearing-vehicle_heading)),distance
