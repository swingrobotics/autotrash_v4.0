from dataclasses import asdict, dataclass
import math

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


@dataclass(frozen=True)
class DetectedObject:
    label: str
    confidence: float
    x: int
    y: int
    width: int
    height: int
    center_x_normalized: float
    bearing_degrees: float
    lidar_distance_m: float | None = None
    in_vehicle_path: bool = False

    def as_dict(self):
        return asdict(self)


class ObjectDetector:
    def __init__(
        self,
        camera_fov_degrees=82.1,
        confidence_threshold=0.45,
        camera_calibration=None,
    ):
        self.camera_fov_degrees = float(camera_fov_degrees)
        self.confidence_threshold = float(confidence_threshold)
        self.camera_calibration = camera_calibration
        self.hog = None
        if cv2 is not None and hasattr(cv2, "HOGDescriptor"):
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    @property
    def available(self):
        return self.hog is not None and np is not None

    def detect_people(self, jpeg):
        if not self.available or not jpeg:
            return []
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return []
        if self.camera_calibration is not None:
            image = self.camera_calibration.undistort(image)
        image = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
        boxes, weights = self.hog.detectMultiScale(
            image,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )
        detections = []
        for (x, y, width, height), weight in zip(boxes, weights):
            confidence = float(weight)
            if confidence < self.confidence_threshold:
                continue
            center_normalized = ((x + width / 2.0) - image.shape[1] / 2.0) / (image.shape[1] / 2.0)
            calibrated_fov = (
                self.camera_calibration.horizontal_fov_degrees()
                if self.camera_calibration is not None
                else None
            )
            bearing = center_normalized * (
                calibrated_fov or self.camera_fov_degrees
            ) / 2.0
            detections.append(
                DetectedObject(
                    "person",
                    confidence,
                    int(x),
                    int(y),
                    int(width),
                    int(height),
                    center_normalized,
                    bearing,
                )
            )
        return detections

    @staticmethod
    def fuse_lidar(detections, lidar_points, maximum_distance_m=2.0, path_half_width_m=0.45):
        fused = []
        for detection in detections:
            distances = []
            for point in lidar_points or []:
                try:
                    bearing = float(point["bearing_degrees"])
                    distance = float(point["distance_mm"]) / 1000.0
                except (KeyError, TypeError, ValueError):
                    continue
                if abs(bearing - detection.bearing_degrees) <= 5.0 and distance > 0:
                    distances.append(distance)
            lidar_distance = min(distances) if distances else None
            lateral = (
                abs(lidar_distance * math.sin(math.radians(detection.bearing_degrees)))
                if lidar_distance is not None
                else None
            )
            in_path = (
                lidar_distance is not None
                and lidar_distance <= maximum_distance_m
                and lateral <= path_half_width_m
            )
            fused.append(
                DetectedObject(
                    detection.label,
                    detection.confidence,
                    detection.x,
                    detection.y,
                    detection.width,
                    detection.height,
                    detection.center_x_normalized,
                    detection.bearing_degrees,
                    lidar_distance,
                    in_path,
                )
            )
        return fused
