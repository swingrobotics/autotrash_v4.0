from dataclasses import asdict, dataclass
import math
import threading
from statistics import median

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


@dataclass(frozen=True)
class LaneResult:
    detected: bool
    confidence: float
    lateral_error_normalized: float | None = None
    lateral_error_m: float | None = None
    heading_error_degrees: float | None = None
    correction_angle_degrees: float = 0.0
    left_line_count: int = 0
    right_line_count: int = 0
    lane_width_pixels: float | None = None
    error: str | None = None
    lane_width_top_pixels: float | None = None
    perspective_ratio: float | None = None
    expected_lane_width_m: float | None = None
    vehicle_width_m: float | None = None
    left_line: dict | None = None
    right_line: dict | None = None
    center_line: dict | None = None
    backend: str = "CLASSICAL_CV"
    marking: str | None = None
    inferred_left: bool = False
    inferred_right: bool = False
    roi: dict | None = None
    image_size: tuple[int, int] | None = None

    def as_dict(self):
        return asdict(self)


class LaneController:
    """General indoor/outdoor lane detector built from OpenCV primitives.

    The detector is color-agnostic at the geometry layer and combines four
    appearance channels before fitting the lane boundaries:
      * dark markings (black tape on a light indoor floor),
      * yellow markings,
      * white markings,
      * strong luminance edges for faded/neutral road paint.

    Candidate pixels are tracked with a sliding-window search and robustly fit
    as x=f(y) quadratic curves, so dashed and curved road lanes are supported.
    Lane validity is based on two-boundary geometry/perspective rather than a
    fixed pixel-width percentage. A recently observed lane-width profile may be
    used to bridge a short single-boundary occlusion, but inferred lanes receive
    lower confidence.
    """

    BACKEND = "CLASSICAL_CV"

    def __init__(
        self,
        expected_lane_width_m=1.0,
        vehicle_width_m=0.4826,
        minimum_lane_clearance_m=0.05,
        lateral_gain=8.0,
        heading_gain=0.25,
        maximum_correction_degrees=5.0,
        camera_calibration=None,
        processing_width=640,
        processing_height=360,
    ):
        self.expected_lane_width_m = float(expected_lane_width_m)
        self.vehicle_width_m = float(vehicle_width_m)
        self.minimum_lane_clearance_m = max(0.0, float(minimum_lane_clearance_m))
        self.lateral_gain = float(lateral_gain)
        self.heading_gain = float(heading_gain)
        self.maximum_correction_degrees = abs(float(maximum_correction_degrees))
        self.camera_calibration = camera_calibration
        self.processing_width = max(320, int(processing_width))
        self.processing_height = max(180, int(processing_height))
        self._previous_width_profile = None
        self._previous_width_y = None
        self._previous_left_coefficients = None
        self._previous_right_coefficients = None
        self._frames_since_two_boundary = 1000
        self._processing_lock = threading.RLock()
        self._cached_jpeg = None
        self._cached_result = None

    @property
    def available(self):
        return cv2 is not None and np is not None

    def set_expected_lane_width_m(self, value):
        value = float(value)
        if math.isfinite(value) and value > 0:
            self.expected_lane_width_m = value

    def reset(self):
        with self._processing_lock:
            self._previous_width_profile = None
            self._previous_width_y = None
            self._previous_left_coefficients = None
            self._previous_right_coefficients = None
            self._frames_since_two_boundary = 1000
            self._cached_jpeg = None
            self._cached_result = None

    def analyze_jpeg(self, jpeg):
        if not self.available:
            return LaneResult(False, 0.0, error="OPENCV_UNAVAILABLE")
        if not jpeg:
            return LaneResult(False, 0.0, error="CAMERA_FRAME_UNAVAILABLE")
        with self._processing_lock:
            # /api/lane and an autonomous control loop can request the same
            # camera bytes concurrently. Reuse that result so dashboard drawing
            # never doubles the Pi vision workload for one frame.
            if jpeg is self._cached_jpeg and self._cached_result is not None:
                return self._cached_result
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                result = LaneResult(False, 0.0, error="JPEG_DECODE_FAILED")
            else:
                result = self.analyze_image(image)
            self._cached_jpeg = jpeg
            self._cached_result = result
            return result

    def analyze_image(self, image):
        if not self.available:
            return LaneResult(False, 0.0, error="OPENCV_UNAVAILABLE")
        if image is None:
            return LaneResult(False, 0.0, error="CAMERA_FRAME_UNAVAILABLE")
        self._frames_since_two_boundary += 1

        minimum_physical_width = self.vehicle_width_m + 2.0 * self.minimum_lane_clearance_m
        if self.expected_lane_width_m <= minimum_physical_width:
            return LaneResult(
                False,
                0.0,
                error="LANE_WIDTH_CONFIGURATION_INVALID",
                expected_lane_width_m=self.expected_lane_width_m,
                vehicle_width_m=self.vehicle_width_m,
                backend=self.BACKEND,
            )

        if self.camera_calibration is not None:
            image = self.camera_calibration.undistort(image)
        image = cv2.resize(
            image,
            (self.processing_width, self.processing_height),
            interpolation=cv2.INTER_AREA,
        )
        height, width = image.shape[:2]
        roi_top = int(height * 0.42)
        roi_bottom = int(height * 0.985)
        roi = image[roi_top:roi_bottom]

        binary, source_masks = self._candidate_mask(roi)
        left_points, right_points = self._sliding_window_points(binary)

        left_fit = self._robust_fit(left_points)
        right_fit = self._robust_fit(right_points)

        # Sliding windows can lose very wide near-field boundaries at the frame
        # edge. Hough segments are only a fallback source of points, not the
        # final geometric model.
        if left_fit is None or right_fit is None:
            hough_left, hough_right = self._hough_points(binary)
            if left_fit is None:
                left_points = left_points + hough_left
                left_fit = self._robust_fit(left_points)
            if right_fit is None:
                right_points = right_points + hough_right
                right_fit = self._robust_fit(right_points)

        marking = self._dominant_marking(source_masks, left_fit, right_fit)
        return self._calculate(
            width,
            height,
            roi.shape[0],
            roi_top,
            left_fit,
            right_fit,
            len(left_points),
            len(right_points),
            marking,
        )

    def _candidate_mask(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)

        yellow = cv2.inRange(hsv, np.array([12, 65, 65]), np.array([42, 255, 255]))

        # White and black markings need local contrast as well as absolute
        # brightness. Without this gate a light indoor floor would itself become
        # a giant "white lane", while dark asphalt could become a black lane.
        local_background = cv2.GaussianBlur(gray_blur, (0, 0), sigmaX=12.0, sigmaY=12.0)
        bright_delta = cv2.subtract(gray_blur, local_background)
        dark_delta = cv2.subtract(local_background, gray_blur)
        _, bright_contrast = cv2.threshold(bright_delta, 12, 255, cv2.THRESH_BINARY)
        _, dark_contrast = cv2.threshold(dark_delta, 12, 255, cv2.THRESH_BINARY)

        white_absolute = cv2.inRange(hls, np.array([0, 170, 0]), np.array([180, 255, 150]))
        white = cv2.bitwise_and(white_absolute, bright_contrast)

        # Dark-marking detection is intentionally enabled only when the scene is
        # bright enough to distinguish black tape/paint from the road surface.
        scene_median = float(np.median(gray_blur))
        if scene_median >= 92.0:
            dark_threshold = int(max(25.0, min(125.0, scene_median * 0.72)))
            black_absolute = cv2.inRange(gray_blur, 0, dark_threshold)
            black = cv2.bitwise_and(black_absolute, dark_contrast)
        else:
            black = np.zeros_like(gray_blur)

        edges = cv2.Canny(gray_blur, 42, 135)
        local_gradient = cv2.morphologyEx(
            gray_blur,
            cv2.MORPH_GRADIENT,
            np.ones((5, 5), np.uint8),
        )
        _, contrast = cv2.threshold(local_gradient, 18, 255, cv2.THRESH_BINARY)
        edge_support = cv2.bitwise_and(edges, contrast)
        edge_support = cv2.dilate(
            edge_support,
            np.ones((3, 3), np.uint8),
            iterations=1,
        )

        color = cv2.bitwise_or(black, cv2.bitwise_or(yellow, white))
        binary = cv2.bitwise_or(color, edge_support)

        # Keep a broad road-ground ROI. It deliberately allows valid near-field
        # boundaries to touch the left/right image edges.
        h, w = binary.shape[:2]
        mask = np.zeros_like(binary)
        polygon = np.array(
            [[
                (0, h - 1),
                (int(w * 0.12), 0),
                (int(w * 0.88), 0),
                (w - 1, h - 1),
            ]],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, polygon, 255)
        binary = cv2.bitwise_and(binary, mask)

        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 7))
        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, close_kernel, iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, open_kernel, iterations=1)

        return binary, {
            "BLACK": black,
            "YELLOW": yellow,
            "WHITE": white,
            "EDGE": edge_support,
        }

    @staticmethod
    def _histogram_base(histogram, start, stop):
        if stop <= start:
            return None
        segment = histogram[start:stop]
        if segment.size == 0:
            return None
        index = int(np.argmax(segment))
        if float(segment[index]) < 3.0:
            return None
        return start + index

    def _sliding_window_points(self, binary):
        active = binary > 0
        h, w = active.shape
        histogram = np.sum(active[int(h * 0.48):, :], axis=0)
        center = w // 2
        guard = max(12, int(w * 0.025))
        left_current = self._histogram_base(histogram, 0, center - guard)
        right_current = self._histogram_base(histogram, center + guard, w)

        nonzero_y, nonzero_x = active.nonzero()
        window_count = 10
        window_height = max(1, h // window_count)
        margin = max(50, int(w * 0.115))
        min_pixels = 18
        left_indices = []
        right_indices = []

        for window in range(window_count):
            y_low = max(0, h - (window + 1) * window_height)
            y_high = h if window == 0 else h - window * window_height

            if left_current is not None:
                good = np.where(
                    (nonzero_y >= y_low)
                    & (nonzero_y < y_high)
                    & (nonzero_x >= left_current - margin)
                    & (nonzero_x < left_current + margin)
                )[0]
                left_indices.append(good)
                if len(good) >= min_pixels:
                    left_current = int(np.median(nonzero_x[good]))

            if right_current is not None:
                good = np.where(
                    (nonzero_y >= y_low)
                    & (nonzero_y < y_high)
                    & (nonzero_x >= right_current - margin)
                    & (nonzero_x < right_current + margin)
                )[0]
                right_indices.append(good)
                if len(good) >= min_pixels:
                    right_current = int(np.median(nonzero_x[good]))

        def collect(chunks):
            if not chunks:
                return []
            chunks = [chunk for chunk in chunks if len(chunk)]
            if not chunks:
                return []
            indices = np.concatenate(chunks)
            # Limit each row to a representative point so a thick tape/line does
            # not overweight the polynomial fit.
            rows = {}
            for index in indices:
                y = int(nonzero_y[index])
                rows.setdefault(y, []).append(int(nonzero_x[index]))
            points = []
            for y, values in rows.items():
                values.sort()
                points.append((float(values[len(values) // 2]), float(y)))
            return points

        return collect(left_indices), collect(right_indices)

    @staticmethod
    def _hough_points(binary):
        h, w = binary.shape[:2]
        lines = cv2.HoughLinesP(
            binary,
            1,
            np.pi / 180,
            threshold=22,
            minLineLength=22,
            maxLineGap=34,
        )
        left = []
        right = []
        if lines is None:
            return left, right
        center = w / 2.0
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            dy = y2 - y1
            dx = x2 - x1
            if abs(dy) < 8:
                continue
            # x=f(y) is stable for near-vertical perspective lane boundaries.
            a = dx / dy
            bottom_x = x1 + a * ((h - 1) - y1)
            target = left if bottom_x < center else right
            steps = max(4, int(abs(dy) / 8))
            for step in range(steps + 1):
                t = step / steps
                target.append(
                    (
                        float(x1 + (x2 - x1) * t),
                        float(y1 + dy * t),
                    )
                )
        return left, right

    @staticmethod
    def _robust_fit(points):
        if np is None or len(points) < 18:
            return None
        values = np.asarray(points, dtype=np.float64)
        x = values[:, 0]
        y = values[:, 1]
        if np.ptp(y) < 45.0:
            return None
        mask = np.ones(len(values), dtype=bool)
        coefficients = None
        for _ in range(4):
            if int(mask.sum()) < 14:
                return None
            coefficients = np.polyfit(y[mask], x[mask], 2)
            predicted = np.polyval(coefficients, y)
            residual = np.abs(predicted - x)
            core = residual[mask]
            threshold = max(
                7.0,
                min(20.0, float(np.percentile(core, 72)) * 1.8 + 2.0),
            )
            next_mask = residual <= threshold
            if np.array_equal(next_mask, mask):
                break
            mask = next_mask
        if coefficients is None or int(mask.sum()) < 14:
            return None
        rows = len(set(int(value) for value in y[mask]))
        coverage = min(1.0, np.ptp(y[mask]) / max(1.0, np.ptp(y)))
        residual_mean = float(
            np.mean(np.abs(np.polyval(coefficients, y[mask]) - x[mask]))
        )
        return {
            "coefficients": coefficients,
            "inliers": int(mask.sum()),
            "rows": rows,
            "coverage": float(coverage),
            "residual": residual_mean,
        }

    @staticmethod
    def _evaluate(fit, y_values):
        if fit is None:
            return None
        return np.polyval(fit["coefficients"], y_values).astype(np.float64)

    @staticmethod
    def _line_document(x_values, y_values, roi_top):
        points = [
            [float(x), float(y + roi_top)]
            for x, y in zip(x_values.tolist(), y_values.tolist())
        ]
        return {
            "bottom_x": points[-1][0],
            "bottom_y": points[-1][1],
            "top_x": points[0][0],
            "top_y": points[0][1],
            "points": points,
        }

    def _dominant_marking(self, masks, left_fit, right_fit):
        fits = [fit for fit in (left_fit, right_fit) if fit is not None]
        if not fits:
            return None
        h, w = next(iter(masks.values())).shape[:2]
        ys = np.linspace(int(h * 0.10), h - 1, 20)
        scores = {key: 0 for key in masks}
        for fit in fits:
            xs = self._evaluate(fit, ys)
            for x, y in zip(xs, ys):
                xi = int(round(x))
                yi = int(round(y))
                if not (0 <= xi < w and 0 <= yi < h):
                    continue
                x0, x1 = max(0, xi - 4), min(w, xi + 5)
                y0, y1 = max(0, yi - 4), min(h, yi + 5)
                for key, mask in masks.items():
                    scores[key] += int(np.count_nonzero(mask[y0:y1, x0:x1]))
        ordered = sorted(scores.items(), key=lambda item: item[1], reverse=True)
        if not ordered or ordered[0][1] <= 0:
            return "EDGE"
        if (
            len(ordered) > 1
            and ordered[1][1] >= ordered[0][1] * 0.70
            and ordered[1][1] > 0
        ):
            return f"{ordered[0][0]}+{ordered[1][0]}"
        return ordered[0][0]

    def _calculate(
        self,
        width,
        image_height,
        roi_height,
        roi_top,
        left_fit,
        right_fit,
        left_count,
        right_count,
        marking,
    ):
        metadata = {
            "expected_lane_width_m": self.expected_lane_width_m,
            "vehicle_width_m": self.vehicle_width_m,
            "backend": self.BACKEND,
            "marking": marking,
            "roi": {
                "top": int(roi_top),
                "bottom": int(roi_top + roi_height),
            },
            "image_size": (int(width), int(image_height)),
        }
        sample_y = np.linspace(
            max(3, int(roi_height * 0.08)),
            roi_height - 1,
            18,
        )
        left_x = self._evaluate(left_fit, sample_y)
        right_x = self._evaluate(right_fit, sample_y)
        inferred_left = False
        inferred_right = False

        if left_x is None and right_x is None:
            return LaneResult(
                False,
                0.0,
                left_line_count=left_count,
                right_line_count=right_count,
                error="LANE_NOT_DETECTED",
                **metadata,
            )

        # Bridge a short one-sided occlusion only after a valid two-sided lane
        # has established a perspective width profile.
        previous_width = self._previous_width_profile
        if (
            previous_width is not None
            and len(previous_width) == len(sample_y)
            and self._frames_since_two_boundary <= 5
        ):
            if left_x is None and right_x is not None:
                left_x = right_x - previous_width
                left_fit = self._fit_from_samples(left_x, sample_y)
                inferred_left = left_fit is not None
            elif right_x is None and left_x is not None:
                right_x = left_x + previous_width
                right_fit = self._fit_from_samples(right_x, sample_y)
                inferred_right = right_fit is not None

        if left_x is None or right_x is None:
            # Preserve the detected physical boundary for the dashboard but do
            # not authorize steering from an unconstrained single line.
            partial_left = (
                None
                if left_x is None
                else self._line_document(left_x, sample_y, roi_top)
            )
            partial_right = (
                None
                if right_x is None
                else self._line_document(right_x, sample_y, roi_top)
            )
            return LaneResult(
                False,
                0.35,
                left_line_count=left_count,
                right_line_count=right_count,
                error="TWO_BOUNDARIES_REQUIRED",
                left_line=partial_left,
                right_line=partial_right,
                inferred_left=inferred_left,
                inferred_right=inferred_right,
                **metadata,
            )

        widths = right_x - left_x
        if np.any(~np.isfinite(widths)) or np.any(widths <= 8.0):
            return LaneResult(
                False,
                0.1,
                left_line_count=left_count,
                right_line_count=right_count,
                error="BOUNDARIES_CROSSED",
                inferred_left=inferred_left,
                inferred_right=inferred_right,
                **metadata,
            )

        center_x_values = (left_x + right_x) / 2.0
        center_limit = width * 0.42
        if (
            np.any(center_x_values < -center_limit)
            or np.any(center_x_values > width + center_limit)
        ):
            return LaneResult(
                False,
                0.1,
                left_line_count=left_count,
                right_line_count=right_count,
                error="LANE_CENTER_INVALID",
                inferred_left=inferred_left,
                inferred_right=inferred_right,
                **metadata,
            )

        lane_width_top = float(widths[0])
        lane_width_bottom = float(widths[-1])
        perspective_ratio = lane_width_bottom / max(lane_width_top, 1e-6)
        # Real curved lanes and calibrated cameras can reduce the apparent width
        # change. Reject only clearly impossible divergence/convergence.
        if perspective_ratio < 0.72 or perspective_ratio > 12.0:
            return LaneResult(
                False,
                0.1,
                left_line_count=left_count,
                right_line_count=right_count,
                lane_width_pixels=lane_width_bottom,
                lane_width_top_pixels=lane_width_top,
                perspective_ratio=perspective_ratio,
                error="PERSPECTIVE_INVALID",
                inferred_left=inferred_left,
                inferred_right=inferred_right,
                **metadata,
            )

        # Width should evolve smoothly with perspective; this rejects unrelated
        # edges that happen to form one plausible pair at a single y position.
        normalized_steps = np.diff(widths) / max(float(np.median(widths)), 1.0)
        width_roughness = (
            float(np.std(normalized_steps)) if len(normalized_steps) else 0.0
        )
        if width_roughness > 0.28:
            return LaneResult(
                False,
                0.2,
                left_line_count=left_count,
                right_line_count=right_count,
                lane_width_pixels=lane_width_bottom,
                lane_width_top_pixels=lane_width_top,
                perspective_ratio=perspective_ratio,
                error="LANE_GEOMETRY_UNSTABLE",
                inferred_left=inferred_left,
                inferred_right=inferred_right,
                **metadata,
            )

        center_bottom = float(center_x_values[-1])
        lookahead_index = max(
            0,
            min(len(sample_y) - 1, int(len(sample_y) * 0.34)),
        )
        center_lookahead = float(center_x_values[lookahead_index])
        y_bottom = float(sample_y[-1])
        y_lookahead = float(sample_y[lookahead_index])

        lateral_normalized = (
            (width / 2.0 - center_bottom)
            / max(lane_width_bottom / 2.0, 1.0)
        )
        lateral_m = lateral_normalized * self.expected_lane_width_m / 2.0
        heading_error = math.degrees(
            math.atan2(
                center_lookahead - center_bottom,
                y_bottom - y_lookahead,
            )
        )
        correction = (
            self.lateral_gain * lateral_normalized
            + self.heading_gain * heading_error
        )
        correction = max(
            -self.maximum_correction_degrees,
            min(self.maximum_correction_degrees, correction),
        )

        support_rows = 0.0
        residual_score = 0.0
        fits = [fit for fit in (left_fit, right_fit) if fit is not None]
        if fits:
            support_rows = min(
                1.0,
                sum(min(80, fit.get("rows", 0)) for fit in fits) / 90.0,
            )
            residual_score = (
                sum(
                    max(0.0, 1.0 - fit.get("residual", 20.0) / 18.0)
                    for fit in fits
                )
                / len(fits)
            )
        perspective_score = max(
            0.0,
            min(1.0, (perspective_ratio - 0.72) / 1.5),
        )
        geometry_score = max(0.0, 1.0 - width_roughness / 0.28)
        confidence = (
            0.52
            + 0.18 * support_rows
            + 0.12 * residual_score
            + 0.10 * geometry_score
            + 0.08 * perspective_score
        )
        if inferred_left or inferred_right:
            confidence = min(confidence * 0.78, 0.64)
        confidence = max(0.0, min(1.0, confidence))

        left_line = self._line_document(left_x, sample_y, roi_top)
        right_line = self._line_document(right_x, sample_y, roi_top)
        center_line = self._line_document(center_x_values, sample_y, roi_top)

        if not inferred_left and not inferred_right:
            self._frames_since_two_boundary = 0
            self._previous_width_profile = widths.copy()
            self._previous_width_y = sample_y.copy()
            self._previous_left_coefficients = (
                left_fit["coefficients"].copy() if left_fit else None
            )
            self._previous_right_coefficients = (
                right_fit["coefficients"].copy() if right_fit else None
            )

        return LaneResult(
            True,
            confidence,
            lateral_normalized,
            lateral_m,
            heading_error,
            correction,
            left_count,
            right_count,
            lane_width_bottom,
            None,
            lane_width_top,
            perspective_ratio,
            self.expected_lane_width_m,
            self.vehicle_width_m,
            left_line,
            right_line,
            center_line,
            self.BACKEND,
            marking,
            inferred_left,
            inferred_right,
            metadata["roi"],
            metadata["image_size"],
        )

    @staticmethod
    def _fit_from_samples(x_values, y_values):
        try:
            coefficients = np.polyfit(y_values, x_values, 2)
        except Exception:
            return None
        return {
            "coefficients": coefficients,
            "inliers": len(y_values),
            "rows": len(y_values),
            "coverage": 1.0,
            "residual": 0.0,
        }
