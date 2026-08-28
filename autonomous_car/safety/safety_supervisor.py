import threading
import time

from autonomous_car.mode_policy import policy_for
from autonomous_car.state import ControlRequest, DriveMode, SafetyContext, SafetyDecision

from .obstacle_checker import ObstacleChecker
from .restart_guard import RestartDelayGuard


class SafetySupervisor:
    ACTIVE_MODES = {
        DriveMode.MANUAL,
        DriveMode.MANUAL_ASSIST,
        DriveMode.RECORD,
        DriveMode.AUTO_AI,
        DriveMode.AUTO_GPS,
        DriveMode.AUTO_LOCAL,
        DriveMode.AUTO,
        # Legacy modes during migration.
        DriveMode.AUTO_ROUTE,
        DriveMode.AUTO_HYBRID,
    }
    AUTONOMOUS_TIMING_MODES = {
        DriveMode.AUTO_AI,
        DriveMode.AUTO_GPS,
        DriveMode.AUTO_LOCAL,
        DriveMode.AUTO,
    }

    def __init__(
        self,
        obstacle_checker=None,
        maximum_throttle=0.35,
        manual_maximum_throttle=None,
        stop_distance_m=0.60,
        crawl_distance_m=0.80,
        slow_distance_m=1.50,
        crawl_throttle=0.12,
        command_timeout_seconds=0.30,
        arduino_timeout_seconds=0.50,
        lidar_timeout_seconds=0.30,
        steering_timeout_seconds=0.35,
        loop_delay_limit_seconds=0.20,
        loop_delay_hard_limit_seconds=0.40,
        autonomous_loop_delay_consecutive_limit=3,
        obstacle_restart_delay_seconds=1.50,
    ):
        self.obstacle_checker = obstacle_checker or ObstacleChecker()
        self.maximum_throttle = max(0.0, min(1.0, float(maximum_throttle)))
        if manual_maximum_throttle is None:
            manual_maximum_throttle = self.maximum_throttle
        self.manual_maximum_throttle = max(
            0.0, min(1.0, float(manual_maximum_throttle))
        )
        self.stop_distance_m = float(stop_distance_m)
        self.crawl_distance_m = float(crawl_distance_m)
        self.slow_distance_m = float(slow_distance_m)
        self.crawl_throttle = max(0.0, min(self.maximum_throttle, float(crawl_throttle)))
        self.command_timeout_seconds = float(command_timeout_seconds)
        self.arduino_timeout_seconds = float(arduino_timeout_seconds)
        self.lidar_timeout_seconds = float(lidar_timeout_seconds)
        self.steering_timeout_seconds = float(steering_timeout_seconds)
        self.loop_delay_limit_seconds = max(0.0, float(loop_delay_limit_seconds))
        self.loop_delay_hard_limit_seconds = max(
            self.loop_delay_limit_seconds,
            float(loop_delay_hard_limit_seconds),
        )
        self.autonomous_loop_delay_consecutive_limit = max(
            1,
            int(autonomous_loop_delay_consecutive_limit),
        )
        self.obstacle_restart_guard = RestartDelayGuard(
            obstacle_restart_delay_seconds
        )
        self._lock = threading.Lock()
        self._last_decision = SafetyDecision(0.0, 0.0, 0.0, 0.0, False, "DISARMED")
        self._last_evaluated_at = time.monotonic()
        self._last_input_source = "none"
        self._last_request_timestamp = None
        self._last_loop_delay_seconds = 0.0
        self._loop_delay_violation_count = 0
        self._loop_delay_source = None

    def _mode_throttle_limit(self, mode):
        try:
            canonical = DriveMode(mode).canonical
        except (ValueError, TypeError):
            canonical = mode
        if canonical in {DriveMode.MANUAL, DriveMode.RECORD}:
            return self.manual_maximum_throttle
        return self.maximum_throttle

    def evaluate(self, request: ControlRequest, context: SafetyContext):
        requested_throttle = max(-1.0, min(1.0, float(request.throttle)))
        requested_steering = max(-1.0, min(1.0, float(request.steering)))
        mode_policy = policy_for(context.mode)
        loop_delay_reason = self._loop_delay_reason(
            context.loop_delay_seconds,
            request.source,
            context.mode,
        )
        stop_reason = self._blocking_reason(
            request,
            context,
            mode_policy,
            loop_delay_reason=loop_delay_reason,
        )
        throttle_limit = self._mode_throttle_limit(context.mode)

        # Computing the nearest obstacle scans every LiDAR point. Do it only
        # when this mode actually enables the hard obstacle fallback and a
        # forward command can use the result. MANUAL/RECORD and learned AI modes
        # do not pay this cost on every control command.
        obstacle_distance_m = None
        if (
            stop_reason is None
            and requested_throttle > 0
            and mode_policy.obstacle_stop_fallback
        ):
            obstacle = self.obstacle_checker.check(context.lidar.value or [])
            obstacle_distance_m = obstacle.distance_m
            throttle_limit, stop_reason = self._obstacle_limit(obstacle_distance_m)
            if stop_reason == "OBSTACLE_STOP":
                self.obstacle_restart_guard.block(stop_reason)
            elif self.obstacle_restart_guard.reason is not None:
                if (
                    obstacle_distance_m is not None
                    and obstacle_distance_m <= self.crawl_distance_m
                ):
                    self.obstacle_restart_guard.block("OBSTACLE_STOP")
                if self.obstacle_restart_guard.remaining() > 0:
                    throttle_limit = 0.0
                    stop_reason = "OBSTACLE_RESTART_DELAY"

        final_throttle = 0.0
        final_steering = 0.0
        allowed = stop_reason is None
        if allowed:
            final_throttle = max(-throttle_limit, min(throttle_limit, requested_throttle))
            final_steering = requested_steering

        decision = SafetyDecision(
            requested_throttle=requested_throttle,
            requested_steering=requested_steering,
            final_throttle=final_throttle,
            final_steering=final_steering,
            allowed=allowed,
            stop_reason=stop_reason,
            obstacle_distance_m=obstacle_distance_m,
            throttle_limit=throttle_limit,
        )
        with self._lock:
            self._last_decision = decision
            self._last_evaluated_at = time.monotonic()
            self._last_input_source = request.source
            self._last_request_timestamp = request.timestamp
        return decision

    def _blocking_reason(self, request, context, mode_policy, loop_delay_reason=None):
        if context.emergency_stop or context.mode == DriveMode.EMERGENCY_STOP:
            return "EMERGENCY_STOP"
        if context.mode == DriveMode.FAULT:
            return "FAULT"

        # Person detection is an explicit external override only in autonomous
        # modes whose policy enables it. MANUAL/RECORD remain pure human control.
        if context.camera_hazard and mode_policy.person_stop:
            return "CAMERA_OBJECT_STOP"

        if not request.enabled or context.mode not in self.ACTIVE_MODES:
            return "DISARMED"

        if mode_policy.require_deadman and not request.deadman_pressed:
            return "DEADMAN_RELEASED"

        if time.monotonic() - request.timestamp > self.command_timeout_seconds:
            return "COMMAND_TIMEOUT"

        # Hard vehicle-safety checks stay active in every drive-capable mode.
        if not context.arduino.is_valid:
            return "ARDUINO_UNAVAILABLE"
        if not context.arduino.fresh(self.arduino_timeout_seconds):
            return "ARDUINO_TIMEOUT"
        if not context.steering.fresh(self.steering_timeout_seconds):
            return "STEERING_SENSOR_TIMEOUT"

        # LiDAR freshness is only a mandatory gate in navigation modes that
        # depend on obstacle/local-planning assistance.
        if mode_policy.require_lidar and not context.lidar.fresh(self.lidar_timeout_seconds):
            return "LIDAR_TIMEOUT"

        if loop_delay_reason is not None:
            return loop_delay_reason
        return None

    def _loop_delay_reason(self, delay_seconds, source, mode):
        """Reject persistent autonomous-loop stalls without faulting on one scheduler hiccup.

        Learned/local controllers target 10 Hz and run alongside camera, LiDAR,
        IMU, HTTP and recording threads on the Raspberry Pi. A single start-to-
        start period just above 200 ms can therefore occur even when inference is
        healthy. It is still unsafe to tolerate a genuinely stalled controller:
        a hard delay faults immediately, while repeated soft overruns fault after
        a short consecutive streak. Legacy/manual semantics remain immediate.
        """

        try:
            delay = max(0.0, float(delay_seconds or 0.0))
        except (TypeError, ValueError):
            delay = 0.0
        source = str(source or "unknown")
        try:
            canonical = DriveMode(mode).canonical
        except (TypeError, ValueError):
            canonical = mode
        autonomous = canonical in self.AUTONOMOUS_TIMING_MODES

        with self._lock:
            if self._loop_delay_source != source:
                self._loop_delay_source = source
                self._loop_delay_violation_count = 0
            self._last_loop_delay_seconds = delay

            if self.loop_delay_limit_seconds <= 0.0 or delay <= self.loop_delay_limit_seconds:
                self._loop_delay_violation_count = 0
                return None

            # A severe stall gets no grace. Keep this below the Arduino watchdog
            # horizon so the software and hardware fail-safe layers agree.
            if delay >= self.loop_delay_hard_limit_seconds:
                self._loop_delay_violation_count += 1
                return "CONTROL_LOOP_DELAY"

            if not autonomous:
                self._loop_delay_violation_count += 1
                return "CONTROL_LOOP_DELAY"

            self._loop_delay_violation_count += 1
            if (
                self._loop_delay_violation_count
                >= self.autonomous_loop_delay_consecutive_limit
            ):
                return "CONTROL_LOOP_DELAY"
            return None

    def _obstacle_limit(self, clearance_m):
        if clearance_m is None:
            return self.maximum_throttle, None
        if clearance_m < self.stop_distance_m:
            return 0.0, "OBSTACLE_STOP"
        if clearance_m < self.crawl_distance_m:
            return min(self.crawl_throttle, self.maximum_throttle), None
        if clearance_m < self.slow_distance_m:
            span = self.slow_distance_m - self.crawl_distance_m
            ratio = (clearance_m - self.crawl_distance_m) / span
            crawl = min(self.crawl_throttle, self.maximum_throttle)
            limit = crawl + ratio * (self.maximum_throttle - crawl)
            return limit, None
        return self.maximum_throttle, None

    def snapshot(self):
        with self._lock:
            return {
                **self._last_decision.as_dict(),
                "maximum_throttle": self.maximum_throttle,
                "manual_maximum_throttle": self.manual_maximum_throttle,
                "last_evaluated_at": self._last_evaluated_at,
                "input_source": self._last_input_source,
                "request_timestamp": self._last_request_timestamp,
                "loop_delay_seconds": self._last_loop_delay_seconds,
                "loop_delay_limit_seconds": self.loop_delay_limit_seconds,
                "loop_delay_hard_limit_seconds": self.loop_delay_hard_limit_seconds,
                "loop_delay_violation_count": self._loop_delay_violation_count,
                "autonomous_loop_delay_consecutive_limit": (
                    self.autonomous_loop_delay_consecutive_limit
                ),
            }
