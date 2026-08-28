from .pure_pursuit import PathPoint, PurePursuit, PursuitResult
from .lane_controller import LaneController, LaneResult
from .lane_candidate_hardening import install_lane_candidate_hardening
from .lane_prior_tracking_hardening import install_lane_prior_tracking_hardening
from .lane_geometry_hardening import install_lane_geometry_hardening
from .lane_pair_hardening import install_lane_pair_hardening
from .throttle_controller import ThrottleCalibration

# Keep one authoritative LaneController API while hardening its runtime geometry:
# 1) prefer BLACK/YELLOW/WHITE lane evidence over unrelated generic edges,
# 2) follow the last accepted boundary before bootstrapping from a new histogram,
# 3) calibrated/raw coordinate consistency + adaptive line/curve fitting,
# 4) pair-wise rejection when only one boundary jumps to an unrelated edge.
install_lane_candidate_hardening()
install_lane_prior_tracking_hardening()
install_lane_geometry_hardening()
install_lane_pair_hardening()

__all__ = [
    "LaneController",
    "LaneResult",
    "PathPoint",
    "PurePursuit",
    "PursuitResult",
    "ThrottleCalibration",
]
