"""Shared camera-mount homography used by rover preview and Worker UFLD.

World axes are +X rover-right, +Y rover-forward, +Z up.  OpenCV camera axes are
+x image-right, +y image-down, +z forward.  The transformation is a ground-plane
homography between the measured physical camera pose and a bounded virtual pose.
At perspective_strength=0 it is identity; at 1 it reaches the requested target
height/pitch while cancelling measured roll/yaw/lateral/longitudinal offsets.
"""

from __future__ import annotations

import math

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
    np = None


DEFAULT_CAMERA_MOUNT_PROFILE = {
    "schema": "SWING_CAMERA_MOUNT_V1",
    "height_m": 0.42,
    "pitch_degrees": -12.0,
    "roll_degrees": 0.0,
    "yaw_degrees": 0.0,
    "lateral_offset_m": 0.0,
    "longitudinal_offset_m": 0.0,
    "target_height_m": 1.20,
    "target_pitch_degrees": -4.0,
    "perspective_strength": 0.0,
}


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"INVALID_CAMERA_{name.upper()}") from error
    if not math.isfinite(number):
        raise ValueError(f"INVALID_CAMERA_{name.upper()}")
    return number


def normalize_camera_mount_profile(document=None):
    source = dict(DEFAULT_CAMERA_MOUNT_PROFILE)
    source.update(dict(document or {}))
    # Backward-compatible aliases from the rover camera-mount UI.
    aliases = {
        "pitch_deg": "pitch_degrees",
        "roll_deg": "roll_degrees",
        "yaw_deg": "yaw_degrees",
        "target_pitch_deg": "target_pitch_degrees",
    }
    for old, new in aliases.items():
        if old in source and new not in (document or {}):
            source[new] = source[old]
    result = {
        "schema": "SWING_CAMERA_MOUNT_V1",
        "height_m": _finite(source.get("height_m"), "height_m"),
        "pitch_degrees": _finite(source.get("pitch_degrees"), "pitch_degrees"),
        "roll_degrees": _finite(source.get("roll_degrees"), "roll_degrees"),
        "yaw_degrees": _finite(source.get("yaw_degrees"), "yaw_degrees"),
        "lateral_offset_m": _finite(source.get("lateral_offset_m"), "lateral_offset_m"),
        "longitudinal_offset_m": _finite(
            source.get("longitudinal_offset_m", 0.0), "longitudinal_offset_m"
        ),
        "target_height_m": _finite(source.get("target_height_m"), "target_height_m"),
        "target_pitch_degrees": _finite(
            source.get("target_pitch_degrees"), "target_pitch_degrees"
        ),
        "perspective_strength": _finite(
            source.get("perspective_strength"), "perspective_strength"
        ),
    }
    limits = {
        "height_m": (0.08, 2.50),
        "pitch_degrees": (-45.0, 20.0),
        "roll_degrees": (-20.0, 20.0),
        "yaw_degrees": (-30.0, 30.0),
        "lateral_offset_m": (-1.00, 1.00),
        "longitudinal_offset_m": (-2.00, 2.00),
        "target_height_m": (0.15, 2.50),
        "target_pitch_degrees": (-30.0, 15.0),
        "perspective_strength": (0.0, 1.0),
    }
    for key, (minimum, maximum) in limits.items():
        if not minimum <= result[key] <= maximum:
            raise ValueError(f"CAMERA_{key.upper()}_OUT_OF_RANGE:{minimum}..{maximum}")
    return result


def _scaled_camera_matrix(camera_matrix, image_size, calibration_size=None):
    if np is None:
        return None
    width, height = image_size
    if camera_matrix is None:
        hfov = 70.0
        focal = width / (2.0 * math.tan(math.radians(hfov) / 2.0))
        return np.asarray(
            [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3).copy()
    if calibration_size and len(calibration_size) == 2:
        source_width = max(1.0, float(calibration_size[0]))
        source_height = max(1.0, float(calibration_size[1]))
        matrix[0, 0] *= width / source_width
        matrix[0, 2] *= width / source_width
        matrix[1, 1] *= height / source_height
        matrix[1, 2] *= height / source_height
    return matrix


def _rx(degrees):
    angle = math.radians(float(degrees))
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rz(degrees):
    angle = math.radians(float(degrees))
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _camera_to_world(profile):
    base = np.asarray(
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    return (
        _rz(-profile["yaw_degrees"])
        @ base
        @ _rx(profile["pitch_degrees"])
        @ _rz(profile["roll_degrees"])
    )


def _ground_projection(matrix, profile):
    c2w = _camera_to_world(profile)
    w2c = c2w.T
    center = np.asarray(
        [
            profile["lateral_offset_m"],
            profile["longitudinal_offset_m"],
            profile["height_m"],
        ],
        dtype=np.float64,
    )
    translation = -w2c @ center
    # Ground plane Z=0, parameterized by world X/Y plus homogeneous 1.
    plane = np.column_stack((w2c[:, 0], w2c[:, 1], translation))
    return matrix @ plane


def _virtual_profile(profile):
    strength = float(profile["perspective_strength"])
    return {
        **profile,
        "height_m": profile["height_m"]
        + (profile["target_height_m"] - profile["height_m"]) * strength,
        "pitch_degrees": profile["pitch_degrees"]
        + (profile["target_pitch_degrees"] - profile["pitch_degrees"]) * strength,
        "roll_degrees": profile["roll_degrees"] * (1.0 - strength),
        "yaw_degrees": profile["yaw_degrees"] * (1.0 - strength),
        "lateral_offset_m": profile["lateral_offset_m"] * (1.0 - strength),
        "longitudinal_offset_m": profile["longitudinal_offset_m"] * (1.0 - strength),
    }


def virtual_camera_homography(image_size, profile, camera_matrix=None, calibration_size=None):
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV/NumPy unavailable")
    width, height = [int(value) for value in image_size]
    if width <= 1 or height <= 1:
        raise ValueError("INVALID_CAMERA_IMAGE_SIZE")
    profile = normalize_camera_mount_profile(profile)
    matrix = _scaled_camera_matrix(camera_matrix, (width, height), calibration_size)
    if profile["perspective_strength"] <= 1e-9:
        identity = np.eye(3, dtype=np.float64)
        return identity, identity.copy()
    actual_projection = _ground_projection(matrix, profile)
    virtual_projection = _ground_projection(matrix, _virtual_profile(profile))
    try:
        determinant = float(np.linalg.det(actual_projection))
        if not math.isfinite(determinant) or abs(determinant) < 1e-9:
            raise ValueError("CAMERA_GROUND_PROJECTION_SINGULAR")
        homography = virtual_projection @ np.linalg.inv(actual_projection)
        homography /= homography[2, 2]
        inverse = np.linalg.inv(homography)
        inverse /= inverse[2, 2]
    except np.linalg.LinAlgError as error:
        raise ValueError("CAMERA_GROUND_PROJECTION_SINGULAR") from error
    if not np.all(np.isfinite(homography)) or not np.all(np.isfinite(inverse)):
        raise ValueError("CAMERA_HOMOGRAPHY_NONFINITE")
    return homography, inverse


def warp_virtual_camera(image, profile, camera_matrix=None, calibration_size=None):
    if image is None:
        raise ValueError("CAMERA_IMAGE_UNAVAILABLE")
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV/NumPy unavailable")
    height, width = image.shape[:2]
    matrix, inverse = virtual_camera_homography(
        (width, height), profile, camera_matrix, calibration_size
    )
    warped = cv2.warpPerspective(
        image,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    return warped, matrix, inverse


def project_points(points, homography):
    if cv2 is None or np is None or not points:
        return []
    array = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(array, np.asarray(homography, dtype=np.float64))
    return [[float(x), float(y)] for x, y in transformed.reshape(-1, 2)]


__all__ = [
    "DEFAULT_CAMERA_MOUNT_PROFILE",
    "normalize_camera_mount_profile",
    "project_points",
    "virtual_camera_homography",
    "warp_virtual_camera",
]
