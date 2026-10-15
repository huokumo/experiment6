"""Coordinate utilities shared by perception and tests (no ROS dependency)."""
import math
import numpy as np


def quaternion_matrix(x, y, z, w):
    q = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def yaw_of(x, y, z, w):
    rotation = quaternion_matrix(x, y, z, w)
    return math.atan2(rotation[1, 0], rotation[0, 0])


def clamp_velocity(vx, vy, omega, max_linear, max_angular):
    if not all(math.isfinite(v) for v in (vx, vy, omega)):
        return 0.0, 0.0, 0.0
    if max_linear <= 0 or max_angular <= 0:
        raise ValueError('Velocity limits must be positive')
    magnitude = math.hypot(vx, vy)
    scale = min(1.0, max_linear / magnitude) if magnitude else 1.0
    return vx * scale, vy * scale, max(-max_angular, min(max_angular, omega))
