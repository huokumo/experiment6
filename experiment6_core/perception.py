"""Depth decoding, projection and conservative sector summaries."""
from dataclasses import dataclass
import math
import numpy as np

UNKNOWN, CLEAR, BLOCKED = 0, 1, 2


def decode_depth(data, width, height, step, encoding, bigendian=False):
    if width <= 0 or height <= 0:
        raise ValueError('Depth dimensions must be positive')
    if encoding == '32FC1':
        dtype = np.dtype('>f4' if bigendian else '<f4')
        scale = 1.0
    elif encoding == '16UC1':
        dtype = np.dtype('>u2' if bigendian else '<u2')
        scale = 0.001
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    if step < width * dtype.itemsize or len(data) < step * height:
        raise ValueError('Depth buffer or row stride is invalid')
    raw = np.ndarray((height, width), dtype=dtype, buffer=data,
                     strides=(step, dtype.itemsize))
    return raw.astype(np.float32) * scale


def pixel_rays(width, height, k, d=(), stride=2):
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(d, dtype=np.float64)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.isfinite(d).all():
        raise ValueError('Invalid depth CameraInfo intrinsics')
    if d.size not in (0,4,5):
        raise ValueError('Expected standard plumb_bob distortion coefficients')
    if stride < 1:
        raise ValueError('Pixel stride must be positive')
    vv, uu = np.mgrid[0:height:stride, 0:width:stride]
    if d.size and np.any(np.abs(d) > 1e-12):
        import cv2
        pixels = np.stack((uu, vv), axis=-1).astype(np.float64).reshape(-1, 1, 2)
        normalized = cv2.undistortPoints(pixels, k, d).reshape(-1, 2)
        return np.column_stack((normalized, np.ones(len(normalized))))
    return np.column_stack(((uu.ravel()-k[0, 2])/k[0, 0],
                            (vv.ravel()-k[1, 2])/k[1, 1], np.ones(uu.size)))


@dataclass(frozen=True)
class PerceptionConfig:
    min_depth: float = 0.1
    max_depth: float = 10.0
    max_range: float = 4.0
    min_height: float = 0.06
    max_height: float = 1.5
    self_radius: float = 0.30
    front_half_angle_deg: float = 10.0
    max_bearing_deg: float = 60.0
    min_valid_ratio: float = 0.35
    min_sector_pixels: int = 20
    blocked_distance: float = 0.85

    def validate(self):
        if not all(math.isfinite(float(v)) for v in self.__dict__.values()):
            raise ValueError('Perception parameters must be finite')
        if not (0 < self.min_depth < self.max_depth and self.max_range > 0 and
                0 <= self.min_height < self.max_height and self.self_radius >= 0 and
                0 < self.front_half_angle_deg < self.max_bearing_deg < 90 and
                0 <= self.min_valid_ratio <= 1 and self.min_sector_pixels > 0 and
                self.self_radius < self.blocked_distance <= self.max_range):
            raise ValueError('Invalid perception parameters')


def sector_ids(bearing, config):
    half = math.radians(config.front_half_angle_deg)
    ids = np.full(len(bearing), -1, dtype=np.int8)
    view = np.abs(bearing) <= math.radians(config.max_bearing_deg)
    ids[view & (bearing > half)] = 0       # robot left = +Y
    ids[view & (np.abs(bearing) <= half)] = 1
    ids[view & (bearing < -half)] = 2
    return ids


def extract_obstacles(depths, rays, rotation, translation, config):
    """Return base-frame obstacle points, nearest ranges, validity and states.

    Coverage denominators use all sampled rays, including invalid depths.
    No point is fabricated for an invalid sample. CLEAR describes observed
    sectors only; it does not certify an unseen corridor or space behind hits.
    """
    config.validate()
    depths = np.asarray(depths, dtype=np.float64).reshape(-1)
    rays = np.asarray(rays, dtype=np.float64).reshape(-1, 3)
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    if len(depths) != len(rays):
        raise ValueError('Depth samples and rays do not match')
    if not np.isfinite(rays).all() or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError('Nonfinite camera transform or rays')
    directions = rays @ rotation.T
    ray_bearing = np.arctan2(directions[:, 1], directions[:, 0])
    expected_sector = sector_ids(ray_bearing, config)
    valid = np.isfinite(depths) & (depths >= config.min_depth) & (depths <= config.max_depth)
    points = directions[valid] * depths[valid, None] + translation
    ranges = np.hypot(points[:, 0], points[:, 1])
    keep = ((points[:, 0] > 0) & (ranges > config.self_radius) &
            (ranges <= config.max_range) & (points[:, 2] >= config.min_height) &
            (points[:, 2] <= config.max_height))
    points = points[keep]
    ranges = ranges[keep]
    point_sector = sector_ids(np.arctan2(points[:, 1], points[:, 0]), config)
    keep = point_sector >= 0
    points, ranges, point_sector = points[keep], ranges[keep], point_sector[keep]
    distances = np.full(3, np.nan)
    ratios = np.zeros(3, dtype=np.float64)
    states = np.full(3, UNKNOWN, dtype=np.uint8)
    for index in range(3):
        sector = expected_sector == index
        count = int(np.count_nonzero(sector))
        ratios[index] = float(np.count_nonzero(valid & sector)) / count if count else 0.0
        observed = count >= config.min_sector_pixels and ratios[index] >= config.min_valid_ratio
        hits = ranges[point_sector == index]
        # Report a close obstacle even when most other pixels are invalid.
        if len(hits) and float(hits.min()) <= config.blocked_distance:
            distances[index] = float(hits.min())
            states[index] = BLOCKED
        elif observed:
            distances[index] = float(hits.min()) if len(hits) else config.max_range
            states[index] = CLEAR
    return points.astype(np.float32), distances, ratios, states
