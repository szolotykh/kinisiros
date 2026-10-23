"""Shared robot geometry, frames and freshness limits for localization."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LocalizationConfig:
    """Defaults for the Omni chassis; inject another profile for another robot."""

    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    body_radius_m: float = 0.30
    costmap_margin_m: float = 0.05
    scan_margin_m: float = 0.15
    odom_max_age_s: float = 0.30
    scan_max_age_s: float = 0.40
    costmap_max_age_s: float = 1.50
    stationary_linear_mps: float = 0.01
    stationary_angular_rps: float = 0.02
    settle_s: float = 1.0
    # Existing Omni pose graphs were recorded with this angular sample count.
    mapping_scan_samples: int = 1080


    def __post_init__(self):
        """Reject profiles that would weaken checks through invalid dimensions or ages."""
        if type(self.mapping_scan_samples) is not int or not 2 <= self.mapping_scan_samples <= 10000:
            raise ValueError("mapping_scan_samples must be an integer in 2..10000")
        for name in ('map_frame', 'odom_frame', 'base_frame'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a nonempty frame name")
        for name in ('body_radius_m', 'odom_max_age_s', 'scan_max_age_s',
                     'costmap_max_age_s', 'stationary_linear_mps',
                     'stationary_angular_rps', 'settle_s'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ('costmap_margin_m', 'scan_margin_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


DEFAULT_LOCALIZATION_CONFIG = LocalizationConfig()


def sensor_health(message, now_ns, max_age_s, expected_frame=None):
    """Describe missing, stale, future-dated or incorrectly framed ROS-like data."""
    if message is None:
        return {"state": "missing", "age_s": None}
    header = message.header
    stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
    age = (now_ns - stamp) / 1e9
    state = "ready"
    if expected_frame is not None and header.frame_id != expected_frame:
        state = "wrong_frame"
    elif age < 0:
        state = "future_timestamp"
    elif age > max_age_s:
        state = "stale"
    return {"state": state, "age_s": age, "frame": header.frame_id}
