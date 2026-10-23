# File: odometry.py
"""Convert completed controller samples into timestamped body-frame velocities."""

import math

from pykinisi import ClockMode, ClockQuality


class OdometryTracker:
    """Track measurement time independently of callback and serial delivery timing."""

    def __init__(self):
        """Start without a previous measurement."""
        self.previous = None

    def reset(self):
        """Forget pose history after a controller odometry reset."""
        self.previous = None

    def update(self, sample):
        """Return a body-frame twist, or None for duplicate/out-of-order samples."""
        if sample.clock_mode != ClockMode.WALL:
            raise ValueError('ROS odometry requires synchronized Unix timestamps')
        if sample.clock_quality not in (ClockQuality.VALID, ClockQuality.STALE):
            raise ValueError('Odometry clock is not ready')
        if not 0 < sample.timestamp_us < (1 << 31) * 1_000_000:
            raise ValueError('Odometry timestamp is outside the ROS Time range')
        if not all(math.isfinite(value) for value in (sample.x, sample.y, sample.t)):
            raise ValueError('Odometry contains a non-finite pose')
        current = (sample.x, sample.y, sample.t, sample.timestamp_us)
        vx = vy = wz = 0.0
        if self.previous is not None:
            x, y, yaw, timestamp_us = self.previous
            elapsed_us = sample.timestamp_us - timestamp_us
            if elapsed_us <= 0:
                return None
            dt = elapsed_us / 1_000_000
            dx = (sample.x - x) / dt
            dy = (sample.y - y) / dt
            c, s = math.cos(sample.t), math.sin(sample.t)
            vx, vy = dx * c + dy * s, -dx * s + dy * c
            delta = sample.t - yaw
            wz = math.atan2(math.sin(delta), math.cos(delta)) / dt
        self.previous = current
        return vx, vy, wz
