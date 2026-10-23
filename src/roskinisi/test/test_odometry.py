# File: test_odometry.py
"""Test acquisition-time velocity estimates independently of ROS and hardware."""

import math

from pykinisi import PlatformOdometrySample
import pytest

from roskinisi.odometry import OdometryTracker


def sample(timestamp=1_789_365_945_599_946, x=0.0, y=0.0, yaw=0.0, mode=1, quality=1):
    """Build a sample using the actual SDK payload type."""
    return PlatformOdometrySample(timestamp, mode, quality, x, y, yaw)


def test_measurement_intervals_and_body_frame():
    """Irregular sample spacing determines velocity, independent of callback timing."""
    tracker = OdometryTracker()
    first = sample(yaw=math.pi / 2)
    assert tracker.update(first) == (0.0, 0.0, 0.0)
    second = sample(first.timestamp_us + 250_000, x=0.5, yaw=math.pi / 2)
    assert tracker.update(second) == pytest.approx((0.0, -2.0, 0.0))


def test_duplicate_and_out_of_order_samples_do_not_change_history():
    """Cached and delayed frames cannot create duplicate TF or corrupt the baseline."""
    tracker = OdometryTracker()
    first = sample()
    tracker.update(first)
    assert tracker.update(first) is None
    assert tracker.update(sample(first.timestamp_us - 1, x=100.0)) is None
    assert tracker.update(sample(first.timestamp_us + 100_000, x=0.1)) == (1.0, 0.0, 0.0)


def test_yaw_wrap_and_reset():
    """Wrap yaw differences and suppress artificial velocity after resetting pose."""
    tracker = OdometryTracker()
    first = sample(yaw=math.pi - 0.01)
    tracker.update(first)
    twist = tracker.update(sample(first.timestamp_us + 100_000, yaw=-math.pi + 0.01))
    assert twist[2] == pytest.approx(0.2)
    tracker.reset()
    assert tracker.update(sample(first.timestamp_us + 200_000, x=100.0)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'mode': 0}, {'quality': 0}, {'quality': 99}, {'timestamp': 0},
    {'timestamp': (1 << 31) * 1_000_000}, {'x': math.nan}, {'yaw': math.inf},
])
def test_invalid_samples_are_rejected(kwargs):
    """Never turn unready timestamps or non-finite values into ROS measurements."""
    with pytest.raises(ValueError):
        OdometryTracker().update(sample(**kwargs))


def test_stale_mapping_is_usable():
    """A previously synchronized clock can continue supplying acquisition times."""
    assert OdometryTracker().update(sample(quality=2)) == (0.0, 0.0, 0.0)
