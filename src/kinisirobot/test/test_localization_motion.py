"""Test movement limits and clearance without driving a physical robot."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinisirobot.localization_motion import (
    LocalizationMotion, costmap_rotation_clearance, rotation_clearance)


def step(motion, now, pose=(0, 0, 0), **overrides):
    """Tick a connected controller with fresh, unobstructed sensors."""
    guards = dict(sensors_fresh=True, clear=True, connected=True)
    guards.update(overrides)
    return motion.step(now, pose, **guards)


def test_turn_sequence_pauses_and_returns_without_translation():
    motion = LocalizationMotion(0, (0, 0, 0))
    yaw = 0.0
    history = []
    for i in range(1, 501):
        az = step(motion, i * 0.05, (0, 0, yaw))
        yaw += az * 0.05
        history.append((yaw, az))
        assert abs(az) <= 0.12
        if not motion.active:
            break
    assert not motion.active
    assert motion.reason.startswith('Small turns complete')
    assert abs(yaw) < math.radians(1.1)
    assert max(y for y, _ in history) > math.radians(13)
    assert min(y for y, _ in history) < math.radians(-13)
    assert sum(az == 0 for _, az in history) >= 100


@pytest.mark.parametrize('guard', ['connected', 'clear', 'sensors_fresh'])
def test_guard_failure_stops_and_does_not_restart(guard):
    motion = LocalizationMotion(0, (0, 0, 0))
    assert step(motion, 0.05) > 0
    assert step(motion, 0.1, **{guard: False}) == 0
    assert not motion.active
    assert step(motion, 0.15) == 0


@pytest.mark.parametrize('pose', [(0.051, 0, 0), (0, 0, math.radians(21)),
                                  (math.nan, 0, 0)])
def test_drift_yaw_and_invalid_pose_stop(pose):
    motion = LocalizationMotion(0, (0, 0, 0))
    assert step(motion, 0.05, pose) == 0
    assert not motion.active


def test_executor_delay_stops():
    motion = LocalizationMotion(0, (0, 0, 0))
    assert step(motion, 0.3) == 0
    assert 'timing' in motion.reason


def test_stuck_robot_cannot_turn_forever():
    motion = LocalizationMotion(0, (0, 0, 0))
    for i in range(1, 502):
        step(motion, i * 0.05)
    assert not motion.active
    assert 'time limit' in motion.reason


def test_heading_wrap_is_not_a_large_turn():
    motion = LocalizationMotion(0, (0, 0, math.pi - 0.01))
    assert step(motion, 0.05, (0, 0, -math.pi + 0.01)) > 0


def scan(ranges):
    """Create a full-circle scan matching the robot's laser metadata."""
    return SimpleNamespace(ranges=ranges, angle_min=0,
                           angle_increment=math.tau / 360,
                           range_min=0.1, range_max=10)


def test_clearance_accounts_for_offset_laser():
    readings = [2.0] * 360
    assert rotation_clearance(scan(readings), (0.115, 0, math.pi))
    # 50 cm from the rear-facing laser is inside the 45 cm base envelope.
    readings[0] = 0.5
    assert not rotation_clearance(scan(readings), (0.115, 0, math.pi))


def test_unknown_sector_is_not_clearance():
    readings = [2.0] * 360
    readings[40:70] = [math.inf] * 30
    assert not rotation_clearance(scan(readings), (0, 0, 0))
    assert not rotation_clearance(scan([math.nan] * 360), (0, 0, 0))


def costmap():
    """Build a free 3 m local costmap centered on the odometry origin."""
    return SimpleNamespace(info=SimpleNamespace(
        width=60, height=60, resolution=.05,
        origin=SimpleNamespace(position=SimpleNamespace(x=-1.5, y=-1.5),
                               orientation=SimpleNamespace(x=0, y=0, z=0, w=1))),
        data=[0] * 3600)


@pytest.mark.parametrize('cost', [-1, 65, 99, 100])
def test_costmap_obstacles_and_unknown_space_block_turns(cost):
    grid = costmap()
    assert costmap_rotation_clearance(grid, (0, 0, 0))
    grid.data[30*60+34] = cost  # Inside swept chassis, not just the center cell.
    assert not costmap_rotation_clearance(grid, (0, 0, 0))


def test_costmap_checks_bounds_and_rotated_origin():
    grid = costmap()
    assert not costmap_rotation_clearance(grid, (1.3, 0, 0))
    grid.info.origin.position.x = 1.5
    grid.info.origin.orientation.z = math.sin(math.pi/4)
    grid.info.origin.orientation.w = math.cos(math.pi/4)
    assert costmap_rotation_clearance(grid, (0, 0, 0))
    grid.data = []
    assert not costmap_rotation_clearance(grid, (0, 0, 0))
