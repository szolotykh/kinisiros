"""Regression tests for preserving saved SLAM scan geometry across sensor modes."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinisirobot.mapping_scan import normalize_mapping_scan


def scan(count=360):
    """Represent the Standard-mode A1 scan while keeping physical time explicit."""
    return NS(header=NS(stamp=123), angle_min=-math.pi + math.pi / 180,
              angle_max=math.pi, angle_increment=math.pi / 180,
              scan_time=.13, time_increment=.13 / (count-1),
              ranges=[float(i + 1) for i in range(count)], intensities=[2.] * count)


def test_standard_scan_matches_saved_1080_bin_geometry():
    """Historical beam angles must not be reinterpreted as 360-bin scans."""
    original = scan()
    result = normalize_mapping_scan(original, 1080)
    assert len(result.ranges) == len(result.intensities) == 1080
    assert result.angle_min == original.angle_min
    assert result.angle_max == original.angle_max
    assert result.angle_increment == pytest.approx(0.00580698, abs=1e-8)
    assert result.angle_min + 1079 * result.angle_increment == pytest.approx(result.angle_max)
    assert result.header.stamp == 123
    assert 1079 * result.time_increment == pytest.approx(original.scan_time)
    assert len(original.ranges) == 360
    assert result.ranges[0] == original.ranges[0]
    assert result.ranges[-1] == original.ranges[-1]


def test_missing_returns_are_not_interpolated_into_obstacles():
    """Keep an unknown sector unknown instead of inventing intermediate ranges."""
    original = scan()
    original.ranges[20:40] = [math.inf] * 20
    result = normalize_mapping_scan(original, 1080)
    assert all(math.isinf(v) for v in result.ranges[65:115])
    assert all(v in original.ranges for v in result.ranges)


def test_already_normalized_scan_keeps_values():
    """Applying the mapping layout again must not rotate or shift its beams."""
    first = normalize_mapping_scan(scan(), 1080)
    second = normalize_mapping_scan(first, 1080)
    assert second.ranges == first.ranges
    assert second.angle_increment == first.angle_increment


@pytest.mark.parametrize('field,value', [('angle_increment', -1.),
    ('angle_max', math.nan), ('ranges', [1.]), ('scan_time', 0.)])
def test_invalid_scan_layout_is_rejected(field, value):
    """Malformed data must never replace a known SLAM laser model."""
    original = scan()
    setattr(original, field, value)
    with pytest.raises(ValueError):
        normalize_mapping_scan(original, 1080)
