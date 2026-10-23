"""Tests for the localization confidence gate."""

import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from kinisirobot.map_localization import (  # noqa: E402
    MappingReadiness, localization_quality, seed_covariance, scan_alignment,
    GlobalLocalizationEvidence,
    PoseContinuity,
)


def test_pose_continuity_compares_with_window_anchor_not_only_previous_sample():
    evidence = PoseContinuity()
    assert evidence.observe((0, 0, 0), (0, 0, 0))
    assert evidence.observe((.15, 0, 0), (0, 0, 0))
    assert not evidence.observe((.3, 0, 0), (0, 0, 0))


def test_global_evidence_needs_consistent_different_views():
    """A turn must preserve the map-to-odom estimate, not just produce low covariance."""
    evidence = GlobalLocalizationEvidence()
    for _ in range(20):
        assert not evidence.observe((1,2,.3), (0,0,0))
    assert evidence.observe((1,2,.3+.25), (0,0,.25))
    # Jumping to another matching room discards the earlier evidence.
    assert not evidence.observe((7,8,.3-.25), (0,0,-.25))


def test_global_evidence_handles_angle_wrap():
    """Crossing -pi/pi during a small turn must not look like a hypothesis jump."""
    evidence = GlobalLocalizationEvidence()
    assert not evidence.observe((1,2,math.pi-.05), (0,0,math.pi-.05))
    assert evidence.observe((1,2,-math.pi+.2), (0,0,-math.pi+.2))


def covariance(x_variance, y_variance, yaw_variance):
    values = [0.0] * 36
    values[0] = x_variance
    values[7] = y_variance
    values[35] = yaw_variance
    return values


def test_accepts_low_uncertainty_pose():
    quality = localization_quality(
        covariance(0.01, 0.0225, math.radians(10.0) ** 2)
    )

    assert quality["confident"] is True
    assert quality["position_std_m"] == 0.15


def test_rejects_uncertain_position():
    quality = localization_quality(
        covariance(0.04, 0.36, math.radians(10.0) ** 2)
    )

    assert quality["confident"] is False
    assert quality["position_std_m"] == 0.6


def test_rejects_uncertain_heading():
    quality = localization_quality(
        covariance(0.04, 0.09, math.radians(30.0) ** 2)
    )

    assert quality["confident"] is False


def test_rejects_invalid_covariance():
    assert localization_quality([])["confident"] is False
    assert localization_quality(covariance(-1.0, 0.1, 0.1))["confident"] is False


def test_converged_amcl_roundoff_does_not_invalidate_pose():
    """Measured AMCL cancellation noise must not produce missing uncertainty."""
    quality = localization_quality(covariance(-1.733e-14, -1.113e-13, -7.1e-15))
    assert quality['confident']
    assert quality['position_std_m'] == quality['yaw_std_rad'] == 0.0
    assert not localization_quality(covariance(-1e-8, 0, 0))['confident']


def test_seed_spread_is_initial_uncertainty_not_a_success_condition():
    """A nearby initial pose starts with small covariance but still needs scan checks."""
    values = seed_covariance(0.30)
    assert values[0] == values[7] == 0.0225
    assert values[35] == math.radians(15.0) ** 2


def test_mapping_waits_for_new_pose_and_new_map():
    """Cached outputs and just a running process must not enable navigation."""
    readiness = MappingReadiness(100)
    readiness.observe("pose", 99)
    readiness.observe("map", 100)
    assert not readiness.ready
    readiness.observe("map", 101)
    assert not readiness.ready
    readiness.observe("pose", 102)
    assert readiness.ready


def room_scan(offset=0.0):
    """Create a square room with mapped walls and a full laser scan at its center."""
    size, resolution = 101, 0.05
    cells = [0] * (size * size)
    for n in range(size):
        cells[n] = cells[(size - 1) * size + n] = 100
        cells[n * size] = cells[n * size + size - 1] = 100
    increment = 2 * math.pi / 360
    ranges = [2.5 / max(abs(math.cos(i * increment)), abs(math.sin(i * increment)))
              for i in range(360)]
    return scan_alignment(ranges, 0.0, increment, 0.1, 12.0,
                          (offset, 0.0, 0.0), size, size, resolution,
                          (-2.5, -2.5, 0.0), cells)


def test_scan_agrees_at_correct_pose_and_rejects_displaced_pose():
    """Covariance alone cannot detect a confident estimate inside the wrong room."""
    assert room_scan()["scan_agrees"]
    assert not room_scan(1.0)["scan_agrees"]


def test_empty_or_unknown_scan_cannot_confirm_localization():
    quality = scan_alignment([float('inf')] * 90, 0, 0.1, 0.1, 10,
                             (0, 0, 0), 10, 10, 0.05, (0, 0, 0), [-1] * 100)
    assert not quality["scan_agrees"]
