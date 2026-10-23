"""Robot-independent sensor diagnostics and localization geometry checks."""

import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinisirobot.localization_config import LocalizationConfig, sensor_health
from kinisirobot.localization_motion import rotation_clearance
from test_localization_motion import scan


@pytest.mark.parametrize('stamp,frame,state', [
    (10, 'odom', 'ready'), (9, 'odom', 'stale'),
    (11, 'odom', 'future_timestamp'), (10, 'map', 'wrong_frame'),
])
def test_sensor_health_explains_individual_failures(stamp, frame, state):
    """Keep sensor age and coordinate errors visible to both callers and the UI."""
    message = NS(header=NS(stamp=NS(sec=stamp, nanosec=0), frame_id=frame))
    assert sensor_health(message, 10_000_000_000, .3, 'odom')['state'] == state
    assert sensor_health(None, 10_000_000_000, .3)['state'] == 'missing'


def test_geometry_can_be_reused_for_a_different_chassis():
    """The same scan has different clearance for differently sized robots."""
    laser = scan([.5] * 360)
    assert rotation_clearance(laser, (0, 0, 0))
    assert not rotation_clearance(laser, (0, 0, 0), LocalizationConfig(body_radius_m=.5))


@pytest.mark.parametrize('values', [dict(body_radius_m=-1), dict(scan_margin_m=-1),
    dict(odom_max_age_s=0), dict(settle_s=float('nan')), dict(base_frame='')])
def test_invalid_profiles_cannot_disable_guards(values):
    """Bad configuration fails at startup, before movement is considered."""
    with pytest.raises(ValueError):
        LocalizationConfig(**values)
