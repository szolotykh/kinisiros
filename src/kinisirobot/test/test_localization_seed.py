"""Check bounded pose refinement without mistaking a seed for confirmed localization."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinisirobot.localization_seed import refine_seed


def room():
    """Return laser observations and a rectangular room with known wall positions."""
    width, height, resolution = 101, 81, .05
    cells = [0] * (width * height)
    for row in range(height):
        for col in range(width):
            if row in (0, height-1) or col in (0, width-1):
                cells[row*width+col] = 100
    grid = NS(info=NS(width=width, height=height, resolution=resolution,
        origin=NS(position=NS(x=-2.5,y=-2.0), orientation=NS(x=0,y=0,z=0,w=1))), data=cells)
    increment = math.tau / 360
    ranges = [min(2.5/max(abs(math.cos(i*increment)),1e-9),
                  2.0/max(abs(math.sin(i*increment)),1e-9)) for i in range(360)]
    scan = NS(ranges=ranges,angle_min=0,angle_increment=increment,range_min=.1,range_max=12.)
    return scan, grid


def test_refinement_recovers_a_nearby_seed_without_leaving_search_radius():
    """A shifted prior should improve from measured walls within the allowed spread."""
    scan, grid = room()
    seed = (.25, .1, math.radians(10))
    result = refine_seed(scan, grid, seed, (0,0,0), .3)
    assert result['scan_match'] > result['original_match'] + .1
    assert result['scan_match'] > .9
    assert math.hypot(result['pose'][0]-seed[0],result['pose'][1]-seed[1]) <= .3 + 1e-9
    assert abs(result['pose'][2]-seed[2]) <= math.radians(20)


def test_correct_seed_is_not_shifted_by_an_equally_good_match():
    """The broad wall tolerance must not move an already good remembered pose."""
    scan, grid = room()
    assert refine_seed(scan,grid,(0,0,0),(0,0,0),.3)['pose'] == (0,0,0)


def test_unknown_readings_never_supply_a_seed():
    """Missing returns and sparse evidence are not a reason to move the prior."""
    scan, grid = room()
    scan.ranges = [math.inf] * 360
    assert refine_seed(scan, grid, (0,0,0),(0,0,0),.3) is None


def test_laser_offset_is_applied_in_the_base_frame():
    """A displaced laser should produce the same match as its equivalent base pose."""
    scan, grid = room()
    result = refine_seed(scan, grid, (-.1,0,0), (.1,0,0), .3)
    assert result['pose'] == (-.1,0,0)
    assert result['scan_match'] > .95
