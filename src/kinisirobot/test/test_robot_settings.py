"""Persistence, map identity and failure handling for remembered robot poses."""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinisirobot.robot_settings import RobotSettings, map_identity  # noqa: E402


def test_preferences_and_per_map_pose_survive_restart(tmp_path):
    settings_file = tmp_path / 'config' / 'settings.json'
    settings = RobotSettings(settings_file)
    settings.set_radius(0.2)
    settings.remember_pose('map-a', 'office', (1.0, -2.0, 0.7))
    reloaded = RobotSettings(settings_file)
    assert reloaded.snapshot('map-a')['last_pose']['x'] == 1.0
    assert reloaded.snapshot()['localization_radius_m'] == 0.2
    assert reloaded.snapshot('map-b')['last_pose'] is None


@pytest.mark.parametrize('radius', [0, -1, 1, float('nan'), True, '0.3'])
def test_invalid_radius_does_not_change_file(tmp_path, radius):
    settings = RobotSettings(tmp_path / 'settings.json')
    settings.set_radius(0.3)
    before = settings.path.read_bytes()
    with pytest.raises(ValueError):
        settings.set_radius(radius)
    assert settings.path.read_bytes() == before


def test_invalid_pose_is_not_persisted(tmp_path):
    settings = RobotSettings(tmp_path / 'settings.json')
    with pytest.raises(ValueError):
        settings.remember_pose('map', 'office', (float('nan'), 0, 0))
    assert not settings.path.exists()


def test_reused_filename_with_different_map_does_not_reuse_pose(tmp_path):
    base = tmp_path / 'office'
    for extension in ('.yaml', '.pgm', '.posegraph', '.data'):
        Path(str(base) + extension).write_bytes(extension.encode())
    first = map_identity(base)
    settings = RobotSettings(tmp_path / 'settings.json')
    settings.remember_pose(first, 'office', (1, 2, 3))
    base.with_suffix('.posegraph').write_bytes(b'new map')
    assert settings.snapshot(map_identity(base))['last_pose'] is None


def test_corrupt_settings_are_not_silently_overwritten(tmp_path):
    path = tmp_path / 'settings.json'
    path.write_text('{')
    with pytest.raises(json.JSONDecodeError):
        RobotSettings(path)
    assert path.read_text() == '{'
