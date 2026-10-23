"""Unit tests for the USB-camera WebRTC lifecycle."""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from kinisirobot import webrtc_camera  # noqa: E402


def test_missing_camera_is_reported_without_opening_it(tmp_path):
    camera = webrtc_camera.WebRTCCamera(device=str(tmp_path / "missing-video"))
    try:
        status = camera.status()
        assert status["available"] is False
        assert status["device"].endswith("missing-video")
        assert status["peers"] == 0
    finally:
        camera.shutdown()


def test_offer_shape_is_validated_before_negotiation(tmp_path, monkeypatch):
    device = tmp_path / "video0"
    device.touch()
    monkeypatch.setattr(webrtc_camera, "AIORTC_IMPORT_ERROR", None)
    monkeypatch.setattr(webrtc_camera, "MediaRelay", object)
    camera = webrtc_camera.WebRTCCamera(device=str(device))
    try:
        with pytest.raises(ValueError, match="type='offer'"):
            camera.create_answer({"type": "answer", "sdp": "test"})
    finally:
        camera.shutdown()


def test_stopping_player_releases_all_media_tracks():
    class Track:
        def __init__(self):
            self.stopped = False

        def stop(self):
            self.stopped = True

    class Player:
        audio = Track()
        video = Track()

    camera = object.__new__(webrtc_camera.WebRTCCamera)
    camera._player = Player()
    camera._stop_player()

    assert camera._player is None
    assert Player.audio.stopped is True
    assert Player.video.stopped is True
