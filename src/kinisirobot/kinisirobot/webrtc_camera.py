#!/usr/bin/env python3
"""WebRTC video source for a Linux USB camera.

The public methods are synchronous so the existing ThreadingHTTPServer can use
the camera while aiortc runs on its own asyncio event loop.
"""

import asyncio
import os
import threading
import uuid

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription
    from aiortc.contrib.media import MediaPlayer, MediaRelay
except ImportError as exc:  # Keep the robot UI usable before camera deps exist.
    RTCPeerConnection = None
    RTCSessionDescription = None
    MediaPlayer = None
    MediaRelay = None
    AIORTC_IMPORT_ERROR = exc
else:
    AIORTC_IMPORT_ERROR = None


class CameraUnavailable(RuntimeError):
    """Raised when WebRTC dependencies or the configured camera are missing."""


class WebRTCCamera:
    """Share one V4L2 camera between browser WebRTC peer connections."""

    def __init__(self, device="/dev/video0", width=640, height=480, fps=20):
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self._loop = None
        self._thread = None
        self._relay = None
        self._player = None
        self._peers = {}
        self._peer_count = 0

        if AIORTC_IMPORT_ERROR is None:
            self._loop = asyncio.new_event_loop()
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()

    def _run_loop(self):
        """Own the asyncio loop used by aiortc and PyAV."""
        asyncio.set_event_loop(self._loop)
        self._relay = MediaRelay()
        self._loop.run_forever()

    def status(self):
        """Return camera availability information for the browser UI."""
        if AIORTC_IMPORT_ERROR is not None:
            return {
                "available": False,
                "error": "WebRTC dependencies are not installed",
                "device": self.device,
                "peers": 0,
            }
        if not self.device or not os.path.exists(self.device):
            return {
                "available": False,
                "error": f"Camera device not found: {self.device}",
                "device": self.device,
                "peers": self._peer_count,
            }
        return {
            "available": True,
            "device": self.device,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "peers": self._peer_count,
        }

    def _require_available(self):
        status = self.status()
        if not status["available"]:
            raise CameraUnavailable(status["error"])

    def _submit(self, coroutine, timeout=20):
        """Run an aiortc coroutine from an HTTP worker thread."""
        if self._loop is None:
            raise CameraUnavailable("WebRTC dependencies are not installed")
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)
        return future.result(timeout=timeout)

    def create_answer(self, offer):
        """Create an SDP answer for a browser offer and return its peer ID."""
        self._require_available()
        if not isinstance(offer, dict):
            raise ValueError("offer must be a JSON object")
        if offer.get("type") != "offer" or not isinstance(offer.get("sdp"), str):
            raise ValueError("offer requires type='offer' and an SDP string")
        return self._submit(self._create_answer(offer))

    async def _create_answer(self, offer):
        """Open the camera lazily and negotiate one receive-only video peer."""
        if self._player is None:
            options = {
                "video_size": f"{self.width}x{self.height}",
                "framerate": str(self.fps),
            }
            try:
                self._player = MediaPlayer(
                    self.device, format="v4l2", options=options
                )
            except Exception as exc:
                raise CameraUnavailable(f"Could not open {self.device}: {exc}") from exc
        if self._player.video is None:
            self._stop_player()
            raise CameraUnavailable(f"No video stream found on {self.device}")

        peer_id = uuid.uuid4().hex
        peer = RTCPeerConnection()
        self._peers[peer_id] = peer
        self._peer_count = len(self._peers)

        @peer.on("connectionstatechange")
        async def on_connectionstatechange():
            if peer.connectionState in ("failed", "closed"):
                await self._remove_peer(peer_id)

        try:
            peer.addTrack(self._relay.subscribe(self._player.video))
            await peer.setRemoteDescription(
                RTCSessionDescription(sdp=offer["sdp"], type=offer["type"])
            )
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
        except Exception:
            await self._remove_peer(peer_id)
            raise

        return {
            "peer_id": peer_id,
            "sdp": peer.localDescription.sdp,
            "type": peer.localDescription.type,
        }

    def close_peer(self, peer_id):
        """Close a browser peer and release the camera when it is the last one."""
        if not peer_id or self._loop is None:
            return False
        return self._submit(self._remove_peer(str(peer_id)), timeout=10)

    async def _remove_peer(self, peer_id):
        peer = self._peers.pop(peer_id, None)
        self._peer_count = len(self._peers)
        if peer is not None and peer.connectionState != "closed":
            await peer.close()
        if not self._peers:
            self._stop_player()
        return peer is not None

    def _stop_player(self):
        """Stop PyAV's capture thread and release the V4L2 device."""
        player = self._player
        self._player = None
        if player is None:
            return
        for track in (player.audio, player.video):
            if track is not None:
                track.stop()

    def shutdown(self):
        """Close every peer and stop the asyncio worker."""
        if self._loop is None:
            return
        try:
            self._submit(self._shutdown(), timeout=10)
        finally:
            self._loop.call_soon_threadsafe(self._loop.stop)
            if self._thread is not None:
                self._thread.join(timeout=5)

    async def _shutdown(self):
        for peer_id in list(self._peers):
            await self._remove_peer(peer_id)
        self._stop_player()
