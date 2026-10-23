"""Persistent robot preferences and last known poses, keyed by saved map."""

import copy
import hashlib
import json
import math
import os
import tempfile
import threading
import time
from pathlib import Path


def map_identity(base):
    """Identify map content so reused filenames cannot select an unrelated pose."""
    digest = hashlib.sha256()
    for extension in (".yaml", ".pgm", ".posegraph", ".data"):
        digest.update(extension.encode())
        with open(str(base) + extension, "rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


class RobotSettings:
    """Atomically persist settings and validated last known map-frame poses."""

    def __init__(self, path):
        """Load preferences; damaged files fail explicitly instead of being overwritten."""
        self.path = Path(path)
        self._lock = threading.Lock()
        self._data = {"version": 1, "localization_radius_m": 0.30, "map_poses": {}}
        if self.path.exists():
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or data.get("version") != 1:
                raise ValueError("Unsupported robot settings file")
            if not isinstance(data.get("map_poses"), dict):
                raise ValueError("Invalid saved map poses")
            self._validate_radius(data.get("localization_radius_m"))
            self._data = data

    @staticmethod
    def _validate_radius(value):
        """Accept the requested 20–30 cm initial search spread."""
        if isinstance(value, bool) or not isinstance(value, (float, int)):
            raise ValueError("Localization radius must be between 0.20 and 0.30 m")
        if not math.isfinite(value) or not 0.20 <= value <= 0.30:
            raise ValueError("Localization radius must be between 0.20 and 0.30 m")

    def snapshot(self, map_id=None):
        """Return public preferences and the last pose for the selected map."""
        with self._lock:
            saved = self._data['map_poses'].get(map_id)
            if (not isinstance(saved, dict) or not all(
                    isinstance(saved.get(key), (int, float))
                    and not isinstance(saved.get(key), bool) and math.isfinite(saved[key])
                    for key in ('x', 'y', 'yaw'))):
                saved = None
            return {
                "localization_radius_m": self._data["localization_radius_m"],
                "last_pose": copy.deepcopy(saved),
            }

    def set_radius(self, value):
        """Persist the initial localization spread for subsequent pose seeds."""
        self._validate_radius(value)
        with self._lock:
            data = copy.deepcopy(self._data)
            data["localization_radius_m"] = float(value)
            self._write(data)

    def remember_pose(self, map_id, name, pose):
        """Store a finite, trusted map-frame pose for a particular map snapshot."""
        if not map_id or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError("A map identity and finite x, y, yaw are required")
        with self._lock:
            data = copy.deepcopy(self._data)
            data["map_poses"][map_id] = {
                "map_name": name, "x": pose[0], "y": pose[1], "yaw": pose[2],
                "saved_at": time.time(),
            }
            self._write(data)

    def _write(self, data):
        """Replace the settings file only after the complete new contents are flushed."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8",
                                             dir=self.path.parent, delete=False) as output:
                temporary = output.name
                json.dump(data, output, indent=2, allow_nan=False)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
            self._data = data
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
