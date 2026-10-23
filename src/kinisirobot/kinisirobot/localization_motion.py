"""Bounded localization turns guarded by live scans and the odom-frame costmap."""

import math

from .localization_config import DEFAULT_LOCALIZATION_CONFIG


def angle_delta(value, reference):
    """Return the shortest signed angular difference in radians."""
    return math.atan2(math.sin(value - reference), math.cos(value - reference))


def costmap_rotation_clearance(grid, pose, config=DEFAULT_LOCALIZATION_CONFIG):
    """Check the complete swept circular footprint against the local costmap.

    Require known cells below the high-cost threshold within a 35 cm radius:
    the 30 cm chassis plus margin. Unknown space and map edges block movement.
    The caller must independently validate map frame, age and live scan data.
    """
    info = grid.info
    if (not math.isfinite(info.resolution) or info.resolution <= 0
            or info.width <= 0 or info.height <= 0
            or len(grid.data) != info.width * info.height):
        return False
    origin = info.origin
    q = origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    dx, dy = pose[0] - origin.position.x, pose[1] - origin.position.y
    if not all(math.isfinite(v) for v in (dx, dy, yaw)):
        return False
    x = math.cos(yaw) * dx + math.sin(yaw) * dy
    y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    radius = config.body_radius_m + config.costmap_margin_m
    if (x - radius < 0 or y - radius < 0 or
            x + radius >= info.width * info.resolution or
            y + radius >= info.height * info.resolution):
        return False
    padding = info.resolution / math.sqrt(2)
    for row in range(math.floor((y-radius)/info.resolution), math.ceil((y+radius)/info.resolution)):
        for col in range(math.floor((x-radius)/info.resolution), math.ceil((x+radius)/info.resolution)):
            if math.hypot((col+.5)*info.resolution-x, (row+.5)*info.resolution-y) <= radius+padding:
                value = grid.data[row*info.width+col]
                if value < 0 or value >= 65:
                    return False
    return True


def _rotation_evidence(scan, laser_pose, config):
    """Return observed sectors, or a blocking reason; poses are in current base."""
    x, y, yaw = laser_pose
    if (not all(math.isfinite(v) for v in (x, y, yaw, scan.angle_min,
                                          scan.angle_increment, scan.range_min,
                                          scan.range_max)) or not scan.ranges
            or scan.angle_increment == 0 or scan.range_max <= scan.range_min):
        return set(), 'Invalid scan layout or laser transform'
    sectors = set()
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or distance <= 0 or distance > scan.range_max:
            continue
        if distance <= scan.range_min:
            return set(), 'LiDAR return is inside its minimum measurable range'
        angle = yaw + scan.angle_min + i * scan.angle_increment
        px, py = x + distance * math.cos(angle), y + distance * math.sin(angle)
        if math.hypot(px, py) < config.body_radius_m + config.scan_margin_m:
            return set(), 'Measured obstacle inside the rotation clearance margin'
        sectors.add(int((math.atan2(py, px) % math.tau) / math.tau * 36) % 36)
    return sectors, None


def rotation_clearance_evidence(observations, config=DEFAULT_LOCALIZATION_CONFIG):
    """Combine fresh, motion-compensated returns, never interpreting infinity as free.

    The caller limits observation age and transforms each laser origin into the
    current base frame. Any observed close obstacle vetoes the entire history.
    This laser-plane check cannot detect hazards above or below that plane.
    """
    sectors = set()
    for scan, laser_pose in observations:
        observed, reason = _rotation_evidence(scan, laser_pose, config)
        if reason:
            return {'clear': False, 'blocked': True, 'reason': reason, 'missing_sectors': []}
        sectors.update(observed)
    missing = sorted(set(range(36)) - sectors)
    return {'clear': not missing, 'blocked': False, 'missing_sectors': missing,
            'reason': (f'LiDAR coverage incomplete: {len(missing)} of 36 sectors unobserved'
                       if missing else '')}


def rotation_clearance(scan, laser_pose, config=DEFAULT_LOCALIZATION_CONFIG):
    """Require finite clearance evidence in every sector of this single scan."""
    return rotation_clearance_evidence([(scan, laser_pose)], config)['clear']


class LocalizationMotion:
    """Turn to +15°, -15°, then the starting heading, pausing at each stop."""

    def __init__(self, now, pose):
        """Anchor movement limits to fresh odometry, never the AMCL estimate."""
        self.origin = pose
        self.started = self.last_tick = now
        self.targets = (math.radians(15), math.radians(-15), 0.0)
        self.index = 0
        self.pause_until = None
        self.active = True
        self.reason = "Turning slowly to gather scans"

    def stop(self, reason):
        """Latch a terminal state; this sequence never retries itself."""
        self.active = False
        self.reason = reason
        return 0.0

    def step(self, now, pose, *, sensors_fresh, clear, connected):
        """Return angular velocity, or zero on any failed guard or travel limit."""
        if not self.active:
            return 0.0
        if not connected:
            return self.stop("Stopped: client disconnected")
        if not sensors_fresh or not all(math.isfinite(v) for v in pose):
            return self.stop("Stopped: fresh LiDAR and odometry required")
        if not clear:
            return self.stop("Stopped: nearby obstacle or incomplete LiDAR coverage")
        if now - self.last_tick > 0.25 or now < self.last_tick:
            return self.stop("Stopped: motion control timing interrupted")
        self.last_tick = now
        if now - self.started >= 25:
            return self.stop("Stopped: localization movement time limit")
        if math.hypot(pose[0] - self.origin[0], pose[1] - self.origin[1]) >= 0.05:
            return self.stop("Stopped: unexpected position drift")
        yaw = angle_delta(pose[2], self.origin[2])
        if abs(yaw) >= math.radians(20):
            return self.stop("Stopped: turn limit reached")
        if self.pause_until is not None:
            if now < self.pause_until:
                return 0.0
            self.pause_until = None
            self.index += 1
            if self.index == len(self.targets):
                return self.stop("Small turns complete; checking localization")
        error = self.targets[self.index] - yaw
        if abs(error) <= math.radians(1):
            self.pause_until = now + 2.0
            self.reason = "Paused to check fresh scans"
            return 0.0
        self.reason = "Turning slowly to gather scans"
        return math.copysign(min(0.12, max(0.04, abs(error))), error)
