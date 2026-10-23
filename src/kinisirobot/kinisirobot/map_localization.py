"""Confidence checks for switching from AMCL localization to map updates."""

import math


MAX_POSITION_STD_M = 0.20
MAX_YAW_STD_RAD = math.radians(20.0)
REQUIRED_CONFIDENT_SAMPLES = 3


class PoseContinuity:
    """Require a consistent map-to-odom transform for every localization prior."""

    def __init__(self, anchored=True):
        self.transform = None
        self.anchored = anchored

    def observe(self, pose, odom):
        """Return false for a hypothesis jump; the new hypothesis starts a new window."""
        yaw = math.atan2(math.sin(pose[2] - odom[2]), math.cos(pose[2] - odom[2]))
        c, s = math.cos(yaw), math.sin(yaw)
        transform = (pose[0] - c*odom[0] + s*odom[1],
                     pose[1] - s*odom[0] - c*odom[1], yaw)
        if not all(math.isfinite(v) for v in transform):
            self.transform = None
            return False
        previous = self.transform
        consistent = previous is None or (
            math.hypot(transform[0]-previous[0], transform[1]-previous[1]) <= .20
            and abs(math.atan2(math.sin(yaw-previous[2]), math.cos(yaw-previous[2]))) <= math.radians(10))
        if previous is None or not consistent or not self.anchored:
            self.transform = transform
        return consistent


class GlobalLocalizationEvidence:
    """Require one consistent map-to-odom estimate observed from different headings."""

    def __init__(self):
        """Start without a trusted global hypothesis or observed turn."""
        self.transform = None
        self.heading = None
        self.diverse = False

    def observe(self, map_pose, odom_pose):
        """Compare stationary scan matches across a measured turn of at least 10°.

        Repeating the same view cannot resolve global ambiguity. Correct poses
        at different headings imply the same map-to-odom transform; a changing
        hypothesis discards earlier evidence instead of combining unrelated fits.
        """
        if not all(math.isfinite(v) for v in (*map_pose, *odom_pose)):
            return False
        yaw = math.atan2(math.sin(map_pose[2]-odom_pose[2]), math.cos(map_pose[2]-odom_pose[2]))
        c, s = math.cos(yaw), math.sin(yaw)
        transform = (map_pose[0]-c*odom_pose[0]+s*odom_pose[1],
                     map_pose[1]-s*odom_pose[0]-c*odom_pose[1], yaw)
        if self.transform is not None:
            angle = math.atan2(math.sin(yaw-self.transform[2]), math.cos(yaw-self.transform[2]))
            if math.hypot(transform[0]-self.transform[0], transform[1]-self.transform[1]) > .20 or abs(angle) > math.radians(10):
                self.transform = None
        if self.transform is None:
            self.transform, self.heading, self.diverse = transform, odom_pose[2], False
        else:
            turn = math.atan2(math.sin(odom_pose[2]-self.heading), math.cos(odom_pose[2]-self.heading))
            self.diverse = self.diverse or abs(turn) >= math.radians(10)
        return self.diverse


def seed_covariance(radius_m):
    """Use ±radius as approximately two standard deviations, not a hard boundary."""
    if not math.isfinite(radius_m) or not 0.20 <= radius_m <= 0.30:
        raise ValueError("Localization radius must be between 0.20 and 0.30 m")
    result = [0.0] * 36
    result[0] = result[7] = (radius_m / 2.0) ** 2
    result[35] = math.radians(15.0) ** 2
    return result


def scan_alignment(ranges, angle_min, angle_increment, range_min, range_max,
                   laser_pose, width, height, resolution, origin, cells):
    """Check independent laser endpoints against occupied cells in the saved map.

    A narrow AMCL prior alone does not prove localization. Require at least
    20 finite returns and 65 percent near mapped surfaces (within 15 cm).
    This is a consistency check, not proof against identical-looking rooms.
    """
    if resolution <= 0 or not all(math.isfinite(v) for v in (*laser_pose, *origin)):
        return {"scan_match": 0.0, "scan_points": 0, "scan_agrees": False}
    c, s = math.cos(origin[2]), math.sin(origin[2])
    tolerance = math.ceil(0.15 / resolution)
    offsets = [(dx, dy) for dx in range(-tolerance, tolerance + 1)
               for dy in range(-tolerance, tolerance + 1)
               if (dx * resolution) ** 2 + (dy * resolution) ** 2 <= 0.15 ** 2]
    matched = total = 0
    for index in range(0, len(ranges), max(1, len(ranges) // 90)):
        distance = ranges[index]
        if not math.isfinite(distance) or not range_min < distance < min(range_max, 10.0):
            continue
        angle = laser_pose[2] + angle_min + index * angle_increment
        x = laser_pose[0] + distance * math.cos(angle) - origin[0]
        y = laser_pose[1] + distance * math.sin(angle) - origin[1]
        gx = math.floor((c * x + s * y) / resolution)
        gy = math.floor((-s * x + c * y) / resolution)
        total += 1
        if any(0 <= gx + dx < width and 0 <= gy + dy < height
               and cells[(gy + dy) * width + gx + dx] >= 65 for dx, dy in offsets):
            matched += 1
    ratio = matched / total if total else 0.0
    return {"scan_match": ratio, "scan_points": total,
            "scan_agrees": total >= 20 and ratio >= 0.65}


class MappingReadiness:
    """Require a new SLAM pose and map before reporting active updates."""

    def __init__(self, started_ns):
        """Record the ROS timestamp immediately before launching SLAM."""
        self.started_ns = started_ns
        self.pose_received = False
        self.map_received = False

    def observe(self, kind, stamp_ns):
        """Ignore cached messages that predate this SLAM startup."""
        if stamp_ns <= self.started_ns:
            return
        if kind == "pose":
            self.pose_received = True
        elif kind == "map":
            self.map_received = True

    @property
    def ready(self):
        """Whether both required outputs have arrived from the new session."""
        return self.pose_received and self.map_received


def localization_quality(covariance):
    """Return AMCL uncertainty and whether it is safe to resume mapping."""
    if len(covariance) != 36:
        return {
            "confident": False,
            "position_std_m": None,
            "yaw_std_rad": None,
        }

    variances = (covariance[0], covariance[7], covariance[35])
    # AMCL's E[x²] - E[x]² can round just below zero after convergence.
    # Clamp only numerical noise, never materially negative covariance.
    if any(not math.isfinite(value) or value < -1e-12 for value in variances):
        return {
            "confident": False,
            "position_std_m": None,
            "yaw_std_rad": None,
        }

    position_std = math.sqrt(max(0.0, variances[0], variances[1]))
    yaw_std = math.sqrt(max(0.0, variances[2]))
    return {
        "confident": (
            position_std <= MAX_POSITION_STD_M
            and yaw_std <= MAX_YAW_STD_RAD
        ),
        "position_std_m": position_std,
        "yaw_std_rad": yaw_std,
    }
