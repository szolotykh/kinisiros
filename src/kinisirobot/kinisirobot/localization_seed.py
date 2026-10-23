"""Bounded scan-to-map search for an AMCL seed, never a localization verdict."""

import math


def refine_seed(scan, grid, seed, laser_offset, radius):
    """Find a better initial pose within the configured neighborhood.

    Compare real laser returns with nearby occupied cells over a 5 cm / 5 degree
    search grid. Missing returns provide no evidence. The result only seeds
    AMCL: independent fresh observations and covariance still gate navigation.
    Ties favor the remembered pose, and an insignificant gain keeps it intact.
    """
    info = grid.info
    if (not .20 <= radius <= .30 or info.resolution <= 0
            or info.width * info.height != len(grid.data)
            or not all(math.isfinite(v) for v in (*seed, *laser_offset,
                         info.resolution, scan.angle_min, scan.angle_increment))):
        return None
    points = []
    lx, ly, lyaw = laser_offset
    for index in range(0, len(scan.ranges), max(1, len(scan.ranges) // 90)):
        distance = scan.ranges[index]
        if math.isfinite(distance) and scan.range_min < distance < min(scan.range_max, 10.0):
            angle = lyaw + scan.angle_min + index * scan.angle_increment
            points.append((lx + distance * math.cos(angle), ly + distance * math.sin(angle)))
    if len(points) < 20:
        return None

    tolerance = math.ceil(.15 / info.resolution)
    offsets = [(dx, dy) for dx in range(-tolerance, tolerance + 1)
               for dy in range(-tolerance, tolerance + 1)
               if math.hypot(dx, dy) * info.resolution <= .15 + 1e-9]
    near_wall = set()
    for index, value in enumerate(grid.data):
        if value < 65:
            continue
        row, col = divmod(index, info.width)
        for dx, dy in offsets:
            if 0 <= col + dx < info.width and 0 <= row + dy < info.height:
                near_wall.add((row + dy) * info.width + col + dx)
    origin = info.origin
    q = origin.orientation
    origin_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    oc, os = math.cos(origin_yaw), math.sin(origin_yaw)

    def score(pose):
        """Count measured endpoints near known walls in the grid's own frame."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        matched = 0
        for px, py in points:
            x = pose[0] + c * px - s * py - origin.position.x
            y = pose[1] + s * px + c * py - origin.position.y
            col = math.floor((oc * x + os * y) / info.resolution)
            row = math.floor((-os * x + oc * y) / info.resolution)
            matched += (0 <= col < info.width and 0 <= row < info.height
                        and row * info.width + col in near_wall)
        return matched / len(points)

    initial = score(seed)
    best_pose, best_score, best_distance = seed, initial, 0.0
    steps = round(radius / .05)
    for ix in range(-steps, steps + 1):
        for iy in range(-steps, steps + 1):
            dx, dy = ix * .05, iy * .05
            if math.hypot(dx, dy) > radius + 1e-9:
                continue
            for ia in range(-4, 5):
                angle = math.radians(ia * 5)
                pose = (seed[0] + dx, seed[1] + dy, seed[2] + angle)
                match = score(pose)
                distance = dx*dx + dy*dy + (.3 * angle)**2
                if match > best_score or (match == best_score and distance < best_distance):
                    best_pose, best_score, best_distance = pose, match, distance
    if best_score < .65 or best_score < initial + .05:
        best_pose, best_score = seed, initial
    return {'pose': best_pose, 'scan_match': best_score, 'original_match': initial,
            'scan_points': len(points)}
