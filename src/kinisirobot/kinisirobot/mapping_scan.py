"""Keep SLAM's scan geometry stable across RPLIDAR acquisition modes."""

import copy
import math


def normalize_mapping_scan(scan, sample_count):
    """Resample angular bins without interpolating across missing measurements.

    Saved SLAM graphs retain their laser model. Replacing a 1080-bin model
    with a 360-bin model changes how historical scans are interpreted. Use
    one fixed layout for mapping; this does not increase sensor resolution.
    The original acquisition timestamp, sweep duration and angular limits
    stay unchanged. Invalid source readings remain unknown in the output.
    """
    if type(sample_count) is not int or not 2 <= sample_count <= 10000:
        raise ValueError("Mapping scan sample count must be in 2..10000")
    source_count = len(scan.ranges)
    if source_count < 2:
        raise ValueError("A complete scan needs at least two angular samples")
    if not all(math.isfinite(value) for value in (
            scan.angle_min, scan.angle_max, scan.angle_increment, scan.scan_time)):
        raise ValueError("Scan geometry and duration must be finite")
    span = scan.angle_max - scan.angle_min
    if span <= 0 or scan.angle_increment <= 0 or scan.scan_time <= 0:
        raise ValueError("Scan angles and duration must increase")
    if not math.isclose(span, scan.angle_increment * (source_count - 1),
                        rel_tol=1e-4, abs_tol=1e-5):
        raise ValueError("Scan length does not match its angular geometry")
    indexes = [min(source_count - 1, round(i * (source_count - 1) / (sample_count - 1)))
               for i in range(sample_count)]
    result = copy.copy(scan)
    result.angle_increment = span / (sample_count - 1)
    result.time_increment = scan.scan_time / (sample_count - 1)
    result.ranges = [scan.ranges[index] for index in indexes]
    result.intensities = ([scan.intensities[index] for index in indexes]
                          if len(scan.intensities) == source_count else [])
    return result
