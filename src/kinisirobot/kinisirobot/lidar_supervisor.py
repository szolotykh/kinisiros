"""Own and recover the RPLIDAR driver while reporting scan health."""

from __future__ import annotations

import glob
import math
import os
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .owned_backend import OwnedBackend


def resolve_serial_port(configured: str, by_id_pattern="/dev/serial/by-id/*"):
    """Select an explicit identity or one recognized adapter; never guess ttyUSB0."""
    if configured:
        return configured
    candidates = sorted(path for path in glob.glob(by_id_pattern) if os.path.exists(path))
    lidar_tokens = ("rplidar", "silicon_labs_cp210")
    likely_lidar = [
        path for path in candidates
        if any(token in os.path.basename(path).lower() for token in lidar_tokens)
        and 'kinisi' not in os.path.basename(path).lower()
    ]
    return likely_lidar[0] if len(likely_lidar) == 1 else None


def scan_timeout_reason(now, started_at, last_scan_at, startup_timeout,
                        scan_timeout):
    """Return a restart reason, retaining startup grace for buffered scans."""
    uptime = now - started_at
    if last_scan_at is None:
        if uptime > startup_timeout:
            return "no scans received before startup timeout"
    elif uptime > startup_timeout and now - last_scan_at > scan_timeout:
        return "scan stream became stale"
    return None


@dataclass
class RestartBackoff:
    """Bound restart frequency without giving up permanent recovery."""

    minimum: float = 2.0
    maximum: float = 60.0
    failures: int = 0

    def failed(self) -> float:
        """Return the next delay and increase the consecutive-failure count."""
        # Bound the exponent before computing it, including multi-day outages.
        cap = max(0, math.ceil(math.log2(self.maximum) - math.log2(self.minimum)))
        delay = self.maximum if self.failures >= cap else math.ldexp(self.minimum, self.failures)
        self.failures += 1
        return delay

    def healthy(self) -> None:
        """A fresh scan resets recovery to the shortest delay."""
        self.failures = 0


class LidarSupervisor(Node):
    """Launch RPLIDAR, detect exits or stale scans, and restart with backoff."""

    def __init__(self):
        super().__init__("lidar_supervisor")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("scan_mode", "Standard")
        self.declare_parameter("angle_compensate", True)
        self.declare_parameter("inverted", False)
        self.declare_parameter("startup_timeout_sec", 12.0)
        self.declare_parameter("scan_timeout_sec", 3.0)
        self.declare_parameter("restart_delay_sec", 2.0)
        self.declare_parameter("maximum_restart_delay_sec", 60.0)

        self._configured_port = self.get_parameter("serial_port").value
        self._port = None
        self._process = None
        self._driver_backend = OwnedBackend()
        self._started_at = None
        self._last_scan_at = None
        self._last_usable_at = None
        self._last_scan_stamp_ns = 0
        self._healthy_scans = 0
        self._scan_reason = 'waiting for scans'
        self._next_start_at = 0.0
        self._reason = "starting"
        self._stopping = False
        self._backoff = RestartBackoff(
            minimum=self._positive("restart_delay_sec"),
            maximum=self._positive("maximum_restart_delay_sec"),
        )

        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)
        self._start_driver()

    def _positive(self, name):
        value = self.get_parameter(name).get_parameter_value().double_value
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def _driver_command(self):
        values = {
            "channel_type": "serial",
            "serial_port": self._port,
            "serial_baudrate": self.get_parameter("serial_baudrate").value,
            "frame_id": self.get_parameter("frame_id").value,
            "scan_mode": self.get_parameter("scan_mode").value,
            "angle_compensate": str(self.get_parameter("angle_compensate").value).lower(),
            "inverted": str(self.get_parameter("inverted").value).lower(),
        }
        command = ["ros2", "run", "rplidar_ros", "rplidar_composition", "--ros-args"]
        for key, value in values.items():
            command.extend(("-p", f"{key}:={value}"))
        return command

    def _start_driver(self):
        if self._stopping or self._process is not None:
            return
        now = time.monotonic()
        if now < self._next_start_at:
            return
        # Retry discovery after a late plug-in. Once selected, keep that exact
        # by-id identity rather than silently adopting a different device.
        self._port = resolve_serial_port(self._configured_port or self._port)
        if self._port is None or not os.path.exists(self._port):
            self._schedule_restart('LiDAR device missing or ambiguous; connect it or set serial_port explicitly')
            return
        try:
            self._process = self._driver_backend.start(self._driver_command(), '~/lidar_driver.log')
        except OSError as exc:
            self._schedule_restart(f"unable to launch driver: {exc}")
            return
        self._started_at = now
        self._last_scan_at = None
        self._last_usable_at = None
        self._last_scan_stamp_ns = 0
        self._healthy_scans = 0
        self._scan_reason = 'waiting for scans'
        self._reason = "waiting for scans"
        self.get_logger().info(
            f"Started RPLIDAR on {self._port} (PID {self._process.process.pid})")

    def _on_scan(self, message):
        """Separate message arrival, a valid stream, and usable range returns."""
        if self._process is None:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        metadata = (message.angle_min, message.angle_increment, message.range_min, message.range_max)
        if (message.header.frame_id != self.get_parameter('frame_id').value
                or not 0 <= age <= self._positive('scan_timeout_sec')
                or stamp <= self._last_scan_stamp_ns
                or len(message.ranges) < 2 or not all(math.isfinite(v) for v in metadata)
                or message.angle_increment == 0 or not 0 <= message.range_min < message.range_max):
            self._healthy_scans = 0
            self._scan_reason = 'Scan rejected: stale/repeated timestamp, wrong frame or invalid layout'
            self._reason = self._scan_reason
            return
        self._last_scan_stamp_ns = stamp
        self._last_scan_at = time.monotonic()
        usable = any(math.isfinite(r) and message.range_min <= r <= message.range_max
                     for r in message.ranges)
        if usable:
            self._last_usable_at = self._last_scan_at
            self._healthy_scans += 1
            if self._healthy_scans >= 5:
                self._backoff.healthy()
            self._scan_reason = 'scans healthy'
        else:
            # A well-formed scan without returns is a coverage problem, not
            # proof that restarting the serial driver will help.
            self._healthy_scans = 0
            self._scan_reason = 'Scan stream active but no usable range returns; check coverage'
        self._reason = self._scan_reason

    def _tick(self):
        if self._stopping:
            return
        now = time.monotonic()
        if self._process is None:
            self._start_driver()
            return
        return_code = self._process.poll()
        if return_code is not None:
            self._terminate_driver()
            self._schedule_restart(f"driver exited with code {return_code}")
            return
        startup_timeout = self._positive("startup_timeout_sec")
        scan_timeout = self._positive("scan_timeout_sec")
        reason = scan_timeout_reason(
            now, self._started_at, self._last_scan_at,
            startup_timeout, scan_timeout)
        if reason:
            self._restart_driver(reason)

    def _restart_driver(self, reason):
        self.get_logger().error(f"RPLIDAR unhealthy: {reason}")
        self._terminate_driver()
        self._schedule_restart(reason)

    def _schedule_restart(self, reason):
        delay = self._backoff.failed()
        self._reason = f"{reason}; retrying in {delay:.0f}s"
        self._next_start_at = time.monotonic() + delay
        self.get_logger().warning(self._reason)

    def _terminate_driver(self):
        # Keep the leader reserved until all group children have been stopped.
        self._driver_backend.stop()
        self._process = None

    def _publish_diagnostics(self):
        now = time.monotonic()
        healthy = (self._process is not None and self._process.poll() is None
                   and self._scan_reason == 'scans healthy'
                   and self._last_usable_at is not None
                   and now - self._last_usable_at <= self._positive("scan_timeout_sec"))
        if healthy:
            level, message = DiagnosticStatus.OK, "RPLIDAR scans are healthy"
        elif self._process is not None:
            level, message = DiagnosticStatus.WARN, self._reason
        else:
            level, message = DiagnosticStatus.ERROR, self._reason
        status = DiagnosticStatus(
            level=level,
            name="kinisirobot/lidar",
            message=message,
            hardware_id=self._port or 'unresolved',
            values=[
                KeyValue(key="serial_port", value=self._port or 'unresolved'),
                KeyValue(key="restart_failures", value=str(self._backoff.failures)),
                KeyValue(key="scan_status", value=self._scan_reason),
            ],
        )
        message_array = DiagnosticArray()
        message_array.header.stamp = self.get_clock().now().to_msg()
        message_array.status = [status]
        self._diagnostics.publish(message_array)

    def destroy_node(self):
        self._stopping = True
        self._terminate_driver()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
