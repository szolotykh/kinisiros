"""Offline checks for deterministic LiDAR recovery policy."""

import sys
import types
from pathlib import Path
from unittest.mock import Mock
from types import SimpleNamespace as NS
import math


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

for module_name, exports in {
    "rclpy": {"init": Mock(), "spin": Mock(), "shutdown": Mock()},
    "rclpy.node": {"Node": object},
    "rclpy.qos": {"qos_profile_sensor_data": Mock()},
    "diagnostic_msgs.msg": {
        "DiagnosticArray": Mock(), "DiagnosticStatus": Mock(), "KeyValue": Mock()},
    "sensor_msgs.msg": {"LaserScan": Mock()},
}.items():
    module = types.ModuleType(module_name)
    module.__dict__.update(exports)
    sys.modules.setdefault(module_name, module)

from kinisirobot.lidar_supervisor import (
    LidarSupervisor,
    RestartBackoff,
    resolve_serial_port,
    scan_timeout_reason,
)


def test_wrapper_exit_cleans_children_before_scheduling_restart():
    """An exited ros2 wrapper must not orphan a live serial-port-owning child."""
    supervisor = LidarSupervisor.__new__(LidarSupervisor)
    supervisor._stopping = False
    supervisor._process = Mock()
    supervisor._process.poll.return_value = 0
    supervisor._driver_backend = Mock()
    supervisor._schedule_restart = Mock()
    supervisor._tick()
    supervisor._driver_backend.stop.assert_called_once()
    assert supervisor._process is None
    supervisor._schedule_restart.assert_called_once_with('driver exited with code 0')


def test_restart_backoff_is_bounded_and_resets():
    backoff = RestartBackoff(minimum=2, maximum=10)
    assert [backoff.failed() for _ in range(5)] == [2, 4, 8, 10, 10]
    backoff.healthy()
    assert backoff.failed() == 2


def test_explicit_serial_port_wins(tmp_path):
    assert resolve_serial_port("/dev/custom", str(tmp_path / "*")) == "/dev/custom"


def test_one_by_id_device_is_preferred(tmp_path):
    device = tmp_path / "usb-rplidar"
    device.write_text("", encoding="utf-8")
    assert resolve_serial_port("", str(tmp_path / "*")) == str(device)


def test_ambiguous_by_id_devices_never_guess_a_port(tmp_path):
    (tmp_path / "one").write_text("", encoding="utf-8")
    (tmp_path / "two").write_text("", encoding="utf-8")
    assert resolve_serial_port("", str(tmp_path / "*")) is None


def test_known_lidar_adapter_is_selected_among_multiple_devices(tmp_path):
    controller = tmp_path / "usb-VsReality_Kinisi_motor_controller-if00"
    lidar = tmp_path / "usb-Silicon_Labs_CP2102_USB_to_UART-if00-port0"
    controller.write_text("", encoding="utf-8")
    lidar.write_text("", encoding="utf-8")
    assert resolve_serial_port("", str(tmp_path / "*")) == str(lidar)


def test_motor_controller_alone_is_not_a_lidar(tmp_path):
    (tmp_path / 'usb-VsReality_Kinisi_motor_controller-if00').touch()
    assert resolve_serial_port('', str(tmp_path / '*')) is None


def test_backoff_survives_multiday_outage():
    backoff = RestartBackoff()
    assert max(backoff.failed() for _ in range(10000)) == 60.0
    backoff.healthy()
    assert backoff.failed() == 2.0


def supervisor():
    """Run real discovery and scan callbacks with fake process/ROS boundaries."""
    node = LidarSupervisor.__new__(LidarSupervisor)
    node._configured_port = ''
    node._port = None
    node._process = None
    node._stopping = False
    node._next_start_at = 0
    node._driver_backend = Mock()
    node._backoff = RestartBackoff()
    node.get_logger = Mock()
    params = {'frame_id':'laser_frame', 'serial_baudrate':115200,
              'scan_mode':'Standard', 'angle_compensate':True, 'inverted':False}
    node.get_parameter = lambda name: NS(value=params[name])
    node._positive = lambda name: 3.
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    node._last_scan_at = node._last_usable_at = None
    node._last_scan_stamp_ns = 0
    node._healthy_scans = 0
    node._scan_reason = 'waiting for scans'
    return node


def reading(stamp=9, ranges=None, frame='laser_frame'):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=stamp, nanosec=0)),
              ranges=[1., 2.] if ranges is None else ranges, angle_min=0.,
              angle_increment=.01, range_min=.1, range_max=10.)


def test_hotplug_rediscovers_missing_lidar_and_pins_its_identity(monkeypatch, tmp_path):
    from kinisirobot import lidar_supervisor as module
    motor = tmp_path / 'usb-VsReality_Kinisi_motor_controller-if00'
    laser = tmp_path / 'usb-Silicon_Labs_CP2102_USB_to_UART-if00-port0'
    motor.touch()
    devices = [str(motor)]
    monkeypatch.setattr(module.glob, 'glob', lambda pattern: devices)
    node = supervisor()
    node._start_driver()
    node._driver_backend.start.assert_not_called()
    assert node._port is None
    laser.touch()
    devices.append(str(laser))
    node._next_start_at = 0
    node._start_driver()
    assert node._port == str(laser)
    node._driver_backend.start.assert_called_once()
    # Removing the identified LiDAR must not adopt the motor controller.
    node._process = None
    laser.unlink()
    node._next_start_at = 0
    node._start_driver()
    node._driver_backend.start.assert_called_once()


def test_stale_wrong_frame_and_empty_scans_do_not_recover_health():
    node = supervisor()
    node._process = Mock()
    node._backoff.failures = 10
    for message in (reading(stamp=1), reading(frame='wrong'), reading(ranges=[])):
        node._on_scan(message)
        assert node._last_scan_at is None
        assert node._last_usable_at is None
        assert node._backoff.failures == 10
        assert 'rejected' in node._scan_reason


def test_usable_scan_recovery_requires_progressing_timestamps():
    node = supervisor()
    node._process = Mock()
    node._backoff.failures = 10
    node._on_scan(reading())
    node._on_scan(reading())
    assert node._backoff.failures == 10
    assert 'rejected' in node._scan_reason
    for nanos in (1, 2, 3, 4, 5):
        message = reading()
        message.header.stamp.nanosec = nanos
        node._on_scan(message)
    assert node._scan_reason == 'scans healthy'
    assert node._backoff.failures == 0


def test_no_returns_is_coverage_not_serial_failure():
    node = supervisor()
    node._process = Mock()
    node._backoff.failures = 10
    node._on_scan(reading(ranges=[math.inf, math.inf]))
    assert node._last_scan_at is not None
    assert node._last_usable_at is None
    assert 'no usable range returns' in node._scan_reason
    assert node._backoff.failures == 10


def test_buffered_scan_does_not_bypass_startup_grace():
    assert scan_timeout_reason(104, 100, 100.5, 12, 3) is None


def test_stale_scan_restarts_after_startup_grace():
    assert scan_timeout_reason(113, 100, 109, 12, 3) == "scan stream became stale"


def test_missing_startup_scan_restarts_after_grace():
    assert scan_timeout_reason(113, 100, None, 12, 3) == (
        "no scans received before startup timeout")
