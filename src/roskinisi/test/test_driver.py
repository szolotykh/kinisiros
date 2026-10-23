# File: test_driver.py
"""Exercise driver lifecycle and publications with ROS stubs and real SDK signatures."""

import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import create_autospec, Mock

from pykinisi import KinisiController, PlatformOdometrySample, RequestTimeoutError
from pykinisi import __version__ as SDK_VERSION
import pytest


def vector():
    """Create zero-initialized ROS-style vector fields."""
    return NS(x=0.0, y=0.0, z=0.0)


def quaternion():
    """Create zero-initialized ROS-style quaternion fields."""
    return NS(x=0.0, y=0.0, z=0.0, w=0.0)


def twist():
    """Create a minimal Twist message."""
    return NS(linear=vector(), angular=vector())


def odometry_message():
    """Create the nested fields written by the driver."""
    return NS(header=NS(stamp=None, frame_id=''), child_frame_id='',
              pose=NS(pose=NS(position=vector(), orientation=quaternion())),
              twist=NS(twist=twist()))


def transform_message():
    """Create the TransformStamped fields written by the driver."""
    return NS(header=NS(stamp=None, frame_id=''), child_frame_id='',
              transform=NS(translation=vector(), rotation=quaternion()))


@pytest.fixture
def driver(monkeypatch):
    """Load the real driver against scoped ROS stubs, without opening serial."""
    parameters = {'port': 'TEST', 'use_sim_time': False}
    nodes = []

    class Node:
        """Provide only the ROS APIs used by this driver."""

        def __init__(self, name):
            """Record resources for cleanup assertions."""
            self.logger = Mock()
            self.destroyed = False
            self.descriptors = {}
            nodes.append(self)

        def declare_parameter(self, name, default, descriptor):
            """Return launch overrides and capture read-only descriptors."""
            self.descriptors[name] = descriptor
            return NS(value=parameters.get(name, default))

        def get_parameter(self, name):
            """Return the simulated ROS clock setting."""
            return NS(value=parameters[name])

        def add_on_set_parameters_callback(self, callback):
            """Save the callback that rejects simulated time at runtime."""
            self.parameter_callback = callback

        def get_logger(self):
            """Return a recording logger."""
            return self.logger

        def create_publisher(self, *args):
            """Return a recording publisher."""
            return Mock()

        def create_timer(self, *args):
            """Return a cancellable timer without scheduling callbacks."""
            return Mock()

        def create_subscription(self, *args):
            """Record subscription setup without an executor."""
            return Mock()

        def destroy_node(self):
            """Record destruction of ROS resources."""
            self.destroyed = True

    exports = {
        'rclpy': {'init': Mock(), 'spin': Mock(), 'ok': Mock(return_value=True),
                  'shutdown': Mock()},
        'rclpy.node': {'Node': Node},
        'rclpy.executors': {'ExternalShutdownException': type(
            'ExternalShutdownException', (Exception,), {})},
        'rcl_interfaces.msg': {'ParameterDescriptor': NS, 'SetParametersResult': NS},
        'builtin_interfaces.msg': {'Time': NS},
        'geometry_msgs.msg': {'Twist': twist, 'TransformStamped': transform_message},
        'nav_msgs.msg': {'Odometry': odometry_message},
        'std_msgs.msg': {'Empty': NS},
        'tf2_ros': {'TransformBroadcaster': Mock(side_effect=lambda node: Mock())},
    }
    for name, values in exports.items():
        if '.' in name:
            parent = name.split('.')[0]
            if parent not in exports:
                monkeypatch.setitem(sys.modules, parent, ModuleType(parent))
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / 'roskinisi/kinisi_controller.py'
    spec = importlib.util.spec_from_file_location('roskinisi.kinisi_controller', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    controller = create_autospec(KinisiController, instance=True)
    controller.connect.return_value = True
    controller.get_subscription_sample.return_value = None
    controller.ready = True
    controller.serial = object()
    controller.last_error = None
    controller.board_info = NS(protocol_major=2, protocol_minor=1, protocol_patch=0)
    factory = create_autospec(KinisiController, return_value=controller)
    monkeypatch.setattr(module, 'KinisiController', factory)
    yield NS(module=module, controller=controller, factory=factory,
             parameters=parameters, nodes=nodes)
    for node in nodes:
        node.shutdown()


def test_startup_and_mecanum_signature(driver):
    """Complete connect before setup; use current SDK signatures for both platforms."""
    driver.parameters['platform_type'] = 'mecanum'
    node = driver.module.KinisiControllerNode()
    driver.factory.assert_called_once_with(wall_clock=True, heartbeat_timeout_ms=500)
    calls = [call[0] for call in driver.controller.method_calls]
    assert calls.index('connect') < calls.index('initialize_mecanum_platform')
    assert calls.index('start_platform_odometry') < calls.index('subscribe_odometry')
    driver.controller.subscribe_odometry.assert_called_once_with(4, 100)
    assert all(descriptor.read_only for descriptor in node.descriptors.values())


@pytest.mark.parametrize('firmware_minor', [1, 3, 5])
def test_sdk_upgrade_preserves_velocity_interface_and_logs_versions(driver, firmware_minor):
    """Keep protocol 2.1+ velocity/odometry support without enabling newer controls."""
    driver.controller.board_info.protocol_minor = firmware_minor
    driver.module.KinisiControllerNode()
    driver.nodes[0].logger.info.assert_any_call(
        f'Connected with pykinisi {SDK_VERSION}; firmware protocol 2.{firmware_minor}.0.')
    driver.controller.initialize_omni_platform.assert_called_once()
    driver.controller.subscribe_odometry.assert_called_once_with(4, 100)
    driver.controller.initialize_platform_position_controller.assert_not_called()
    driver.controller.get_imu.assert_not_called()


def test_stale_velocity_expires_while_connection_is_alive(driver):
    """An SDK heartbeat must not keep a stale ROS velocity latched."""
    node = driver.module.KinisiControllerNode()
    message = twist()
    message.linear.x = .1
    node.cmd_vel_callback(message)
    with node._command_lock:
        node._last_command_at -= 2
        node._check_command_age()
    deadline = time.monotonic() + 1
    while node._command_active and time.monotonic() < deadline:
        time.sleep(.005)
    driver.controller.set_platform_target_velocity.assert_called_with(0., 0., 0.)
    assert driver.controller.ready
    assert not node._command_active


def test_fresh_velocity_is_not_stopped_by_command_watchdog(driver):
    """Normal command streaming stays active; an explicit zero disarms expiry."""
    node = driver.module.KinisiControllerNode()
    message = twist()
    message.angular.z = .1
    node.cmd_vel_callback(message)
    node._check_command_age()
    driver.controller.set_platform_target_velocity.assert_called_once_with(0., 0., .1)
    node.cmd_vel_callback(twist())
    assert not node._command_active


def test_uses_measurement_time_and_does_not_poll_or_republish(driver):
    """Publish the exact SDK timestamp once, identically in odometry and TF."""
    node = driver.module.KinisiControllerNode()
    node.odom_publish_callback()
    node.odom_publisher.publish.assert_not_called()
    driver.controller.get_subscription_sample.return_value = PlatformOdometrySample(
        1_789_365_945_599_946, 1, 1, 1.0, 2.0, 0.5)
    node.odom_publish_callback()
    node.odom_publish_callback()
    node.odom_publisher.publish.assert_called_once()
    odom = node.odom_publisher.publish.call_args.args[0]
    tf = node.tf_broadcaster.sendTransform.call_args.args[0]
    assert odom.header.stamp.sec == 1_789_365_945
    assert odom.header.stamp.nanosec == 599_946_000
    assert odom.header.stamp == tf.header.stamp
    assert odom.pose.pose.position.x == tf.transform.translation.x == 1.0
    driver.controller.get_platform_odometry.assert_not_called()


def test_reset_flushes_cache_and_history(driver):
    """Only post-reset readings can contribute to the next velocity estimate."""
    node = driver.module.KinisiControllerNode()
    node._odometry.update(PlatformOdometrySample(1_000_000, 1, 1, 9.0, 0.0, 0.0))
    driver.controller.reset_mock()
    node.reset_odometry_callback(NS())
    assert [call[0] for call in driver.controller.method_calls] == [
        'unsubscribe_odometry', 'reset_platform_odometry', 'subscribe_odometry']
    assert node._odometry.previous is None


@pytest.mark.parametrize('overrides', [
    {'port': 'NOT_SET'}, {'heartbeat_timeout_ms': 0}, {'heartbeat_timeout_ms': True},
    {'odometry_frequency_hz': 0}, {'odometry_interval_ms': 99},
    {'use_sim_time': True}, {'platform_type': 'unknown'},
    {'command_timeout_ms': 0}, {'command_timeout_ms': True},
])
def test_invalid_parameters_never_open_serial(driver, overrides):
    """Reject invalid configuration without partially initializing hardware."""
    driver.parameters.update(overrides)
    with pytest.raises(ValueError):
        driver.module.KinisiControllerNode()
    driver.factory.assert_not_called()
    assert driver.nodes[0].destroyed


def test_sim_time_cannot_be_enabled_after_startup(driver):
    """Do not mix host Unix acquisition timestamps with ROS simulation time."""
    node = driver.module.KinisiControllerNode()
    assert not node.parameter_callback([NS(name='use_sim_time', value=True)]).successful


def test_custom_intervals_preserve_firmware_tick_quantization(driver):
    """Use the same integer-millisecond calculation rule as the firmware."""
    driver.parameters.update(heartbeat_timeout_ms=1000, odometry_frequency_hz=333,
                             odometry_interval_ms=6)
    driver.module.KinisiControllerNode()
    driver.factory.assert_called_once_with(wall_clock=True, heartbeat_timeout_ms=1000)
    driver.controller.set_odometry_frequency.assert_called_once_with(333)
    driver.controller.subscribe_odometry.assert_called_once_with(4, 6)


def test_failed_init_never_starts_odometry_or_motion(driver):
    """A failed clock/setup handshake cannot reach platform initialization."""
    driver.controller.connect.return_value = False
    driver.controller.ready = False
    driver.controller.serial = None
    with pytest.raises(driver.module.ConnectionClosedError):
        driver.module.KinisiControllerNode()
    driver.controller.initialize_omni_platform.assert_not_called()
    driver.controller.start_platform_controller.assert_not_called()
    driver.controller.disconnect.assert_called_once()


def test_velocity_forwarding_and_command_failure(driver):
    """Preserve cmd_vel units and close the session on an unsuccessful command."""
    node = driver.module.KinisiControllerNode()
    command = twist()
    command.linear.x, command.linear.y, command.angular.z = 0.2, -0.1, 0.5
    node.cmd_vel_callback(command)
    driver.controller.set_platform_target_velocity.assert_called_once_with(0.2, -0.1, 0.5)
    driver.controller.set_platform_target_velocity.side_effect = RequestTimeoutError('No ACK')
    with pytest.raises(RequestTimeoutError):
        node.cmd_vel_callback(command)
    driver.controller.stop_platform_controller.assert_not_called()
    driver.controller.disconnect.assert_called_once()


@pytest.mark.parametrize('stall_zero', [False, True])
def test_blocked_velocity_or_zero_ack_revokes_session(driver, stall_zero):
    """Watchdog can revoke SDK heartbeats without acquiring the blocked I/O lock."""
    driver.parameters['command_timeout_ms'] = 100
    node = driver.module.KinisiControllerNode()
    entered, release, disconnected = threading.Event(), threading.Event(), threading.Event()
    errors = []
    driver.controller.disconnect.side_effect = disconnected.set
    def target(*values):
        if (not any(values)) == stall_zero:
            entered.set()
            if not release.wait(2):
                errors.append('blocked test command was not released')
    driver.controller.set_platform_target_velocity.side_effect = target
    message = twist()
    message.linear.x = .1
    sender = threading.Thread(target=node.cmd_vel_callback, args=(message,))
    sender.start()
    try:
        assert entered.wait(1)
        assert disconnected.wait(.7), 'ACK stall left heartbeats enabled'
        assert node._closed
        assert node._command_stop.is_set()
        count = driver.controller.set_platform_target_velocity.call_count
        node.cmd_vel_callback(message)
        assert driver.controller.set_platform_target_velocity.call_count == count
    finally:
        release.set()
        sender.join(2)
    assert not errors


def test_startup_failure_stops_and_disconnects(driver):
    """Cleanup covers failures after the hardware connection has been established."""
    driver.controller.subscribe_odometry.side_effect = RequestTimeoutError('No ACK')
    with pytest.raises(RequestTimeoutError):
        driver.module.KinisiControllerNode()
    driver.controller.stop_platform_controller.assert_called_once()
    driver.controller.coast_platform.assert_called_once()
    driver.controller.disconnect.assert_called_once()
    assert driver.nodes[0].destroyed


def test_shutdown_is_idempotent_and_survives_stop_failure(driver):
    """Always close serial even when the controller cannot acknowledge shutdown."""
    node = driver.module.KinisiControllerNode()
    driver.controller.reset_mock()
    driver.controller.stop_platform_controller.side_effect = RequestTimeoutError('No ACK')
    node.shutdown()
    node.shutdown()
    assert [call[0] for call in driver.controller.method_calls] == [
        'stop_platform_controller', 'coast_platform', 'unsubscribe_odometry', 'disconnect']
    node.odom_timer.cancel.assert_called_once()


def test_connection_loss_never_publishes_cached_pose(driver):
    """Lose readiness before reading the cache and stop the ROS callbacks."""
    node = driver.module.KinisiControllerNode()
    driver.controller.ready = False
    with pytest.raises(driver.module.ConnectionClosedError):
        node.odom_publish_callback()
    node.odom_publisher.publish.assert_not_called()
    driver.controller.disconnect.assert_called_once()


def test_main_cleans_up_after_callback_exception(driver):
    """Executor failures must still stop hardware and destroy ROS resources."""
    driver.module.rclpy.spin.side_effect = RequestTimeoutError('No ACK')
    with pytest.raises(RequestTimeoutError):
        driver.module.main()
    assert driver.nodes[0].destroyed
    driver.controller.disconnect.assert_called_once()
    driver.module.rclpy.shutdown.assert_called_once()
