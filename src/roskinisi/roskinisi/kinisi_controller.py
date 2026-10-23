#!/usr/bin/env python3
# File: kinisi_controller.py
"""Bridge ROS velocity commands and timestamped Kinisi protocol-v2 odometry."""

import math
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from pykinisi import ClockQuality, ConnectionClosedError, KinisiController, KinisiError
from pykinisi import __version__ as SDK_VERSION
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty
import tf2_ros

from .odometry import OdometryTracker


PLATFORM_SOURCE = 4


class KinisiControllerNode(Node):
    """Own one physical controller session, with explicit startup and shutdown."""

    def __init__(self):
        """Finish INIT/clock setup before configuring motion and subscribing."""
        super().__init__('kinisi_controller')
        self.kinisi_controller = None
        self.odom_timer = None
        self._closed = False
        self._command_lock = threading.RLock()
        self._command_io_lock = threading.Lock()
        self._command_stop = threading.Event()
        self._last_command_at = None
        self._command_active = False
        self._command_inflight_at = None
        self._expiry_pending = False
        self._odometry = OdometryTracker()
        self._clock_quality = None
        try:
            self._configure()
            self.kinisi_controller = KinisiController(
                wall_clock=True, heartbeat_timeout_ms=self.heartbeat_timeout_ms)
            if not self.kinisi_controller.connect(self.port):
                raise ConnectionClosedError(
                    f'Cannot connect to {self.port}: {self.kinisi_controller.last_error}')
            board = self.kinisi_controller.board_info
            self.get_logger().info(
                f'Connected with pykinisi {SDK_VERSION}; firmware protocol '
                f'{board.protocol_major}.{board.protocol_minor}.{board.protocol_patch}.')
            self._initialize_platform()
            self.kinisi_controller.set_odometry_frequency(self.odometry_frequency_hz)
            self.kinisi_controller.start_platform_odometry()
            self.kinisi_controller.reset_platform_odometry()
            self.kinisi_controller.subscribe_odometry(
                PLATFORM_SOURCE, self.odometry_interval_ms)
            self.kinisi_controller.start_platform_controller(
                kp=0.8, ki=0.2, kd=0, integral_limit=30)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
            self.odom_timer = self.create_timer(
                self.odometry_interval_ms / 1000, self.odom_publish_callback)
            self.cmd_vel_subscriber = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            self.reset_odom_subscriber = self.create_subscription(
                Empty, '/reset_odometry', self.reset_odometry_callback, 10)
            threading.Thread(target=self._command_watchdog, daemon=True,
                             name='kinisi-command-watchdog').start()
            self.get_logger().info('Kinisi session ready; odometry streaming enabled.')
        except BaseException:
            # Setup can fail after opening serial or starting a controller.
            self.shutdown()
            super().destroy_node()
            raise

    def _configure(self):
        """Validate startup-only parameters before opening the hardware connection."""
        defaults = {
            'port': 'NOT_SET', 'platform_type': 'omni',
            'heartbeat_timeout_ms': 500, 'odometry_frequency_hz': 20,
            'odometry_interval_ms': 100,
            'command_timeout_ms': 500,
        }
        for name, default in defaults.items():
            value = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
            setattr(self, name, value)
        if not self.port or self.port == 'NOT_SET':
            raise ValueError('Set the port parameter to the controller serial device')
        if self.platform_type not in ('omni', 'mecanum'):
            raise ValueError('platform_type must be omni or mecanum')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('This physical driver requires use_sim_time=false')
        self.add_on_set_parameters_callback(self._validate_clock_parameter)
        for name, minimum, maximum in (
                ('heartbeat_timeout_ms', 100, 60000),
                ('command_timeout_ms', 100, 2000),
                ('odometry_frequency_hz', 1, 1000),
                ('odometry_interval_ms', 2, 3600000)):
            value = getattr(self, name)
            if type(value) is not int or not minimum <= value <= maximum:
                raise ValueError(f'{name} must be an integer in {minimum}..{maximum}')
        period_ms = 1000 // self.odometry_frequency_hz
        if self.odometry_interval_ms < 2 * period_ms:
            raise ValueError('Subscription interval must span at least two calculations')

    def _validate_clock_parameter(self, parameters):
        """Keep ROS time in the same domain as the synchronized hardware clock."""
        for parameter in parameters:
            if parameter.name == 'use_sim_time' and parameter.value:
                return SetParametersResult(
                    successful=False, reason='Physical controller timestamps use Unix time')
        return SetParametersResult(successful=True)

    def _initialize_platform(self):
        """Preserve existing geometry, encoder calibration and motor polarity."""
        if self.platform_type == 'omni':
            self.kinisi_controller.initialize_omni_platform(
                is_reversed_0=True, is_reversed_1=True, is_reversed_2=True,
                is_encoder_reversed_0=False, is_encoder_reversed_1=False,
                is_encoder_reversed_2=False, wheels_diameter=0.096,
                robot_radius=0.185, encoder_resolution=1425.1)
        else:
            self.kinisi_controller.initialize_mecanum_platform(
                is_reversed_0=False, is_reversed_1=False, is_reversed_2=False,
                is_reversed_3=False, is_encoder_reversed_0=False,
                is_encoder_reversed_1=False, is_encoder_reversed_2=False,
                is_encoder_reversed_3=False, length=0.5, width=0.4,
                wheels_diameter=0.1, encoder_resolution=1425.1)

    def _require_ready(self):
        """Reject a failed session instead of publishing cached data or restarting motion."""
        if not self.kinisi_controller.ready:
            raise ConnectionClosedError(
                f'Controller session lost: {self.kinisi_controller.last_error}')

    def reset_odometry_callback(self, msg):
        """Flush pre-reset telemetry before accepting new poses."""
        if self._closed:
            return
        try:
            self._require_ready()
            # The unsubscribe ACK clears the SDK cache after earlier streamed frames.
            self.kinisi_controller.unsubscribe_odometry(PLATFORM_SOURCE)
            self.kinisi_controller.reset_platform_odometry()
            self._odometry.reset()
            self.kinisi_controller.subscribe_odometry(
                PLATFORM_SOURCE, self.odometry_interval_ms)
        except KinisiError:
            self.shutdown()
            raise

    def odom_publish_callback(self):
        """Publish each fresh cached sample once, using measurement time for odom/TF."""
        if self._closed:
            return
        try:
            self._require_ready()
            sample = self.kinisi_controller.get_subscription_sample(PLATFORM_SOURCE)
            if sample is None:
                return
            twist = self._odometry.update(sample)
            if twist is None:
                return
            if sample.clock_quality != self._clock_quality:
                if sample.clock_quality == ClockQuality.STALE:
                    self.get_logger().warning('Time sync is stale; using the last mapping.')
                self._clock_quality = sample.clock_quality
            self._publish_sample(sample, twist)
        except (KinisiError, ValueError):
            self.shutdown()
            raise

    def _publish_sample(self, sample, twist):
        """Publish world-frame pose and body-frame twist with one acquisition timestamp."""
        seconds, microseconds = divmod(sample.timestamp_us, 1_000_000)
        stamp = Time(sec=seconds, nanosec=microseconds * 1000)
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = sample.x
        odom.pose.pose.position.y = sample.y
        # Planar yaw needs no external quaternion conversion dependency.
        odom.pose.pose.orientation.z = math.sin(sample.t / 2)
        odom.pose.pose.orientation.w = math.cos(sample.t / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = twist[:2]
        odom.twist.twist.angular.z = twist[2]
        self.odom_publisher.publish(odom)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = sample.x
        transform.transform.translation.y = sample.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def cmd_vel_callback(self, msg):
        """Forward finite body-frame velocities only while the session is ready."""
        if self._closed:
            return
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Ignoring a non-finite cmd_vel')
            return
        received = time.monotonic()
        try:
            with self._command_io_lock:
                with self._command_lock:
                    if self._closed:
                        return
                    # Do not revive a command that waited behind blocked I/O.
                    if time.monotonic() - received >= self.command_timeout_ms / 1000:
                        return
                    self._require_ready()
                    self._last_command_at = received
                    self._command_inflight_at = time.monotonic()
                    self._command_active = self._command_active or any(values)
                # Never hold the watchdog state lock across a synchronous ACK.
                self.kinisi_controller.set_platform_target_velocity(*values)
                with self._command_lock:
                    self._command_inflight_at = None
                    if not self._closed:
                        self._command_active = any(values)
        except KinisiError:
            self._abort_command_session('Velocity request failed; revoking controller session')
            raise

    def _check_command_age(self):
        """Expire latched motion independently of ROS callbacks and SDK heartbeats."""
        with self._command_lock:
            if self._closed:
                return
            now = time.monotonic()
            stalled = (self._command_inflight_at is not None
                       and now - self._command_inflight_at >= self.command_timeout_ms / 1000)
            expired = (self._command_active and self._last_command_at is not None
                       and now - self._last_command_at >= self.command_timeout_ms / 1000)
            queue_stop = expired and self._command_inflight_at is None and not self._expiry_pending
            if queue_stop:
                self._expiry_pending = True
        if stalled:
            self._abort_command_session('Velocity ACK deadline exceeded; revoking controller session')
        elif queue_stop:
            # The watchdog must also remain live if the zero command stalls.
            threading.Thread(target=self._expire_command, daemon=True,
                             name='kinisi-expire-command').start()

    def _expire_command(self):
        """Serialize a stop with writes, rechecking for a newer operator command."""
        try:
            with self._command_io_lock:
                with self._command_lock:
                    if (self._closed or not self._command_active
                            or time.monotonic() - self._last_command_at < self.command_timeout_ms / 1000):
                        return
                    self._command_inflight_at = time.monotonic()
                self.kinisi_controller.set_platform_target_velocity(0.0, 0.0, 0.0)
                with self._command_lock:
                    self._command_inflight_at = None
                    self._command_active = False
                self.get_logger().warning('Velocity command expired; motor target set to zero')
        except Exception:
            self._abort_command_session('Unable to acknowledge zero velocity; revoking controller session')
        finally:
            with self._command_lock:
                self._expiry_pending = False

    def _abort_command_session(self, reason):
        """Stop SDK heartbeats even while a velocity request is awaiting its ACK.

        Disconnect cancels pending SDK I/O. The controller's configured hardware
        heartbeat timeout is the final physical stop bound, not this host timer.
        A failed session is never automatically rearmed.
        """
        with self._command_lock:
            if self._closed:
                return
            self._closed = True
            self._command_stop.set()
        if self.kinisi_controller is not None:
            self.kinisi_controller.disconnect()
        if self.odom_timer is not None:
            self.odom_timer.cancel()
        self.get_logger().error(reason)

    def _command_watchdog(self):
        """Stop stale commands even if the ROS executor stops processing callbacks."""
        while not self._command_stop.wait(0.05):
            try:
                self._check_command_age()
            except Exception as exc:
                self.get_logger().error(f'Command watchdog failed; closing controller: {exc}')
                self._abort_command_session('Command watchdog failed')
                return

    def shutdown(self):
        """Revoke a stalled request or stop outputs before normal teardown."""
        with self._command_lock:
            if self._closed:
                return
            inflight = self._command_inflight_at is not None
            if not inflight:
                self._closed = True
                self._command_stop.set()
        if inflight:
            self._abort_command_session('Shutdown during velocity request')
            return
        if self.odom_timer is not None:
            self.odom_timer.cancel()
        controller = self.kinisi_controller
        if controller is None:
            return
        try:
            if controller.serial is not None:
                for operation in (controller.stop_platform_controller, controller.coast_platform):
                    try:
                        operation()
                    except KinisiError as error:
                        self.get_logger().warning(f'Could not complete motor stop: {error}')
                if controller.ready:
                    try:
                        controller.unsubscribe_odometry(PLATFORM_SOURCE)
                    except KinisiError as error:
                        self.get_logger().warning(f'Could not unsubscribe: {error}')
        finally:
            controller.disconnect()

    def destroy_node(self):
        """Release the hardware connection before destroying ROS resources."""
        self.shutdown()
        return super().destroy_node()


def main(args=None):
    """Run the driver and always disconnect on shutdown or callback failure."""
    rclpy.init(args=args)
    node = None
    try:
        node = KinisiControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
