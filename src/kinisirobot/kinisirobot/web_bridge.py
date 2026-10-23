#!/usr/bin/env python3
"""Kinisi robot web bridge.

A ROS 2 node + HTTP server that lets a browser:
  * see the live SLAM map (/map) with the robot's pose, and
  * drive the robot with WASD/QE (publishes geometry_msgs/Twist on /cmd_vel).

Run it ON THE ROBOT PI (needs access to the ROS graph), with the ROS env sourced:

    source /opt/ros/humble/setup.bash
    source ~/development/kinisiros/install/setup.bash
    ros2 run kinisirobot kinisi_web_bridge --port 8080

Then open http://<pi-host>:8080/ in a browser on the same network.

Only depends on rclpy + the Python stdlib (no aiohttp/websockets needed).
"""

import argparse
import copy
import gzip
import json
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
from collections import deque
from functools import wraps
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, quote, urlparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptyService
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
import tf2_ros

from ament_index_python.packages import get_package_share_directory

from .webrtc_camera import CameraUnavailable, WebRTCCamera
from .map_bundle import (
    MAX_BUNDLE_BYTES,
    MapBundleError,
    create_map_bundle,
    install_map_bundle,
    safe_map_name,
)
from .map_localization import (
    MappingReadiness,
    GlobalLocalizationEvidence,
    PoseContinuity,
    REQUIRED_CONFIDENT_SAMPLES,
    localization_quality,
    seed_covariance,
    scan_alignment,
)
from .robot_settings import RobotSettings, map_identity
from .localization_motion import LocalizationMotion, rotation_clearance_evidence, costmap_rotation_clearance
from .localization_config import DEFAULT_LOCALIZATION_CONFIG, sensor_health
from .owned_backend import OwnedBackend
from .mapping_scan import normalize_mapping_scan
from .localization_seed import refine_seed

HERE = os.path.dirname(os.path.abspath(__file__))
try:
    PACKAGE_SHARE = get_package_share_directory("kinisirobot")
except Exception:  # Source-tree execution before a colcon install.
    PACKAGE_SHARE = os.path.dirname(HERE)
INDEX_HTML = os.path.join(PACKAGE_SHARE, "web", "index.html")
DEFAULT_SLAM_PARAMS = os.path.join(
    PACKAGE_SHARE, "config", "mapper_params_online_async.yaml"
)
MAPS_DIR = os.path.expanduser("~/development/kinisiros/maps")


def serialized_map_operation(method):
    """Keep uploads, resets, saves and backend handoffs from replacing each other."""
    @wraps(method)
    def locked(self, *args, **kwargs):
        """Hold one reentrant lock for a complete map operation."""
        with self._map_operation_lock:
            if self._closing:
                raise RuntimeError("Robot bridge is shutting down")
            return method(self, *args, **kwargs)
    return locked


class WebBridge(Node):
    def __init__(self, slam_params=DEFAULT_SLAM_PARAMS, camera=None,
                 localization_config=DEFAULT_LOCALIZATION_CONFIG, update_loaded_map=False):
        """Wire ROS and HTTP-facing operations to the robot localization profile."""
        super().__init__("web_bridge")
        self._lock = threading.Lock()
        self._closing = False
        self._map = None
        self.slam_params = slam_params
        self.localization_config = localization_config
        self._slam_backend = OwnedBackend()
        self._localization_backend = OwnedBackend()
        self._sensor_health = {}

        # /map from slam_toolbox is latched (reliable + transient_local).
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        # Nav2 costmaps (also latched/transient_local like /map). Shown as an
        # optional translucent overlay in the web UI: global = whole-map cost
        # (static obstacles + inflation), local = live rolling window around
        # the robot (fresh obstacle detections).
        self._global_costmap = None
        self._local_costmap = None
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_global_costmap, map_qos)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_local_costmap, map_qos)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_navigation_velocity, 10)
        self._reset_odom_pub = self.create_publisher(Empty, "/reset_odometry", 10)

        # --- Autonomous navigation (Nav2) ---------------------------------
        # AMCL initial-pose seed, the planned path for display, and a
        # NavigateToPose action client for "click a goal -> drive there".
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._plan = None
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_lock = threading.Lock()
        self._nav_goal_handle = None
        self._nav_state = "idle"  # idle|pending|active|succeeded|failed|canceled|no_server
        self._nav_goal = None     # {x, y, yaw}
        self._queued_nav_goal = None
        self._nav_request_id = 0

        # Loading a map uses a read-only AMCL phase before SLAM is allowed to
        # add scans. Several confident AMCL poses trigger the transition back
        # to update-map mode at the localized pose.
        self._map_phase_lock = threading.Lock()
        self._map_operation_lock = threading.RLock()
        self._map_phase = "normal"
        self._map_phase_error = None
        self._localization_generation = 0
        self._localization_samples = 0
        self._localization_quality = None
        self._localization_map_base = None
        self.settings = RobotSettings(os.path.expanduser("~/.config/kinisi/robot_settings.json"))
        self._active_map_id = None
        self._last_pose_save_at = 0.0
        self.create_timer(5.0, self._remember_current_pose)
        self._pose_seed_ns = 0
        self._last_amcl_stamp_ns = 0
        self._localization_started_at = None
        self._localization_fresh_since = None
        self._global_evidence = GlobalLocalizationEvidence()
        self._pose_continuity = PoseContinuity()
        self._runtime_continuity = PoseContinuity(anchored=False)
        self._update_loaded_map = update_loaded_map
        self._map_update_error = None
        self._mapping_diagnostics = None
        self._slam_candidate = None
        self._amcl_candidate = None
        self._mapping_good_samples = 0
        self._mapping_bad_samples = 0
        self._last_mapping_stamp_ns = 0
        self._last_localization_observation_ns = 0
        self._last_runtime_checked_ns = 0
        self._fresh_mapping_good_samples = 0
        self._runtime_bad_samples = 0
        self._localization_backend_started_ns = 0
        self._amcl_map_ready_ns = 0
        self.create_subscription(Log, "/rosout", self._on_amcl_log, 100)
        self._scans = deque(maxlen=90)
        self._mapping_scan_pub = self.create_publisher(LaserScan, "/scan_mapping", qos_profile_sensor_data)
        self._scan_layout_error = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self._mapping_readiness = None
        self._mapping_process = None
        self._mapping_started_at = None
        self._mapping_reference_grid = None
        self._pending_mapping_grid = None
        self._last_map_at = None
        # Serialize automatic commands with STOP and manual takeover. The
        # routine uses odometry for limits, since AMCL is still uncertain.
        self._motion_lock = threading.RLock()
        self._localization_motion = None
        self._auto_localization_pending = False
        self._auto_localization_cancelled = False
        self._motion_stop_latched = False
        self._allow_localization_motion = True
        self._transient_motion_hold = False
        self._motion_recovery_since = None
        self._motion_retries = 0
        self._auto_localization_message = ""
        self._localization_search_scope = "remembered"
        self._motion_blocker = "Waiting for live sensor data"
        self._motion_settled_after_ns = 0
        self._motion_odom = None
        self.create_subscription(Odometry, "/odom", self._on_motion_odom, 10)
        self.create_timer(0.05, self._tick_localization_motion)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self._on_slam_pose, 10)
        self.create_timer(1.0, self._check_mapping_startup)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            10,
        )
        self._amcl_state_client = self.create_client(GetState, "/amcl/get_state")
        self._nomotion_client = self.create_client(EmptyService, "/request_nomotion_update")
        self._nomotion_future = None
        self._nomotion_requested_at = 0.0
        self._global_localization_client = self.create_client(EmptyService, "/reinitialize_global_localization")
        self.create_timer(1.0, self._update_stationary_localization)
        self.create_timer(0.2, self._monitor_navigation_health)

        # Emergency-stop generation counter. force_stop() briefly spams zero
        # Twists (in a background thread) to beat Nav2's ~20 Hz /cmd_vel stream
        # and the *latching* base, which otherwise keeps executing Nav2's last
        # non-zero command after a cancel. A real movement command (drive()
        # with non-zero velocity) bumps the counter so teleop immediately
        # supersedes an in-flight stop-spam.
        self._stop_gen = 0
        self._stop_lock = threading.Lock()

        # --- Front-end heartbeat / dead-man's switch ----------------------
        # The browser UI POSTs /heartbeat a few times per second. If the beats
        # stop (tab closed, browser crash, Wi-Fi drop) the watchdog issues a
        # safety stop, because the base LATCHES its last velocity and would
        # otherwise keep driving (e.g. into a wall) with no operator connected.
        self._hb_lock = threading.Lock()
        self._hb_last = 0.0          # time.monotonic() of the last heartbeat
        self._hb_timeout = 1.5       # seconds of silence -> considered gone
        self._hb_enabled = False     # armed once the first heartbeat arrives
        self._hb_alive = False       # current connected state (edge-triggered)
        threading.Thread(target=self._heartbeat_watchdog, daemon=True).start()
        self.create_timer(0.5, self._dispatch_queued_nav_goal)

        # USB-camera WebRTC transport. The V4L2 device opens only after a
        # browser requests a stream and is released with the final peer.
        self.camera = camera or WebRTCCamera()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            "web_bridge node ready (subscribed /map, publishing /cmd_vel)"
        )

    def _on_map(self, msg):
        """Cache the map and confirm actual SLAM output during startup."""
        with self._map_phase_lock:
            verifying = self._map_phase in ('starting_mapping', 'waiting_mapping', 'localization_error', 'recovering_localization') and self._mapping_reference_grid is not None
        with self._lock:
            if verifying:
                if self._mapping_readiness is None or self._stamp_ns(msg.header) <= self._mapping_readiness.started_ns:
                    return
                self._pending_mapping_grid = msg
            else:
                self._map = msg
                self._last_map_at = time.monotonic()
        if msg.info.width > 0 and msg.info.height > 0:
            self._observe_mapping_output("map", msg.header)

    def _on_slam_pose(self, msg):
        """Confirm a new scan-matched SLAM pose after switching backends."""
        if msg.header.frame_id == self.localization_config.map_frame:
            self._slam_candidate = msg
            with self._map_phase_lock:
                verifying = self._map_phase in ('starting_mapping', 'waiting_mapping') and self._mapping_reference_grid is not None
                generation = self._localization_generation
            if verifying:
                if self._mapping_readiness is None or self._stamp_ns(msg.header) <= self._mapping_readiness.started_ns:
                    return
                quality = self._scan_quality(msg, reference_grid=self._mapping_reference_grid)
                if quality.get('scan_agrees'):
                    p = msg.pose.pose
                    odom_pose = self._odometry_at(msg.header.stamp)
                    if odom_pose is None:
                        return
                    quality['pose_consistent'] = self._pose_continuity.observe(
                        (p.position.x, p.position.y, self._yaw(p.orientation)), odom_pose)
                    quality['scan_agrees'] = quality['pose_consistent']
                self._mapping_diagnostics = {**quality, 'pose': {
                    'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
                    'yaw': self._yaw(msg.pose.pose.orientation)}}
                stamp = self._stamp_ns(msg.header)
                if stamp <= self._last_mapping_stamp_ns:
                    return
                self._last_mapping_stamp_ns = stamp
                if quality.get('scan_match') is None:
                    return  # Missing evidence is not a measured mismatch.
                if not quality.get('scan_agrees'):
                    self._mapping_good_samples = 0
                    self._mapping_bad_samples += 1
                    self.get_logger().warning(f'SLAM verification: {self._mapping_diagnostics}')
                    if self._mapping_bad_samples >= 3:
                        self._recover_mapping('SLAM scans disagree with the saved map')
                    return
                self._mapping_bad_samples = 0
                self._mapping_good_samples += 1
                if self._mapping_good_samples < REQUIRED_CONFIDENT_SAMPLES:
                    return
            if self._map_phase == 'updating':
                self._record_runtime_observation(msg)
            self._observe_mapping_output("pose", msg.header)
            if self.map_mode() in ("normal", "updating"):
                pose = msg.pose.pose
                self._remember_pose((pose.position.x, pose.position.y, self._yaw(pose.orientation)))

    def _remember_pose(self, pose, force=False):
        """Save trusted poses at most every five seconds to limit disk writes."""
        if not self._active_map_id or not self._localization_map_base:
            return
        if not force and time.monotonic() - self._last_pose_save_at < 5.0:
            return
        try:
            self.settings.remember_pose(self._active_map_id,
                                        os.path.basename(self._localization_map_base), pose)
            self._last_pose_save_at = time.monotonic()
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"Could not remember robot position: {exc}")

    def _remember_current_pose(self):
        """Capture the settled pose even when SLAM stops emitting stationary scans."""
        if self.map_mode() != "updating" or not self._active_map_id:
            return
        try:
            transform = self._tf_buffer.lookup_transform(self.localization_config.map_frame, self.localization_config.base_frame, rclpy.time.Time())
            age = self.get_clock().now().nanoseconds - self._stamp_ns(transform.header)
            if not 0 <= age <= 2_000_000_000:
                return
            p = transform.transform
            self._remember_pose((p.translation.x, p.translation.y, self._yaw(p.rotation)))
        except Exception as exc:
            self.get_logger().debug(f"Position not ready to remember: {exc}")

    def _on_scan(self, msg):
        """Retain raw scans for AMCL checks and publish a stable layout for SLAM."""
        with self._lock:
            self._scans.append(msg)
        if (self._slam_candidate is not None
                and self._map_phase in ('starting_mapping', 'waiting_mapping', 'updating', 'mapping_paused')):
            self._corroborate_slam_pose(msg)
        if self._map_phase in ('mapping_paused', 'mapping_error'):
            return  # Preserve the unsaved graph until fresh evidence is verified.
        try:
            normalized = normalize_mapping_scan(msg, self.localization_config.mapping_scan_samples)
            self._mapping_scan_pub.publish(normalized)
            self._scan_layout_error = None
        except ValueError as exc:
            if self._scan_layout_error != str(exc):
                self.get_logger().warning(f"Mapping scan rejected: {exc}")
            self._scan_layout_error = str(exc)

    def _corroborate_slam_pose(self, scan):
        """Verify the real SLAM output with fresh scans even while the base stays still."""
        candidate = self._slam_candidate
        if self._stamp_ns(scan.header) <= self._stamp_ns(candidate.header):
            return
        observation = copy.deepcopy(candidate)
        if self._map_phase in ('starting_mapping', 'waiting_mapping'):
            if (self._stamp_ns(scan.header) - self._stamp_ns(candidate.header) > 3_000_000_000
                    or not self._stationary_observation(self._stamp_ns(scan.header))):
                return
            # Only stationary corroboration may reuse a single startup pose.
            observation.header.stamp = scan.header.stamp
            self._on_slam_pose(observation)
            self._slam_candidate = candidate
        else:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.localization_config.map_frame, self.localization_config.base_frame,
                    rclpy.time.Time.from_msg(scan.header.stamp)).transform
            except Exception:
                return
            observation.header.stamp = scan.header.stamp
            observation.pose.pose.position.x = transform.translation.x
            observation.pose.pose.position.y = transform.translation.y
            observation.pose.pose.orientation = transform.rotation
            self._record_runtime_observation(observation)

    def _on_motion_odom(self, msg):
        """Cache measured base movement for the bounded localization routine."""
        with self._lock:
            self._motion_odom = msg
        # Include manual/external motion between AMCL callbacks in the settling
        # window, so delayed stationary observations cannot bypass it.
        with self._motion_lock:
            _, health, stationary = self._odometry_state()
            if health["state"] != "ready" or not stationary:
                self._motion_settled_after_ns = (self.get_clock().now().nanoseconds
                    + int(self.localization_config.settle_s * 1e9))

    def _on_amcl_log(self, msg):
        """Observe Humble AMCL's map-receipt acknowledgement for this backend.

        Humble's global-localization service dereferences its map without a
        null check. Active lifecycle state alone does not prove map receipt.
        This acknowledgement is emitted while AMCL holds its map mutex; the
        subsequent service uses the same mutex, so conversion finishes first.
        Missing/changed diagnostics fail closed instead of risking a crash.
        """
        if msg.name != "amcl" or not re.match(r"Received a \d+ X \d+ map @", msg.msg):
            return
        stamp = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        with self._map_phase_lock:
            if (self._map_phase in ("starting_localization", "requesting_localization", "localizing")
                    and stamp > self._localization_backend_started_ns):
                self._amcl_map_ready_ns = stamp

    def _wait_for_amcl_map(self, generation):
        """Wait for the new backend to consume a map containing free space."""
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            with self._map_phase_lock:
                if generation != self._localization_generation:
                    return False
                ready = self._amcl_map_ready_ns > self._localization_backend_started_ns
            with self._lock:
                grid = self._map
            if ready and grid is not None:
                if not any(value == 0 for value in grid.data):
                    self._set_localization_error(generation, "Saved map has no free space for localization")
                    return False
                return True
            time.sleep(0.1)
        self._set_localization_error(generation, "AMCL has not acknowledged the loaded map; robot remains stopped")
        return False

    def _odometry_state(self):
        """Read odometry independently of clearance so all handoffs can be guarded."""
        with self._lock:
            odom = self._motion_odom
        cfg = self.localization_config
        now_ns = self.get_clock().now().nanoseconds
        health = sensor_health(odom, now_ns, cfg.odom_max_age_s, cfg.odom_frame)
        pose = (math.nan, math.nan, math.nan)
        stationary = False
        if health["state"] == "ready":
            if odom.child_frame_id != cfg.base_frame:
                health["state"] = "wrong_child_frame"
            else:
                p, v = odom.pose.pose, odom.twist.twist
                pose = (p.position.x, p.position.y, self._yaw(p.orientation))
                stationary = (all(math.isfinite(value) for value in pose)
                              and math.hypot(v.linear.x, v.linear.y) < cfg.stationary_linear_mps
                              and abs(v.angular.z) < cfg.stationary_angular_rps)
        return pose, health, stationary

    def _stationary_observation(self, stamp_ns):
        """Require recent source scans and settled odometry for every SLAM handoff."""
        _, health, stationary = self._odometry_state()
        now_ns = self.get_clock().now().nanoseconds
        ready = health["state"] == "ready" and stationary
        if not ready:
            self._motion_settled_after_ns = now_ns + int(self.localization_config.settle_s * 1e9)
        return (ready and stamp_ns > self._motion_settled_after_ns
                and 0 <= now_ns - stamp_ns <= 1_000_000_000)

    def _odometry_at(self, stamp):
        """Pair localization with odometry at acquisition time, never latest time."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self.localization_config.odom_frame, self.localization_config.base_frame,
                rclpy.time.Time.from_msg(stamp)).transform
            pose = (transform.translation.x, transform.translation.y, self._yaw(transform.rotation))
            return pose if all(math.isfinite(v) for v in pose) else None
        except Exception:
            return None

    def _motion_inputs(self):
        """Report each sensor independently; require every motion guard to pass."""
        cfg = self.localization_config
        pose, odom_health, stationary = self._odometry_state()
        with self._lock:
            scan = self._scans[-1] if self._scans else None
            scans = list(self._scans)
            costmap = self._local_costmap
        now_ns = self.get_clock().now().nanoseconds
        health = {
            "odometry": odom_health,
            "lidar": sensor_health(scan, now_ns, cfg.scan_max_age_s),
            "costmap": sensor_health(costmap, now_ns, cfg.costmap_max_age_s, cfg.odom_frame),
            "laser_transform": {"state": "waiting"},
        }
        scan_clear = False
        coverage = {'clear': False, 'reason': 'Waiting for fresh scan coverage', 'missing_sectors': []}
        if health["lidar"]["state"] == "ready":
            try:
                mount = self._tf_buffer.lookup_transform(
                    cfg.base_frame, scan.header.frame_id, rclpy.time.Time()).transform
                health['laser_transform'] = {'state': 'ready'}
                latest = rotation_clearance_evidence([(scan, (mount.translation.x,
                    mount.translation.y, self._yaw(mount.rotation)))], cfg)
                observations = []
                for observation in reversed(scans):
                    age_ns = now_ns - self._stamp_ns(observation.header)
                    if not 0 <= age_ns <= int(min(.4, cfg.scan_max_age_s) * 1e9):
                        continue
                    try:
                        laser = self._tf_buffer.lookup_transform(
                            cfg.odom_frame, observation.header.frame_id,
                            rclpy.time.Time.from_msg(observation.header.stamp)).transform
                    except Exception:
                        continue
                    dx, dy = laser.translation.x - pose[0], laser.translation.y - pose[1]
                    c, s = math.cos(pose[2]), math.sin(pose[2])
                    observations.append((observation, (c*dx+s*dy, -s*dx+c*dy,
                                                       self._yaw(laser.rotation)-pose[2])))
                    if len(observations) >= 6:
                        break
                # A newly arrived close return always vetoes old clear views,
                # even if its odometry TF has not arrived yet. Only time-paired
                # scans contribute positive clearance evidence.
                coverage = latest if latest['blocked'] else rotation_clearance_evidence(observations, cfg)
                scan_clear = coverage['clear']
            except Exception:
                health["laser_transform"] = {"state": "missing"}
        fresh = all(item["state"] == "ready" for item in health.values())
        clear = (fresh and scan_clear and costmap_rotation_clearance(costmap, pose, cfg))
        labels = {"odometry": "Odometry", "lidar": "LiDAR", "costmap": "Local costmap",
                  "laser_transform": "Laser transform"}
        problems = [f"{labels[name]}: {item['state'].replace('_', ' ')}"
                    for name, item in health.items() if item["state"] != "ready"]
        self._sensor_health = health
        health['lidar']['coverage'] = coverage
        self._motion_blocker = ("; ".join(problems) + ". Robot remains stopped" if problems else
                               "" if clear else (coverage['reason'] if not scan_clear else
                                                  'Costmap obstacle or unknown space inside the footprint')
                               + '; robot remains stopped')
        with self._hb_lock:
            connected = self._hb_alive and time.monotonic() - self._hb_last < self._hb_timeout
        if fresh and clear and not connected:
            self._motion_blocker = "Waiting for the client connection; robot remains stopped"
        elif fresh and clear and not stationary:
            self._motion_blocker = "Waiting for the robot to settle"
        return pose, fresh, clear, connected, stationary

    def resume_localization(self):
        """Explicitly rearm a paused session without reloading its saved map."""
        with self._motion_lock:
            with self._map_phase_lock:
                if self._map_phase != "localizing":
                    raise ValueError("No localization session is available to resume")
                if not self._auto_localization_cancelled:
                    raise ValueError("Localization is not paused")
                self._localization_samples = 0
                self._localization_started_at = time.monotonic()
                self._pose_seed_ns = self.get_clock().now().nanoseconds
            self._localization_motion = None
            self._auto_localization_cancelled = False
            self._motion_stop_latched = False
            self._auto_localization_pending = True
            self._auto_localization_message = "Checking sensors before resuming localization"

    def start_localization_motion(self):
        """Start one short turn sequence once automatic recovery guards pass."""
        with self._motion_lock:
            with self._map_phase_lock:
                if self._map_phase != "localizing":
                    raise ValueError("Localization is not ready for movements")
            if self._localization_motion and self._localization_motion.active:
                raise ValueError("Localization movements are already running")
            with self._nav_lock:
                if self._nav_state in ("pending", "active"):
                    raise ValueError("Cancel navigation before localization movements")
            pose, fresh, clear, connected, stationary = self._motion_inputs()
            if not (fresh and connected and stationary and all(math.isfinite(v) for v in pose)):
                raise ValueError("A connected client, fresh sensors and a stationary robot are required")
            if not clear:
                raise ValueError("LiDAR must show clearance around the robot before turning")
            with self._stop_lock:
                self._stop_gen += 1
            self._localization_motion = LocalizationMotion(time.monotonic(), pose)

    def _automatic_localization(self):
        """Automatically help one map load; never restart after STOP or a fault."""
        with self._motion_lock:
            if not self._auto_localization_pending or self._auto_localization_cancelled:
                return
            if self._motion_stop_latched or not self._allow_localization_motion:
                return
            with self._map_phase_lock:
                if self._map_phase != "localizing" or self._localization_started_at is None:
                    return
                elapsed = time.monotonic() - self._localization_started_at
            if elapsed < 3.0:
                return  # Give stationary matching a chance before moving.
            if elapsed > 120:
                self._auto_localization_message = ('Localization needs more usable views; '
                                                   'robot remains stopped. Check LiDAR coverage and the selected map.')
                return
            self._motion_inputs()
            self._auto_localization_message = self._motion_blocker
            try:
                self.start_localization_motion()
            except ValueError:
                return  # Missing sensors/clearance: stay stopped and report why.
            self._auto_localization_pending = False
            self._auto_localization_message = "Automatically gathering views for localization"

    def _cancel_localization_motion(self, reason="Localization movements stopped"):
        """Latch cancellation before another automatic command can publish."""
        with self._motion_lock:
            self._auto_localization_pending = False
            self._auto_localization_cancelled = True
            self._auto_localization_message = reason
            if self._localization_motion and self._localization_motion.active:
                self._localization_motion.stop(reason)
                self._publish_velocity(0.0, 0.0, 0.0)

    def _tick_localization_motion(self):
        """Issue bounded turns at 20 Hz, checking all guards on every command."""
        with self._motion_lock:
            motion = self._localization_motion
            if motion is None or not motion.active:
                return
            try:
                with self._map_phase_lock:
                    localizing = self._map_phase == "localizing"
                if not localizing:
                    self._cancel_localization_motion("Stopped: localization phase changed")
                    return
                pose, fresh, clear, connected, _ = self._motion_inputs()
                az = motion.step(time.monotonic(), pose, sensors_fresh=fresh,
                                 clear=clear, connected=connected)
                if az:
                    self._motion_settled_after_ns = self.get_clock().now().nanoseconds + int(self.localization_config.settle_s * 1e9)
                self._publish_velocity(0.0, 0.0, az)
                if not motion.active:
                    self._transient_motion_hold = any(word in motion.reason for word in (
                        'disconnected', 'fresh LiDAR', 'incomplete LiDAR', 'timing interrupted'))
                    self.force_stop()
            except Exception as exc:
                self._cancel_localization_motion("Stopped: localization motion error")
                self.force_stop()
                self.get_logger().error(f"Localization movement failed: {exc}")

    def _scan_quality(self, msg, reference_grid=None):
        """Reject a confident prior when measured surfaces disagree with the map."""
        with self._lock:
            grid = reference_grid if reference_grid is not None else self._map
            scans = list(self._scans)
        stamp_ns = self._stamp_ns(msg.header)
        scan = next((s for s in reversed(scans) if self._stamp_ns(s.header) == stamp_ns), None)
        if grid is None or scan is None:
            return {"scan_agrees": False, "scan_match": None, "reason": "Waiting for the time-paired scan and map"}
        try:
            laser = self._tf_buffer.lookup_transform(
                self.localization_config.base_frame, scan.header.frame_id, rclpy.time.Time()
            ).transform
        except Exception:
            return {"scan_agrees": False, "scan_match": None, "reason": "Waiting for the laser transform"}
        pose = msg.pose.pose
        yaw = self._yaw(pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        laser_pose = (pose.position.x + c * laser.translation.x - s * laser.translation.y,
                      pose.position.y + s * laser.translation.x + c * laser.translation.y,
                      yaw + self._yaw(laser.rotation))
        origin = grid.info.origin
        return scan_alignment(scan.ranges, scan.angle_min, scan.angle_increment,
                              scan.range_min, scan.range_max, laser_pose,
                              grid.info.width, grid.info.height, grid.info.resolution,
                              (origin.position.x, origin.position.y, self._yaw(origin.orientation)),
                              grid.data)

    @staticmethod
    def _stamp_ns(header):
        """Convert a message timestamp into integer nanoseconds."""
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _observe_mapping_output(self, kind, header):
        """Enable navigation only after the new SLAM session publishes."""
        stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        with self._map_phase_lock:
            if self._map_phase not in ("starting_mapping", "waiting_mapping") or self._mapping_readiness is None:
                return
            if self._mapping_process is not None and self._mapping_process.poll() is not None:
                return  # A dead backend cannot recover merely from queued outputs.
            self._mapping_readiness.observe(kind, stamp_ns)
            if self._mapping_readiness.ready:
                self._map_phase = "updating"
                self._last_localization_observation_ns = stamp_ns
                self._map_phase_error = None
                self._mapping_reference_grid = None
                with self._lock:
                    if self._pending_mapping_grid is not None:
                        self._map = self._pending_mapping_grid
                        self._pending_mapping_grid = None
                        self._last_map_at = time.monotonic()

    def _check_mapping_startup(self):
        """Report failed SLAM startup instead of showing a false active state."""
        with self._map_phase_lock:
            if self._map_phase not in ("starting_mapping", "waiting_mapping", "updating", "mapping_paused"):
                return
            process = self._mapping_process
            failed = process is not None and process.poll() is not None
            timed_out = (self._map_phase == "starting_mapping"
                         and self._mapping_started_at is not None
                         and time.monotonic() - self._mapping_started_at > 45.0)
            if failed or timed_out:
                self._map_phase = "localization_error" if failed else "waiting_mapping"
                self._map_phase_error = (
                    "Map updates stopped. Reload the saved map to retry."
                    if failed else "Still waiting for fresh SLAM data. Startup will continue automatically."
                )
            recover = failed or (self._mapping_started_at is not None
                                and self._map_phase == 'waiting_mapping'
                                and time.monotonic() - self._mapping_started_at > 60)
        if recover:
            self._recover_mapping('SLAM stopped or could not verify startup before the deadline')

    def _on_global_costmap(self, msg):
        with self._lock:
            self._global_costmap = msg

    def _on_local_costmap(self, msg):
        with self._lock:
            self._local_costmap = msg

    def _on_plan(self, msg):
        # Downsample the planned path to keep the JSON payload small.
        pts = msg.poses
        step = max(1, len(pts) // 200)
        with self._nav_lock:
            self._plan = [
                {"x": p.pose.position.x, "y": p.pose.position.y}
                for p in pts[::step]
            ]

    # ---- Autonomous navigation ------------------------------------------
    @staticmethod
    def _yaw_to_quat(yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w); x=y=0

    def set_initial_pose(self, x, y, yaw, automatic=False):
        """Search near a remembered or operator-selected map-frame pose."""
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("Pose must contain finite x, y and heading")
        if not self._initialpose_pub.get_subscription_count():
            raise ValueError("Localization is not ready to receive a pose yet")
        if not automatic:
            self._cancel_localization_motion("Stopped: initial pose changed")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.localization_config.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qz, qw = self._yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = seed_covariance(self.settings.snapshot()["localization_radius_m"])
        with self._map_phase_lock:
            if not automatic:
                self._localization_search_scope = 'manual'
                self._global_evidence = GlobalLocalizationEvidence()
            self._pose_seed_ns = self._stamp_ns(msg.header)
            self._last_amcl_stamp_ns = self._pose_seed_ns
            self._localization_samples = 0
            self._localization_quality = None
            self._pose_continuity = PoseContinuity()
            self._localization_started_at = time.monotonic()
            self._localization_fresh_since = None
            if self._map_phase in ("awaiting_pose", "localizing", "requesting_localization"):
                self._map_phase = "localizing"
        self._initialpose_pub.publish(msg)
        self.get_logger().info(f"Published /initialpose x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def request_nav_goal(self, x, y, yaw):
        """Accept a destination now; defer navigation until the map is ready."""
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return False
        with self._map_operation_lock, self._motion_lock:
            phase = self.map_mode()
            if phase in ("localization_error", 'mapping_error', "unknown", "static"):
                return False
            # One intent owns both immediate and delayed dispatch.
            with self._nav_lock:
                old_handle = self._nav_goal_handle
                self._nav_request_id += 1
                self._queued_nav_goal = None
                self._nav_goal_handle = None
                self._nav_goal = None
                self._nav_state = 'idle'
            if old_handle is not None:
                old_handle.cancel_goal_async()
                self.force_stop()
            if phase not in ("localized", "updating"):
                with self._map_phase_lock:
                    generation = self._localization_generation
                with self._nav_lock:
                    self._nav_request_id += 1
                    self._nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
                    self._queued_nav_goal = (generation, dict(self._nav_goal))
                    self._nav_state = "waiting_localization"
                return True
            return self.send_nav_goal(x, y, yaw)

    def _dispatch_queued_nav_goal(self):
        """Dispatch once, under the same lock as Stop; never rearm stopped motion."""
        if not self._map_operation_lock.acquire(blocking=False):
            return
        try:
            with self._motion_lock:
                with self._nav_lock:
                    queued = self._queued_nav_goal
                if queued is None or self._closing:
                    return
                generation, goal = queued
                with self._map_phase_lock:
                    phase = self._map_phase
                    current_generation = self._localization_generation
                with self._hb_lock:
                    alive = (self._hb_alive and
                             time.monotonic() - self._hb_last <= self._hb_timeout)
                if generation != current_generation or not alive or phase == "localization_error":
                    self.cancel_nav()
                    return
                if phase not in ("localized", "updating") or not self._nav_client.server_is_ready():
                    return
                with self._nav_lock:
                    self._queued_nav_goal = None
                self.send_nav_goal(**goal)
        finally:
            self._map_operation_lock.release()

    def send_nav_goal(self, x, y, yaw):
        """Send a NavigateToPose goal (map frame)."""
        if self.map_mode() not in ('localized', 'updating'):
            self.get_logger().warn("Navigation goal rejected while localizing")
            return False
        if not self._navigation_inputs_ready():
            with self._nav_lock:
                self._nav_state = 'blocked'
                self._nav_goal = None
            return False
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            with self._nav_lock:
                self._nav_state = "no_server"
            self.get_logger().warn("navigate_to_pose action server unavailable")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.localization_config.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = self._yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        with self._nav_lock:
            self._nav_request_id += 1
            request_id = self._nav_request_id
            self._nav_state = "pending"
            self._nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda future: self._on_goal_response(future, request_id))
        self.get_logger().info(f"Sent nav goal x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        return True

    def _on_goal_response(self, future, request_id):
        handle = future.result()
        with self._motion_lock, self._nav_lock:
            if request_id != self._nav_request_id:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._nav_state = "failed"
                return
            self._nav_goal_handle = handle
            self._nav_state = "active"
            with self._stop_lock:
                self._stop_gen += 1
        handle.get_result_async().add_done_callback(
            lambda future: self._on_goal_result(future, request_id))

    def _on_goal_result(self, future, request_id):
        status = future.result().status
        # action_msgs/GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        mapping = {4: "succeeded", 5: "canceled", 6: "failed"}
        with self._nav_lock:
            if request_id != self._nav_request_id:
                return
            self._nav_state = mapping.get(status, "failed")
            self._nav_goal_handle = None

    def cancel_nav(self, preserve_waiting=False, latch_motion=True):
        """Cancel the active navigation goal and force the robot to stop.

        Cancelling the goal alone is not enough: the base latches Nav2's last
        commanded velocity, and Nav2's controller/recovery behaviors keep
        publishing /cmd_vel for a short window while the cancel propagates. So
        we also force_stop() to overwrite that with a sustained zero.
        """
        with self._motion_lock:
            if latch_motion:
                self._motion_stop_latched = True
            with self._nav_lock:
                handle = self._nav_goal_handle
                self._nav_request_id += 1
                if not preserve_waiting or self._queued_nav_goal is None:
                    self._queued_nav_goal = None
                    self._nav_state = "canceled"
                    self._nav_goal = None
                self._nav_goal_handle = None
                self._plan = None
            if handle is not None:
                handle.cancel_goal_async()
            self.force_stop()

    def force_stop(self, duration=2.0, rate_hz=20.0):
        """Halt the base robustly. Publishes a zero Twist repeatedly for a
        short window (in a background thread, so the HTTP request returns at
        once) to win the race against Nav2's trailing /cmd_vel and to overwrite
        the latched velocity on the controller. A subsequent non-zero drive()
        or force_stop() supersedes this one via the generation counter."""
        with self._motion_lock:
            self._cancel_localization_motion()
            with self._stop_lock:
                self._stop_gen += 1
                gen = self._stop_gen
            self._publish_velocity(0.0, 0.0, 0.0)
        threading.Thread(
            target=self._spam_stop, args=(gen, duration, rate_hz), daemon=True
        ).start()

    def _spam_stop(self, gen, duration, rate_hz):
        """Repeat zero commands without racing a new operator motion request."""
        period = 1.0 / rate_hz
        for _ in range(max(1, int(duration * rate_hz))):
            with self._motion_lock:
                with self._stop_lock:
                    if self._stop_gen != gen:
                        return
                self._publish_velocity(0.0, 0.0, 0.0)
            time.sleep(period)

    # ---- Front-end heartbeat / dead-man's switch ------------------------
    def heartbeat(self):
        """Record a liveness ping from the web UI (arms the dead-man's switch)."""
        with self._hb_lock:
            self._hb_last = time.monotonic()
            self._hb_enabled = True
            was_alive = self._hb_alive
            self._hb_alive = True
        if not was_alive:
            self.get_logger().info("web UI heartbeat connected")

    def _heartbeat_watchdog(self, poll_hz=10.0):
        """Stop the robot if the UI's heartbeat goes silent. Edge-triggered:
        fires the safety stop exactly once on the connected->lost transition so
        it does not fight a reconnecting client or a fresh command."""
        period = 1.0 / poll_hz
        while rclpy.ok():
            time.sleep(period)
            # Independent of ROS spin: a stalled motion timer must not leave
            # its last nonzero velocity latched while HTTP heartbeats continue.
            with self._motion_lock:
                motion = self._localization_motion
                if motion and motion.active and time.monotonic() - motion.last_tick > 0.25:
                    self._cancel_localization_motion("Stopped: motion timer stalled")
                    self._transient_motion_hold = True
                    self.force_stop()
            with self._hb_lock:
                if not self._hb_enabled:
                    continue
                silent = time.monotonic() - self._hb_last
                lost = self._hb_alive and silent > self._hb_timeout
                if lost:
                    self._hb_alive = False
            if lost:
                self.get_logger().warn(
                    "web UI heartbeat LOST (%.1fs silent) -> safety stop" % silent
                )
                try:
                    self.safety_stop()
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f"safety_stop failed: {e}")

    def safety_stop(self):
        """Dead-man's-switch stop: cancel any active nav goal and force the base
        to a sustained zero (defeats the latch). Same effect as the operator's
        emergency stop, but triggered automatically when the UI disconnects."""
        with self._motion_lock:
            with self._hb_lock:
                self._hb_alive = False  # A leave event revokes liveness immediately.
            recoverable = (self._transient_motion_hold or self._auto_localization_pending or
                           bool(self._localization_motion and self._localization_motion.active))
            self.cancel_nav(latch_motion=False)
            self._transient_motion_hold = recoverable

    def nav_status(self):
        with self._nav_lock:
            return {
                "state": self._nav_state,
                "goal": self._nav_goal,
                "path": self._plan,
            }

    def localization_status(self):
        """Return the current load/localize/update transition state."""
        with self._motion_lock:
            motion = self._localization_motion
            movement = {"active": bool(motion and motion.active),
                        "message": (motion.reason if motion else self._auto_localization_message),
                        "paused": self._auto_localization_cancelled,
                        "can_resume": self._auto_localization_cancelled}
            sensors = dict(self._sensor_health)
        with self._map_phase_lock:
            quality = dict(self._localization_quality or {})
            return {
                "phase": self._map_phase,
                "movement": movement,
                "sensors": sensors,
                "error": self._map_phase_error,
                "confident_samples": self._localization_samples,
                "required_samples": REQUIRED_CONFIDENT_SAMPLES,
                "search_radius_m": self.settings.snapshot()["localization_radius_m"],
                "search_scope": self._localization_search_scope,
                "last_map_age_s": (None if self._last_map_at is None else
                                   max(0.0, time.monotonic() - self._last_map_at)),
                "map_update_error": self._map_update_error,
                "mapping_diagnostics": self._mapping_diagnostics,
                "motion_stop_latched": self._motion_stop_latched,
                "navigation_ready": self._map_phase in ('localized', 'updating'),
                **quality,
            }

    def _set_localization_error(self, generation, message):
        """Keep a failed load read-only and expose the reason to the UI."""
        with self._map_phase_lock:
            if generation != self._localization_generation:
                return
            self._map_phase = "localization_error"
            self._map_phase_error = message
        self.get_logger().error(message)

    def _request_saved_localization(self, generation):
        """Make failures visible instead of silently losing a startup worker."""
        try:
            self._seed_saved_localization(generation)
        except Exception as exc:
            self._set_localization_error(generation, f'Unable to initialize localization: {exc}')

    def _seed_saved_localization(self, generation):
        """Wait for active AMCL, then seed the last pose saved for this map."""
        deadline = time.monotonic() + 30.0
        client = self._amcl_state_client
        while time.monotonic() < deadline:
            with self._map_phase_lock:
                if generation != self._localization_generation:
                    return
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            ready = threading.Event()
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda _: ready.set())
            if not ready.wait(2.0):
                future.cancel()
                continue
            try:
                active = future.result().current_state.id == 3
            except Exception:
                active = False
            if not active or not self._initialpose_pub.get_subscription_count():
                time.sleep(0.25)
                continue
            if not self._wait_for_amcl_map(generation):
                return
            saved = self.settings.snapshot(self._active_map_id)["last_pose"]
            if saved:
                saved = self._refine_saved_pose(saved)
            with self._motion_lock:
                with self._map_phase_lock:
                    if generation != self._localization_generation:
                        return
                    self._map_phase = "requesting_localization"
                if saved:
                    self.set_initial_pose(saved["x"], saved["y"], saved["yaw"], automatic=True)
            if not saved:
                self._request_global_localization(generation)
            return
        self._set_localization_error(generation, "Localization did not start. Reload the map to retry.")

    def _refine_saved_pose(self, saved):
        """Improve a remembered seed from current scans without declaring success."""
        with self._lock:
            grid = self._map
            scan = self._scans[-1] if self._scans else None
        now_ns = self.get_clock().now().nanoseconds
        if (grid is None or sensor_health(scan, now_ns, self.localization_config.scan_max_age_s)['state'] != 'ready'):
            return saved
        try:
            laser = self._tf_buffer.lookup_transform(
                self.localization_config.base_frame, scan.header.frame_id, rclpy.time.Time()).transform
            result = refine_seed(scan, grid, (saved['x'], saved['y'], saved['yaw']),
                                 (laser.translation.x, laser.translation.y, self._yaw(laser.rotation)),
                                 self.settings.snapshot()['localization_radius_m'])
            if result is not None:
                x, y, yaw = result['pose']
                self.get_logger().info(
                    f"Local pose search: scan agreement {result['original_match']:.0%} -> {result['scan_match']:.0%}")
                return {**saved, 'x': x, 'y': y, 'yaw': yaw}
        except Exception as exc:
            self.get_logger().warning(f"Using remembered pose without scan refinement: {exc}")
        return saved

    def _request_global_localization(self, generation):
        """Search the saved map automatically when no trusted prior is available."""
        if not self._wait_for_amcl_map(generation):
            return
        client = self._global_localization_client
        if not client.wait_for_service(timeout_sec=5.0):
            self._set_localization_error(generation, "Automatic map search service is unavailable")
            return
        with self._motion_lock:
            with self._map_phase_lock:
                if generation != self._localization_generation:
                    return
            future = client.call_async(EmptyService.Request())
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        if not ready.wait(5.0):
            future.cancel()
            self._set_localization_error(generation, "Automatic map search timed out")
            return
        try:
            future.result()
        except Exception as exc:
            self._set_localization_error(generation, f"Automatic map search failed: {exc}")
            return
        with self._motion_lock:
            with self._map_phase_lock:
                if generation != self._localization_generation:
                    return
                self._localization_search_scope = "global"
                self._pose_seed_ns = self.get_clock().now().nanoseconds
                self._last_amcl_stamp_ns = self._pose_seed_ns
                self._localization_started_at = time.monotonic()
                self._localization_samples = 0
                self._localization_quality = None
                self._map_phase = "localizing"
                self._global_evidence = GlobalLocalizationEvidence()
            if not self._auto_localization_cancelled:
                # One additional bounded sequence for a whole-map hypothesis.
                self._localization_motion = None
                self._auto_localization_pending = True

    def _update_stationary_localization(self):
        """Keep localization diagnostics fresh; count search time only with live sensors."""
        with self._map_phase_lock:
            phase = self._map_phase
        if phase not in ('localizing', 'localized'):
            return
        # Refresh diagnostics even while movements are paused or blocked.
        with self._motion_lock:
            self._motion_inputs()
        inputs_ready = all(self._sensor_health[name]['state'] == 'ready'
                           for name in ('odometry', 'lidar'))
        now = time.monotonic()
        if not inputs_ready:
            self._localization_fresh_since = None
            return
        if self._localization_fresh_since is None:
            self._localization_fresh_since = now
        if phase == 'localizing':
            self._recover_transient_motion()
            self._automatic_localization()
        with self._map_phase_lock:
            if self._map_phase not in ('localizing', 'localized'):
                return
            if (self._map_phase == 'localizing' and self._localization_search_scope == "remembered" and
                    now - self._localization_fresh_since > 35
                    and not (self._localization_motion and self._localization_motion.active)):
                self._map_phase = "requesting_localization"
                generation = self._localization_generation
                threading.Thread(target=self._request_global_localization,
                                 args=(generation,), daemon=True).start()
                return
        if self._nomotion_future is not None and not self._nomotion_future.done():
            if time.monotonic() - self._nomotion_requested_at < 3.0:
                return
            self._nomotion_future.cancel()
            self._nomotion_future = None
        if self._nomotion_client.service_is_ready():
            self._nomotion_requested_at = time.monotonic()
            self._nomotion_future = self._nomotion_client.call_async(EmptyService.Request())

    def _on_amcl_pose(self, msg):
        """Resume map updates after consecutive confident AMCL estimates."""
        stamp_ns = self._stamp_ns(msg.header)
        # Serialize the final decision with manual commands and the turn timer.
        with self._motion_lock:
            self._amcl_candidate = msg
            if self._map_phase == 'localized':
                self._record_runtime_observation(msg)
                return
            self._process_amcl_pose(msg, stamp_ns)

    def _record_runtime_observation(self, msg):
        """Continue checking localization after enabling navigation."""
        stamp = self._stamp_ns(msg.header)
        age = self.get_clock().now().nanoseconds - stamp
        if msg.header.frame_id != self.localization_config.map_frame or not 0 <= age <= 2_000_000_000:
            return
        if stamp <= self._last_runtime_checked_ns:
            return
        quality = localization_quality(msg.pose.covariance)
        quality.update(self._scan_quality(msg))
        quality['confident'] = quality['confident'] and quality['scan_agrees']
        if quality['confident']:
            p = msg.pose.pose
            odom_pose = self._odometry_at(msg.header.stamp)
            if odom_pose is None:
                self._localization_quality = {**quality, 'confident': False,
                                              'reason': 'Waiting for time-paired odometry'}
                return
            if not self._runtime_continuity.observe(
                    (p.position.x, p.position.y, self._yaw(p.orientation)), odom_pose):
                quality['confident'] = False
                quality['reason'] = 'Position estimate jumped relative to odometry'
                self._localization_quality = quality
                self._last_runtime_checked_ns = stamp
                self._fresh_mapping_good_samples = 0
                self._lose_localization(quality['reason'])
                return
        self._localization_quality = quality
        if quality.get('scan_match') is not None:
            self._last_runtime_checked_ns = stamp
        if quality['confident']:
            self._last_localization_observation_ns = stamp
            self._runtime_bad_samples = 0
            p = msg.pose.pose
            self._remember_pose((p.position.x, p.position.y, self._yaw(p.orientation)))
            if self._map_phase == 'mapping_paused':
                if self._stationary_observation(stamp) and self._navigation_inputs_ready():
                    self._fresh_mapping_good_samples += 1
                    if self._fresh_mapping_good_samples >= REQUIRED_CONFIDENT_SAMPLES:
                        self._map_phase = 'updating'
                        self._map_phase_error = None
                else:
                    self._fresh_mapping_good_samples = 0
        elif quality.get('scan_match') is not None:
            self._fresh_mapping_good_samples = 0
            self._runtime_bad_samples += 1
            if self._runtime_bad_samples >= 3:
                self._lose_localization('Live scans no longer confirm the robot position')

    def _navigation_inputs_ready(self):
        """Validate live sensors and the position provider, not just cached UI state."""
        _, fresh, _, connected, _ = self._motion_inputs()
        if not fresh or not connected:
            return False
        try:
            transform = self._tf_buffer.lookup_transform(
                self.localization_config.map_frame, self.localization_config.base_frame, rclpy.time.Time())
            age = self.get_clock().now().nanoseconds - self._stamp_ns(transform.header)
            if not -1_000_000_000 <= age <= 1_000_000_000:
                return False
        except Exception:
            return False
        age = self.get_clock().now().nanoseconds - self._last_localization_observation_ns
        return 0 <= age <= 3_000_000_000

    def _monitor_navigation_health(self):
        """Refresh diagnostics in every phase and fail closed on navigation health loss."""
        with self._motion_lock:
            self._motion_inputs()
            if self._amcl_candidate is not None and self._map_phase in ('localizing', 'localized'):
                self._on_amcl_pose(self._amcl_candidate)
            # Odometry TF can arrive after its laser scan. Revisit buffered
            # observations rather than requiring arrival ordering across topics.
            phases = ('starting_mapping', 'waiting_mapping', 'updating', 'mapping_paused')
            if self._slam_candidate is not None and self._map_phase in phases:
                with self._lock:
                    scans = list(self._scans)[-6:]
                now_ns = self.get_clock().now().nanoseconds
                for scan in reversed(scans):
                    if self._map_phase not in phases:
                        break
                    if 0 <= now_ns - self._stamp_ns(scan.header) <= 400_000_000:
                        self._corroborate_slam_pose(scan)
            if self._map_phase not in ('localized', 'updating'):
                return
            # A disconnected browser cancels the goal, not a still-valid position.
            with self._hb_lock:
                connected = self._hb_alive
            if connected and not self._navigation_inputs_ready():
                self._lose_localization('Waiting for fresh sensors, transform and position evidence')

    def _lose_localization(self, reason):
        """Cancel the goal and revalidate the existing AMCL hypothesis without moving blindly."""
        if self._map_phase == 'updating':
            self._recover_mapping(reason)
            return
        if self._map_phase != 'localized':
            return
        self.cancel_nav(latch_motion=False)
        with self._map_phase_lock:
            self._map_phase = 'localizing'
            self._localization_samples = 0
            self._pose_seed_ns = self.get_clock().now().nanoseconds
            self._localization_started_at = time.monotonic()
            self._localization_fresh_since = None
            self._localization_search_scope = 'remembered'
            self._pose_continuity = PoseContinuity()
            self._global_evidence = GlobalLocalizationEvidence()
        self._auto_localization_message = reason
        self._transient_motion_hold = True

    def _recover_transient_motion(self):
        """Retry temporary holds twice, after stable guards; never override explicit Stop."""
        if (not self._transient_motion_hold or self._motion_stop_latched
                or not self._allow_localization_motion or self._motion_retries >= 2):
            return
        _, fresh, clear, connected, stationary = self._motion_inputs()
        if not (fresh and clear and connected and stationary):
            self._motion_recovery_since = None
            return
        now = time.monotonic()
        if self._motion_recovery_since is None:
            self._motion_recovery_since = now
        if now - self._motion_recovery_since < 2:
            return
        self._motion_retries += 1
        self._transient_motion_hold = False
        self._localization_motion = None
        self._auto_localization_cancelled = False
        self._auto_localization_pending = True

    def _process_amcl_pose(self, msg, stamp_ns):
        """Evaluate one observation while holding the motion lock through handoff."""
        if not self._stationary_observation(stamp_ns):
            with self._map_phase_lock:
                self._localization_samples = 0
            return
        with self._map_phase_lock:
            if (self._map_phase != 'localizing' or msg.header.frame_id != self.localization_config.map_frame
                    or stamp_ns <= self._last_amcl_stamp_ns):
                return
        quality = localization_quality(msg.pose.covariance)
        quality.update(self._scan_quality(msg))
        quality["confident"] = quality["confident"] and quality["scan_agrees"]
        if quality.get('scan_match') is None:
            self._localization_quality = quality
            return  # Retry after the matching scan/TF arrives; do not consume its stamp.
        pose = msg.pose.pose
        yaw = self._yaw(pose.orientation)
        if not all(math.isfinite(value) for value in (pose.position.x, pose.position.y, yaw)):
            quality["confident"] = False
        if quality['confident']:
            odom_pose = self._odometry_at(msg.header.stamp)
            if odom_pose is None:
                quality.update(confident=False, reason='Waiting for time-paired odometry')
                self._localization_quality = quality
                return
            else:
                quality['pose_consistent'] = self._pose_continuity.observe(
                    (pose.position.x, pose.position.y, yaw), odom_pose)
                quality['confident'] = quality['pose_consistent']
        if quality['confident'] and self._localization_search_scope == 'global':
            quality['views_confirmed'] = self._global_evidence.observe(
                (pose.position.x, pose.position.y, yaw), odom_pose)
            quality['confident'] = quality['views_confirmed']

        with self._map_phase_lock:
            if self._map_phase != 'localizing' or stamp_ns <= self._last_amcl_stamp_ns:
                return
            self._last_amcl_stamp_ns = stamp_ns
            self._localization_quality = quality
            if quality["confident"]:
                self._localization_samples += 1
            else:
                self._localization_samples = 0
            if (self._localization_samples < REQUIRED_CONFIDENT_SAMPLES
                    or stamp_ns - self._pose_seed_ns < 3_000_000_000):
                return
            localized_pose = (pose.position.x, pose.position.y, yaw)
            generation = self._localization_generation
            map_base = self._localization_map_base
            self._last_localization_observation_ns = stamp_ns
            self._last_runtime_checked_ns = stamp_ns
            self._runtime_bad_samples = 0
            self._runtime_continuity = PoseContinuity(anchored=False)
            self._runtime_continuity.transform = self._pose_continuity.transform
            self._map_phase = 'starting_mapping' if self._update_loaded_map else 'localized'
        self.force_stop()
        self._remember_pose(localized_pose, force=True)
        if not self._update_loaded_map:
            self.get_logger().info(f'Position confirmed; navigation ready at {localized_pose}; map read-only')
            return
        self.get_logger().info(
            f'Position confirmed; starting mapping at {localized_pose}')
        threading.Thread(target=self._complete_map_localization,
                         args=(generation, map_base, localized_pose), daemon=True).start()

    def _stop_slam_backend(self):
        """Stop only the SLAM session launched by this bridge."""
        self._stop_backend(self._slam_backend, {'slam_toolbox'})

    def _stop_localization_backend(self):
        """Stop this bridge's AMCL launch and discard its pending service call."""
        self._stop_backend(self._localization_backend, {'amcl', 'map_server'})
        if self._nomotion_future is not None:
            self._nomotion_future.cancel()
            self._nomotion_future = None

    def _stop_backend(self, backend, names):
        """Wait for an owned provider to leave discovery before allowing another load."""
        if backend.process is None:
            return
        backend.stop()
        # Discovery can retain an abruptly stopped participant after its process
        # exits. Keep the map operation serialized through that drain period.
        deadline = time.monotonic() + 25.0
        while rclpy.ok() and not self._closing:
            if not names.intersection(self.get_node_names()):
                return
            if time.monotonic() >= deadline:
                raise ValueError('Previous map provider is still leaving ROS discovery; retry shortly')
            time.sleep(.1)

    def _require_managed_backends(self):
        """Refuse to replace externally managed map providers instead of killing them."""
        external = []
        names = set(self.get_node_names())
        if "slam_toolbox" in names and self._slam_backend.process is None:
            external.append("SLAM")
        if names.intersection(("amcl", "map_server")) and self._localization_backend.process is None:
            external.append("localization")
        if external:
            raise ValueError("Externally managed " + ", ".join(external) +
                             " is running. Restart the stack with the updated launcher before changing maps.")

    @serialized_map_operation
    def start_map_localization(self, map_base, allow_motion=True):
        """Load a static map and localize near its remembered pose before updates."""
        self._require_managed_backends()
        self.cancel_nav(latch_motion=False)
        with self._motion_lock:
            self._localization_motion = None
            self._motion_settled_after_ns = 0
            self._auto_localization_pending = allow_motion
            self._auto_localization_cancelled = not allow_motion
            self._allow_localization_motion = allow_motion
            self._motion_stop_latched = not allow_motion
            self._transient_motion_hold = False
            self._motion_recovery_since = None
            self._motion_retries = 0
            self._auto_localization_message = ("Waiting for localization and live sensors"
                                               if allow_motion else "Localization movements stopped")
            self._localization_search_scope = "remembered"
        with self._map_phase_lock:
            self._localization_generation += 1
            generation = self._localization_generation
            self._map_phase = "starting_localization"
            self._map_phase_error = None
            self._localization_samples = 0
            self._localization_quality = None
            self._localization_map_base = map_base
            self._amcl_candidate = None
            self._localization_fresh_since = None
            self._pose_continuity = PoseContinuity()
            self._global_evidence = GlobalLocalizationEvidence()
            self._last_localization_observation_ns = 0
            self._last_runtime_checked_ns = 0
            self._mapping_reference_grid = None
            self._pending_mapping_grid = None
            self._active_map_id = map_identity(map_base)
            self._mapping_started_at = None
            self._mapping_readiness = None
            self._mapping_process = None

        self._stop_slam_backend()
        self._stop_localization_backend()
        time.sleep(1.5)
        with self._lock:
            self._map = None
        with self._map_phase_lock:
            self._localization_backend_started_ns = self.get_clock().now().nanoseconds
            self._amcl_map_ready_ns = 0
        args = [
            "ros2", "launch", "kinisirobot", "navigation.launch.py",
            "slam:=false", "navigation:=false",
            f"map:={map_base}.yaml", "use_sim_time:=false",
        ]
        self._localization_backend.start(args, "~/map_localization.log")
        threading.Thread(
            target=self._request_saved_localization,
            args=(generation,),
            daemon=True,
        ).start()

    @serialized_map_operation
    def _complete_map_localization(self, generation, map_base, pose):
        """Replace read-only AMCL with update-map SLAM at a localized pose."""
        try:
            with self._map_phase_lock:
                if generation != self._localization_generation:
                    return
            self.cancel_nav(preserve_waiting=True, latch_motion=False)
            self._remember_pose(pose, force=True)
            with self._lock:
                self._mapping_reference_grid = self._map
                self._pending_mapping_grid = None
            self._stop_localization_backend()
            time.sleep(1.0)
            self._launch_slam(map_base, pose)
        except Exception as exc:  # noqa: BLE001
            self._recover_mapping(f"Unable to start map updates: {exc}")
            return
        self.get_logger().info("Waiting for fresh SLAM pose and map after localization")

    def _recover_mapping(self, reason):
        """One bounded rollback to read-only AMCL; never retry the failed SLAM handoff."""
        if self._localization_map_base is None:
            # New-map mode has no disk snapshot to roll back to. Keep its graph
            # and stop forwarding scans, rather than starting AMCL on None.yaml.
            with self._map_phase_lock:
                alive = self._mapping_process is not None and self._mapping_process.poll() is None
                self._map_phase = 'mapping_paused' if alive else 'mapping_error'
                self._map_phase_error = reason
                self._fresh_mapping_good_samples = 0
            self.cancel_nav(latch_motion=False)
            return
        with self._map_phase_lock:
            if self._map_phase == 'recovering_localization':
                return
            generation = self._localization_generation
            self._map_phase = 'recovering_localization'
            self._map_update_error = reason
            self._update_loaded_map = False
        self.cancel_nav(latch_motion=False)
        threading.Thread(target=self._rollback_to_localization, args=(generation,), daemon=True).start()

    @serialized_map_operation
    def _rollback_to_localization(self, generation):
        """Recover the same saved map while preserving operator Stop and map ownership."""
        if generation != self._localization_generation:
            return
        try:
            self.start_map_localization(self._localization_map_base, allow_motion=False)
        except Exception as exc:
            self._set_localization_error(self._localization_generation, f'Localization recovery failed: {exc}')

    @serialized_map_operation
    def start_map_updates(self):
        """Explicitly opt into extending a verified saved map, without accepting motion."""
        with self._motion_lock:
            if self._map_phase != 'localized' or not self._navigation_inputs_ready():
                raise ValueError('Wait for verified localization before starting map updates')
            _, _, stationary = self._odometry_state()
            if not stationary:
                raise ValueError('Stop the robot before starting map updates')
            pose = self._pose()
            if pose is None:
                raise ValueError('Waiting for a fresh robot pose')
            self._map_phase = 'starting_mapping'
            self._map_update_error = None
            generation = self._localization_generation
        threading.Thread(target=self._complete_map_localization,
                         args=(generation, self._localization_map_base,
                               (pose['x'], pose['y'], pose['yaw'])), daemon=True).start()

    @serialized_map_operation
    def _stop_rejected_mapping(self, generation):
        """Stop an unverified SLAM instance without touching a newer map session."""
        with self._map_phase_lock:
            if generation != self._localization_generation:
                return
        self._stop_slam_backend()

    @staticmethod
    def _yaw(q):
        """Return the planar yaw represented by a geometry quaternion."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _grid_to_dict(self, m, target_frame=None):
        """Serialize an OccupancyGrid, optionally transforming it into a frame."""
        info = m.info
        frame_id = m.header.frame_id
        x = info.origin.position.x
        y = info.origin.position.y
        yaw = self._yaw(info.origin.orientation)

        # The rolling local costmap is expressed in ``odom``, while the map and
        # browser canvas are expressed in ``map``. Apply map<-odom here so SLAM
        # pose corrections do not make the overlay appear to drift away from
        # the robot. The grid itself remains unchanged; its origin and yaw
        # describe how the browser should place it.
        if target_frame and frame_id and frame_id != target_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame, frame_id, rclpy.time.Time()
                ).transform
            except Exception:
                return None
            transform_yaw = self._yaw(transform.rotation)
            cos_yaw = math.cos(transform_yaw)
            sin_yaw = math.sin(transform_yaw)
            x, y = (
                transform.translation.x + cos_yaw * x - sin_yaw * y,
                transform.translation.y + sin_yaw * x + cos_yaw * y,
            )
            yaw += transform_yaw
            frame_id = target_frame

        return {
            "frame_id": frame_id,
            "width": info.width,
            "height": info.height,
            "resolution": info.resolution,
            "origin": {
                "x": x,
                "y": y,
            },
            "yaw": yaw,
            "data": list(m.data),
        }

    def _costmaps_payload(self, which):
        """Return {name: grid} for the requested costmaps that are available."""
        out = {}
        if not which:
            return out
        with self._lock:
            g = self._global_costmap
            l = self._local_costmap
        if "global" in which and g is not None:
            grid = self._grid_to_dict(g, self.localization_config.map_frame)
            if grid is not None:
                out["global"] = grid
        if "local" in which and l is not None:
            grid = self._grid_to_dict(l, self.localization_config.map_frame)
            if grid is not None:
                out["local"] = grid
        return out

    def map_payload(self, costmaps=()):
        with self._lock:
            m = self._map
        mode = self.map_mode()
        if m is None:
            payload = {"available": False, "pose": self._pose(),
                       "nav": self.nav_status(), "map_mode": mode}
        else:
            info = m.info
            payload = {
                "available": True,
                "width": info.width,
                "height": info.height,
                "resolution": info.resolution,
                "origin": {
                    "x": info.origin.position.x,
                    "y": info.origin.position.y,
                },
                "data": list(m.data),
                "pose": self._pose(),
                "nav": self.nav_status(),
                "map_mode": mode,
            }
        payload["localization"] = self.localization_status()
        # Costmaps are only serialized when the UI asks (overlay toggled on),
        # so the poll stays cheap when the overlay is hidden.
        cm = self._costmaps_payload(costmaps)
        if cm:
            payload["costmaps"] = cm
        return payload

    def map_mode(self):
        """Report which map backend is live: updating (slam) or static (amcl)."""
        with self._map_phase_lock:
            phase = self._map_phase
        if phase != "normal":
            return phase
        try:
            names = self.get_node_names()
        except Exception:
            return "unknown"
        if "slam_toolbox" in names:
            return "updating"
        if "amcl" in names:
            return "static"
        return "unknown"

    def _pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.localization_config.map_frame, self.localization_config.base_frame, rclpy.time.Time()
            )
            age = self.get_clock().now().nanoseconds - self._stamp_ns(t.header)
            if not -1_000_000_000 <= age <= 1_000_000_000:
                return None
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return {
            "x": t.transform.translation.x,
            "y": t.transform.translation.y,
            "yaw": yaw,
        }

    def drive(self, lx, ly, az):
        """Let manual commands take over and cancel localization movements."""
        if not all(math.isfinite(value) for value in (lx, ly, az)):
            raise ValueError('Velocity must be finite')
        with self._motion_lock:
            # The server, not a separate browser request, owns arbitration.
            with self._nav_lock:
                navigating = self._nav_state in ('pending', 'active', 'waiting_localization')
            if navigating:
                self.cancel_nav()
            self._motion_stop_latched = True
            self._cancel_localization_motion("Stopped: manual control")
            with self._map_phase_lock:
                if self._closing or self._map_phase in ("starting_localization", "starting_mapping", "waiting_mapping", 'mapping_paused', 'mapping_error'):
                    lx = ly = az = 0.0
            with self._stop_lock:
                self._stop_gen += 1  # This explicit manual command supersedes stop repetition.
            if lx or ly or az:
                self._motion_settled_after_ns = self.get_clock().now().nanoseconds + int(self.localization_config.settle_s * 1e9)
            self._publish_velocity(lx, ly, az)

    def _publish_velocity(self, lx, ly, az):
        """Publish a velocity after the caller has checked its motion guards."""
        tw = Twist()
        tw.linear.x = float(lx)
        tw.linear.y = float(ly)
        tw.angular.z = float(az)
        self._cmd_pub.publish(tw)

    def _on_navigation_velocity(self, msg):
        """Arbitrate Nav2 output so stop/health loss cannot be overridden by late commands."""
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            return
        with self._motion_lock:
            with self._nav_lock:
                active = self._nav_state == 'active'
            if not active:
                return  # Do not interfere with manual or bounded localization movement.
            if self._closing or self._map_phase not in ('localized', 'updating') or not self._navigation_inputs_ready():
                self.cancel_nav(latch_motion=False)
                return
            self._publish_velocity(*values)

    def reset_odometry(self):
        """Ask the controller node to zero the platform odometry."""
        self._cancel_localization_motion("Stopped: odometry reset")
        self._reset_odom_pub.publish(Empty())

    def _launch_slam(self, map_base=None, start_pose=None):
        """Launch update-map SLAM, optionally at a localized map pose."""
        with self._map_phase_lock:
            self._mapping_readiness = MappingReadiness(self.get_clock().now().nanoseconds)
            self._mapping_started_at = time.monotonic()
            self._mapping_process = None
            self._mapping_good_samples = self._mapping_bad_samples = 0
            self._last_mapping_stamp_ns = 0
            self._mapping_diagnostics = None
            self._slam_candidate = None
        args = [
            "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
            "--ros-args", "--params-file", self.slam_params,
            "-p", "use_sim_time:=false",
            "-p", "mode:=mapping",
            "-p", "scan_topic:=/scan_mapping",
        ]
        if map_base:
            args.extend(["-p", f"map_file_name:={map_base}"])
            if start_pose is None:
                args.extend(["-p", "map_start_at_dock:=true"])
            else:
                pose_value = "[%.8f,%.8f,%.8f]" % start_pose
                args.extend(["-p", f"map_start_pose:={pose_value}"])
        self._mapping_process = self._slam_backend.start(args, "~/slam.log")

    @serialized_map_operation
    def restart_slam(self, map_base=None):
        """Reset odometry and restart update-map SLAM."""
        self._require_managed_backends()
        self.cancel_nav()
        with self._map_phase_lock:
            self._localization_generation += 1
            self._map_phase = "starting_mapping"
            self._map_phase_error = None
            self._active_map_id = None
            self._localization_map_base = None
            self._last_localization_observation_ns = self._last_runtime_checked_ns = 0
            self._runtime_continuity = PoseContinuity(anchored=False)
            self._runtime_bad_samples = self._fresh_mapping_good_samples = 0
            self._mapping_reference_grid = None
            self._pending_mapping_grid = None
            self._mapping_started_at = None
            self._mapping_readiness = None
            self._mapping_process = None
        self.reset_odometry()
        self._stop_slam_backend()
        self._stop_localization_backend()
        time.sleep(2.0)
        with self._lock:
            self._map = None
        self._launch_slam(map_base)
        mode = f"saved map '{map_base}'" if map_base else "fresh map"
        self.get_logger().info(f"Restarted slam_toolbox ({mode}).")

    @serialized_map_operation
    def save_map(self, name):
        """Persist the current map to MAPS_DIR/<name>.

        Writes the occupancy grid (.pgm + .yaml, usable for localization) and,
        when slam_toolbox is running (update-map mode), the serialized
        pose-graph (.posegraph + .data) so it can be continued/extended later.
        """
        mode = self.map_mode()
        if mode not in ('localized', 'updating', 'mapping_paused'):
            return {
                "ok": False,
                "error": "Wait for map localization to finish before saving",
            }

        name = re.sub(r"[^A-Za-z0-9_-]", "", (name or "").strip()) or "kinisi_map"
        name = safe_map_name(name)
        os.makedirs(MAPS_DIR, exist_ok=True)
        if mode == 'localized':
            base = self._localization_map_base
            name = os.path.basename(base)
            archive = base + '.kinisi-map.zip'
            create_map_bundle(base, name, archive)
            return {'ok': True, 'name': name, 'occupancy': True, 'posegraph': True,
                    'bundle': True, 'download_url': '/maps/download?name=' + quote(name)}
        # Only fresh files from this attempt can become the next saved snapshot.
        self.cancel_nav()
        with tempfile.TemporaryDirectory(dir=MAPS_DIR) as staging:
            return self._save_map_snapshot(name, staging)

    def _save_map_snapshot(self, name, staging):
        """Create a complete map set before replacing any previous saved map."""
        base = os.path.join(staging, name)
        result = {
            "name": name,
            "occupancy": False,
            "posegraph": False,
            "bundle": False,
        }

        # 1) Occupancy grid via map_saver_cli (subscribes to /map).
        try:
            r = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", base,
                 "--ros-args", "-p", "save_map_timeout:=10.0"],
                capture_output=True, text=True, timeout=40,
            )
            result["occupancy"] = (r.returncode == 0) and os.path.exists(base + ".yaml")
            if not result["occupancy"]:
                result["error"] = (r.stderr or r.stdout or "map_saver failed").strip()[-300:]
        except Exception as e:  # noqa: BLE001
            result["error"] = str(e)

        # 2) Serialized pose-graph (only meaningful while slam_toolbox runs).
        try:
            serialized = subprocess.run(
                ["ros2", "service", "call", "/slam_toolbox/serialize_map",
                 "slam_toolbox/srv/SerializePoseGraph",
                 "{filename: '%s'}" % base],
                capture_output=True, text=True, timeout=40,
            )
            result["posegraph"] = (serialized.returncode == 0
                                   and all(os.path.isfile(base + ext) for ext in ('.posegraph', '.data')))
        except Exception as e:  # noqa: BLE001
            result.setdefault("error", str(e))

        if result["occupancy"] and result["posegraph"]:
            try:
                archive = base + ".kinisi-map.zip"
                create_map_bundle(base, name, archive)
                with open(archive, 'rb') as source:
                    installed = install_map_bundle(source.read(), MAPS_DIR, require_posegraph=True)
                base = installed['base']
                os.replace(archive, base + '.kinisi-map.zip')
                result["bundle"] = True
                result["download_url"] = "/maps/download?name=" + quote(name)
                if result["posegraph"]:
                    self._active_map_id = map_identity(base)
                    self._localization_map_base = base
                    pose = self._pose()
                    if pose:
                        self._remember_pose((pose["x"], pose["y"], pose["yaw"]), force=True)
            except (MapBundleError, ValueError, OSError) as exc:
                result["error"] = str(exc)

        result["ok"] = bool(result["occupancy"] and result["bundle"])
        if not result['ok']:
            result.setdefault('error', 'Map snapshot incomplete; previous saved map was not replaced')
        self.get_logger().info(
            f"save_map '{name}': occupancy={result['occupancy']} "
            f"posegraph={result['posegraph']} bundle={result['bundle']}"
        )
        return result

    @staticmethod
    def map_bundle_path(name):
        """Return the path of a named downloadable map bundle."""
        return os.path.join(MAPS_DIR, safe_map_name(name) + ".kinisi-map.zip")

    @serialized_map_operation
    def load_map_bundle(self, payload):
        """Install a map and verify the remembered pose before allowing updates."""
        self._require_managed_backends()
        self._map_update_error = None
        result = install_map_bundle(payload, MAPS_DIR, require_posegraph=True)
        self.start_map_localization(result["base"])
        result["ok"] = True
        result["phase"] = "localizing"
        result.pop("base", None)
        return result


bridge = None  # set in main()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):  # silence per-request logging
        pass

    def _send(self, code, body=b"", ctype="text/plain", gzipped=False):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        if gzipped:
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            try:
                with open(INDEX_HTML, "rb") as f:
                    html = f.read()
            except OSError:
                self._send(500, b"kinisirobot web/index.html is not installed")
                return
            self._send(200, html, "text/html; charset=utf-8")
        elif path == "/map":
            # Optional costmap overlay: /map?costmap=global|local|both
            qs = parse_qs(urlparse(self.path).query)
            cmparam = (qs.get("costmap", [""])[0] or "").lower()
            which = set()
            if cmparam in ("global", "both", "all"):
                which.add("global")
            if cmparam in ("local", "both", "all"):
                which.add("local")
            payload = json.dumps(bridge.map_payload(which)).encode("utf-8")
            if "gzip" in self.headers.get("Accept-Encoding", ""):
                payload = gzip.compress(payload)
                self._send(200, payload, "application/json", gzipped=True)
            else:
                self._send(200, payload, "application/json")
        elif path == "/camera/status":
            body = json.dumps(bridge.camera.status()).encode("utf-8")
            self._send(200, body, "application/json")
        elif path == "/robot/settings":
            body = json.dumps(bridge.settings.snapshot(bridge._active_map_id)).encode("utf-8")
            self._send(200, body, "application/json")
        elif path == "/maps/download":
            query = parse_qs(urlparse(self.path).query)
            try:
                name = query.get("name", [""])[0]
                bundle_path = bridge.map_bundle_path(name)
                with open(bundle_path, "rb") as bundle_file:
                    body = bundle_file.read()
            except (MapBundleError, ValueError) as exc:
                self._send(400, str(exc).encode("utf-8"))
                return
            except OSError:
                self._send(404, b"map bundle not found")
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/zip")
            self.send_header(
                "Content-Disposition",
                f'attachment; filename="{name}.kinisi-map.zip"',
            )
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self._send(404, b"not found")

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/robot/settings":
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 1024:
                    raise ValueError("Invalid settings request size")
                data = json.loads(self.rfile.read(length))
                bridge.settings.set_radius(data["localization_radius_m"])
            except (ValueError, KeyError, TypeError) as exc:
                self._send(400, json.dumps({"ok": False, "error": str(exc)}).encode(), "application/json")
                return
            except OSError:
                self._send(500, b'{"ok":false,"error":"Could not save robot settings"}', "application/json")
                return
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/webrtc/offer":
            length = int(self.headers.get("Content-Length", 0) or 0)
            try:
                offer = json.loads(self.rfile.read(length) if length else b"{}")
                answer = bridge.camera.create_answer(offer)
            except (ValueError, TypeError) as exc:
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
                self._send(400, body, "application/json")
                return
            except CameraUnavailable as exc:
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
                self._send(503, body, "application/json")
                return
            except Exception as exc:  # noqa: BLE001
                bridge.get_logger().error(f"WebRTC negotiation failed: {exc}")
                body = json.dumps(
                    {"ok": False, "error": "WebRTC negotiation failed"}
                ).encode("utf-8")
                self._send(500, body, "application/json")
                return
            self._send(
                200, json.dumps(answer).encode("utf-8"), "application/json"
            )
            return
        if path == "/webrtc/close":
            length = int(self.headers.get("Content-Length", 0) or 0)
            try:
                data = json.loads(self.rfile.read(length) if length else b"{}")
            except (ValueError, TypeError):
                data = {}
            closed = bridge.camera.close_peer(data.get("peer_id"))
            body = json.dumps({"ok": True, "closed": closed}).encode("utf-8")
            self._send(200, body, "application/json")
            return
        if path == "/heartbeat":
            # Dead-man's switch ping from the UI. If these stop arriving the
            # watchdog issues a safety stop (the base latches its last command).
            bridge.heartbeat()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/reset_odom":
            bridge.cancel_nav()
            bridge.reset_odometry()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/estop":
            # Operator emergency stop: cancel any nav goal and force the base to
            # halt (defeats the latch + Nav2's trailing /cmd_vel).
            bridge.cancel_nav()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/session/leave":
            bridge.safety_stop()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/mapping/start":
            try:
                bridge.start_map_updates()
            except (ValueError, RuntimeError) as exc:
                self._send(409, json.dumps({'ok': False, 'error': str(exc)}).encode(), 'application/json')
                return
            self._send(200, b'{"ok":true}', 'application/json')
            return
        if path == "/localization/resume":
            try:
                bridge.resume_localization()
            except ValueError as exc:
                self._send(409, json.dumps({"ok": False, "error": str(exc)}).encode(), "application/json")
                return
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/reset_map":
            try:
                bridge.restart_slam()
            except ValueError as exc:
                self._send(409, json.dumps({"ok": False, "error": str(exc)}).encode(), "application/json")
                return
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/save_map":
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(raw or b"{}")
            except (ValueError, TypeError):
                data = {}
            result = bridge.save_map(data.get("name", "kinisi_map"))
            code = 200 if result.get("ok") else 500
            self._send(code, json.dumps(result).encode("utf-8"), "application/json")
            return
        if path == "/maps/load":
            try:
                length = int(self.headers.get("Content-Length", 0) or 0)
            except ValueError:
                length = 0
            if length <= 0 or length > MAX_BUNDLE_BYTES:
                self._send(
                    413,
                    b'{"ok":false,"error":"map bundle is empty or too large"}',
                    "application/json",
                )
                return
            try:
                result = bridge.load_map_bundle(self.rfile.read(length))
            except (MapBundleError, ValueError) as exc:
                body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
                self._send(400, body, "application/json")
                return
            except Exception as exc:  # noqa: BLE001
                bridge.get_logger().error(f"Map load failed: {exc}")
                body = json.dumps(
                    {"ok": False, "error": "Unable to load map"}
                ).encode("utf-8")
                self._send(500, body, "application/json")
                return
            self._send(
                200, json.dumps(result).encode("utf-8"), "application/json"
            )
            return
        if path in ("/nav_goal", "/set_initial_pose", "/cancel_nav"):
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(raw or b"{}")
            except (ValueError, TypeError):
                self._send(400, b"bad json")
                return
            if path == "/cancel_nav":
                bridge.cancel_nav()
                self._send(200, b'{"ok":true}', "application/json")
                return
            try:
                x = float(data["x"])
                y = float(data["y"])
                yaw = float(data.get("yaw", 0.0))
            except (KeyError, ValueError, TypeError):
                self._send(400, b'{"ok":false,"error":"need x,y[,yaw]"}',
                           "application/json")
                return
            if path == "/set_initial_pose":
                try:
                    bridge.set_initial_pose(x, y, yaw)
                except ValueError as exc:
                    self._send(409, json.dumps({"ok": False, "error": str(exc)}).encode(), "application/json")
                    return
                self._send(200, b'{"ok":true}', "application/json")
            else:  # /nav_goal
                ok = bridge.request_nav_goal(x, y, yaw)
                body = json.dumps({"ok": ok, "state": bridge.nav_status()["state"],
                                   **({} if ok else {"error": "Navigation unavailable; check map status and Nav2"})}).encode()
                self._send(200 if ok else 503, body, "application/json")
            return
        if path != "/cmd_vel":
            self._send(404, b"not found")
            return
        length = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(raw or b"{}")
            lx = float(data.get("lx", 0.0))
            ly = float(data.get("ly", 0.0))
            az = float(data.get("az", 0.0))
        except (ValueError, TypeError):
            self._send(400, b"bad json")
            return
        try:
            bridge.drive(lx, ly, az)
        except ValueError as exc:
            self._send(400, json.dumps({'ok': False, 'error': str(exc)}).encode(), 'application/json')
            return
        self._send(200, b'{"ok":true}', "application/json")


def main():
    global bridge
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--slam-params", default=DEFAULT_SLAM_PARAMS)
    ap.add_argument("--start-mapping", action="store_true", help="Own a fresh SLAM backend")
    ap.add_argument("--localize-map", help="Resume read-only localization of a saved map basename")
    ap.add_argument("--no-localization-motion", action="store_true",
                    help="Preserve stopped motion when restarting saved-map localization")
    ap.add_argument('--update-loaded-map', action='store_true',
                    help='Opt into SLAM map extension after localization; default keeps AMCL read-only')
    ap.add_argument("--camera-device", default="/dev/video0")
    ap.add_argument("--camera-width", type=int, default=640)
    ap.add_argument("--camera-height", type=int, default=480)
    ap.add_argument("--camera-fps", type=int, default=20)
    args = ap.parse_args()

    rclpy.init()
    camera = WebRTCCamera(
        device=args.camera_device,
        width=args.camera_width,
        height=args.camera_height,
        fps=args.camera_fps,
    )
    bridge = WebBridge(slam_params=args.slam_params, camera=camera,
                       update_loaded_map=args.update_loaded_map)

    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    bridge.get_logger().info(f"HTTP server on http://{args.host}:{args.port}/")
    def start_backend():
        """Report failed startup in the UI instead of losing an exception in a thread."""
        try:
            if args.localize_map:
                bridge.start_map_localization(args.localize_map, allow_motion=not args.no_localization_motion)
            elif args.start_mapping:
                bridge.restart_slam()
        except Exception as exc:
            bridge._set_localization_error(bridge._localization_generation, str(exc))

    if args.localize_map or args.start_mapping:
        threading.Thread(target=start_backend, daemon=True).start()

    def request_shutdown(signum, frame):
        """Let SIGTERM stop HTTP so the finally block also releases owned backends."""
        bridge._closing = True
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge._closing = True
        bridge.drive(0.0, 0.0, 0.0)  # safety stop
        with bridge._map_phase_lock:
            bridge._localization_generation += 1
        with bridge._map_operation_lock:
            bridge._stop_slam_backend()
            bridge._stop_localization_backend()
        bridge.camera.shutdown()
        httpd.server_close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
