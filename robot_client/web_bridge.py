#!/usr/bin/env python3
"""Kinisi robot web bridge.

A single-file ROS 2 node + HTTP server that lets a browser:
  * see the live SLAM map (/map) with the robot's pose, and
  * drive the robot with WASD/QE (publishes geometry_msgs/Twist on /cmd_vel).

Run it ON THE ROBOT PI (needs access to the ROS graph), with the ROS env sourced:

    source /opt/ros/humble/setup.bash
    source ~/development/kinisiros/install/setup.bash
    python3 web_bridge.py --port 8080

Then open http://<pi-host>:8080/ in a browser on the same network.

Only depends on rclpy + the Python stdlib (no aiohttp/websockets needed).
"""

import argparse
import gzip
import json
import math
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
import tf2_ros

HERE = os.path.dirname(os.path.abspath(__file__))
INDEX_HTML = os.path.join(HERE, "index.html")
DEFAULT_SLAM_PARAMS = os.path.expanduser(
    "~/development/kinisiros/src/kinisirobot/config/mapper_params_online_async.yaml"
)
MAPS_DIR = os.path.expanduser("~/development/kinisiros/maps")


class WebBridge(Node):
    def __init__(self, slam_params=DEFAULT_SLAM_PARAMS):
        super().__init__("web_bridge")
        self._lock = threading.Lock()
        self._map = None
        self.slam_params = slam_params

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

        # --- Camera snapshot (Raspberry Pi Camera Module 3 / IMX708) ------
        # This Pi has no libcamera/rpicam/ffmpeg, so a live stream isn't
        # available; instead cam_grab.py grabs a raw unicam frame and software-
        # debayers it (~5 s/frame). Captures run ONLY while the UI camera panel
        # is open (each /camera.jpg request kicks a refresh if one isn't already
        # running), so there is zero camera use / CPU contention with Nav2 when
        # the panel is closed.
        self._cam_lock = threading.Lock()
        self._cam_jpeg = None        # bytes of the most recent frame
        self._cam_ts = 0.0           # time.time() the frame was captured
        self._cam_capturing = False
        self._cam_script = os.path.join(HERE, "cam_grab.py")
        self._cam_out = os.path.expanduser("~/cam.jpg")
        # Seed with the last saved frame (if any) so the panel shows something
        # immediately while the first fresh capture runs.
        try:
            with open(self._cam_out, "rb") as f:
                self._cam_jpeg = f.read()
        except OSError:
            pass

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            "web_bridge node ready (subscribed /map, publishing /cmd_vel)"
        )

    def _on_map(self, msg):
        with self._lock:
            self._map = msg

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

    def set_initial_pose(self, x, y, yaw):
        """Seed AMCL with the robot's current pose (map frame)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qz, qw = self._yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Moderate covariance (x, y, yaw) so AMCL converges from the seed.
        cov = [0.0] * 36
        cov[0] = 0.25       # x
        cov[7] = 0.25       # y
        cov[35] = 0.0685    # yaw (~15 deg)
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        self.get_logger().info(f"Published /initialpose x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def send_nav_goal(self, x, y, yaw):
        """Send a NavigateToPose goal (map frame)."""
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            with self._nav_lock:
                self._nav_state = "no_server"
            self.get_logger().warn("navigate_to_pose action server unavailable")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = self._yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        with self._nav_lock:
            self._nav_state = "pending"
            self._nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().info(f"Sent nav goal x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        return True

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            with self._nav_lock:
                self._nav_state = "failed"
            return
        with self._nav_lock:
            self._nav_goal_handle = handle
            self._nav_state = "active"
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        # action_msgs/GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        mapping = {4: "succeeded", 5: "canceled", 6: "failed"}
        with self._nav_lock:
            self._nav_state = mapping.get(status, "failed")
            self._nav_goal_handle = None

    def cancel_nav(self):
        """Cancel the active navigation goal and force the robot to stop.

        Cancelling the goal alone is not enough: the base latches Nav2's last
        commanded velocity, and Nav2's controller/recovery behaviors keep
        publishing /cmd_vel for a short window while the cancel propagates. So
        we also force_stop() to overwrite that with a sustained zero.
        """
        with self._nav_lock:
            handle = self._nav_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        with self._nav_lock:
            self._nav_state = "canceled"
            self._nav_goal_handle = None
        self.force_stop()

    def force_stop(self, duration=2.0, rate_hz=20.0):
        """Halt the base robustly. Publishes a zero Twist repeatedly for a
        short window (in a background thread, so the HTTP request returns at
        once) to win the race against Nav2's trailing /cmd_vel and to overwrite
        the latched velocity on the controller. A subsequent non-zero drive()
        or force_stop() supersedes this one via the generation counter."""
        with self._stop_lock:
            self._stop_gen += 1
            gen = self._stop_gen
        threading.Thread(
            target=self._spam_stop, args=(gen, duration, rate_hz), daemon=True
        ).start()

    def _spam_stop(self, gen, duration, rate_hz):
        period = 1.0 / rate_hz
        for _ in range(max(1, int(duration * rate_hz))):
            with self._stop_lock:
                if self._stop_gen != gen:
                    return  # superseded by a newer stop or a movement command
            self.drive(0.0, 0.0, 0.0)
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
        with self._nav_lock:
            handle = self._nav_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
            with self._nav_lock:
                self._nav_state = "canceled"
                self._nav_goal_handle = None
        self.force_stop()

    # --- Camera snapshot ---------------------------------------------------
    def camera_snapshot(self):
        """Return (jpeg_bytes_or_None, age_seconds_or_None) for the latest frame
        and kick a background refresh (deduplicated while one is in flight)."""
        self._camera_kick()
        with self._cam_lock:
            jpeg = self._cam_jpeg
            ts = self._cam_ts
        age = (time.time() - ts) if ts else None
        return jpeg, age

    def _camera_kick(self):
        """Start a capture in the background unless one is already running."""
        with self._cam_lock:
            if self._cam_capturing:
                return
            self._cam_capturing = True
        threading.Thread(target=self._camera_capture_worker, daemon=True).start()

    def _camera_capture_worker(self):
        tmp = self._cam_out + ".part.jpg"
        try:
            r = subprocess.run(
                ["python3", self._cam_script, tmp],
                cwd=HERE, capture_output=True, text=True, timeout=30,
            )
            if r.returncode == 0 and os.path.exists(tmp):
                with open(tmp, "rb") as f:
                    data = f.read()
                os.replace(tmp, self._cam_out)   # keep for next-startup seed
                with self._cam_lock:
                    self._cam_jpeg = data
                    self._cam_ts = time.time()
            else:
                msg = (r.stderr or r.stdout or "")[-200:]
                self.get_logger().warn("camera capture failed: %s" % msg)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("camera capture timed out")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn("camera capture error: %s" % e)
        finally:
            try:
                if os.path.exists(tmp):
                    os.remove(tmp)
            except OSError:
                pass
            with self._cam_lock:
                self._cam_capturing = False

    def nav_status(self):
        with self._nav_lock:
            return {
                "state": self._nav_state,
                "goal": self._nav_goal,
                "path": self._plan,
            }

    @staticmethod
    def _grid_to_dict(m):
        """Serialize a nav_msgs/OccupancyGrid to the compact dict the UI draws."""
        info = m.info
        return {
            "width": info.width,
            "height": info.height,
            "resolution": info.resolution,
            "origin": {
                "x": info.origin.position.x,
                "y": info.origin.position.y,
            },
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
            out["global"] = self._grid_to_dict(g)
        if "local" in which and l is not None:
            out["local"] = self._grid_to_dict(l)
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
        # Costmaps are only serialized when the UI asks (overlay toggled on),
        # so the poll stays cheap when the overlay is hidden.
        cm = self._costmaps_payload(costmaps)
        if cm:
            payload["costmaps"] = cm
        return payload

    def map_mode(self):
        """Report which map backend is live: updating (slam) or static (amcl)."""
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
                "map", "base_link", rclpy.time.Time()
            )
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
        if lx or ly or az:
            # A real movement command supersedes any in-flight stop-spam so the
            # operator regains control immediately after an emergency stop.
            with self._stop_lock:
                self._stop_gen += 1
        tw = Twist()
        tw.linear.x = float(lx)
        tw.linear.y = float(ly)
        tw.angular.z = float(az)
        self._cmd_pub.publish(tw)

    def reset_odometry(self):
        """Ask the controller node to zero the platform odometry."""
        self._reset_odom_pub.publish(Empty())

    def restart_slam(self):
        """Reset odometry and restart slam_toolbox for a fresh, empty map."""
        self.reset_odometry()
        subprocess.run(["pkill", "-f", "async_slam_toolbox_node"], check=False)
        subprocess.run(["pkill", "-f", "online_async_launch"], check=False)
        time.sleep(2.0)
        logf = open(os.path.expanduser("~/slam.log"), "ab")
        subprocess.Popen(
            [
                "ros2", "launch", "slam_toolbox", "online_async_launch.py",
                f"slam_params_file:={self.slam_params}",
                # Real robot has no /clock; slam_toolbox's launch defaults
                # use_sim_time to true, which freezes its timers/TF handling and
                # the map stops updating. Force wall-clock time.
                "use_sim_time:=false",
            ],
            stdout=logf,
            stderr=logf,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.get_logger().info("Restarted slam_toolbox (fresh map).")

    def save_map(self, name):
        """Persist the current map to MAPS_DIR/<name>.

        Writes the occupancy grid (.pgm + .yaml, usable for localization) and,
        when slam_toolbox is running (update-map mode), the serialized
        pose-graph (.posegraph + .data) so it can be continued/extended later.
        """
        name = re.sub(r"[^A-Za-z0-9_-]", "", (name or "").strip()) or "kinisi_map"
        os.makedirs(MAPS_DIR, exist_ok=True)
        base = os.path.join(MAPS_DIR, name)
        result = {"name": name, "path": base, "occupancy": False, "posegraph": False}

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
            subprocess.run(
                ["ros2", "service", "call", "/slam_toolbox/serialize_map",
                 "slam_toolbox/srv/SerializePoseGraph",
                 "{filename: '%s'}" % base],
                capture_output=True, text=True, timeout=40,
            )
            result["posegraph"] = os.path.exists(base + ".posegraph")
        except Exception as e:  # noqa: BLE001
            result.setdefault("error", str(e))

        result["ok"] = bool(result["occupancy"])
        self.get_logger().info(
            f"save_map '{name}': occupancy={result['occupancy']} "
            f"posegraph={result['posegraph']}"
        )
        return result


bridge = None  # set in main()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):  # silence per-request logging
        pass

    def _send(self, code, body=b"", ctype="text/plain", gzipped=False):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
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
                self._send(500, b"index.html not found next to web_bridge.py")
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
        elif path == "/camera.jpg":
            # Latest camera still (kicks a background refresh). 503 until the
            # very first frame exists so the UI can show a "capturing…" state.
            jpeg, age = bridge.camera_snapshot()
            if jpeg is None:
                self._send(503, b"capturing", "text/plain")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            if age is not None:
                self.send_header("X-Frame-Age", "%.1f" % age)
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
        else:
            self._send(404, b"not found")

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/heartbeat":
            # Dead-man's switch ping from the UI. If these stop arriving the
            # watchdog issues a safety stop (the base latches its last command).
            bridge.heartbeat()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/reset_odom":
            bridge.reset_odometry()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/estop":
            # Operator emergency stop: cancel any nav goal and force the base to
            # halt (defeats the latch + Nav2's trailing /cmd_vel).
            bridge.cancel_nav()
            self._send(200, b'{"ok":true}', "application/json")
            return
        if path == "/reset_map":
            bridge.restart_slam()
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
                bridge.set_initial_pose(x, y, yaw)
                self._send(200, b'{"ok":true}', "application/json")
            else:  # /nav_goal
                ok = bridge.send_nav_goal(x, y, yaw)
                body = b'{"ok":true}' if ok else b'{"ok":false,"error":"no nav server"}'
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
        bridge.drive(lx, ly, az)
        self._send(200, b'{"ok":true}', "application/json")


def main():
    global bridge
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--slam-params", default=DEFAULT_SLAM_PARAMS)
    args = ap.parse_args()

    rclpy.init()
    bridge = WebBridge(slam_params=args.slam_params)

    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    bridge.get_logger().info(f"HTTP server on http://{args.host}:{args.port}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.drive(0.0, 0.0, 0.0)  # safety stop
        httpd.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
