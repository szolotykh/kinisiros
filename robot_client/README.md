# Kinisi web client

A zero-dependency browser client for the OMNI robot. Runs **on the Pi** and talks
to ROS 2 through a small `rclpy` node embedded in an HTTP server. Open it at
`http://rospi.local:8080/`.

## Files

| File            | Purpose                                                              |
| --------------- | ------------------------------------------------------------------- |
| `web_bridge.py` | ROS 2 node + `ThreadingHTTPServer`. Serves the UI and bridges HTTP ⇄ ROS (`/map`, `/cmd_vel`, Nav2 actions, slam_toolbox services, camera). Depends only on `rclpy` + the Python stdlib. |
| `index.html`    | Single-page UI (map canvas, WASD/joystick driving, Nav2 goal/pose tools, costmap overlay, camera panel). No build step, no external JS. |
| `cam_grab.py`   | One-shot camera capture (raw unicam → NumPy demosaic → PIL JPEG). Invoked by `web_bridge.py` on demand. See the **Camera view** section of `../OMNI_ROBOT_SETUP.md`. |

## Running

```bash
cd ~/development/kinisiros/robot_client
python3 web_bridge.py --port 8080        # --host 0.0.0.0 by default
```

Normally you don't run it by hand — `start-omni.ps1` (on the PC) deploys this
folder and launches `web_bridge.py` as part of the stack. It needs
`kinisi_controller` running for driving/odometry, and slam_toolbox/rplidar for a
live map.

## UI features

- **Map view** – live `/map` with the robot drawn at its **real footprint**
  (0.30 m radius), pan/zoom, heading arrow.
- **Driving** – `W`/`S` forward-back, `A`/`D` rotate, `Q`/`E` strafe, `Space` stop.
- **Autonomous navigation** – *Set Pose* seeds AMCL, *Set Goal* (click + drag to
  aim) sends a Nav2 goal, *Cancel Navigation* aborts.
- **Costmap overlay** – toggle the global/local Nav2 costmap on the map.
- **Camera** – 📷 toggle shows a periodic snapshot (see below). Off by default.
- **Map tools** – *Reset Position* (zero odom), *Reset Map* (fresh SLAM),
  *Save Map* (occupancy grid + pose-graph).
- **Heartbeat safety stop** – the UI pings `/heartbeat` continuously; if the
  browser disconnects, the watchdog issues a safety stop (the base **latches**
  its last `/cmd_vel`, so an explicit stop is required).

## HTTP API

`web_bridge.py` exposes:

| Method & path        | Body / query                    | Effect                                              |
| -------------------- | ------------------------------- | --------------------------------------------------- |
| `GET /`              | –                               | The UI (`index.html`).                              |
| `GET /map`           | `?costmap=global\|local\|both`  | Occupancy grid (+ optional costmap) as JSON; gzip if the client accepts it. |
| `GET /camera.jpg`    | –                               | Latest camera still (`image/jpeg`); kicks a background capture. `503 capturing` until the first frame exists. `X-Frame-Age` header = seconds since capture. |
| `POST /cmd_vel`      | `{lx, ly, az}`                  | Drive: linear x/y + angular z (m/s, rad/s).         |
| `POST /heartbeat`    | –                               | Dead-man's-switch ping.                             |
| `POST /estop`        | –                               | Emergency stop: cancel nav + force the base to halt.|
| `POST /reset_odom`   | –                               | Zero odometry.                                      |
| `POST /reset_map`    | –                               | Restart slam_toolbox with a fresh map.              |
| `POST /save_map`     | `{name}`                        | Save occupancy grid (+ pose-graph in update-map).   |
| `POST /nav_goal`     | `{x, y, yaw?}`                  | Send a Nav2 `NavigateToPose` goal (map frame).      |
| `POST /set_initial_pose` | `{x, y, yaw?}`              | Seed AMCL initial pose.                             |
| `POST /cancel_nav`   | –                               | Cancel the active Nav2 goal.                        |

## Camera panel

The Pi has **no** libcamera/rpicam/ffmpeg, so there is no live video — the panel
requests `GET /camera.jpg` every ~2 s while open. Each request serves the cached
frame immediately and starts a background `cam_grab.py` capture if one isn't
already running, so the image refreshes roughly every ~2 s (capture ≈ 1.6 s).
Captures only run while the panel is open (zero cost when closed). The camera is
mounted upside down, so `cam_grab.py` rotates frames 180° by default
(`ROTATE` env: `0/90/180/270`). Capture tuning env vars: `FRAMES` (warm-up
frames, default 2), `JPEG_QUALITY` (default 85), `EXPOSURE`/`AGAIN`/`VBLANK`.
