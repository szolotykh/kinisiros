# Omni Robot Setup (Kinisi + ROS 2)

How to bring up the **3-wheel omni** Kinisi robot on the Raspberry Pi and drive it
with ROS 2 `Twist` commands. Verified working end-to-end (rotation + translation
confirmed via `/odom`).

## Quick start: one-command bring-up (`start-omni.ps1`)

From the Windows PC, `start-omni.ps1` deploys the latest code and brings up the
**whole stack** on the Pi over SSH (base + rosbridge + web UI + Nav2 + mapping),
all detached so it survives the SSH session:

```powershell
cd C:\Development\kinisi_controller\kinisiros
.\start-omni.ps1                       # start everything (navigate + map at once)
.\start-omni.ps1 -Action status        # show what's up
.\start-omni.ps1 -Action restart       # redeploy code + clean relaunch (needed for src changes)
.\start-omni.ps1 -Action stop          # stop the stack
```

Key options:

| Option        | Values / default                                        | Notes                                              |
| ------------- | ------------------------------------------------------- | -------------------------------------------------- |
| `-Action`     | `start` (default) `stop` `status` `restart`             | `restart` force-relaunches everything.             |
| `-Mode`       | `update-map` (default) `localization` `mapping` `none`  | `update-map` = Nav2 + SLAM together; `none` = robot + UI only. |
| `-RobotHost`  | `rospi.local` (default)                                 | Pass the Pi's **IP** when mDNS won't resolve.      |
| `-SkipDeploy` | switch                                                  | Don't scp code; just (re)start what's on the Pi.   |
| `-SlamMap`    | basename                                                | Continue extending a serialized map (update-map).  |

On `start`/`restart` it **scp's** `robot_client/` (`web_bridge.py`, `index.html`,
`cam_grab.py`) plus the two edited workspace sources (`kinisi_controller.py`,
`nav2_params.yaml`) to the Pi, then relaunches `web_bridge`. The Pi workspace is
built `--symlink-install`, so copied sources are live after a **restart** (no
`colcon build` needed). Nothing survives a Pi **reboot** — just re-run the script.

> **mDNS/DHCP is flaky from Windows.** `rospi.local` intermittently fails to
> resolve and the Pi's DHCP lease changes. If the script can't reach the Pi, get
> its IP with `ssh -i ~/.ssh/pi_kinisi szolotykh@rospi.local "hostname -I"` (or
> scan the LAN for the host that accepts the `pi_kinisi` key) and pass
> `-RobotHost <ip>`.

The sections below document the same components **manually** on the Pi, for
reference and troubleshooting.

## Hardware

- **Controller:** Kinisi STM32F405 motor controller (drives the 3 omni wheels with
  quadrature encoder feedback), connected to the Pi over **USB serial**.
- **Compute:** Raspberry Pi running ROS 2 **Humble**.
- **Platform:** omni (3 wheels at 120°).

### Omni platform parameters

These are set by the ROS node when it initializes the platform
(`src/roskinisi/roskinisi/kinisi_controller.py`):

| Parameter            | Value    | Notes                                  |
| -------------------- | -------- | -------------------------------------- |
| `platform_type`      | `omni`   |                                        |
| `wheels_diameter`    | `0.096` m|                                        |
| `robot_radius`       | `0.18` m | center to wheel                        |
| `encoder_resolution` | `1425.1` | counts per wheel rev (goBILDA 50.9:1)  |
| `is_reversed_0/1/2`  | `True`   | all three wheels reversed              |

Platform PID controller gains: `kp=1`, `ki=0.1`, `kd=0`, `integral_limit=30`.

## Connecting to the Pi

```bash
ssh -i ~/.ssh/pi_kinisi -o IdentitiesOnly=yes szolotykh@rospi.local
```

- Prefer the hostname `rospi.local` — the IP is DHCP and changes (has been
  `192.168.50.212` and `192.168.68.103`).
- Stable serial device (survives `ttyACM0`/`ttyACM1` renumbering):
  `/dev/serial/by-id/usb-VsReality_Kinisi_motor_controller_395433603435-if00`
  (currently → `/dev/ttyACM0`).

> **Note:** the ROS node and the `kinisi-serial-proxy` both need exclusive access
> to the serial port — do **not** run them at the same time.

## Build the workspace (on the Pi)

```bash
cd ~/development/kinisiros
colcon build --symlink-install
source install/setup.bash          # also sourced from ~/.bashrc
```

Dependencies (already installed on the Pi): `pykinisi`, `pyserial`,
`transforms3d`, `ros-humble-tf-transformations`.

## Run the motor controller node

Only the `kinisi_controller` node is required to drive the robot (lidar/nav2 are
optional):

```bash
source /opt/ros/humble/setup.bash
source ~/development/kinisiros/install/setup.bash
ros2 run roskinisi kinisi_controller --ros-args -p port:=/dev/ttyACM0
```

To run detached (survives the SSH session), logging to a file:

```bash
setsid nohup ros2 run roskinisi kinisi_controller --ros-args -p port:=/dev/ttyACM0 \
  </dev/null >~/kinisi_node.log 2>&1 &
```

On startup the node:
- connects to the controller,
- initializes the omni platform,
- starts the platform PID controller and odometry,
- blinks the status LED,
- subscribes to `/cmd_vel` (`geometry_msgs/Twist`) and publishes `/odom` +
  the `odom → base_link` TF.

> **Restart fresh after a controller power-cycle.** In-firmware platform state
> (init, PID, odometry) is lost when the controller loses power, but the node
> process keeps running. If commands stop moving the robot, kill and re-run the
> node so it re-initializes the platform.

### Odometry (world-frame, fixed in firmware)

The controller's platform odometry is now integrated in the **world frame**: the
firmware rotates each tick's body-frame motion increment by the current heading
(midpoint R(θ)) before accumulating it, in `odometry_manager.c` (via the shared
`odometry_integrator.c` helpers). So the raw firmware `x`/`y`/`t` are already a
correct world-frame pose for straight lines, in-place spins **and curved paths**.

`kinisi_controller.py` therefore just **passes the firmware pose through** to
`/odom` and the `odom → base_link` TF — no node-side reconstruction.

> **History:** earlier firmware accumulated the body-frame increment *without*
> the R(θ) rotation, so any curve (driving while turning) corrupted x/y badly
> (a ~0.2 m circle reported metres of phantom X). If you flash **old** firmware,
> odometry will drift on curves again — reflash the current firmware
> (`pio run -e genericSTM32F405RG -t upload`). Covered by the native unit test
> `pio test -e test_platforms -f test_platform_odometry`.


### Full launch (with lidar + robot_state_publisher)

```bash
ros2 launch kinisirobot rsp.launch.py
```

Starts `robot_state_publisher`, `kinisi_controller`, `rplidar_composition`
(`/dev/ttyUSB0`), and the `base_link → laser_frame` static transform.

## Driving with Twist commands

`Twist` fields map to the omni platform velocity:

- `linear.x` — forward/back (m/s)
- `linear.y` — strafe left/right (m/s, omni only)
- `angular.z` — rotate in place (rad/s)

**Use `--once`.** The firmware **latches** the last commanded velocity until it
changes, so a single publish keeps the robot moving. Rate publishers
(`-r 10`) under short `timeout`s often drop messages due to ROS 2 discovery
latency.

Rotate in place, then stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
sleep 2
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"        # stop
```

Drive forward, then stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
sleep 2
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"        # stop
```

> **Always send an empty `{}` Twist to stop** — because the velocity is latched,
> the robot keeps going until you do.

## Checking status

```bash
ros2 topic list
ros2 topic info /cmd_vel            # expect Subscription count: 1 (the node)
ros2 topic echo --once /odom        # live pose feedback
```

Verified drive test: `angular.z=1.0` → odom heading increased ~1.0 rad/s;
`linear.x=0.1` for 2 s → moved ~0.43 m forward.

## Mapping (SLAM)

We use **slam_toolbox** in **online async** mode with
`src/kinisirobot/config/mapper_params_online_async.yaml` (frames: `map`/`odom`/
`base_link`, `scan_topic: /scan`, `mode: mapping`, resolution `0.05`, max range
`12.0` m). Lidar is an **RPLIDAR A1** on `/dev/ttyUSB0`.

Bring-up order (with the `kinisi_controller` node already running):

```bash
source /opt/ros/humble/setup.bash
source ~/development/kinisiros/install/setup.bash

# 1. Lidar (publishes /scan on frame laser_frame)
# Use the stable by-id path: the A1's CP2102 adapter re-enumerates
# ttyUSB0<->ttyUSB1 across USB resets, which silently kills /scan.
LIDAR=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p channel_type:=serial -p serial_port:=$LIDAR -p serial_baudrate:=115200 \
  -p angle_compensate:=true -p frame_id:=laser_frame -p scan_mode:=Sensitivity -p inverted:=false

# 2. Static TF base_link -> laser_frame
# The RPLIDAR is mounted rotated 180 deg about Z relative to base_link (its 0 deg
# beam points to the robot's REAR), so the transform needs yaw = pi. Without this
# the lidar reports the world moving opposite to odometry, SLAM's scan matcher
# fights the odom motion prior, and the map smears (scans never overlap).
# Determined empirically: driving +x, the lidar-observed motion bearing is ~180 deg.
ros2 run tf2_ros static_transform_publisher 0 0 0 3.14159 0 0 base_link laser_frame

# 3. SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/development/kinisiros/src/kinisirobot/config/mapper_params_online_async.yaml \
  use_sim_time:=false
```

TF chain must resolve `map → odom → base_link → laser_frame`
(`map→odom` from slam, `odom→base_link` from the controller node,
`base_link→laser_frame` from the static publisher). Verify:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo --once /map | grep -E "width|height"   # grows as you drive
```

**Build the map by driving** (slow in-place rotation captures the surroundings;
add short forward legs). Then **save**:

```bash
mkdir -p ~/development/kinisiros/maps
cd ~/development/kinisiros/maps
ros2 run nav2_map_server map_saver_cli -f simple_map
```

Produces `simple_map.pgm` + `simple_map.yaml` (used later for
localization/nav2). A blank-ish `/map` usually means the lidar isn't publishing
`/scan` or the TF chain is broken.

## Autonomous navigation (Nav2)

Drive the robot to a goal **on a previously-saved map** using Nav2: localization
is **AMCL** (omni motion model) against the static map, planning is NavFn, and the
local controller is **DWB configured holonomic** (it uses `vx`, `vy` and `vtheta`,
so the omni base can strafe). Config: `src/kinisirobot/config/nav2_params.yaml`;
launch: `src/kinisirobot/launch/navigation.launch.py`.

> **Use the AS-RECORDED map, not the normalized one.** AMCL matches live scans
> against the map, so a map that was rotated/"normalized" for viewing will not
> localize. The launch defaults to the package map
> `share/kinisirobot/maps/kinisi_map.yaml`.

Bring-up (SLAM must **not** be running — both it and map_server+amcl provide
`map → odom`):

```bash
# 1. Base: controller + lidar + robot_state_publisher + base_link->laser_frame TF
ros2 launch kinisirobot rsp.launch.py

# 2. Nav2: map_server + amcl + planner + controller + behaviors + bt_navigator
#    (two lifecycle managers auto-activate everything)
ros2 launch kinisirobot navigation.launch.py
# override the map with:  map:=/abs/path/to/map.yaml
```

On start AMCL warns `AMCL cannot publish a pose ... Please set the initial pose`
and the global costmap logs `frame map does not exist` — this is expected until
you **seed the initial pose**:

```bash
# Tell AMCL where the robot actually is on the map (x, y, yaw). Or use RViz
# "2D Pose Estimate", or the web client "Set Pose" tool.
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, \
   orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, \
   0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"
```

Once seeded, `map → odom` is published by AMCL and the TF chain
`map → odom → base_link → laser_frame` is complete. Send a goal:

```bash
# Verify planning WITHOUT driving (plans only, no motion):
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 0.6, y: 0.0}, \
   orientation: {w: 1.0}}}, use_start: false}"

# Actually drive to a goal:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \
   orientation: {w: 1.0}}}}"
```

Easiest is the **web client** (`robot_client/`): its *Autonomous navigation* panel
lets you click **Set Pose** (seed AMCL) then **Set Goal** (click + drag to aim) and
watches the planned path + nav state; **Cancel Navigation** aborts. Velocities in
`nav2_params.yaml` are intentionally conservative (`max_vel 0.26 m/s`) — tune on
hardware.

### Update-map mode (navigate while extending the map)

Instead of localizing against a *static* map, you can run Nav2 on top of
**slam_toolbox (mapping)** so the map keeps updating while the robot drives to
goals — useful to fill gaps or map new areas. Selected with `slam:=true`:

```bash
# Start a fresh map and refine it while navigating:
ros2 launch kinisirobot navigation.launch.py slam:=true

# Or continue extending a previously-saved map (basename of a serialized
# .posegraph/.data pair — NOT the .yaml):
ros2 launch kinisirobot navigation.launch.py slam:=true \
     slam_map:=/home/szolotykh/maps/kinisi_map
```

In this mode slam_toolbox provides both `map → odom` (so **no Set Pose needed**)
and a continuously-updated `/map` that Nav2's costmaps track. Do **not** also run
`navigation.launch.py` in the default (AMCL) mode or a separate slam_toolbox — only
one thing may publish `map → odom`. Save the improved map with the web client's
**Save Map** button, or:

```bash
# occupancy grid (.pgm/.yaml, for later localization):
ros2 run nav2_map_server map_saver_cli -f ~/maps/kinisi_map
# serialized pose-graph (.posegraph/.data, to continue updating later):
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/szolotykh/maps/kinisi_map'}"
```

## Web client (map view + WASD driving)

A browser client lives in `../robot_client/` (`web_bridge.py` + `index.html`).
It shows the live `/map` with the robot pose and drives via `/cmd_vel`. Run it
**on the Pi** (with `kinisi_controller`, and slam/rplidar for a map, running):

```bash
cd ~/development/kinisiros/robot_client   # deploy target of robot_client/
python3 web_bridge.py --port 8080
```

Open `http://rospi.local:8080/`. Controls: `W`/`S` forward/back, `A`/`D` rotate,
`Q`/`E` strafe, `Space` = stop. **Reset Position** zeroes odometry; **Reset Map**
restarts slam_toolbox with a fresh map; **Save Map** persists the current map
(occupancy grid, plus the slam_toolbox pose-graph when in update-map mode). The
map status line shows whether the map is `updating (SLAM)` or `static (AMCL)`.
The robot is drawn at its **real footprint** (0.30 m radius). Extra panels:

- **Autonomous navigation** — *Set Pose* (seed AMCL), *Set Goal* (click + drag to
  aim), *Cancel Navigation*.
- **Costmap overlay** — toggle the Nav2 global/local costmap on the map
  (`GET /map?costmap=global|local|both`).
- **Camera** — 📷 toggle a periodic snapshot (see **Camera view** below). Off by
  default because capture is expensive.
- **Heartbeat safety stop** — the UI pings `/heartbeat` continuously; if the
  browser disconnects, the watchdog issues a safety stop. This matters because
  the base **latches** its last `/cmd_vel`, so it keeps moving until told to stop.

Only depends on `rclpy` + the stdlib. See `robot_client/README.md` for the full
HTTP API and file layout.

## Camera view

The robot carries a **Raspberry Pi Camera Module 3 (IMX708)**. The web UI's 📷
panel shows a periodic still (there is **no live video** — see below).

### One-time hardware enablement (Ubuntu 22.04 / linux-raspi 5.15)

The stock Ubuntu 22.04 kernel ships **no** imx708 driver or device-tree overlay,
so Camera Module 3 isn't detected out of the box. Bring-up required backported
kernel modules (built against the exact running kernel, kept in `~/cam_backport`
on the Pi):

1. **`imx708.ko`** — the sensor driver. Installed into
   `/lib/modules/$(uname -r)/updates/`, then `depmod -a`. Its `modinfo` *vermagic*
   must match the running kernel.
2. **`imx708.dtbo`** — the device-tree overlay. Copied to the boot `overlays/`
   dir and enabled in `config.txt`:
   ```
   dtoverlay=imx708
   camera_auto_detect=0      # or auto-detect overrides the manual overlay
   ```
   Then **reboot**. (If undetected, try `dtoverlay=imx708,cam0` / `,cam1`.)
3. **`dw9807-vcm.ko`** — the DW9807/DW9817 autofocus VCM driver. The imx708
   declares a lens-focus device; **without this driver unicam never finishes
   probing and `/dev/video0` is never created.** `modprobe dw9807-vcm`.

After reboot the kernel log shows the imx708 module ID and `/dev/video0` +
`/dev/v4l-subdev*` appear. `media-ctl -d /dev/media0 -p` shows the unicam graph.

### Capture pipeline (`robot_client/cam_grab.py`)

There is no working libcamera/rpicam/ffmpeg on this Pi, so capture is done
directly off **unicam**: `media-ctl`/`v4l2-ctl` configure the sensor (binned
2304×1296 RG10, manual exposure/gain, VCM focus), grab a couple of raw frames,
then software-process them. The processing is deliberately **NumPy + PIL, not
OpenCV**: `import cv2` alone costs ~1.6 s per process on this Pi and dominated
capture time. Pipeline:

1. Read the last raw RG10 frame; **half-resolution binned demosaic** in NumPy
   (each 2×2 Bayer block → one RGB pixel) → 1152×648.
2. Gray-world white balance + gamma.
3. Rotate 180° (camera is mounted upside down) and JPEG-encode with PIL.

Result: **~1.6 s/frame, ~56 KB JPEG** (was ~4.7 s and ~250 KB with the OpenCV
path). Useful env vars: `ROTATE` (`0/90/180/270`, default 180), `FRAMES`
(warm-up frames, default 2), `JPEG_QUALITY` (default 85),
`EXPOSURE`/`AGAIN`/`VBLANK`.

### How the UI uses it

`web_bridge.py` serves `GET /camera.jpg`: it returns the cached frame immediately
and kicks a background `cam_grab.py` capture if one isn't already running. The UI
polls every ~2 s **only while the camera panel is open**, so the image refreshes
~every 2 s and there is **zero capture cost when the panel is closed**.

## Troubleshooting

| Symptom | Fix |
| ------- | --- |
| SSH to the IP times out | Use `rospi.local` (IP is DHCP). |
| `rospi.local` won't resolve from Windows | mDNS is flaky and the DHCP lease changes. Get the IP via `ssh ... szolotykh@rospi.local "hostname -I"` when it briefly resolves, or scan the LAN for the host that accepts the `pi_kinisi` key, then pass `-RobotHost <ip>` to `start-omni.ps1`. |
| `Can't open serial connection` | Check the port; ensure the proxy isn't holding it. `sudo chmod 777 /dev/ttyACM0` if permissions block access. |
| Commands logged but robot doesn't move | Controller was likely power-cycled — restart the node to re-init the platform. Also check motor power/battery. |
| Rotation/forward command "missed" | Use `ros2 topic pub --once` instead of a rate publisher with a short timeout. |
| Robot won't stop | Publish an empty `{}` Twist (velocity is latched). |
| Map stops updating / `/scan` silent | The A1's `/dev/ttyUSB*` re-enumerated (e.g. ttyUSB0→ttyUSB1) after a USB reset; the rplidar node is on a dead device. Restart it on the by-id path `/dev/serial/by-id/usb-Silicon_Labs_CP2102_...-port0`. Check with `ros2 topic hz /scan`. |
| Pose/map drifts badly when driving in curves | Old firmware odometry bug (body-frame accumulation without heading rotation). Fixed in current firmware — reflash with `pio run -e genericSTM32F405RG -t upload`. Straight lines/spins are unaffected. |
| Camera panel shows "capturing…" forever | First capture not done yet (~1.6 s) or `cam_grab.py` failed. Check `~/web_bridge.log` for `camera capture failed`; confirm `/dev/video0` exists (VCM driver loaded) and the user is in the `video` group. |
| Camera image upside down / rotated | Set `ROTATE` (`0/90/180/270`) in `cam_grab.py`'s environment; default is 180 for the inverted mount. |
| Robot keeps moving after the browser is closed | Expected — the base latches the last `/cmd_vel`. The UI heartbeat watchdog stops it; otherwise publish an empty `{}` Twist or hit E-Stop. |
