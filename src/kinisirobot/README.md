# Kinisi Omni robot application

`kinisirobot` contains the Omni-specific ROS 2 application: robot description,
LiDAR supervision, Nav2, AMCL, SLAM, saved-map localization, camera, and browser
control. The reusable motor-controller interface remains in `roskinisi`.

## Build and run

```bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/kinisirobot/requirements.txt
colcon build --symlink-install --packages-select roskinisi kinisirobot
source install/setup.bash

ros2 launch kinisirobot rsp.launch.py
ros2 launch kinisirobot navigation.launch.py backend:=false
ros2 run kinisirobot kinisi_web_bridge --port 8080 --start-mapping
```

Open `http://rospi.local:8080/` from the robot's local network.

## Package responsibilities

- `kinisirobot/web_bridge.py`: web API, map view, teleoperation, Nav2 goals,
  camera signaling, map bundle operations, and localization workflow.
- `kinisirobot/lidar_supervisor.py`: owns the RPLIDAR process, monitors `/scan`,
  reports diagnostics, and restarts unhealthy drivers with bounded backoff.
- `kinisirobot/map_localization.py`: localization confidence and safe map-update
  handoff checks.
- `kinisirobot/robot_settings.py`: atomic, per-map trusted-pose persistence.
- `launch/`: base/sensor and navigation startup.
- `config/`: Nav2, AMCL, and SLAM parameters.
- `description/`: Omni URDF/Xacro.
- `web/`: browser application assets.

## Saved-map behavior

Loading a map first starts a read-only AMCL session. A remembered pose is used
only when it belongs to the exact map contents; otherwise AMCL starts global
localization. The application checks covariance and live scan agreement and can
perform bounded, obstacle-checked turns to collect distinct views. Navigation
waits until several consistent observations agree. Users can select a goal
on the loaded map during this process: it appears as **Waiting for localization**
and starts automatically after localization is verified. A new
selection replaces the waiting goal. Stop, Cancel, browser heartbeat loss, a map
change, or a localization error cancels it; reconnecting does not restore it.
Selecting a goal does not stop or rearm localization movements.

After localization succeeds, AMCL stays active and the UI shows **Position
confirmed**. Navigation uses the saved map without editing it. **Enable map
updates** is a separate, stationary operation; `--update-loaded-map` opts into
the same transition at startup. SLAM output must be verified before it replaces
the displayed map. Failure triggers one read-only AMCL recovery with no automatic
turns, not an endless SLAM retry. Live sensor or localization loss cancels goals.

For a maintenance restart that must preserve stopped localization motion, use
`kinisi_web_bridge --localize-map /absolute/path/to/map --no-localization-motion`
(map basename without `.yaml`). This still allows stationary matching.

## LiDAR recovery

`rsp.launch.py` starts the LiDAR supervisor rather than launching the driver
directly. An explicit `lidar_serial_port` is preferred. When it is empty, the
supervisor selects a unique RPLIDAR/Silicon Labs CP210 adapter by-id. It never
selects an arbitrary single device or guesses `/dev/ttyUSB0`. Unrecognized or
ambiguous devices wait with an actionable diagnostic; configure their exact
by-id path explicitly. A CP210 name identifies the adapter, not its attached
sensor, so explicit configuration is preferred when other adapters are present.
Discovery is retried after hotplug; once identified, that device identity stays
pinned across disconnects. Driver exit, missing startup
scans, and stale scan streams trigger bounded restarts and `/diagnostics` status.
Driver logs are in `~/lidar_driver.log`. Restart cleanup owns the entire process
group, including a driver child whose `ros2 run` wrapper has already exited.
Retry delay remains bounded through multi-day outages. Scan health distinguishes
fresh, progressing, correctly framed data from empty/malformed scans and missing
range coverage. Five consecutive usable scans reset recovery backoff. A valid
stream containing only missing returns reports a coverage warning, not healthy
clearance, and does not trigger a pointless serial restart loop.

Example override:

```bash
ros2 launch kinisirobot rsp.launch.py \
  lidar_serial_port:=/dev/serial/by-id/your-rplidar-device \
  lidar_scan_mode:=Standard
```

## Tests

Offline tests do not connect to or move a robot:

```bash
python3 -m pytest src/kinisirobot/test -q
python3 -m pytest src/roskinisi/test -q
```

The live Playwright workflow in `test/browser_map_workflow.py` is intentionally
opt-in because it resets and reloads the robot's active map.
