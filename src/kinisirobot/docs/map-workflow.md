# Map loading and localization

A new map starts mapping immediately. Loading a saved map starts read-only
localization near its remembered pose, or searches the map if there is no
usable prior. Fresh laser observations must agree with the map before updating
it. Every handoff requires fresh, stationary odometry and a settled observation.
The remembered pose is first refined against current scans within the configured
20–30 cm neighborhood and ±20 degrees. This supplies an initial guess; it never
bypasses the independent confidence checks. When needed, the robot can turn up
to ±15 degrees at no more than 0.12 rad/s, pausing to collect observations.
After localization is confirmed, AMCL remains running and goals can execute on
the saved, read-only map. Goal placement is available while localization is still
pending; execution waits for verified position and live sensor/TF health.
Map extension is separate: use **Enable map updates** while stopped, or explicitly
opt into it at bridge startup with `--update-loaded-map`.
Whole-map searches additionally require consistent matches from headings at
least 10 degrees apart. Repeated scans at one stationary heading cannot approve
a global match. Every prior must also have consistent map-to-odom estimates.
Pose continuity uses odometry TF at the observation's acquisition timestamp,
not the latest odometry. Missing paired TF waits for evidence. Runtime checks
deduplicate both accepted and rejected timestamps.
SLAM startup is checked against the original grid and confirmed transform with
fresh, time-paired scans. A stationary SLAM pose can be corroborated for up to
three seconds while the base remains settled. Missing scans/TF are waiting
states, not map mismatches. Three valid matches and fresh SLAM map output are
required before accepting map updates. Repeated mismatch, provider failure, or
a 60-second startup deadline triggers one read-only AMCL rollback with automatic
motion disabled. Diagnostics retain the SLAM pose, score, count and reason.
This rollback applies only to saved maps. With a new, unsaved map, live-input
loss instead pauses scan forwarding and retains the running SLAM graph. Saving
is still allowed. Three distinct, verified stationary observations can resume
mapping without restoring a canceled goal. A dead new-map backend is reported
as an error, never restarted against a nonexistent saved map or silently reset.

The client reports missing or stale LiDAR, odometry, local costmap, and laser
transform data separately. Missing data prevents automatic movement. STOP,
manual control, lost browser connection, and switching tabs stop movements and
cancel navigation goals. Explicit Stop/manual takeover remains latched. A
temporary sensor/connection hold can retry bounded localization turns at most
twice, after guards stay healthy for two seconds, unless Stop or maintenance
mode prohibits it. There is no Resume button. Stationary matching continues.
Repeated leave events preserve the temporary hold. Blur/hidden events do not
send manual commands or impersonate explicit Stop. Unfocused/hidden pages stop
heartbeats, and a leave event immediately revokes server-side liveness.
Sensor status continues updating while paused. Missing scans or odometry do
not consume the local-search timeout or trigger a premature whole-map search.

Mapping startup waits for new map and pose messages from its current session.
After 45 seconds the client shows a waiting message; healthy output can still
complete startup until the rollback deadline. Browser
status requests time out after four seconds and retry automatically.

For bridge-owned backends (`backend:=false`), Nav2 controller and behavior output
is remapped to `/cmd_vel_nav`. The bridge forwards it to `/cmd_vel` only for its
active goal while position/sensors/TF are ready. Stop invalidates that intent,
so late Nav2 commands cannot defeat the stop. The reusable controller driver
independently expires nonzero commands after `command_timeout_ms` (500 ms by
default), even if ROS spin stalls. A stalled velocity/zero ACK revokes the serial
session; final physical stopping depends on the firmware heartbeat timeout.
See the controller driver guide for timing limits and required bench tests.
Manual drive atomically cancels active/pending/queued navigation before publishing;
late Nav2 output is ignored. Standalone external-backend launch remains a
separate configuration; it does not use the bridge's goal/velocity gate.

## Laser scan compatibility

Serialized SLAM maps retain the laser's angular layout. The Omni bridge publishes
`/scan_mapping` with 1080 angular bins to match its existing saved pose graphs,
even when the RPLIDAR runs in Standard mode with 360 bins. Nearest-neighbor
resampling preserves missing returns and the acquisition timestamp; it does not
increase sensor resolution. AMCL, navigation, and movement clearance continue
using the original `/scan` measurements.

Turn clearance combines at most six time-paired scans no older than 400 ms (or
the configured freshness limit, if shorter), with laser poses transformed into
the current base frame. Only finite returns contribute observed sectors. A close
return in any included scan, including the newest scan before its odometry TF
arrives, vetoes motion. All 36 sectors and the swept costmap footprint must still
be clear; persistent blind sectors remain blocked, with a specific coverage
diagnostic rather than a claim that LiDAR is offline. Validate this short history
with recorded scans and supervised obstacle tests before deployment.

Keep the mapping layout, angular limits, and laser mounting transform consistent
when reusing a pose graph. `LocalizationConfig.mapping_scan_samples` configures
the layout for another robot; this is not automatic conversion of arbitrary
saved laser models.

## Backend ownership

Use `start-omni.ps1 -Action restart -Mode update-map` after updating an older
installation. In update-map and mapping modes the web bridge owns the map
backend. Navigation starts with `backend:=false`, so it does not create a
second map provider. The launcher passes a saved `-SlamMap` to localization
before allowing map updates.

For a manual setup, start the controller and LiDAR, start navigation with
`ros2 launch kinisirobot navigation.launch.py backend:=false`, then start:

```sh
ros2 run kinisirobot kinisi_web_bridge --start-mapping
# Or localize a saved map before updating it:
ros2 run kinisirobot kinisi_web_bridge --localize-map /absolute/path/to/map_basename
```

The bridge stops only process groups it launches, including their ROS children.
SIGTERM and Ctrl+C release these backends. It refuses map replacement when it
detects externally managed map providers. Stop those through their launcher;
the bridge never terminates them by process name. The launcher's standalone
localization mode remains externally managed.
Replacing an owned backend waits for its nodes to leave ROS discovery, so an
immediate reset/load does not mistake the old nodes for another installation.
The URDF's robot state publisher is the sole source of the laser mounting
transform; do not also launch a static publisher for `laser_frame`.

## Reusing the components

- `localization_config.py` holds the chassis dimensions, frames, freshness
  limits, and structured sensor diagnostics. Inject a `LocalizationConfig`
  into `WebBridge` for another robot, keeping its ROS frame and footprint
  configuration consistent with that profile.
- `localization_motion.py` evaluates bounded turns and clearance independently
  of ROS. Unknown scan sectors and unknown costmap cells remain obstacles to
  automatic movement.
- `map_localization.py` evaluates observation confidence and tracks new-session
  mapping output without ROS dependencies.
- `mapping_scan.py` normalizes the mapping laser layout independently of ROS.
- `localization_seed.py` searches near a remembered pose without ROS dependencies.
- `owned_backend.py` handles Linux process groups independently of ROS launch
  internals. It retains the group leader until shutdown to prevent PID reuse.

Offline regressions run with `python -m pytest src/kinisirobot/test -q`.
The pytest browser tests require Playwright and its Chromium installation;
their requests are intercepted locally, so they never contact a robot. Physical
clearance, ROS lifecycle transitions, and real sensor timing still require a
robot integration test.

`src/kinisirobot/test/browser_map_workflow.py` is a separate, opt-in live test. It
resets the working map, uploads a backup bundle, stops automatic movements, and
checks the localization handoff and subsequent stationary map stability. Run it
only with a robot prepared for testing and an existing map backup:

```sh
python src/kinisirobot/test/browser_map_workflow.py --url http://ROBOT:8080 \
  --bundle /path/to/map.kinisi-map.zip --output /path/to/test-results
```

This stationary check does not validate navigation accuracy while driving.
With the robot prepared for movement, add `--allow-localization-motion` to test
the bounded automatic turns. The test stops the robot once localization is ready,
then observes map stability; it never sends a navigation goal.
