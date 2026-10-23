# Kinisi ROS 2 workspace

This repository contains two ROS 2 packages with a strict dependency boundary:

- `roskinisi` is the reusable Kinisi motor-controller driver. It translates
  velocity commands, publishes timestamped odometry, and exposes controller
  operations. It has no Omni, LiDAR, mapping, camera, or web-client policy.
- `kinisirobot` is the Omni robot application. It owns the robot description,
  LiDAR supervision, Nav2, AMCL, SLAM, saved-map localization, camera, and web UI.

The dependency direction is `kinisirobot -> roskinisi -> pykinisi`.

## Build

On Ubuntu with ROS 2 Humble installed:

```bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --find-links sdk-wheels -r src/roskinisi/requirements.txt
python3 -m pip install -r src/kinisirobot/requirements.txt
colcon build --symlink-install
source install/setup.bash
```

The driver now targets `pykinisi>=2.5.0,<3`. As checked on October 22, 2026,
PyPI still provides 2.3.1; build the 2.5.0 wheel from the sibling `pykinisi`
checkout **before** the install step above:

```bash
python3 -m pip wheel --no-deps --no-build-isolation --wheel-dir sdk-wheels ../pykinisi
```

Use the clean SDK revision `2fd834b6c390e94c405e002a9797889a20c7525b`
for the tested 2.5.0 build. A wheel built on Windows is also usable on the Pi
(`py3-none-any`). This does not publish an SDK release. See the
[driver guide](src/roskinisi/docs/driver.md) for the upgrade sequence; stop the
robot stack before replacing its SDK or package sources. Colcon does not
install or upgrade the SDK itself.

Run the base and sensors with:

```bash
ros2 launch kinisirobot rsp.launch.py
```

Run navigation and the web application with:

```bash
ros2 launch kinisirobot navigation.launch.py
ros2 run kinisirobot kinisi_web_bridge
```

See [the Omni setup guide](OMNI_ROBOT_SETUP.md) and
[saved-map workflow](src/kinisirobot/docs/map-workflow.md) for operational details.

## Release status

The code is being prepared for an initial ROS package release. Offline tests
must pass before every release candidate. Physical LiDAR, localization,
navigation, emergency-stop, and map round-trip tests are required before the
final tag. See [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).

The actionable remaining work is tracked in [TODO.md](TODO.md).
