ROS 2 driver for Kinisi firmware protocol 2.1+, using `pykinisi>=2.5.0,<3`.
Publishes timestamped odometry and TF, and accepts `/cmd_vel` commands.
See the [driver guide](docs/driver.md) for setup, parameters, and connection handling.

Install the SDK in the Python environment used by ROS before building:

```bash
python3 -m pip install --find-links sdk-wheels -r src/roskinisi/requirements.txt
```

Run this from the workspace root. SDK 2.5.0 is currently available from the
source checkout, not PyPI (checked October 22, 2026). Build or copy its wheel
into `sdk-wheels` first; see the [driver guide](docs/driver.md). Firmware does
not need protocol 2.5 for the existing velocity and streamed-odometry interface.

Build ws:
colcon build --symlink-install

Run node:
```
ros2 run roskinisi kinisi_controller
```
Arguments:
* `port`- Serial port name


Example:
```
ros2 run roskinisi kinisi_controller --ros-args -p port:=/dev/ttyACM0
```

Publish cmd_vel
```
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.05
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
