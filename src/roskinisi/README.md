
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
  x: 50.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```