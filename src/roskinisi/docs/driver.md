# Kinisi ROS 2 driver

The driver uses firmware protocol 2.1+ and the Python SDK (`pykinisi>=2.5.0,<3`).
Install the SDK into the Python environment used by ROS, then build the workspace
with `colcon build --symlink-install` and source `install/setup.bash`.
Colcon alone does not install the pip dependency. The ROS dependencies are
declared in `package.xml` and can be installed with the usual rosdep workflow.

```bash
python3 -m pip install --find-links sdk-wheels -r src/roskinisi/requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 run roskinisi kinisi_controller --ros-args -p port:=/dev/ttyACM0
```

### SDK 2.5 upgrade

As checked on October 22, 2026, PyPI's latest release is 2.3.1, while the clean
SDK checkout at `2fd834b6c390e94c405e002a9797889a20c7525b` is 2.5.0. From
the ROS workspace root, build a portable wheel with:

```bash
python3 -m pip wheel --no-deps --no-build-isolation --wheel-dir sdk-wheels ../pykinisi
```

This needs local setuptools/wheel build tooling. Transfer the resulting
`pykinisi-2.5.0-py3-none-any.whl` to the Pi's `sdk-wheels` directory when ready.
For an offline SDK upgrade in an existing environment with `pyserial` installed:

```bash
python3 -m pip install --no-index --find-links sdk-wheels -r src/roskinisi/requirements.txt
python3 -c "import pykinisi; print(pykinisi.__version__, pykinisi.__file__)"
```

Stop the robot stack before upgrading; keep its previous SDK artifact and package
sources for rollback, preserve maps/settings, then rebuild **both** `roskinisi`
and `kinisirobot`. Restart initially with localization motion disabled and verify
stationary odometry, TF, LiDAR and logs before supervised movement. The Windows
launcher copies/builds ROS sources but does not install the SDK. Do not use it
for this upgrade until the SDK is installed in the Python environment used by
ROS (normally `/usr/bin/python3` on Humble).

The SDK package version is separate from firmware protocol and hardware revision.
SDK 2.5 connects to protocol 2.1+; newer position/sensor commands are gated by
firmware support. This driver still uses the same velocity and odometry APIs,
with unchanged geometry, polarity, units and watchdog settings. It does not
enable position control, read IMU sensors, or change robot localization policy.
Connection logs report the actual SDK and firmware protocol versions.

## Startup parameters

These parameters are read-only after startup. Restart the node to change them.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `port` | required | Controller serial device |
| `platform_type` | `omni` | `omni` or `mecanum`; retains the existing geometry and polarity settings |
| `heartbeat_timeout_ms` | `500` | Firmware connection timeout, 100–60,000 ms |
| `command_timeout_ms` | `500` | Host receipt-age and velocity ACK deadline, 100–2,000 ms |
| `odometry_frequency_hz` | `20` | Background calculation frequency, 1–1,000 Hz |
| `odometry_interval_ms` | `100` | Subscription and ROS publication timer interval |

The subscription interval must span at least two calculation periods. The
controller uses millisecond calculation periods: `period_ms = 1000 // frequency`.
For the default 20 Hz calculation rate, subscriptions must be 100 ms or slower.
The driver rejects invalid combinations before opening the serial connection.

This physical driver requires `use_sim_time=false`. The Python SDK synchronizes
the controller to host Unix time, not ROS simulation time.

## Odometry

Connection setup finishes INIT and clock synchronization before odometry starts.
The controller calculates continuously and streams the latest completed platform
sample. The ROS timer reads the SDK cache without sending a GET for each update.
Repeated or out-of-order timestamps are skipped.

`/odom` and the `odom` to `base_link` transform use the same acquisition timestamp.
Position is in `odom`; finite-difference velocity uses measurement intervals and
is rotated into `base_link`. A skipped stream sample therefore does not assume a
fixed delivery interval. The first sample has zero estimated velocity.

`/reset_odometry` temporarily unsubscribes, resets the controller pose and local
velocity history, and subscribes again. This prevents old cached poses and reset
jumps from producing false velocity estimates.

When time synchronization is stale, the last clock mapping is still used and a
warning is logged. Uptime-only timestamps and unready clocks are not published.

## Connection handling

The SDK enables the configured watchdog after setup. It sends a heartbeat only
after other outgoing traffic has been idle for one fifth of that timeout.
Heartbeats run in the SDK's background thread; no ROS ping timer is required.

Session loss stops publication and terminates the driver; it does not reconnect
and resume movement automatically. On normal shutdown, the
driver attempts to stop the platform controller and coast its motors before
unsubscribing and closing serial. If communication is already lost, the firmware
watchdog provides the fallback. It must be enabled for subscriptions.

Heartbeats monitor the serial connection, not the velocity publisher. An
independent driver thread checks command age every 50 ms and schedules zero when
a nonzero command expires. Publish velocity commands regularly (for example at
10 Hz). Receipt time is recorded before sending, not after receiving an ACK.
Blocking serial calls never hold the watchdog's state lock; even the expiry-zero
request is watched. A velocity request that fails or exceeds the ACK deadline
revokes the SDK session, cancels pending I/O and stops its heartbeats. No command
can rearm that failed session; restart the driver after investigating the cause.

`command_timeout_ms` is not a guaranteed physical stopping time. Under an ACK or
transport stall, firmware stops on `heartbeat_timeout_ms` after the last valid
traffic, plus host scheduling/transport latency. On expiry of a received command,
the driver first attempts zero; a stalled zero also triggers session revocation.
Bench-test both blocked nonzero and blocked zero requests with wheels lifted,
then measure actual stopping distance under supervision. This is not a replacement
for an independent physical emergency stop or hardware-enforced velocity expiry.

## Tests

Portable tests use ROS message/node stubs and the real SDK method signatures:

```bash
python3 -m pytest test/test_odometry.py test/test_driver.py
```

Run `colcon test --packages-select roskinisi` in a ROS 2 environment as well.
