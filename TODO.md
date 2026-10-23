# Kinisi Omni project TODOs

The October 22 review fixes and SDK upgrade pass 232 offline tests against the
built pykinisi 2.5.0 wheel. Robot deployment is deferred at the user's
request; the hostname also failed to resolve during the initial read-only check.
The tasks below require a ROS 2 Humble machine, the connected robot, or a release decision.

## Next: ROS environment validation

- [ ] With the stack stopped, install the tested pykinisi 2.5.0 wheel in ROS's
  Python environment; preserve the previous SDK and sources for rollback.
- [ ] Run `rosdep install --from-paths src --ignore-src -r -y` on Ubuntu with ROS 2 Humble.
- [ ] Re-run a clean `colcon build --symlink-install` after the latest changes.
- [ ] Re-run `colcon test` and review `colcon test-result --verbose`. The prior
  revision passed 166 tests with six expected skips; that does not validate
  the September 23 or October 22 changes.
- [ ] Re-run smoke tests of the installed base launch, web bridge, LiDAR diagnostics,
  odometry, TF, and packaged web assets.

## LiDAR reliability

- [x] Select the robot's stable `/dev/serial/by-id/...` LiDAR path automatically.
- [ ] Verify cold-boot scan startup repeatedly.
- [ ] Boot with LiDAR unplugged and the motor controller present, then connect
  LiDAR; verify discovery never opens the motor-controller port.
- [x] Verify recovery after terminating the RPLIDAR driver.
- [ ] Verify recovery after stale scans and a USB disconnect/reconnect. Stale
  scans are detected, but a forcibly wedged CP2102 currently requires a
  physical reconnect or a privileged targeted USB reset.
- [ ] Capture ROS, supervisor, kernel, and USB logs for any remaining failure.
- [ ] Tune startup, stale-scan, and restart-backoff timing from hardware results.

## Localization and map safety

- [ ] Validate localization from the last trusted pose for the same map.
- [ ] Validate global localization when no saved pose exists.
- [ ] Validate recovery after the robot is picked up and moved.
- [ ] Confirm bounded left-right localization turns are obstacle-safe.
- [ ] Confirm a goal can be selected while localizing but executes only after
  confidence is stable and live sensor checks pass.
- [ ] Confirm read-only AMCL localization enables navigation without requiring
  SLAM or map updates.
- [ ] Confirm optional map updates require validation and failed SLAM startup
  restores read-only localization without rearming motion.
- [ ] Confirm localization loss stops motion and prevents map corruption.
- [ ] Test sensor loss during a new, unsaved map: retain/save the graph and
  resume only after distinct stationary evidence; a dead backend must not reset it.
- [ ] Replay delayed AMCL/odometry and sparse scans; verify timestamp pairing,
  short-history clearance and close-obstacle vetoes.
- [ ] Validate map save/load round trips, including the pose graph and saved pose.

## Navigation and operator safety

- [ ] Test autonomous goals, replanning, cancellation, and manual takeover.
- [ ] Test browser disconnect, heartbeat stop, and emergency stop.
- [ ] Test repeated blur/hide/unload events, focus return, and explicit Stop;
  no background window may keep automatic-motion permission alive.
- [ ] Bench-test velocity expiration while SDK heartbeats remain alive, and
  confirm late Nav2 commands cannot override Stop in the bridge-managed stack.
- [ ] Bench-test blocked velocity and blocked zero ACKs: verify session revocation
  stops SDK heartbeats, firmware stops motors, and the driver cannot auto-rearm.
- [ ] Confirm the 30 cm configured radius against physical measurements.
- [ ] Confirm odometry and TF remain stable while translating and rotating.
  Stationary `/odom` at 10 Hz and `odom -> base_link` TF have been verified.
- [ ] Confirm the camera and web UI cannot block core robot startup.

## Release candidate

- [ ] Review hardware results and resolve any failures.
- [ ] Choose final package versions and changelog dates.
- [ ] Perform a clean-install release-candidate test.
- [ ] Review the final diff and Git metadata.
- [ ] Tag and publish only after physical validation and explicit approval.

See `RELEASE_CHECKLIST.md` for the release gate and completed offline scope.
