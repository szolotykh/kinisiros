# ROS release checklist

## Offline release-candidate checks

- [x] `roskinisi` contains only reusable motor-controller integration.
- [x] `kinisirobot` contains all Omni-specific application code and assets.
- [x] Package manifests have complete dependencies, maintainers, licenses, and versions.
- [ ] Publish pykinisi 2.5.0 with approval, or supply its tested wheel for installation;
  PyPI still provided only 2.3.1 when checked on October 22.
- [ ] Verify the SDK 2.5.0 wheel installs in the Pi's ROS Python environment and
  repeat the stationary controller/odometry smoke test before enabling motion.
- [ ] `rosdep install --from-paths src --ignore-src -r -y` succeeds on ROS 2 Humble.
- [ ] Re-run `colcon build --symlink-install` after the October 22 safety/recovery fixes.
- [x] Unit and browser tests pass locally against the pykinisi 2.5.0 wheel
  (232 tests, October 22; ROS-only lint tests excluded).
- [x] Build both ROS package wheels offline; Humble colcon validation remains pending.
- [ ] Re-run `colcon test` and `colcon test-result --verbose` after these fixes.
- [ ] Re-run installed-stack smoke tests after the latest launch, configuration,
  bridge, controller, and web changes.
- [x] Changelogs and public documentation match the release candidate.

## Robot validation required before the final tag

- [ ] Cold boot repeatedly starts the controller and LiDAR.
- [ ] LiDAR supervisor recovers from driver exit, stale scans, and USB reconnect.
  Driver-exit recovery passes; a forcibly wedged USB device still needs a
  privileged USB reset or physical reconnect.
- [ ] New-map mode starts without localization.
- [ ] Unsaved new-map sensor loss preserves a saveable graph and recovers without loading `None.yaml`.
- [ ] A saved map with a trusted pose localizes before enabling updates.
- [ ] A saved map without a trusted pose completes global active localization.
- [ ] Localization loss stops the robot and prevents map corruption.
- [ ] Navigation, replanning, manual stop, browser disconnect, and emergency stop are safe.
- [ ] Command-source loss expires velocity while SDK heartbeats remain alive (bench test).
- [ ] Blocked nonzero and zero ACKs revoke the SDK session; measure the firmware
  heartbeat fallback stop time on a bench, not an unattended floor test.
- [ ] Background/duplicate browser leave events cannot rearm explicit Stop.
- [ ] Delayed poses use paired odometry; sparse scan history never masks a close obstacle.
- [ ] New goals cannot be replaced by older queued intents at localization completion.
- [ ] Optional SLAM failure restores read-only AMCL without rearming stopped motion.
- [ ] Confirm the conservative 30 cm footprint against physical chassis measurements.
- [ ] Save/load round trips preserve the occupancy map and pose graph.
- [ ] Camera and web UI remain optional and do not block core robot startup.

## Publishing

- [ ] Choose final package versions and replace `unreleased` changelog dates.
- [ ] Review the complete diff and verify the configured Git author.
- [ ] Create and test the release candidate tag.
- [ ] Publish only after physical validation and explicit approval.
