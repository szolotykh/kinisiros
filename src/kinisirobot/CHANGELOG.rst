^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kinisirobot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (unreleased)
------------------
* Package the Omni robot application separately from the reusable controller driver.
* Add portable saved-map bundles and automatic localization before map updates.
* Remember trusted poses per map and support global localization when no pose exists.
* Add guarded localization turns, scan-to-map verification, and clear UI state.
* Add WebRTC camera streaming, map rotation, and Nav2 goal control.
* Add supervised LiDAR startup and stale-scan recovery.
* Add offline unit and browser regression coverage.
* Keep AMCL localization and navigation separate from optional map extension.
* Recover rejected SLAM handoffs to read-only localization with structured diagnostics.
* Fix remembered-pose startup, goal supersession, and continuous readiness checks.
* Align Nav2 footprint with the robot model; retain LiDAR process-group ownership.
* Stage complete map snapshots and stop deployment after build failures.
* Make manual takeover authoritative and browser-disconnect recovery idempotent.
* Pair pose checks with measurement-time odometry and deduplicate rejected evidence.
* Preserve unsaved mapping sessions through transient sensor loss.
* Discover LiDAR conservatively after hotplug, bound long-outage backoff, and validate scan health.
* Use short, motion-compensated clearance history with explicit coverage diagnostics.
