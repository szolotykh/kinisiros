^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roskinisi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (unreleased)
------------------
* Add Kinisi protocol 2.1 support through ``pykinisi``.
* Update the SDK baseline to ``pykinisi>=2.5.0,<3`` while retaining firmware protocol 2.1+ support.
* Report the actual SDK and firmware protocol versions at connection time.
* Publish timestamped wheel odometry and the ``odom`` to ``base_link`` transform.
* Keep this package limited to the reusable motor-controller ROS interface.
* Expire stale velocity commands independently of ROS spin and SDK connection heartbeats.
* Revoke the SDK session on blocked velocity or zero-command ACKs without waiting for the command I/O lock.
