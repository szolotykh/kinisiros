"""Regression checks for October review failures; never connect to a robot."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

from test_map_workflow import bridge_module, make_bridge, motion_bridge, pose_message  # noqa: F401


def test_manual_takeover_revokes_nav_before_publishing(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._nav_state = 'active'
    bridge._nav_goal_handle = handle = Mock()
    bridge.drive(0., 0., .1)
    handle.cancel_goal_async.assert_called_once()
    assert bridge._nav_state == 'canceled'
    bridge._publish_velocity.reset_mock()
    bridge._on_navigation_velocity(NS(linear=NS(x=.26, y=0.), angular=NS(z=0.)))
    bridge._publish_velocity.assert_not_called()


def test_manual_takeover_invalidates_pending_action_acceptance(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._nav_state = 'pending'
    previous = bridge._nav_request_id
    bridge.drive(0., 0., 0.)
    handle = Mock(accepted=True)
    bridge._on_goal_response(Mock(result=Mock(return_value=handle)), previous)
    handle.cancel_goal_async.assert_called_once()
    assert bridge._nav_state == 'canceled'


def test_duplicate_disconnect_keeps_recovery_and_explicit_stop_wins(bridge_module, monkeypatch):
    bridge = motion_bridge(bridge_module)
    bridge._auto_localization_pending = True
    bridge.force_stop = bridge_module.WebBridge.force_stop.__get__(bridge)
    monkeypatch.setattr(bridge_module.threading, 'Thread', Mock())
    for _ in range(3):
        bridge.safety_stop()
        assert bridge._transient_motion_hold
        assert not bridge._motion_stop_latched
    bridge._motion_recovery_since = bridge_module.time.monotonic() - 3
    bridge._recover_transient_motion()
    assert bridge._auto_localization_pending
    bridge.cancel_nav()
    bridge.safety_stop()
    bridge._recover_transient_motion()
    assert bridge._motion_stop_latched
    assert not bridge._auto_localization_pending


def test_runtime_pose_uses_measurement_time_odometry(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._runtime_continuity.transform = (0., 0., 0.)
    bridge._odometry_state.return_value = ((0., 0., .3), {'state': 'ready'}, False)
    bridge._odometry_at = bridge_module.WebBridge._odometry_at.__get__(bridge)
    bridge._tf_buffer = Mock()
    bridge._tf_buffer.lookup_transform.return_value.transform = NS(
        translation=NS(x=0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.))
    time_type = Mock()
    monkeypatch.setattr(bridge_module.rclpy, 'time', NS(Time=time_type), raising=False)
    pose = pose_message(9)
    pose.pose.pose.position.x = pose.pose.pose.position.y = 0.
    bridge._record_runtime_observation(pose)
    assert bridge._map_phase == 'localized'
    assert bridge._localization_quality['confident']
    time_type.from_msg.assert_called_once_with(pose.header.stamp)
    bridge._tf_buffer.lookup_transform.assert_called_once_with('odom', 'base_link', time_type.from_msg.return_value)


def test_missing_paired_odometry_waits_without_consuming_evidence(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._odometry_at.return_value = None
    bridge._odometry_at.side_effect = None
    bridge._record_runtime_observation(pose_message(9))
    assert bridge._last_runtime_checked_ns == 0
    assert bridge._runtime_bad_samples == 0
    assert bridge._map_phase == 'localized'


def test_duplicate_rejected_pose_counts_once(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'updating'
    bridge._scan_quality.return_value = {'scan_agrees': False, 'scan_match': .4}
    for _ in range(3):
        bridge._record_runtime_observation(pose_message(9))
    assert bridge._runtime_bad_samples == 1
    assert bridge._map_phase == 'updating'


def test_new_map_failure_retains_live_graph_and_allows_save(bridge_module, monkeypatch, tmp_path):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'updating'
    bridge._localization_map_base = None
    bridge._mapping_process = backend = Mock()
    backend.poll.return_value = None
    bridge.start_map_localization = Mock()
    bridge._save_map_snapshot = Mock(return_value={'ok': True})
    monkeypatch.setattr(bridge_module, 'MAPS_DIR', str(tmp_path))
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    bridge._recover_mapping('LiDAR temporarily unavailable')
    assert bridge._map_phase == 'mapping_paused'
    assert bridge._mapping_process is backend
    assert bridge.save_map('recovery')['ok']
    bridge.start_map_localization.assert_not_called()
    worker.assert_not_called()


def test_dead_new_map_process_reports_error_without_loading_none(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'updating'
    bridge._localization_map_base = None
    bridge._mapping_process = Mock()
    bridge._mapping_process.poll.return_value = 1
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    bridge._recover_mapping('SLAM process exited')
    assert bridge._map_phase == 'mapping_error'
    assert bridge._map_phase_error == 'SLAM process exited'
    worker.assert_not_called()


def test_paused_new_map_resumes_only_after_distinct_good_stationary_scans(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'mapping_paused'
    bridge._map_phase_error = 'Waiting for scans'
    for sec in (8, 8, 9):
        bridge._record_runtime_observation(pose_message(sec))
        assert bridge._map_phase == 'mapping_paused'
    bridge._record_runtime_observation(pose_message(10))
    assert bridge._map_phase == 'updating'
    assert bridge._map_phase_error is None
    bridge._nav_client.send_goal_async.assert_not_called()


def test_paused_new_map_does_not_forward_unverified_scan(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'mapping_paused'
    bridge._scans = []
    bridge._mapping_scan_pub = Mock()
    bridge._on_scan(NS(header=pose_message(10).header))
    bridge._mapping_scan_pub.publish.assert_not_called()


def test_clearance_history_fills_complementary_gaps_but_never_hides_obstacles():
    from test_localization_motion import scan
    from kinisirobot.localization_motion import rotation_clearance_evidence
    first = [2.] * 360
    second = [2.] * 360
    first[40:80] = [math.inf] * 40
    second[120:160] = [math.inf] * 40
    views = [(scan(first), (0., 0., 0.)), (scan(second), (0., 0., 0.))]
    assert not rotation_clearance_evidence(views[:1])['clear']
    assert rotation_clearance_evidence(views)['clear']
    second[125] = .2
    evidence = rotation_clearance_evidence(views)
    assert not evidence['clear']
    assert evidence['blocked']
    assert 'obstacle' in evidence['reason']


def test_unknown_coverage_remains_blocked_with_specific_reason():
    from test_localization_motion import scan
    from kinisirobot.localization_motion import rotation_clearance_evidence
    evidence = rotation_clearance_evidence([(scan([math.inf] * 360), (0., 0., 0.))])
    assert not evidence['clear']
    assert len(evidence['missing_sectors']) == 36
    assert 'coverage incomplete' in evidence['reason']


def test_motion_history_requires_fresh_paired_scans_and_new_obstacles_veto(bridge_module, monkeypatch):
    from test_localization_motion import scan, costmap
    bridge = motion_bridge(bridge_module)
    bridge._motion_inputs = bridge_module.WebBridge._motion_inputs.__get__(bridge)
    bridge._local_costmap = costmap()
    bridge._local_costmap.header = NS(frame_id='odom', stamp=NS(sec=10, nanosec=0))
    previous = scan([2.] * 360)
    previous.header = NS(frame_id='laser_frame', stamp=NS(sec=9, nanosec=800_000_000))
    current = scan([2.] * 360)
    current.ranges[40:80] = [math.inf] * 40
    current.header = NS(frame_id='laser_frame', stamp=NS(sec=9, nanosec=900_000_000))
    bridge._scans = [previous, current]
    time_type = Mock()
    time_type.from_msg.side_effect = lambda stamp: stamp
    monkeypatch.setattr(bridge_module.rclpy, 'time', NS(Time=time_type), raising=False)
    transform = NS(transform=NS(translation=NS(x=0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    def lookup(target, source, stamp):
        if target == 'odom' and stamp is current.header.stamp:
            raise LookupError('odometry TF has not arrived yet')
        return transform
    bridge._tf_buffer = Mock()
    bridge._tf_buffer.lookup_transform.side_effect = lookup
    _, fresh, clear, _, _ = bridge._motion_inputs()
    assert fresh and clear
    current.ranges[60] = .2
    assert not bridge._motion_inputs()[2]
    assert 'obstacle' in bridge._motion_blocker
    current.ranges[60] = math.inf
    previous.header.stamp.nanosec = 500_000_000  # Older than 400 ms.
    assert not bridge._motion_inputs()[2]
    assert 'coverage incomplete' in bridge._motion_blocker


def test_time_paired_rotation_history_compensates_changed_heading(bridge_module, monkeypatch):
    from test_localization_motion import scan, costmap
    bridge = motion_bridge(bridge_module)
    bridge._motion_inputs = bridge_module.WebBridge._motion_inputs.__get__(bridge)
    bridge._odometry_state.return_value = ((0., 0., math.pi/2), {'state':'ready'}, True)
    bridge._local_costmap = costmap()
    bridge._local_costmap.header = NS(frame_id='odom', stamp=NS(sec=10, nanosec=0))
    laser = scan([2.] * 360)
    laser.header = NS(frame_id='laser_frame', stamp=NS(sec=10, nanosec=0))
    bridge._scans = [laser]
    bridge._tf_buffer = Mock()
    bridge._tf_buffer.lookup_transform.return_value.transform = NS(
        translation=NS(x=.1, y=0.), rotation=NS(x=0., y=0., z=0., w=1.))
    monkeypatch.setattr(bridge_module.rclpy, 'time', NS(Time=Mock()), raising=False)
    evaluate = Mock(return_value={'clear':True, 'blocked':False, 'reason':'', 'missing_sectors':[]})
    monkeypatch.setattr(bridge_module, 'rotation_clearance_evidence', evaluate)
    bridge._motion_inputs()
    compensated = evaluate.call_args.args[0][0][1]
    assert abs(compensated[0]) < 1e-10
    assert abs(compensated[1] + .1) < 1e-10
    assert abs(compensated[2] + math.pi/2) < 1e-10


def test_paused_mapping_retries_scan_after_odometry_tf_arrives(bridge_module, monkeypatch):
    bridge = motion_bridge(bridge_module)
    bridge._map_phase = 'mapping_paused'
    bridge._slam_candidate = pose_message(8)
    latest = NS(header=NS(stamp=NS(sec=9, nanosec=900_000_000)))
    paired = NS(header=NS(stamp=NS(sec=9, nanosec=700_000_000)))
    bridge._scans = [paired, latest]
    time_type = Mock()
    time_type.from_msg.side_effect = lambda stamp: stamp
    monkeypatch.setattr(bridge_module.rclpy, 'time', NS(Time=time_type), raising=False)
    def lookup(target, source, stamp):
        if stamp is latest.header.stamp:
            raise LookupError('TF is not ready for the latest scan yet')
        return NS(transform=NS(translation=NS(x=1., y=2.),
                               rotation=NS(x=0., y=0., z=0., w=1.)))
    bridge._tf_buffer = Mock()
    bridge._tf_buffer.lookup_transform.side_effect = lookup
    bridge._monitor_navigation_health()
    assert bridge._last_localization_observation_ns == 9_700_000_000
    assert bridge._fresh_mapping_good_samples == 1
    bridge._monitor_navigation_health()
    assert bridge._fresh_mapping_good_samples == 1  # Rechecking is not new evidence.


def test_amcl_pose_waits_for_paired_tf_then_counts_once(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._odometry_at.side_effect = None
    bridge._odometry_at.return_value = None
    bridge._on_amcl_pose(pose_message(9))
    assert bridge._localization_samples == 0
    assert bridge._last_amcl_stamp_ns < 9_000_000_000
    bridge._odometry_at.return_value = (0., 0., 0.)
    bridge._monitor_navigation_health()
    assert bridge._localization_samples == 1
    bridge._monitor_navigation_health()
    assert bridge._localization_samples == 1
