"""Regressions for review findings, using real bridge paths with fake ROS boundaries."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

from test_map_workflow import (  # noqa: F401
    bridge_module, make_bridge, motion_bridge, pose_message, immediate_service,
)


def test_default_localization_keeps_amcl_and_does_not_start_slam(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    bridge._update_loaded_map = False
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    for sec in (4, 5, 6):
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._map_phase == 'localized'
    worker.assert_not_called()
    bridge._remember_pose.assert_called_once()


def test_remembered_pose_worker_executes_real_refinement_path(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._amcl_state_client = immediate_service(NS(current_state=NS(id=3)))
    bridge._initialpose_pub = Mock()
    bridge._active_map_id = 'map'
    bridge._map = None
    bridge._scans = []
    saved = {'x': 1., 'y': 2., 'yaw': .3}
    bridge.settings = Mock()
    bridge.settings.snapshot.return_value = {'last_pose': saved}
    bridge.set_initial_pose = Mock()
    bridge._request_saved_localization(1)
    bridge.set_initial_pose.assert_called_once_with(1., 2., .3, automatic=True)


def test_worker_exception_becomes_visible_error(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._seed_saved_localization = Mock(side_effect=ValueError('bad saved pose'))
    bridge._request_saved_localization(1)
    assert bridge._map_phase == 'localization_error'
    assert 'bad saved pose' in bridge._map_phase_error


def test_missing_paired_slam_scan_waits_instead_of_claiming_mismatch(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'starting_mapping'
    bridge._mapping_reference_grid = object()
    bridge._mapping_readiness = bridge_module.MappingReadiness(1_000_000_000)
    bridge._scan_quality.return_value = {'scan_agrees': False, 'scan_match': None, 'reason': 'missing scan'}
    for sec in (3, 4, 5):
        bridge._on_slam_pose(pose_message(sec))
    assert bridge._map_phase == 'starting_mapping'
    assert bridge._mapping_diagnostics['reason'] == 'missing scan'
    bridge.force_stop.assert_not_called()


def test_scan_validation_honors_explicit_reference(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    q = NS(x=0., y=0., z=0., w=1.)
    origin = NS(position=NS(x=0., y=0.), orientation=q)
    def grid(width):
        return NS(info=NS(width=width, height=1, resolution=.05, origin=origin), data=[0]*width)
    message = pose_message(4)
    bridge._scans = [NS(header=message.header, ranges=[1.], angle_min=0., angle_increment=.1,
                        range_min=.01, range_max=10.)]
    bridge._map = grid(1)
    bridge._tf_buffer = Mock()
    bridge._tf_buffer.lookup_transform.return_value.transform = NS(translation=NS(x=0., y=0.), rotation=q)
    monkeypatch.setattr(bridge_module.rclpy, 'time', NS(Time=Mock()), raising=False)
    score = Mock(return_value={'scan_agrees': True})
    monkeypatch.setattr(bridge_module, 'scan_alignment', score)
    bridge_module.WebBridge._scan_quality(bridge, message, reference_grid=grid(2))
    assert score.call_args.args[6] == 2


def test_inconsistent_remembered_poses_cannot_approve_navigation(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._update_loaded_map = False
    for sec, x in ((4, 1.), (5, 4.), (6, 1.)):
        message = pose_message(sec)
        message.pose.pose.position.x = x
        bridge._on_amcl_pose(message)
    assert bridge._map_phase == 'localizing'


def test_new_immediate_goal_removes_older_waiting_intent(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge.request_nav_goal(1., 2., 0.)
    bridge._map_phase = 'localized'
    bridge.request_nav_goal(3., 4., 0.)
    bridge._dispatch_queued_nav_goal()
    assert bridge._nav_goal == {'x': 3., 'y': 4., 'yaw': 0.}
    assert bridge._queued_nav_goal is None
    bridge._nav_client.send_goal_async.assert_called_once()


def test_runtime_health_loss_cancels_goal_and_revalidates_amcl(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._motion_inputs = Mock()
    handle = bridge._nav_goal_handle = Mock()
    bridge._nav_state = 'active'
    bridge._navigation_inputs_ready.return_value = False
    bridge._monitor_navigation_health()
    handle.cancel_goal_async.assert_called_once()
    assert bridge._map_phase == 'localizing'
    assert bridge._nav_state == 'canceled'


def test_confident_runtime_pose_jump_stops_navigation(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._record_runtime_observation(pose_message(8))
    jumped = pose_message(9)
    jumped.pose.pose.position.x = 4.
    bridge._record_runtime_observation(jumped)
    assert bridge._map_phase == 'localizing'
    assert bridge._nav_state == 'canceled'
    assert not bridge._localization_quality['confident']


def test_slam_failure_stops_navigation_and_schedules_single_rollback(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'updating'
    bridge._nav_goal_handle = handle = Mock()
    bridge._mapping_process = Mock()
    bridge._mapping_process.poll.return_value = 1
    bridge._mapping_started_at = 0.
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    bridge._check_mapping_startup()
    bridge._check_mapping_startup()
    assert bridge._map_phase == 'recovering_localization'
    assert not bridge._update_loaded_map
    handle.cancel_goal_async.assert_called_once()
    worker.return_value.start.assert_called_once()


def test_rollback_is_read_only_and_does_not_rearm_motion(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge.start_map_localization = Mock()
    bridge._rollback_to_localization(1)
    bridge.start_map_localization.assert_called_once_with('/maps/office', allow_motion=False)
    bridge.start_map_localization.reset_mock()
    bridge._rollback_to_localization(0)
    bridge.start_map_localization.assert_not_called()


def test_explicit_stop_blocks_transient_recovery(bridge_module, monkeypatch):
    bridge = motion_bridge(bridge_module)
    bridge._transient_motion_hold = True
    bridge._motion_stop_latched = True
    bridge._motion_recovery_since = 1.
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 10.)
    bridge._recover_transient_motion()
    assert not bridge._auto_localization_pending
    bridge._motion_stop_latched = False
    bridge._recover_transient_motion()
    assert bridge._auto_localization_pending
    assert bridge._motion_retries == 1


def test_managed_nav_commands_cannot_override_stop(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._nav_state = 'active'
    message = NS(linear=NS(x=.1, y=0.), angular=NS(z=0.))
    bridge._on_navigation_velocity(message)
    bridge._publish_velocity.assert_called_once_with(.1, 0., 0.)
    bridge.cancel_nav()
    bridge._publish_velocity.reset_mock()
    bridge._on_navigation_velocity(message)
    bridge._publish_velocity.assert_not_called()


def test_managed_nav_commands_fail_closed_on_stale_inputs(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._map_phase = 'localized'
    bridge._nav_state = 'active'
    bridge._navigation_inputs_ready.return_value = False
    bridge._on_navigation_velocity(NS(linear=NS(x=.1, y=0.), angular=NS(z=0.)))
    bridge._publish_velocity.assert_not_called()
    assert bridge._nav_state == 'canceled'


def test_failed_serialization_cannot_reuse_old_graph(bridge_module, tmp_path, monkeypatch):
    bridge = make_bridge(bridge_module)
    old = tmp_path / 'maps'
    stage = tmp_path / 'stage'
    old.mkdir()
    stage.mkdir()
    for ext in ('.yaml', '.pgm', '.posegraph', '.data'):
        (old / ('office' + ext)).write_bytes(b'old')
    def command(args, **kwargs):
        if 'map_saver_cli' in args:
            for ext in ('.yaml', '.pgm'):
                (stage / ('office' + ext)).write_bytes(b'new')
            return NS(returncode=0, stdout='', stderr='')
        return NS(returncode=1, stdout='', stderr='failed')
    monkeypatch.setattr(bridge_module.subprocess, 'run', command)
    monkeypatch.setattr(bridge_module, 'MAPS_DIR', str(old))
    result = bridge._save_map_snapshot('office', str(stage))
    assert not result['ok']
    assert (old / 'office.pgm').read_bytes() == b'old'
