"""Exercise bridge state transitions without a ROS installation or robot motion."""

import importlib
import sys
import threading
import types
from pathlib import Path
from unittest.mock import Mock

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


@pytest.fixture
def bridge_module(monkeypatch):
    """Stub ROS imports only; execute the actual bridge workflow methods."""
    def missing_package(_name):
        """Exercise the source-tree asset fallback outside a ROS install."""
        raise LookupError("not installed")

    exports = {
        'ament_index_python': {},
        'ament_index_python.packages': {
            'get_package_share_directory': missing_package},
        'rclpy': {'ok': Mock(return_value=True)}, 'rclpy.node': {'Node': object},
        'rclpy.action': {'ActionClient': Mock()},
        'rclpy.qos': {name: Mock() for name in (
            'QoSProfile', 'QoSDurabilityPolicy', 'QoSHistoryPolicy',
            'QoSReliabilityPolicy', 'qos_profile_sensor_data')},
        'nav_msgs.msg': {'OccupancyGrid': Mock(), 'Path': Mock(), 'Odometry': Mock()},
        'geometry_msgs.msg': {name: Mock() for name in ('Twist', 'PoseWithCovarianceStamped', 'PoseStamped')},
        'std_msgs.msg': {'Empty': Mock()}, 'std_srvs.srv': {'Empty': Mock()},
        'lifecycle_msgs.srv': {'GetState': Mock()},
        'rcl_interfaces.msg': {'Log': Mock()},
        'sensor_msgs.msg': {'LaserScan': Mock()},
        'nav2_msgs.action': {'NavigateToPose': Mock()}, 'tf2_ros': {},
    }
    for name, attributes in exports.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, 'kinisirobot.web_bridge', raising=False)
    return importlib.import_module('kinisirobot.web_bridge')


def make_bridge(module):
    """Set up a seeded localization session with mocked process boundaries."""
    bridge = module.WebBridge.__new__(module.WebBridge)
    bridge._closing = False
    bridge.get_clock = Mock()
    bridge.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    bridge.get_node_names = Mock(return_value=[])
    bridge._lock = threading.Lock()
    bridge._map_phase_lock = threading.Lock()
    bridge._motion_lock = threading.RLock()
    bridge._map_operation_lock = threading.RLock()
    bridge._nav_lock = threading.Lock()
    bridge._stop_lock = threading.Lock()
    bridge._stop_gen = 0
    bridge._nav_goal_handle = None
    bridge._nav_request_id = 0
    bridge._nav_goal = None
    bridge._queued_nav_goal = None
    bridge._nav_state = 'idle'
    bridge._nav_client = Mock()
    bridge._hb_lock = threading.Lock()
    bridge._hb_alive = True
    bridge._hb_last = module.time.monotonic()
    bridge._hb_timeout = 1.5
    bridge._motion_settled_after_ns = 0
    bridge.localization_config = module.DEFAULT_LOCALIZATION_CONFIG
    bridge._sensor_health = {}
    bridge._localization_fresh_since = None
    bridge._stationary_observation = Mock(return_value=True)
    bridge._localization_motion = None
    bridge._auto_localization_pending = False
    bridge._auto_localization_cancelled = False
    bridge._motion_stop_latched = False
    bridge._allow_localization_motion = True
    bridge._transient_motion_hold = False
    bridge._motion_recovery_since = None
    bridge._motion_retries = 0
    bridge._auto_localization_message = ''
    bridge._localization_search_scope = 'remembered'
    bridge._motion_blocker = ''
    bridge._wait_for_amcl_map = Mock(return_value=True)
    bridge._localization_started_at = module.time.monotonic() - 4
    bridge.force_stop = Mock()
    bridge._map_phase = 'localizing'
    bridge._mapping_process = None
    bridge._mapping_reference_grid = None
    bridge._pending_mapping_grid = None
    bridge._global_evidence = module.GlobalLocalizationEvidence()
    bridge._pose_continuity = module.PoseContinuity()
    bridge._runtime_continuity = module.PoseContinuity(anchored=False)
    bridge._update_loaded_map = True
    bridge._map_update_error = None
    bridge._mapping_diagnostics = None
    bridge._mapping_good_samples = bridge._mapping_bad_samples = 0
    bridge._last_mapping_stamp_ns = 0
    bridge._last_localization_observation_ns = 0
    bridge._last_runtime_checked_ns = 0
    bridge._fresh_mapping_good_samples = 0
    bridge._runtime_bad_samples = 0
    bridge._slam_candidate = None
    bridge._amcl_candidate = None
    bridge._remember_pose = Mock()
    bridge._odometry_state = Mock(return_value=((0, 0, 0), {'state': 'ready'}, True))
    bridge._odometry_at = Mock(side_effect=lambda stamp: bridge._odometry_state()[0])
    bridge._navigation_inputs_ready = Mock(return_value=True)
    bridge._last_amcl_stamp_ns = 1_000_000_000
    bridge._pose_seed_ns = 1_000_000_000
    bridge._localization_samples = 0
    bridge._localization_generation = 1
    bridge._localization_map_base = '/maps/office'
    bridge._scan_quality = Mock(return_value={'scan_agrees': True, 'scan_match': 0.9})
    bridge.get_logger = Mock()
    return bridge


def pose_message(sec, variance=0.01):
    """Represent an AMCL pose whose header identifies its source scan."""
    from types import SimpleNamespace as NS
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = variance
    return NS(header=NS(frame_id='map', stamp=NS(sec=sec, nanosec=0)),
              pose=NS(covariance=covariance, pose=NS(
                  position=NS(x=1.0, y=2.0), orientation=NS(x=0, y=0, z=0, w=1))))


def test_queued_destination_waits_for_localization_and_dispatches_once(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge.send_nav_goal = Mock(return_value=True)
    assert bridge.request_nav_goal(1, 2, .3)
    assert bridge._nav_state == 'waiting_localization'
    bridge._dispatch_queued_nav_goal()
    bridge.send_nav_goal.assert_not_called()
    bridge._map_phase = 'waiting_mapping'
    bridge.cancel_nav(preserve_waiting=True)
    assert bridge._nav_state == 'waiting_localization'
    bridge._dispatch_queued_nav_goal()
    bridge.send_nav_goal.assert_not_called()
    bridge._map_phase = 'updating'
    bridge._dispatch_queued_nav_goal()
    bridge._dispatch_queued_nav_goal()
    bridge.send_nav_goal.assert_called_once_with(x=1., y=2., yaw=.3)


@pytest.mark.parametrize('reason', ['stop', 'disconnect', 'map_change', 'error', 'stale_heartbeat'])
def test_waiting_destination_is_canceled_safely(bridge_module, reason):
    bridge = make_bridge(bridge_module)
    bridge.send_nav_goal = Mock()
    bridge.request_nav_goal(1, 2, 0)
    if reason == 'stop':
        bridge.cancel_nav()
    elif reason == 'disconnect':
        bridge.safety_stop()
    elif reason == 'map_change':
        bridge._localization_generation += 1
    elif reason == 'error':
        bridge._map_phase = 'localization_error'
    else:
        bridge._hb_last -= 10
    bridge._dispatch_queued_nav_goal()
    bridge._map_phase = 'updating'
    bridge._dispatch_queued_nav_goal()
    assert bridge._queued_nav_goal is None
    assert bridge._nav_state == 'canceled'
    bridge.send_nav_goal.assert_not_called()


def test_latest_destination_replaces_waiting_destination(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge.send_nav_goal = Mock()
    bridge.request_nav_goal(1, 2, 0)
    bridge.request_nav_goal(3, 4, .5)
    bridge._map_phase = 'updating'
    bridge._nav_client.server_is_ready.return_value = False
    bridge._dispatch_queued_nav_goal()
    bridge.send_nav_goal.assert_not_called()
    bridge._nav_client.server_is_ready.return_value = True
    bridge._dispatch_queued_nav_goal()
    bridge.send_nav_goal.assert_called_once_with(x=3., y=4., yaw=.5)


def test_stop_cancels_late_action_acceptance(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._nav_request_id = 4
    bridge.cancel_nav()
    future = Mock()
    future.result.return_value.accepted = True
    bridge._on_goal_response(future, 4)
    future.result.return_value.cancel_goal_async.assert_called_once()
    assert bridge._nav_state == 'canceled'


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_destination_rejected(bridge_module, value):
    bridge = make_bridge(bridge_module)
    assert not bridge.request_nav_goal(value, 2, 0)
    assert bridge._queued_nav_goal is None


def test_narrow_prior_with_wrong_scan_never_starts_mapping(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    bridge._scan_quality.return_value = {'scan_agrees': False, 'scan_match': 0.1}
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    for sec in range(2, 9):
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._map_phase == 'localizing'
    assert bridge._localization_samples == 0
    worker.assert_not_called()


def test_duplicate_and_pre_seed_poses_do_not_unlock_mapping(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    for sec in (0, 1, 2, 2, 2):
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._localization_samples == 1
    worker.assert_not_called()


def test_bad_observation_resets_consecutive_matches(bridge_module, monkeypatch):
    bridge = make_bridge(bridge_module)
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    bridge._on_amcl_pose(pose_message(2))
    bridge._on_amcl_pose(pose_message(3))
    bridge._on_amcl_pose(pose_message(4, variance=9.0))
    bridge._on_amcl_pose(pose_message(5))
    bridge._on_amcl_pose(pose_message(6))
    worker.assert_not_called()
    bridge._on_amcl_pose(pose_message(7))
    assert bridge._map_phase == 'starting_mapping'
    worker.return_value.start.assert_called_once()
    bridge._on_amcl_pose(pose_message(8))
    worker.return_value.start.assert_called_once()


def test_process_launch_does_not_immediately_report_updating(bridge_module):
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'starting_mapping'
    bridge._mapping_readiness = bridge_module.MappingReadiness(2_000_000_000)
    bridge._observe_mapping_output('map', pose_message(1).header)
    bridge._observe_mapping_output('pose', pose_message(3).header)
    assert bridge._map_phase == 'starting_mapping'
    bridge._observe_mapping_output('map', pose_message(4).header)
    assert bridge._map_phase == 'updating'


def test_unverified_slam_grid_cannot_replace_loaded_map(bridge_module, monkeypatch):
    """A failed SLAM handoff must neither expose a distorted grid nor enable goals."""
    from types import SimpleNamespace as NS
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'starting_mapping'
    bridge._map = reference = object()
    bridge._mapping_reference_grid = reference
    bridge._mapping_readiness = bridge_module.MappingReadiness(1_000_000_000)
    bad_grid = NS(header=pose_message(2).header, info=NS(width=20,height=20))
    bridge._on_map(bad_grid)
    assert bridge._map is reference
    bridge._scan_quality.return_value = {'scan_agrees': False, 'scan_match': .2}
    monkeypatch.setattr(bridge_module.threading, 'Thread', Mock())
    for sec in (3, 4, 5):
        bridge._on_slam_pose(pose_message(sec))
    assert bridge._map_phase == 'recovering_localization'
    bridge._on_map(bad_grid)
    assert bridge._map is reference
    bridge.force_stop.assert_called_once()


def test_matching_slam_pose_releases_pending_grid(bridge_module):
    """Only a checked SLAM pose and fresh grid finish the map handoff."""
    from types import SimpleNamespace as NS
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'starting_mapping'
    bridge._map = reference = object()
    bridge._mapping_reference_grid = reference
    bridge._mapping_readiness = bridge_module.MappingReadiness(1_000_000_000)
    bridge._remember_pose = Mock()
    grid = NS(header=pose_message(2).header, info=NS(width=20,height=20))
    bridge._on_map(grid)
    for sec in (3, 4, 5):
        bridge._on_slam_pose(pose_message(sec))
    assert bridge._map_phase == 'updating'
    assert bridge._map is grid


def test_global_match_at_one_heading_cannot_unlock_mapping(bridge_module, monkeypatch):
    """Repeated stationary matches after global resampling are insufficient evidence."""
    bridge = make_bridge(bridge_module)
    bridge._localization_search_scope = 'global'
    bridge._odometry_state = Mock(return_value=((0,0,0), {'state':'ready'}, True))
    monkeypatch.setattr(bridge_module.threading, 'Thread', Mock())
    for sec in range(2, 15):
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._map_phase == 'localizing'
    assert not bridge._localization_quality['views_confirmed']


def test_explicit_pose_replaces_global_search_with_a_local_prior(bridge_module):
    """An operator-supplied position still needs scan checks, but is no longer global."""
    bridge = motion_bridge(bridge_module)
    bridge._localization_search_scope = 'global'
    bridge._initialpose_pub = Mock()
    bridge._initialpose_pub.get_subscription_count.return_value = 1
    bridge.get_clock.return_value.now.return_value.to_msg.return_value = pose_message(10).header.stamp
    bridge.settings = Mock()
    bridge.settings.snapshot.return_value = {'localization_radius_m': .3}
    bridge.set_initial_pose(1,2,.3)
    assert bridge._localization_search_scope == 'manual'
    assert bridge._map_phase == 'localizing'
    assert bridge._localization_samples == 0


def motion_bridge(module):
    """Build the real movement/stop integration around mocked sensor inputs."""
    bridge = make_bridge(module)
    bridge._stop_lock = threading.Lock()
    bridge._stop_gen = 0
    bridge._nav_lock = threading.Lock()
    bridge._nav_state = 'idle'
    bridge._nav_goal_handle = None
    bridge._publish_velocity = Mock()
    bridge._motion_inputs = Mock(return_value=((0, 0, 0), True, True, True, True))
    bridge.get_clock = Mock()
    bridge.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    return bridge


def test_manual_command_cancels_turns_permanently(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge.start_localization_motion()
    bridge._tick_localization_motion()
    assert bridge._publish_velocity.call_args.args[2] > 0
    bridge.drive(0, 0, 0)
    assert not bridge._localization_motion.active
    bridge._publish_velocity.reset_mock()
    bridge._tick_localization_motion()
    bridge._publish_velocity.assert_not_called()


def test_emergency_stop_prevents_later_automatic_commands(bridge_module, monkeypatch):
    bridge = motion_bridge(bridge_module)
    bridge.start_localization_motion()
    # Exercise the real stop entry point; don't launch its background thread.
    bridge.force_stop = bridge_module.WebBridge.force_stop.__get__(bridge)
    monkeypatch.setattr(bridge_module.threading, 'Thread', Mock())
    bridge.cancel_nav()
    bridge._publish_velocity.assert_called_with(0.0, 0.0, 0.0)
    assert not bridge._localization_motion.active
    bridge._publish_velocity.reset_mock()
    bridge._tick_localization_motion()
    bridge._publish_velocity.assert_not_called()


@pytest.mark.parametrize('inputs', [
    ((0, 0, 0), False, True, True, True),
    ((0, 0, 0), True, False, True, True),
    ((0, 0, 0), True, True, False, True),
    ((0, 0, 0), True, True, True, False),
])
def test_start_requires_fresh_clear_stationary_connected_state(bridge_module, inputs):
    bridge = motion_bridge(bridge_module)
    bridge._motion_inputs.return_value = inputs
    with pytest.raises(ValueError):
        bridge.start_localization_motion()
    assert bridge._localization_motion is None
    bridge._publish_velocity.assert_not_called()


def test_mapping_waits_for_stationary_scans_after_turns(bridge_module, monkeypatch):
    bridge = motion_bridge(bridge_module)
    bridge._stationary_observation = bridge_module.WebBridge._stationary_observation.__get__(bridge)
    bridge._odometry_state = Mock(return_value=((0,0,0), {"state": "ready"}, True))
    bridge._motion_settled_after_ns = 10_000_000_000
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    for sec in (8, 9, 10):
        bridge._on_amcl_pose(pose_message(sec))
    worker.assert_not_called()
    bridge._odometry_state.return_value = ((0,0,0), {"state": "ready"}, False)
    bridge.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
    bridge._on_amcl_pose(pose_message(11))
    assert bridge._localization_samples == 0
    bridge._motion_inputs.return_value = ((0, 0, 0), True, True, True, True)
    bridge._odometry_state.return_value = ((0,0,0), {"state": "ready"}, True)
    for sec in (13, 14, 15):
        bridge.get_clock.return_value.now.return_value.nanoseconds = sec * 1_000_000_000
        bridge._on_amcl_pose(pose_message(sec))
    bridge.force_stop.assert_called_once()
    worker.return_value.start.assert_called_once()


def test_sensor_loss_during_turn_publishes_zero(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge.start_localization_motion()
    bridge._tick_localization_motion()
    bridge._motion_inputs.return_value = ((0, 0, 0), False, True, True, True)
    bridge._tick_localization_motion()
    assert not bridge._localization_motion.active
    bridge._publish_velocity.assert_called_with(0.0, 0.0, 0.0)
    bridge.force_stop.assert_called_once()


def test_stale_stop_worker_cannot_cancel_new_movement(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge.start_localization_motion()
    bridge._spam_stop(0, 0.1, 20)
    assert bridge._localization_motion.active
    bridge._publish_velocity.assert_not_called()


def test_independent_watchdog_stops_stalled_ros_timer(bridge_module, monkeypatch):
    bridge = motion_bridge(bridge_module)
    bridge.start_localization_motion()
    bridge._localization_motion.last_tick -= 1
    bridge._hb_lock = threading.Lock()
    bridge._hb_enabled = False
    monkeypatch.setattr(bridge_module.rclpy, 'ok', Mock(side_effect=[True, False]), raising=False)
    monkeypatch.setattr(bridge_module.time, 'sleep', Mock())
    bridge._heartbeat_watchdog()
    assert not bridge._localization_motion.active
    assert 'timer stalled' in bridge._localization_motion.reason
    bridge._publish_velocity.assert_called_with(0.0, 0.0, 0.0)
    bridge.force_stop.assert_called_once()


def test_automatic_localization_starts_without_an_operator_request(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._auto_localization_pending = True
    bridge._automatic_localization()
    assert bridge._localization_motion.active
    assert not bridge._auto_localization_pending
    first = bridge._localization_motion
    bridge._automatic_localization()
    assert bridge._localization_motion is first


def test_pending_automatic_movement_waits_for_sensors_and_stop_cancels_it(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._auto_localization_pending = True
    bridge._motion_inputs.return_value = ((0,0,0), False, False, True, True)
    bridge._motion_blocker = 'Waiting for fresh LiDAR'
    bridge._automatic_localization()
    assert bridge._localization_motion is None
    assert bridge._auto_localization_message == 'Waiting for fresh LiDAR'
    bridge.drive(0,0,0)
    bridge._motion_inputs.return_value = ((0,0,0), True, True, True, True)
    bridge._automatic_localization()
    assert bridge._localization_motion is None


def immediate_service(result):
    """Model a ROS service completion without a ROS executor."""
    future = Mock()
    future.result.return_value = result
    future.add_done_callback.side_effect = lambda callback: callback(future)
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    return client


def test_map_without_remembered_pose_requests_global_localization(bridge_module):
    from types import SimpleNamespace as NS
    bridge = motion_bridge(bridge_module)
    bridge._amcl_state_client = immediate_service(NS(current_state=NS(id=3)))
    bridge._initialpose_pub = Mock()
    bridge._initialpose_pub.get_subscription_count.return_value = 1
    bridge._active_map_id = 'test-map'
    bridge.settings = Mock()
    bridge.settings.snapshot.return_value = {'last_pose': None}
    bridge._request_global_localization = Mock()
    bridge.set_initial_pose = Mock()
    bridge._request_saved_localization(1)
    bridge._request_global_localization.assert_called_once_with(1)
    bridge.set_initial_pose.assert_not_called()


def test_global_search_completion_requires_no_set_pose(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._global_localization_client = immediate_service(object())
    bridge._request_global_localization(1)
    assert bridge._localization_search_scope == 'global'
    assert bridge._map_phase == 'localizing'
    assert bridge._pose_seed_ns == 10_000_000_000


def test_previous_map_global_search_result_is_ignored(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._global_localization_client = immediate_service(object())
    bridge._request_global_localization(0)
    bridge._global_localization_client.call_async.assert_not_called()
    assert bridge._localization_search_scope == 'remembered'


def test_global_search_never_calls_amcl_before_map_acknowledgement(bridge_module):
    bridge = motion_bridge(bridge_module)
    bridge._wait_for_amcl_map.return_value = False
    bridge._global_localization_client = immediate_service(object())
    bridge._request_global_localization(1)
    bridge._global_localization_client.call_async.assert_not_called()


def test_amcl_map_acknowledgement_rejects_old_or_unrelated_logs(bridge_module):
    from types import SimpleNamespace as NS
    bridge = make_bridge(bridge_module)
    bridge._amcl_map_ready_ns = 0
    bridge._localization_backend_started_ns = 10_000_000_000
    for name, sec, message in [('amcl', 9, 'Received a 238 X 335 map @ 0.050 m/pix'),
                                ('map_server', 11, 'Received a 238 X 335 map @ 0.050 m/pix'),
                                ('amcl', 11, 'Activating')]:
        bridge._on_amcl_log(NS(name=name, stamp=NS(sec=sec,nanosec=0), msg=message))
    assert bridge._amcl_map_ready_ns == 0
    bridge._on_amcl_log(NS(name='amcl', stamp=NS(sec=12,nanosec=0),
                          msg='Received a 238 X 335 map @ 0.050 m/pix'))
    assert bridge._amcl_map_ready_ns == 12_000_000_000


@pytest.mark.parametrize('map_stamp,frame,allowed', [(10,'odom',True), (8,'odom',False),
                                                   (11,'odom',False), (10,'map',False)])
def test_motion_requires_current_odom_frame_costmap(bridge_module, monkeypatch, map_stamp, frame, allowed):
    from types import SimpleNamespace as NS
    from test_localization_motion import costmap, scan
    bridge = motion_bridge(bridge_module)
    bridge._motion_inputs = bridge_module.WebBridge._motion_inputs.__get__(bridge)
    bridge._lock = threading.Lock()
    bridge._motion_odom = pose_message(10)
    bridge._motion_odom.header.frame_id = 'odom'
    bridge._motion_odom.child_frame_id = 'base_link'
    bridge._motion_odom.pose.pose.position.x = 0
    bridge._motion_odom.pose.pose.position.y = 0
    bridge._motion_odom.twist = NS(twist=NS(linear=NS(x=0,y=0), angular=NS(z=0)))
    laser = scan([2.0]*360)
    laser.header = pose_message(10).header
    bridge._scans = [laser]
    bridge._local_costmap = costmap()
    bridge._local_costmap.header = pose_message(map_stamp).header
    bridge._local_costmap.header.frame_id = frame
    bridge._tf_buffer = Mock()
    bridge._tf_buffer.lookup_transform.return_value = NS(transform=NS(
        translation=NS(x=0,y=0), rotation=NS(x=0,y=0,z=0,w=1)))
    monkeypatch.setattr(bridge_module.rclpy, 'time', NS(Time=Mock()), raising=False)
    bridge._hb_lock = threading.Lock()
    bridge._hb_alive = True
    bridge._hb_last = bridge_module.time.monotonic()
    bridge._hb_timeout = 1.5
    _, fresh, clear, _, _ = bridge._motion_inputs()
    assert fresh is allowed
    assert clear is allowed


@pytest.mark.parametrize('state,stationary', [('ready', False), ('stale', True), ('missing', True)])
def test_handoff_always_requires_live_stationary_odometry(bridge_module, monkeypatch, state, stationary):
    """Manual motion and missing data cannot bypass checks before automatic turns."""
    bridge = motion_bridge(bridge_module)
    bridge._stationary_observation = bridge_module.WebBridge._stationary_observation.__get__(bridge)
    bridge._odometry_state = Mock(return_value=((0, 0, 0), {'state': state}, stationary))
    worker = Mock()
    monkeypatch.setattr(bridge_module.threading, 'Thread', worker)
    for sec in (10, 11, 12):
        bridge.get_clock.return_value.now.return_value.nanoseconds = sec * 1_000_000_000
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._map_phase == 'localizing'
    assert bridge._localization_samples == 0
    worker.assert_not_called()
    bridge._odometry_state.return_value = ((0, 0, 0), {'state': 'ready'}, True)
    # Settling ends at 13 seconds; old observations must not unlock the map.
    for sec in (14, 15, 16):
        bridge.get_clock.return_value.now.return_value.nanoseconds = sec * 1_000_000_000
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._map_phase == 'starting_mapping'
    worker.return_value.start.assert_called_once()


def test_delayed_pose_cannot_authorize_handoff(bridge_module):
    """A fresh stationary robot does not make an old AMCL estimate current."""
    bridge = motion_bridge(bridge_module)
    bridge._stationary_observation = bridge_module.WebBridge._stationary_observation.__get__(bridge)
    bridge._odometry_state = Mock(return_value=((0, 0, 0), {'state': 'ready'}, True))
    for sec in (3, 4, 5):
        bridge._on_amcl_pose(pose_message(sec))
    assert bridge._localization_samples == 0


def test_slow_mapping_start_recovers_without_reloading(bridge_module):
    """Late healthy output from this session recovers after the startup warning."""
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'starting_mapping'
    bridge._mapping_process = Mock()
    bridge._mapping_process.poll.return_value = None
    bridge._mapping_started_at = bridge_module.time.monotonic() - 46
    bridge._mapping_readiness = bridge_module.MappingReadiness(2_000_000_000)
    bridge._check_mapping_startup()
    assert bridge._map_phase == 'waiting_mapping'
    bridge._observe_mapping_output('map', pose_message(1).header)
    bridge._observe_mapping_output('pose', pose_message(50).header)
    assert bridge._map_phase == 'waiting_mapping'
    bridge._observe_mapping_output('map', pose_message(51).header)
    assert bridge._map_phase == 'updating'
    assert bridge._map_phase_error is None


def test_dead_mapping_backend_cannot_recover_from_queued_output(bridge_module, monkeypatch):
    """Backend failure stays an error even when its final messages arrive late."""
    bridge = make_bridge(bridge_module)
    bridge._map_phase = 'waiting_mapping'
    bridge._mapping_process = Mock()
    bridge._mapping_process.poll.return_value = 1
    bridge._mapping_started_at = bridge_module.time.monotonic() - 46
    bridge._mapping_readiness = bridge_module.MappingReadiness(2_000_000_000)
    bridge._observe_mapping_output('pose', pose_message(50).header)
    bridge._observe_mapping_output('map', pose_message(51).header)
    assert bridge._map_phase != 'updating'
    monkeypatch.setattr(bridge_module.threading, 'Thread', Mock())
    bridge._check_mapping_startup()
    assert bridge._map_phase == 'recovering_localization'


def test_explicit_resume_rearms_existing_session_but_rechecks_guards(bridge_module):
    """A tab stop is permanent until resume; resume itself never sends movement."""
    bridge = motion_bridge(bridge_module)
    bridge._auto_localization_pending = True
    bridge.drive(0, 0, 0)
    bridge._automatic_localization()
    assert bridge._localization_motion is None
    bridge.resume_localization()
    assert bridge._auto_localization_pending
    assert not bridge._auto_localization_cancelled
    assert bridge._localization_generation == 1
    bridge._localization_started_at -= 4
    bridge._motion_inputs.return_value = ((0,0,0), False, False, True, True)
    bridge._automatic_localization()
    assert bridge._localization_motion is None
    bridge._motion_inputs.return_value = ((0,0,0), True, True, True, True)
    bridge._automatic_localization()
    assert bridge._localization_motion.active


def test_external_backend_is_not_killed_or_replaced(bridge_module):
    """Multiple installations can share a host without name-based termination."""
    bridge = make_bridge(bridge_module)
    bridge.get_node_names = Mock(return_value=['slam_toolbox', 'amcl'])
    bridge._slam_backend = Mock(process=None)
    bridge._localization_backend = Mock(process=None)
    with pytest.raises(ValueError, match='Externally managed'):
        bridge._require_managed_backends()
    bridge._slam_backend.stop.assert_not_called()
    bridge._localization_backend.stop.assert_not_called()


def test_backend_stop_discards_old_nomotion_request(bridge_module):
    """A request to a replaced AMCL instance must not block the next session."""
    bridge = make_bridge(bridge_module)
    bridge._localization_backend = Mock()
    future = bridge._nomotion_future = Mock()
    bridge._stop_localization_backend()
    future.cancel.assert_called_once()
    assert bridge._nomotion_future is None


def test_stop_waits_for_discovery_before_next_load(bridge_module, monkeypatch):
    """Dead owned nodes in the DDS cache must not look like external providers."""
    bridge = make_bridge(bridge_module)
    backend = Mock(process=object())
    bridge.get_node_names.side_effect = [['amcl', 'map_server'], ['map_server'], []]
    pause = Mock()
    monkeypatch.setattr(bridge_module.time, 'sleep', pause)
    bridge._stop_backend(backend, {'amcl', 'map_server'})
    backend.stop.assert_called_once()
    assert pause.call_count == 2


def test_paused_localization_updates_health_without_broadening_search(bridge_module):
    """Missing scans must not consume the local search window or hide diagnostics."""
    bridge = motion_bridge(bridge_module)
    bridge._auto_localization_cancelled = True
    bridge._localization_fresh_since = bridge_module.time.monotonic() - 100
    bridge._sensor_health = {'odometry': {'state': 'ready'}, 'lidar': {'state': 'stale'}}
    bridge._nomotion_client = Mock()
    bridge._update_stationary_localization()
    bridge._motion_inputs.assert_called_once()
    assert bridge._localization_fresh_since is None
    assert bridge._localization_search_scope == 'remembered'
    bridge._nomotion_client.call_async.assert_not_called()


def test_missing_lidar_does_not_blame_healthy_odometry(bridge_module):
    """The progress message should identify the failed source, not a generic pair."""
    bridge = motion_bridge(bridge_module)
    bridge._lock = threading.Lock()
    bridge._scans = []
    bridge._local_costmap = None
    bridge._odometry_state = Mock(return_value=((0,0,0), {'state': 'ready', 'age_s': .01}, True))
    bridge._hb_lock = threading.Lock()
    bridge._hb_alive = True
    bridge._hb_last = bridge_module.time.monotonic()
    bridge._hb_timeout = 1.5
    bridge._motion_inputs = bridge_module.WebBridge._motion_inputs.__get__(bridge)
    _, fresh, clear, _, _ = bridge._motion_inputs()
    assert not fresh and not clear
    assert 'LiDAR: missing' in bridge._motion_blocker
    assert 'Odometry:' not in bridge._motion_blocker
    assert bridge._sensor_health['odometry']['state'] == 'ready'
