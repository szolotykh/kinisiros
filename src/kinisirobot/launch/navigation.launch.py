"""Autonomous navigation bring-up for the Kinisi OMNI robot (Nav2).

Two map backends, selected with the `slam` argument:

  slam:=false  (default) -- LOCALIZATION mode.
      map_server + amcl localize against a previously-saved static map. The
      map does not change while you drive.

  slam:=true             -- UPDATE-MAP mode.
      slam_toolbox (online async, mapping) provides map->odom AND a /map that
      keeps updating, so the robot refines / extends the map while it drives
      to goals. Optionally continue from a saved map with `slam_map`.

In BOTH modes the Nav2 navigation servers (controller/planner/behavior/
bt_navigator) run so you can send `nav2_msgs/action/NavigateToPose` goals
(web client "Set Goal", or RViz "2D Goal Pose").

Prerequisites (run these first / alongside):
  * kinisi_controller node   -> /odom + odom->base_link TF, consumes /cmd_vel
  * rplidar                  -> /scan on frame laser_frame
  * base_link->laser_frame static TF (yaw=pi)  [rsp.launch.py provides all three]
  * robot_state_publisher    (rsp.launch.py)

Do NOT run a separate slam_toolbox at the same time -- in update-map mode this
launch already starts one, and running two providers of map->odom fights over TF.

Typical usage on the Pi:
  ros2 launch kinisirobot rsp.launch.py                     # controller + lidar + TF
  ros2 launch kinisirobot navigation.launch.py              # localize on static map
  ros2 launch kinisirobot navigation.launch.py slam:=true   # update the map while navigating

Continue updating a previously-saved map (basename, no extension -- expects
<name>.posegraph + <name>.data next to it):
  ros2 launch kinisirobot navigation.launch.py slam:=true \
       slam_map:=/home/szolotykh/development/kinisiros/maps/kinisi_map

Override the static map (localization mode only):
  ros2 launch kinisirobot navigation.launch.py map:=/abs/path/to/map.yaml

NOTE: for localization mode use the AS-RECORDED map (kinisi_map/simple_map),
not the normalized one.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    slam_map = LaunchConfiguration('slam_map').perform(context)
    slam_params = LaunchConfiguration('slam_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    # --- Localization backend (slam:=false): map_server + amcl --------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=UnlessCondition(slam),
        parameters=[params_file,
                    {'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml}],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=UnlessCondition(slam),
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=UnlessCondition(slam),
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['map_server', 'amcl']}],
    )

    # --- Update-map backend (slam:=true): slam_toolbox (mapping) ------------
    # Provides both map->odom and a continuously-updated /map. Continue from a
    # saved map by passing slam_map (basename of a serialized .posegraph/.data).
    slam_overrides = {'use_sim_time': use_sim_time, 'mode': 'mapping'}
    if slam_map:
        slam_overrides['map_file_name'] = slam_map
        slam_overrides['map_start_at_dock'] = True
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(slam),
        parameters=[slam_params, slam_overrides],
    )

    # --- Navigation servers (both modes) -----------------------------------
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': navigation_nodes}],
    )

    return [
        map_server, amcl, lifecycle_localization,
        slam_toolbox,
        controller_server, planner_server, behavior_server, bt_navigator,
        lifecycle_navigation,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('kinisirobot')
    default_map = os.path.join(pkg_share, 'maps', 'kinisi_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true = update-map mode (slam_toolbox mapping); '
                        'false = localize against a static map (map_server+amcl).'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Static map YAML to localize against (slam:=false only).'),
        DeclareLaunchArgument(
            'slam_map', default_value='',
            description='Basename of a serialized map to continue updating '
                        '(slam:=true only). Empty = start a fresh map.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 parameters YAML.'),
        DeclareLaunchArgument(
            'slam_params', default_value=default_slam_params,
            description='slam_toolbox parameters YAML (slam:=true only).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (false on the real robot).'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically transition lifecycle nodes to active.'),
        OpaqueFunction(function=launch_setup),
    ])
