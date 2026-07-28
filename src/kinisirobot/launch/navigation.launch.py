"""Autonomous navigation bring-up for the Kinisi OMNI robot (Nav2 + AMCL).

Localizes against a previously-saved map and drives the robot to goals sent as
`nav2_msgs/action/NavigateToPose` (e.g. from the web client "Set Goal" tool or
RViz "2D Goal Pose").

Prerequisites (run these first / alongside):
  * kinisi_controller node   -> /odom + odom->base_link TF, consumes /cmd_vel
  * rplidar                  -> /scan on frame laser_frame
  * base_link->laser_frame static TF (yaw=pi)  [rsp.launch.py provides all three]
  * robot_state_publisher    (rsp.launch.py)
Do NOT run slam_toolbox at the same time -- both slam and map_server+amcl provide
map->odom, and running both fights over TF.

Typical usage on the Pi:
  ros2 launch kinisirobot rsp.launch.py          # controller + lidar + TF
  ros2 launch kinisirobot navigation.launch.py   # map_server + amcl + nav2

Override the map:
  ros2 launch kinisirobot navigation.launch.py map:=/abs/path/to/map.yaml

NOTE: use the AS-RECORDED map (kinisi_map/simple_map), not the normalized one.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kinisirobot')
    default_map = os.path.join(pkg_share, 'maps', 'kinisi_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the as-recorded map YAML to navigate against.')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameters YAML.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (false on the real robot).')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically transition lifecycle nodes to active.')

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml}],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': localization_nodes}],
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

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_localization,
        lifecycle_navigation,
    ])
