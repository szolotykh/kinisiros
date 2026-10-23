import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('kinisirobot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # kinisi_controller node
    #ros2 run roskinisi kinisi_controller --ros-args -p port:=/dev/ttyACM0
    node_kinisi_controller = Node(
        package='roskinisi',
        executable='kinisi_controller',
        name='kinisi_controller',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0'}]
    )

    # The supervisor owns the RPLIDAR process, monitors /scan, and restarts
    # boundedly on startup failure, driver exit, or a stale scan stream.
    node_rplidar = Node(
        package='kinisirobot',
        executable='kinisi_lidar_supervisor',
        name='lidar_supervisor',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'scan_mode': LaunchConfiguration('lidar_scan_mode'),
            'angle_compensate': True,
            'startup_timeout_sec': LaunchConfiguration('lidar_startup_timeout'),
            'scan_timeout_sec': LaunchConfiguration('lidar_scan_timeout'),
            'inverted': False}]
    )
    
    # robot_state_publisher owns the laser mount from lidar.xacro. A second
    # publisher for laser_frame would give consumers conflicting TF parents.

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_scan_mode', default_value='Standard',
            description='RPLIDAR scan mode supported by the connected model.'),
        DeclareLaunchArgument(
            'lidar_serial_port', default_value='',
            description='RPLIDAR serial device. Empty selects a unique recognized LiDAR adapter by-id; missing/ambiguous devices wait without guessing.'),
        DeclareLaunchArgument(
            'lidar_startup_timeout', default_value='12.0',
            description='Seconds to wait for the first scan before restarting the driver.'),
        DeclareLaunchArgument(
            'lidar_scan_timeout', default_value='3.0',
            description='Seconds without scans before restarting the driver.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        node_kinisi_controller,
        node_rplidar
    ])
