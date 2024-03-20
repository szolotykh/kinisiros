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

    #ros2 run rplidar_ros rplidar_composition --ros-args -p channel_type:=serial -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p angle_compensate:=true -p frame_id:=laser_frame -p scan_mode:=Sensitivity -p inverted:=false
    # rplidar node
    node_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'angle_compensate': True,
            'frame_id': 'laser_frame',
            'scan_mode': 'Sensitivity',
            'inverted': False}]
    )

    # Nav2 (Not running yet, just for reference)
    config_dir = os.path.join(get_package_share_directory('kinisirobot'), 'config')
    amcl_config_file = os.path.join(config_dir, 'amcl_config.yaml')
    node_av2_amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_file],
        )
    
    # ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
    # static_transform_publisher node
    node_static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.112', '0', '0.075', '3.14', '0', '3.14', 'base_link', 'laser_frame']
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        #node_kinisi_controller,
        #node_rplidar,
        node_static_transform_publisher
    ])