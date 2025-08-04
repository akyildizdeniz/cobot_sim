#!/usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('cobot_sim')
    
    # Default config file path
    default_config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the robot configuration YAML file'
    )

    return LaunchDescription([
        # Launch argument
        config_file_arg,
        
        # Nodes with parameters where applicable
        Node(
            package='cobot_sim', 
            executable='proximity_sensor_node',   
            name='proximity_sensor', 
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='cobot_sim', 
            executable='emergency_stop_monitor_node',    
            name='estop_monitor', 
            output='screen'
        ),
        Node(
            package='cobot_sim', 
            executable='speed_controller_node',   
            name='speed_controller', 
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='cobot_sim', 
            executable='state_logger_node',       
            name='state_logger', 
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='cobot_sim', 
            executable='visualizer_node',         
            name='visualizer', 
            output='screen'
        ),
        
        # Robot speed control node with parameters
        Node(
            package='cobot_sim', 
            executable='robot_motion_control_node', 
            name='robot_motion_control', 
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
    ])