#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='stereo_camera_launch').find('stereo_camera_launch')
    
    # Path to config files
    config_file = os.path.join(pkg_share, 'config', 'stereo_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    
    return LaunchDescription([
        # Stereo camera node
        Node(
            package='jetson_stereo_camera',
            executable='jetson_stereo_node',
            name='jetson_stereo_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        
        # # Static transforms
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='left_camera_tf',
        #     arguments=['0.0', '0.03', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'camera_left_link']
        # ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='right_camera_tf',
        #     arguments=['0.0', '-0.03', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'camera_right_link']
        # )
    ])