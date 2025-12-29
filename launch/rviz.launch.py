#!/usr/bin/env python3
"""
RViz visualization launch file for JeTank robot.

Launches RViz with comprehensive configuration displaying:
  - Robot model + TF tree
  - Camera images (left/right/depth)
  - Point cloud
  - Laser scan
  - Map (SLAM/Nav2)
  - AMCL particle cloud
  - Navigation plans (global/local)
  - Costmaps (global/local)
  - Odometry + velocity visualization

Designed for remote operation:
  - Run on laptop while robot runs on Jetson
  - Connect via ROS_DOMAIN_ID and network

Usage:
  # Launch RViz only (robot running elsewhere):
  ros2 launch jetank_ros_main rviz.launch.py

  # With simulation time:
  ros2 launch jetank_ros_main rviz.launch.py use_sim_time:=true

  # Custom config file:
  ros2 launch jetank_ros_main rviz.launch.py \\
    rviz_config:=/path/to/custom.rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate RViz launch description."""

    # Get package directory
    pkg_jetank_main = get_package_share_directory('jetank_ros_main')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_jetank_main, 'rviz', 'unified.rviz'),
        description='Path to RViz configuration file'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        rviz_node
    ])
