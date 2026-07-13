#!/usr/bin/env python3
"""Thin full-bringup wrapper around unified.launch.py.

Kept for backwards compatibility with the documented command
``ros2 launch jetank_ros_main main.launch.py``. All bringup logic lives in
unified.launch.py (URDF + motor + stereo + IMU + lidar, plus optional
layers); this wrapper only pins ``enable_web_control:=false`` to preserve
main.launch.py's historical hardware-only scope. Other unified arguments
(``enable_moveit``, ``enable_navigation``, ...) pass straight through when
set on the CLI.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    unified_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetank_ros_main'),
                'launch',
                'unified.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_web_control': 'false',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        unified_launch,
    ])
