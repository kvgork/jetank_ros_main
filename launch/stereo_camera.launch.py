#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetank_perception'),
                'launch',
                'stereo_camera.launch.py'
            ])
        ]),
        # launch_arguments={
        #     'namespace': 'my_robot/stereo',
        #     'camera_width': '1280',  # Override config file value
        #     'log_level': 'debug'
        # }.items()
    )
    return LaunchDescription([stereo_launch])
