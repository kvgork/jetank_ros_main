#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    lidar_source_arg = DeclareLaunchArgument(
        'lidar_source',
        default_value='rplidar',
        description='LiDAR source: rplidar (C1M1 hardware) or pointcloud (stereo camera)'
    )

    lidar_source = LaunchConfiguration('lidar_source')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # use_rplidar for URDF: true when lidar_source is rplidar (enables laser TF frame)
    use_rplidar = PythonExpression(
        ["'true' if '", lidar_source, "' == 'rplidar' else 'false'"]
    )

    # URDF launch (includes laser TF frame when use_rplidar=true)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetank_ros_main'),
                'launch',
                'urdf.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rplidar': use_rplidar
        }.items()
    )

    # Motor controller launch
    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetank_ros_main'),
                'launch',
                'motor_controller.launch.py'
            ])
        ])
    )

    # Stereo camera launch
    stereo_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetank_perception'),
                'launch',
                'stereo_camera.launch.py'
            ])
        ])
    )

    # Laser scan source: C1M1 rplidar hardware or pointcloud conversion
    laser_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('jetank_navigation'),
                'launch',
                'laser_scan_converter.launch.py'
            ])
        ]),
        launch_arguments={
            'lidar_source': lidar_source
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        lidar_source_arg,
        urdf_launch,
        motor_controller_launch,
        stereo_camera_launch,
        laser_scan_launch,
    ])
