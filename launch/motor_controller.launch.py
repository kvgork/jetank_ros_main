#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    config = os.path.join(
        get_package_share_directory('jetank_ros_main'),
        'config',
        'motor_params.yaml'
    )

    # Robot controller node
    robot_controller_node = Node(
        package='jetank_motor_control',
        executable='robot_controller',
        name='robot_controller',
        parameters=[config],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        robot_controller_node,
    ])
