#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    # Config lives with the node it configures (jetank_motor_control), not
    # here -- co-locating them keeps the declared parameter names and the
    # YAML keys that set them from drifting apart unnoticed.
    config = os.path.join(
        get_package_share_directory('jetank_motor_control'),
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
