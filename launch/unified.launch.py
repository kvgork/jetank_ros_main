#!/usr/bin/env python3
"""
Unified launch file for complete JeTank robot system.

Launches integrated system with:
  - Robot description (URDF/TF)
  - Motor control (base mobility + odometry)
  - Stereo perception (depth + point cloud)
  - Laser scan converter (navigation sensor)
  - MoveIt2 arm control (optional, manipulation)
  - Navigation stack (optional, SLAM or Nav2)

Launch arguments:
  use_sim_time: Use simulation clock (default: false)
  enable_navigation: Enable Nav2/SLAM (default: true)
  enable_moveit: Enable MoveIt2 arm control (default: true)
  navigation_mode: 'slam' or 'nav2' (default: 'slam')
  map_file: Map YAML for nav2 mode (default: '')

Usage:
  # Full system (hardware):
  ros2 launch jetank_ros_main unified.launch.py

  # Full system (simulation):
  ros2 launch jetank_ros_main unified.launch.py use_sim_time:=true

  # Base mobility only (no navigation/arm):
  ros2 launch jetank_ros_main unified.launch.py \\
    enable_navigation:=false enable_moveit:=false

  # Navigation with existing map:
  ros2 launch jetank_ros_main unified.launch.py \\
    navigation_mode:=nav2 map_file:=/path/to/map.yaml

  # Arm control only (no navigation):
  ros2 launch jetank_ros_main unified.launch.py \\
    enable_navigation:=false enable_moveit:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Generate unified launch description."""

    # ============================================================================
    # PACKAGE DIRECTORIES
    # ============================================================================
    pkg_jetank_main = get_package_share_directory('jetank_ros_main')
    pkg_jetank_perception = get_package_share_directory('jetank_perception')
    pkg_jetank_navigation = get_package_share_directory('jetank_navigation')
    pkg_jetank_description = get_package_share_directory('jetank_description')

    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_moveit = LaunchConfiguration('enable_moveit')
    navigation_mode = LaunchConfiguration('navigation_mode')
    map_file = LaunchConfiguration('map_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable navigation stack (SLAM/Nav2)'
    )

    declare_enable_moveit = DeclareLaunchArgument(
        'enable_moveit',
        default_value='true',
        description='Enable MoveIt2 arm control'
    )

    declare_navigation_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='slam',
        choices=['slam', 'nav2'],
        description='Navigation mode: slam for mapping, nav2 for navigation'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to map YAML file (required for nav2 mode)'
    )

    # ============================================================================
    # LAYER 1: ROBOT DESCRIPTION
    # ============================================================================

    # Robot state publisher (URDF + TF tree)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'urdf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Static TF: world → base_footprint (MoveIt2 virtual joint)
    world_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_moveit)
    )

    # ============================================================================
    # LAYER 2: HARDWARE INTERFACES
    # ============================================================================

    # Motor controller (base mobility + odometry)
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_main, 'launch', 'motor_controller.launch.py')
        )
    )

    # Stereo camera (perception pipeline)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_perception, 'launch', 'stereo_camera.launch.py')
        ),
        launch_arguments={
            'namespace': 'stereo_camera',
            'publish_camera_transforms': 'false'  # TF handled by URDF
        }.items()
    )

    # Laser scan converter (PointCloud2 → LaserScan)
    laser_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'laser_scan_converter.launch.py')
        )
    )

    # ============================================================================
    # LAYER 3: MOVEIT2 ARM CONTROL (Conditional)
    # ============================================================================

    # MoveIt2 configuration
    robot_description_file = os.path.join(
        pkg_jetank_description, 'urdf', 'jetank_ros2_control.urdf.xacro'
    )

    moveit_config = (
        MoveItConfigsBuilder('jetank', package_name='jetank_moveit_config')
        .robot_description(file_path=robot_description_file)
        .robot_description_semantic(file_path='config/jetank.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    # ros2_control node (hardware interface for arm)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([
                FindPackageShare('jetank_motor_control'),
                'config',
                'jetank_controllers.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_moveit)
    )

    # Controller spawners (delayed start for proper initialization)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_moveit)
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_moveit)
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper_controller_spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_moveit)
    )

    # MoveIt2 move_group (motion planning server)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_moveit)
    )

    # ============================================================================
    # LAYER 4: NAVIGATION STACK (Conditional)
    # ============================================================================

    # SLAM Toolbox (mapping mode)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
                enable_navigation,
                " and '", navigation_mode, "' == 'slam'"
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Nav2 stack (navigation mode)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
                enable_navigation,
                " and '", navigation_mode, "' == 'nav2'"
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items()
    )

    # ============================================================================
    # LAUNCH INFORMATION
    # ============================================================================

    launch_info = LogInfo(
        msg=[
            '\n========================================\n',
            'JeTank Unified System Launch\n',
            '========================================\n',
            '  Use Sim Time: ', use_sim_time, '\n',
            '  Navigation: ', enable_navigation, ' (', navigation_mode, ')\n',
            '  MoveIt2: ', enable_moveit, '\n',
            '  Map File: ', map_file, '\n',
            '========================================\n'
        ]
    )

    # ============================================================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================================================

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_navigation)
    ld.add_action(declare_enable_moveit)
    ld.add_action(declare_navigation_mode)
    ld.add_action(declare_map_file)

    # Launch info
    ld.add_action(launch_info)

    # Layer 1: Robot description
    ld.add_action(urdf_launch)
    ld.add_action(world_to_base_tf)

    # Layer 2: Hardware interfaces
    ld.add_action(motor_launch)
    ld.add_action(camera_launch)
    ld.add_action(laser_scan_launch)

    # Layer 3: MoveIt2 (conditional)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(gripper_controller_spawner)
    ld.add_action(move_group_node)

    # Layer 4: Navigation (conditional)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)

    return ld
