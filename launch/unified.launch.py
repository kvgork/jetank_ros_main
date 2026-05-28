#!/usr/bin/env python3
"""
Unified launch file for complete JeTank robot system.

Launches integrated system with:
  - Robot description (URDF/TF)
  - Motor control (base mobility + odometry)
  - Stereo perception (depth + point cloud)
  - Laser scan converter (navigation sensor)
  - Web control interface (optional, browser-based remote control)
  - MoveIt2 arm control (optional, manipulation)
  - Navigation stack (optional, SLAM or Nav2)

Launch arguments:
  use_sim_time:       Use simulation clock (default: false)
  enable_web_control: Enable browser remote control (default: true)
  web_port:           Port for web control server (default: 8080)
  enable_navigation:  Enable Nav2/SLAM (default: false)
  enable_moveit:      Enable MoveIt2 arm control (default: false)
  navigation_mode:    'slam' or 'nav2' (default: 'slam')
  map_file:           Map YAML for nav2 mode (default: '')

Usage:
  # Full system (hardware) with web control:
  ros2 launch jetank_ros_main unified.launch.py

  # Disable web control:
  ros2 launch jetank_ros_main unified.launch.py enable_web_control:=false

  # Custom web port:
  ros2 launch jetank_ros_main unified.launch.py web_port:=9090

  # Base mobility only (no navigation/arm):
  ros2 launch jetank_ros_main unified.launch.py \\
    enable_navigation:=false enable_moveit:=false

  # Navigation with existing map:
  ros2 launch jetank_ros_main unified.launch.py \\
    navigation_mode:=nav2 map_file:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_moveit_actions(context, *args, **kwargs):
    """Build the MoveIt2 actions on demand.

    Loaded lazily (only when enable_moveit:=true) so the launch file does
    not parse the SRDF / URDF / planning configs at import time. If
    jetank_moveit_config is missing files, this raises only when MoveIt
    is actually requested.
    """
    # Import inside the function so 'moveit_configs_utils' is not required
    # for users who never enable MoveIt.
    from moveit_configs_utils import MoveItConfigsBuilder

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_jetank_description = get_package_share_directory('jetank_description')
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
            publish_robot_description_semantic=True,
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                moveit_config.robot_description['robot_description'],
                value_type=str,
            )},
            PathJoinSubstitution([
                FindPackageShare('jetank_motor_control'),
                'config',
                'jetank_controllers.yaml',
            ]),
            {'use_sim_time': use_sim_time},
        ],
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'{controller}_spawner',
            arguments=[controller, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        )
        for controller in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')
    ]

    # Wrap XML string parameters so launch_ros doesn't try to parse them as YAML.
    moveit_params = moveit_config.to_dict()
    for xml_key in ('robot_description', 'robot_description_semantic'):
        if xml_key in moveit_params and isinstance(moveit_params[xml_key], str):
            moveit_params[xml_key] = ParameterValue(moveit_params[xml_key], value_type=str)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_params, {'use_sim_time': use_sim_time}],
    )

    return [ros2_control_node, *spawners, move_group_node]


def generate_launch_description():
    """Generate unified launch description."""

    # ============================================================================
    # PACKAGE DIRECTORIES
    # ============================================================================
    pkg_jetank_main = get_package_share_directory('jetank_ros_main')
    pkg_jetank_perception = get_package_share_directory('jetank_perception')
    pkg_jetank_navigation = get_package_share_directory('jetank_navigation')
    # jetank_description is only needed by the lazy MoveIt builder.

    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_web_control = LaunchConfiguration('enable_web_control')
    web_port = LaunchConfiguration('web_port')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_moveit = LaunchConfiguration('enable_moveit')
    navigation_mode = LaunchConfiguration('navigation_mode')
    map_file = LaunchConfiguration('map_file')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_web_control = DeclareLaunchArgument(
        'enable_web_control',
        default_value='true',
        description='Enable browser-based web control interface'
    )

    declare_web_port = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the web control HTTP server'
    )

    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable navigation stack (SLAM/Nav2)'
    )

    declare_enable_moveit = DeclareLaunchArgument(
        'enable_moveit',
        default_value='false',
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
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rplidar': 'true'
        }.items()
    )

    # Static TF: world → base_footprint (MoveIt2 virtual joint)
    world_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([
            "'", enable_moveit, "' == 'true'"
        ]))
    )

    # ============================================================================
    # WEB CONTROL (Conditional)
    # ============================================================================

    pkg_web_control = get_package_share_directory('jetank_web_control')

    web_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_web_control, 'launch', 'web_control.launch.py')
        ),
        launch_arguments={'web_port': web_port}.items(),
        condition=IfCondition(PythonExpression([
            "'", enable_web_control, "' == 'true'"
        ]))
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

    # IMU (ICM-20948 on Waveshare IMX219-83 Stereo Camera module)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'imu.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Laser scan source: C1M1 RPLidar hardware
    laser_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_navigation, 'launch', 'lidar.launch.py')
        )
    )

    # ============================================================================
    # LAYER 3: MOVEIT2 ARM CONTROL (Conditional, lazy-loaded)
    # ============================================================================
    # The MoveIt nodes are built inside _build_moveit_actions(), which is only
    # invoked when enable_moveit:=true. That keeps MoveItConfigsBuilder (which
    # reads + parses several SRDF/URDF/YAML files) out of import-time, so a
    # missing or broken jetank_moveit_config does NOT prevent the base system
    # from launching.

    moveit_group = GroupAction(
        [OpaqueFunction(function=_build_moveit_actions)],
        condition=IfCondition(enable_moveit),
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
                "'", enable_navigation, "' == 'true' and '", navigation_mode, "' == 'slam'"
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
                "'", enable_navigation, "' == 'true' and '", navigation_mode, "' == 'nav2'"
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
            '  Use Sim Time:   ', use_sim_time, '\n',
            '  Web Control:    ', enable_web_control, ' (port ', web_port, ')\n',
            '  Navigation:     ', enable_navigation, ' (', navigation_mode, ')\n',
            '  MoveIt2:        ', enable_moveit, '\n',
            '  LiDAR: RPLidar C1M1 (hardware)\n',
            '  IMU: ICM-20948 (imu/data_raw, imu/magnetic_field)\n',
            '  Map File:       ', map_file, '\n',
            '========================================\n'
        ]
    )

    # ============================================================================
    # BUILD LAUNCH DESCRIPTION
    # ============================================================================

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_web_control)
    ld.add_action(declare_web_port)
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
    ld.add_action(imu_launch)
    ld.add_action(laser_scan_launch)

    # Web control (conditional)
    ld.add_action(web_control_launch)

    # Layer 3: MoveIt2 (conditional, lazy-loaded - see _build_moveit_actions)
    ld.add_action(moveit_group)

    # Layer 4: Navigation (conditional)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)

    return ld
